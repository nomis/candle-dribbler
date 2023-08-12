/*
 * candle-dribbler - ESP32 Zigbee light controller
 * Copyright 2023  Simon Arlott
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "nutt/light.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <nvs.h>
#include <nvs_handle.hpp>
#include <driver/gpio.h>
#include <ha/esp_zigbee_ha_standard.h>

#include <memory>
#include <string>
#include <thread>

#ifndef ESP_ZB_HA_ON_OFF_LIGHT_SWITCH_DEVICE_ID
# define ESP_ZB_HA_ON_OFF_LIGHT_SWITCH_DEVICE_ID 0x0103
#endif

namespace nutt {

std::unique_ptr<nvs::NVSHandle> Light::nvs_;

Light::Light(size_t index, gpio_num_t switch_pin, bool switch_active_low,
		gpio_num_t relay_pin, bool relay_active_low) : index_(index),
		switch_pin_(switch_pin), switch_active_low_(switch_active_low),
		relay_pin_(relay_pin), relay_active_low_(relay_active_low),
		enable_(enable_nvs()),
		primary_ep_(*new light::PrimaryEndpoint{*this}),
		secondary_ep_(*new light::SecondaryEndpoint{*this}),
		tertiary_ep_(*new light::TertiaryEndpoint{*this}),
		switch_status_ep_(*new light::SwitchStatusEndpoint{*this}) {
	gpio_config_t switch_config = {
		.pin_bit_mask = 1ULL << switch_pin_,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config_t relay_config = {
		.pin_bit_mask = 1ULL << relay_pin_,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	ESP_ERROR_CHECK(gpio_config(&switch_config));
	ESP_ERROR_CHECK(gpio_set_level(relay_pin_, relay_inactive()));
	ESP_ERROR_CHECK(gpio_config(&relay_config));

	switch_change_state_ = switch_state_ = gpio_get_level(switch_pin_);
	switch_active_ = switch_state_ == switch_active();
	ESP_LOGI(TAG, "Light %u switch is %d", index_, switch_active_);

	ESP_ERROR_CHECK(gpio_isr_handler_add(switch_pin_, light_interrupt_handler, this));
}

bool Light::open_nvs() {
	if (!nvs_) {
		nvs_ = nvs::open_nvs_handle("light", NVS_READWRITE, nullptr);
	}

	return !!nvs_;
}

bool Light::enable_nvs() {
	if (open_nvs()) {
		uint8_t value = 1;

		if (nvs_->get_item(("e" + std::to_string(index_)).c_str(), value) == ESP_OK) {
			return value != 0;
		}
	}

	return true;
}

void Light::enable_nvs(bool state) {
	if (open_nvs()) {
		uint8_t value = state ? 1 : 0;

		nvs_->set_item(("e" + std::to_string(index_)).c_str(), value);
		nvs_->commit();
	}
}

void Light::attach(Device &device) {
	device_ = &device;
	device.add(*this, {
		primary_ep_,
		secondary_ep_,
		tertiary_ep_,
		switch_status_ep_,
		*new light::EnableEndpoint{*this},
	});

	ESP_ERROR_CHECK(gpio_set_intr_type(switch_pin_, GPIO_INTR_ANYEDGE));
	ESP_ERROR_CHECK(gpio_intr_enable(switch_pin_));
}

unsigned long Light::run() {
	unsigned long wait_ms = ULONG_MAX;
	unsigned long switch_change_count_copy = switch_change_count_irq_;
	uint64_t now_us = esp_timer_get_time();
	int level = gpio_get_level(switch_pin_);

	if (switch_change_count_ != switch_change_count_copy) {
		switch_change_count_ = switch_change_count_copy;
		switch_change_us_ = now_us;
	}

	if (switch_change_state_ != level) {
		switch_change_state_ = level;
		switch_change_us_ = now_us;
	}

	if (switch_state_ != switch_change_state_) {
		if (now_us - switch_change_us_ >= DEBOUNCE_US) {
			switch_state_ = switch_change_state_;
			switch_active_ = switch_state_ == switch_active();
			primary_switch(switch_active_, true);
		} else {
			wait_ms = (DEBOUNCE_US - (now_us - switch_change_us_)) / 1000U;
		}
	}

	return wait_ms;
}

void light_interrupt_handler(void *arg) {
	static_cast<Light*>(arg)->interrupt_handler();
}

void Light::interrupt_handler() {
	Device *device = device_;

	switch_change_count_irq_++;

	if (device)
		device->wake_up_isr();
}

bool Light::primary_on() const {
	std::lock_guard lock{mutex_};
	return primary_on_;
}

bool Light::secondary_on() const {
	std::lock_guard lock{mutex_};
	return secondary_on_;
}

bool Light::tertiary_on() const {
	std::lock_guard lock{mutex_};
	return tertiary_on_;
}

bool Light::switch_on() const {
	std::lock_guard lock{mutex_};
	return switch_active_;
}

bool Light::enable() const {
	std::lock_guard lock{mutex_};
	return enable_;
}

bool Light::on() const {
	return on_;
}

void Light::primary_switch(bool state, bool local) {
	std::lock_guard lock{mutex_};

	ESP_LOGI(TAG, "Light %u set primary switch %d -> %d (%s)",
		index_, primary_on_, state, local ? "local" : "remote");
	primary_on_ = state;

	if (!state) {
		secondary_switch_locked(state, local);
	}
	update_state();
}

void Light::secondary_switch(bool state, bool local) {
	std::lock_guard lock{mutex_};

	secondary_switch_locked(state, local);
	update_state();
}

void Light::secondary_switch_locked(bool state, bool local) {
	ESP_LOGI(TAG, "Light %u set secondary switch %d -> %d (%s)",
		index_, secondary_on_, state, local ? "local" : "remote");
	secondary_on_ = state;
}

void Light::tertiary_switch(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGI(TAG, "Light %u set tertiary switch %d -> %d",
		index_, tertiary_on_, state);
	tertiary_on_ = state;
	update_state();
}

void Light::enable(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGI(TAG, "Light %u set enable %d -> %d",
		index_, enable_, state);
	enable_nvs(state);
	enable_ = state;
	update_state();
}

void Light::update_state() {
	bool on = (enable_ && primary_on_) || secondary_on_ || tertiary_on_;
	ESP_LOGI(TAG, "Light %u update state %d -> %d", index_, on_, on);
	on_ = on;
	gpio_set_level(relay_pin_, on_ ? relay_active() : relay_inactive());
	request_refresh();
}

void Light::request_refresh() {
	device_->request_refresh();
}

void Light::refresh() {
	primary_ep_.refresh();
	secondary_ep_.refresh();
	tertiary_ep_.refresh();
	switch_status_ep_.refresh();
}

namespace light {

PrimaryEndpoint::PrimaryEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		light_(light), state_(light_.primary_on()) {
}

void PrimaryEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t light_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

esp_err_t PrimaryEndpoint::set_attr_value(uint16_t cluster_id,
		uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			if (data->type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
					&& data->size == sizeof(uint8_t)) {
				state_ = *(uint8_t *)data->value != 0;
				light_.primary_switch(state_, false);
				return ESP_OK;
			}
		}
	}
	return ESP_ERR_INVALID_ARG;
}

void PrimaryEndpoint::refresh() {
	bool new_state = light_.primary_on();

	if (new_state != state_) {
		uint8_t value = new_state ? 1 : 0;

		ESP_LOGI(TAG, "Light %u report primary switch %u", light_.index(), value);

		update_attr_value(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value);
		state_ = new_state;
	}
}

SecondaryEndpoint::SecondaryEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		light_(light), state_(light_.secondary_on()) {
}

void SecondaryEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t light_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void SecondaryEndpoint::refresh() {
	bool new_state = light_.secondary_on();

	if (new_state != state_) {
		uint8_t value = new_state ? 1 : 0;

		ESP_LOGI(TAG, "Light %u report secondary switch %u", light_.index(), value);

		update_attr_value(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value);
		state_ = new_state;
	}
}

esp_err_t SecondaryEndpoint::set_attr_value(uint16_t cluster_id,
		uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			if (data->type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
					&& data->size == sizeof(uint8_t)) {
				state_ = *(uint8_t *)data->value != 0;
				light_.secondary_switch(state_, false);
				return ESP_OK;
			}
		}
	}
	return ESP_ERR_INVALID_ARG;
}

TertiaryEndpoint::TertiaryEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		light_(light), state_(light_.tertiary_on()) {
}

void TertiaryEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t light_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void TertiaryEndpoint::refresh() {
	bool new_state = light_.tertiary_on();

	if (new_state != state_) {
		uint8_t value = new_state ? 1 : 0;

		ESP_LOGI(TAG, "Light %u report tertiary switch %u", light_.index(), value);

		update_attr_value(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value);
		state_ = new_state;
	}
}

esp_err_t TertiaryEndpoint::set_attr_value(uint16_t cluster_id,
		uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			if (data->type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
					&& data->size == sizeof(uint8_t)) {
				state_ = *(uint8_t *)data->value != 0;
				light_.tertiary_switch(state_);
				return ESP_OK;
			}
		}
	}
	return ESP_ERR_INVALID_ARG;
}

uint32_t SwitchStatusEndpoint::type_{
	  (  0x03 << 24)  /* Group: Binary Input            */
	| (  0x00 << 16)  /* Type:  Application Domain HVAC */
	|  0x004B         /* Index: Lighting Status BI      */
};

SwitchStatusEndpoint::SwitchStatusEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_SWITCH_DEVICE_ID),
		light_(light), state_(light_.switch_on() ? 1 : 0) {
}

void SwitchStatusEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_binary_input_cluster_cfg_t input_cfg = {
		.out_of_service = 0,
		.status_flags = 0,
	};

	esp_zb_attribute_list_t *input_cluster = esp_zb_binary_input_cluster_create(&input_cfg);

	ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID, &state_));

	ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_APPLICATION_TYPE_ID, &type_));

	ESP_ERROR_CHECK(esp_zb_binary_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_DESCRIPTION_ID,
			ZigbeeString("Light " + std::to_string(light_.index())).data()));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_binary_input_cluster(&cluster_list,
		input_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void SwitchStatusEndpoint::refresh() {
	uint8_t new_state = light_.switch_on() ? 1 : 0;

	if (new_state != state_) {
		ESP_LOGI(TAG, "Light %u report switch state %u", light_.index(), new_state);

		update_attr_value(ESP_ZB_ZCL_CLUSTER_ID_BINARY_INPUT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_BINARY_INPUT_PRESENT_VALUE_ID,
			&new_state);
		state_ = new_state;
	}
}

esp_err_t SwitchStatusEndpoint::set_attr_value(uint16_t cluster_id,
		uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			if (data->type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
					&& data->size == sizeof(uint8_t)) {
				state_ = *(uint8_t *)data->value != 0;
				light_.request_refresh();
				return ESP_OK;
			}
		}
	}
	return ESP_ERR_INVALID_ARG;
}

EnableEndpoint::EnableEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CONFIGURATION_TOOL_DEVICE_ID),
		light_(light), state_(light_.enable()) {
}

void EnableEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t switch_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&switch_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

esp_err_t EnableEndpoint::set_attr_value(uint16_t cluster_id,
		uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			if (data->type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
					&& data->size == sizeof(uint8_t)) {
				state_ = *(uint8_t *)data->value != 0;
				light_.enable(state_);
				return ESP_OK;
			}
		}
	}
	return ESP_ERR_INVALID_ARG;
}

} // namespace light

} // namespace nutt
