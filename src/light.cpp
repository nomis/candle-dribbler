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
#include <freertos/FreeRTOS.h>
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
		persistent_enable_(persistent_enable_nvs()),
		temporary_enable_(persistent_enable_),
		primary_ep_(*new light::PrimaryEndpoint{*this}),
		secondary_ep_(*new light::SecondaryEndpoint{*this}),
		switch_status_ep_(*new light::SwitchStatusEndpoint{*this}),
		temporary_enable_ep_(*new light::TemporaryEnableEndpoint{*this}) {
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

	ESP_ERROR_CHECK(gpio_isr_handler_add(switch_pin_, light_interrupt_handler, this));
}

bool Light::open_nvs() {
	if (!nvs_) {
		nvs_ = nvs::open_nvs_handle("light", NVS_READWRITE, nullptr);
	}

	return !!nvs_;
}

bool Light::persistent_enable_nvs() {
	if (open_nvs()) {
		uint8_t value = 1;

		if (nvs_->get_item(("e" + std::to_string(index_)).c_str(), value) == ESP_OK) {
			return value != 0;
		}
	}

	return true;
}

void Light::persistent_enable_nvs(bool state) {
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
		switch_status_ep_,
		temporary_enable_ep_,
		*new light::PersistentEnableEndpoint{*this},
	});

	ESP_ERROR_CHECK(gpio_set_intr_type(switch_pin_, GPIO_INTR_ANYEDGE));
	ESP_ERROR_CHECK(gpio_intr_enable(switch_pin_));
}

TickType_t Light::run() {
	TickType_t wait = portMAX_DELAY;
	unsigned long switch_change_count_copy = switch_change_count_irq_;
	uint64_t now_us = esp_timer_get_time();
	int level = gpio_get_level(switch_pin_);

	if (switch_change_count_copy != switch_change_count_) {
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
			wait = std::max(static_cast<uint64_t>(0U), (DEBOUNCE_US - (now_us - switch_change_us_))) / static_cast<uint64_t>(1000U) / portTICK_PERIOD_MS;
		}
	}

	return wait;
}

void light_interrupt_handler(void *arg) {
	static_cast<Light*>(arg)->interrupt_handler();
}

void Light::interrupt_handler() {
	Device *device = device_;

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

bool Light::switch_on() const {
	std::lock_guard lock{mutex_};
	return switch_active_;
}

bool Light::temporary_enable() const {
	std::lock_guard lock{mutex_};
	return temporary_enable_;
}

bool Light::persistent_enable() const {
	std::lock_guard lock{mutex_};
	return persistent_enable_;
}

bool Light::on() const {
	return on_;
}

void Light::primary_switch(bool state, bool local) {
	std::lock_guard lock{mutex_};

	ESP_LOGI(TAG, "Light %u set primary switch %d -> %d (%s)",
		index_, primary_on_, state, local ? "local" : "remote");
	primary_on_ = state;
	update_state();
}

void Light::secondary_switch(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGI(TAG, "Light %u set secondary switch %d -> %d",
		index_, secondary_on_, state);
	secondary_on_ = state;
	update_state();
}

void Light::temporary_enable(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGI(TAG, "Light %u set temporary enable %d -> %d",
		index_, temporary_enable_, state);
	temporary_enable_ = state;
	update_state();
}

void Light::persistent_enable(bool state) {
	std::lock_guard lock{mutex_};

	ESP_LOGI(TAG, "Light %u set persistent enable %d -> %d",
		index_, persistent_enable_, state);
	persistent_enable_nvs(state);
	persistent_enable_ = state;
	ESP_LOGI(TAG, "Light %u set temporary enable %d -> %d (auto)",
		index_, temporary_enable_, state);
	temporary_enable_ = state;
	update_state();
}

void Light::update_state() {
	bool on = (temporary_enable_ && primary_on_) || secondary_on_;
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
	switch_status_ep_.refresh();
	temporary_enable_ep_.refresh();
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

uint8_t PrimaryEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			state_ = *(uint8_t *)value != 0;
			light_.primary_switch(state_, false);
			return 0;
		}
	}
	return -1;
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

uint8_t SecondaryEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			state_ = *(uint8_t *)value != 0;
			light_.secondary_switch(state_);
			return 0;
		}
	}
	return -1;
}

SwitchStatusEndpoint::SwitchStatusEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_SWITCH_DEVICE_ID),
		light_(light), state_(light_.switch_on()) {
}

void SwitchStatusEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t light_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void SwitchStatusEndpoint::refresh() {
	bool new_state = light_.switch_on();

	if (new_state != state_) {
		uint8_t value = new_state ? 1 : 0;

		ESP_LOGI(TAG, "Light %u report switch state %u", light_.index(), value);

		update_attr_value(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value);
		state_ = new_state;
	}
}

uint8_t SwitchStatusEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			state_ = *(uint8_t *)value != 0;
			light_.request_refresh();
			return 0;
		}
	}
	return -1;
}

TemporaryEnableEndpoint::TemporaryEnableEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CONFIGURATION_TOOL_DEVICE_ID),
		light_(light), state_(light_.temporary_enable()) {
}

void TemporaryEnableEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t switch_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&switch_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

void TemporaryEnableEndpoint::refresh() {
	bool new_state = light_.temporary_enable();

	if (new_state != state_) {
		uint8_t value = new_state ? 1 : 0;

		ESP_LOGI(TAG, "Light %u report temporary enable %u", light_.index(), value);

		update_attr_value(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value);
		state_ = new_state;
	}
}

uint8_t TemporaryEnableEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			state_ = *(uint8_t *)value != 0;
			light_.temporary_enable(state_);
			return 0;
		}
	}
	return -1;
}

PersistentEnableEndpoint::PersistentEnableEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CONFIGURATION_TOOL_DEVICE_ID),
		light_(light), state_(light_.persistent_enable()) {
}

void PersistentEnableEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t switch_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&switch_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

uint8_t PersistentEnableEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			state_ = *(uint8_t *)value != 0;
			light_.persistent_enable(state_);
			return 0;
		}
	}
	return -1;
}

} // namespace light

} // namespace nutt
