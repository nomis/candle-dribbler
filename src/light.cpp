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
#include <nvs.h>
#include <nvs_handle.hpp>
#include <ha/esp_zigbee_ha_standard.h>

#include <memory>
#include <string>

namespace nutt {

std::unique_ptr<nvs::NVSHandle> Light::nvs_;

Light::Light(size_t index, int switch_pin, int relay_pin)
		: index_(index), switch_pin_(switch_pin), relay_pin_(relay_pin),
		persistent_enable_(persistent_enable_nvs()),
		temporary_enable_(persistent_enable_),
		primary_ep_(*new light::PrimaryEndpoint{*this}),
		secondary_ep_(*new light::SecondaryEndpoint{*this}),
		status_ep_(*new light::StatusEndpoint{*this}),
		temporary_enable_ep_(*new light::TemporaryEnableEndpoint{*this}) {
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
	device.add(*this, {
		primary_ep_,
		secondary_ep_,
		status_ep_,
		temporary_enable_ep_,
		*new light::PersistentEnableEndpoint{*this},
	});
}

void Light::primary_switch(bool state) {
	ESP_LOGI(TAG, "Light %u primary switch %d", index_, state);
	primary_on_ = state;
	update_state();
}

void Light::secondary_switch(bool state) {
	ESP_LOGI(TAG, "Light %u secondary switch %d", index_, state);
	secondary_on_ = state;
	update_state();
}

void Light::temporary_enable(bool state) {
	ESP_LOGI(TAG, "Light %u temporary enable %d", index_, state);
	temporary_enable_ = state;
	update_state();
}

void Light::persistent_enable(bool state) {
	ESP_LOGI(TAG, "Light %u persistent enable %d", index_, state);
	persistent_enable_nvs(state);
	persistent_enable_ = state;
	temporary_enable_ = state;
	update_state();
}

void Light::update_state() {
	on_ = (temporary_enable_ && primary_on_) || secondary_on_;
	ESP_LOGI(TAG, "Light %u state %d", index_, on_);
	status_ep_.refresh();
	primary_ep_.refresh();
	secondary_ep_.refresh();
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
			light_.primary_switch(state_);
			return 0;
		}
	}
	return -1;
}

void PrimaryEndpoint::refresh() {
	bool new_state = light_.primary_on();

	if (new_state != state_) {
		uint8_t value = new_state ? 1 : 0;

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

StatusEndpoint::StatusEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		light_(light), state_(light_.on()) {
}

void StatusEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t light_cfg = {
		.on_off = state_,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
}

void StatusEndpoint::refresh() {
	bool new_state = light_.on();

	if (new_state != state_) {
		uint8_t value = new_state ? 1 : 0;

		update_attr_value(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
			ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE,
			ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
			&value);
		state_ = new_state;
	}
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
