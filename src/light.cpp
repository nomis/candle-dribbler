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
#include <ha/esp_zigbee_ha_standard.h>

namespace nutt {

Light::Light(size_t index, int switch_pin, int relay_pin)
		: index_(index), switch_pin_(switch_pin), relay_pin_(relay_pin),
		primary_ep_(*new light::PrimaryEndpoint{*this}),
		status_ep_(*new light::StatusEndpoint{*this}) {
}

void Light::attach(Device &device) {
	device.add(*this, {
		primary_ep_,
		*new light::SecondaryEndpoint{*this},
		status_ep_,
		*new light::TemporaryEnableEndpoint{*this},
		*new light::PersistentEnableEndpoint{*this},
	});
}

void Light::primary_switch(bool state) {
	ESP_LOGI(TAG, "Light %u primary switch %d", index_, state);
}

void Light::secondary_switch(bool state) {
	ESP_LOGI(TAG, "Light %u secondary switch %d", index_, state);
}

void Light::temporary_enable(bool state) {
	ESP_LOGI(TAG, "Light %u temporary enable %d", index_, state);
}

void Light::persistent_enable(bool state) {
	ESP_LOGI(TAG, "Light %u persistent enable %d", index_, state);
}


namespace light {

PrimaryEndpoint::PrimaryEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		light_(light) {
}

void PrimaryEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t light_cfg = {
		.on_off = 0,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

uint8_t PrimaryEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			light_.primary_switch(*(uint8_t *)value != 0);
			return 0;
		}
	}
	return -1;
}

SecondaryEndpoint::SecondaryEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		light_(light) {
}

void SecondaryEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t light_cfg = {
		.on_off = 0,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&light_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

uint8_t SecondaryEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			light_.secondary_switch(*(uint8_t *)value != 0);
			return 0;
		}
	}
	return -1;
}

StatusEndpoint::StatusEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		light_(light) {
}

void StatusEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
}

TemporaryEnableEndpoint::TemporaryEnableEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CONFIGURATION_TOOL_DEVICE_ID),
		light_(light) {
}

void TemporaryEnableEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t switch_cfg = {
		.on_off = 1,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&switch_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

uint8_t TemporaryEnableEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			light_.temporary_enable(*(uint8_t *)value != 0);
			return 0;
		}
	}
	return -1;
}

PersistentEnableEndpoint::PersistentEnableEndpoint(Light &light)
		: ZigbeeEndpoint(BASE_EP_ID + light.index(),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_CONFIGURATION_TOOL_DEVICE_ID),
		light_(light) {
}

void PersistentEnableEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_on_off_cluster_cfg_t switch_cfg = {
		.on_off = 1,
	};

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(&cluster_list,
		esp_zb_on_off_cluster_create(&switch_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

uint8_t PersistentEnableEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
		if (attr_id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
			light_.persistent_enable(*(uint8_t *)value != 0);
			return 0;
		}
	}
	return -1;
}

} // namespace light

} // namespace nutt
