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

#include "nutt/zigbee.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_zigbee_core.h>

#include <algorithm>
#include <string_view>

extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
	nutt::ZigbeeDevice::instance_->signal_handler(
		static_cast<esp_zb_app_signal_type_t>(*signal_struct->p_app_signal),
		signal_struct->esp_err_status,
		esp_zb_app_signal_get_params(signal_struct->p_app_signal));
}

namespace nutt {

ZigbeeDevice *ZigbeeDevice::instance_{nullptr};
uint8_t ZigbeeDevice::power_source_{4}; /* DC */

ZigbeeString::ZigbeeString(const std::string_view text) {
	size_t length = std::max(text.length(), MAX_LENGTH);

	value_.reserve(1 + text.length());
	value_.push_back(text.length());
	value_.insert(value_.end(), text.cbegin(), text.cbegin() + length);
}

std::string ZigbeeAddress::to_string() const {
	std::vector<char> data(24);

	snprintf(data.data(), data.size(),
		"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		value_[7], value_[6], value_[5], value_[4],
		value_[3], value_[2], value_[1], value_[0]);

	return {data.data(), data.size()};
}

ZigbeeDevice::ZigbeeDevice() {
	assert(!instance_);
	instance_ = this;

	esp_zb_platform_config_t platform_config{};

	platform_config.radio_config.radio_mode = RADIO_MODE_NATIVE;
	platform_config.host_config.host_connection_mode = HOST_CONNECTION_MODE_NONE;

	ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

	esp_zb_cfg_t config{};

	config.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
	config.install_code_policy = false,
	config.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16MIN;
	config.nwk_cfg.zed_cfg.keep_alive = 3000; /* milliseconds */

	esp_zb_init(&config);

	endpoint_list_ = esp_zb_ep_list_create();
}

void ZigbeeDevice::add(ZigbeeEndpoint &endpoint) {
	if (endpoints_.emplace(endpoint.id(), endpoint).second) {
		auto *cluster_list = esp_zb_zcl_cluster_list_create();
		esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(nullptr);

		/*
		 * Adding clusters to the list in a different order from the order they
		 * were created causes the wrong roles to be configured 😣
		 *
		 * Reusing the basic cluster works but it may be unsupported, so create
		 * it every time.
		 */

		/* Integers are used by reference but strings are immediately [copied] by value 🤷 */
		ESP_ERROR_CHECK(esp_zb_cluster_update_attr(basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source_));

		endpoint.configure_basic_cluster(*basic_cluster);

		ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list,
			basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

		endpoint.configure_cluster_list(*cluster_list);

		ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoint_list_, cluster_list,
			endpoint.id(), endpoint.profile_id(), endpoint.device_id()));

		endpoint.attach(*this);
	}
}

void ZigbeeDevice::start() {
	ESP_ERROR_CHECK(esp_zb_device_register(endpoint_list_));
	esp_zb_device_add_set_attr_value_cb(attr_value_cb);
	ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK));
	ESP_ERROR_CHECK(esp_zb_start(true /* TODO true */));
	xTaskCreate(task, "zigbee_main", 4096, this, 5, NULL); /* TODO std::thread */
}

void ZigbeeDevice::run() {
	esp_zb_main_loop_iteration();
	ESP_LOGE(TAG, "Zigbee main loop stopped");
	esp_restart();
}

void ZigbeeDevice::task(void *arg) {
	ZigbeeDevice *dev = static_cast<ZigbeeDevice*>(arg);
	dev->run();
}

void ZigbeeDevice::attr_value_cb(uint8_t status, uint8_t endpoint_id, uint16_t cluster_id, uint16_t attr_id, void *value) {
	instance_->set_attr_value(endpoint_id, cluster_id, attr_id, value);
}

uint8_t ZigbeeDevice::set_attr_value(uint8_t endpoint_id, uint16_t cluster_id, uint16_t attr_id, void *value) {
	auto it = endpoints_.find(endpoint_id);

	if (it != endpoints_.end()) {
		return it->second.set_attr_value(cluster_id, attr_id, value);
	} else {
		return -1;
	}
}

void ZigbeeDevice::update_attr_value(uint8_t endpoint_id, uint16_t cluster_id, uint8_t cluster_role, uint16_t attr_id, void *value) {
	esp_zb_zcl_set_attribute_val(endpoint_id, cluster_id, cluster_role, attr_id, value, false);
}

ZigbeeEndpoint::ZigbeeEndpoint(ep_id_t id, uint16_t profile_id, uint16_t device_id)
	: id_(id), profile_id_(profile_id), device_id_(device_id) {
}

void ZigbeeEndpoint::attach(ZigbeeDevice &device) {
	device_ = &device;
}

uint8_t ZigbeeEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) {
	ESP_LOGW("ZigbeeEndpoint", "set_attr_value not supported");
	return -1;
}

void ZigbeeEndpoint::update_attr_value(uint16_t cluster_id, uint8_t cluster_role, uint16_t attr_id, void *value) {
	if (device_) {
		device_->update_attr_value(id_, cluster_id, cluster_role, attr_id, value);
	}
}

inline void ZigbeeDevice::signal_handler(esp_zb_app_signal_type_t type, esp_err_t status, void *data) {
	switch (type) {
	case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
		break;

	case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
		ESP_LOGI(TAG, "Zigbee stack initialized");
		ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION));
		break;

	case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
		if (status == ESP_OK) {
			ESP_LOGI(TAG, "Start network steering");
			ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING));
		} else {
			/* commissioning failed */
			ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %d)", status);
		}
		break;

	case ESP_ZB_BDB_SIGNAL_STEERING:
		if (status == ESP_OK) {
			esp_zb_ieee_addr_t extended_pan_id;
			esp_zb_get_extended_pan_id(extended_pan_id);
			ESP_LOGI(TAG, "Joined network successfully (%s/0x%04x@%u)",
					ZigbeeAddress{extended_pan_id}.to_string().c_str(),
					esp_zb_get_pan_id(), esp_zb_get_current_channel());
		} else {
			ESP_LOGI(TAG, "Network steering was not successful (status: %d)", status);
			esp_zb_scheduler_alarm(start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
		}
		break;

	case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
		if (status == ESP_OK && data) {
			struct device_unavailable_params {
				esp_zb_ieee_addr_t long_addr;
				uint16_t short_addr;
			} *params = static_cast<struct device_unavailable_params*>(data);

			ESP_LOGW(TAG, "Device unavailable (%s/0x%04x)",
				ZigbeeAddress{params->long_addr}.to_string().c_str(),
				params->short_addr);
		}
		break;

	case ESP_ZB_NLME_STATUS_INDICATION:
		if (status == ESP_OK && data) {
			struct nlme_status_indication {
				uint8_t status;
				uint16_t network_addr;
				uint8_t unknown_command_id;
			} __attribute__((packed)) *params = static_cast<struct nlme_status_indication*>(data);

			ESP_LOGW(TAG, "NLME status indication: %02x 0x%04x %02x",
				params->status, params->network_addr, params->unknown_command_id);
		}
		break;

	default:
		ESP_LOGW(TAG, "Unknown signal: %u/0x%02x, status: %d, data: %p", type, type, status, data);
		break;
	}
}

void ZigbeeDevice::start_top_level_commissioning(uint8_t mode_mask) {
	ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

} // namespace nutt
