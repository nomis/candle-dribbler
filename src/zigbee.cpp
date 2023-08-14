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

#include <esp_err.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_zigbee_core.h>

#include <algorithm>
#include <string_view>
#include <thread>

#include "nutt/thread.h"

extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
	nutt::ZigbeeDevice::instance_->signal_handler(
		static_cast<esp_zb_app_signal_type_t>(*signal_struct->p_app_signal),
		signal_struct->esp_err_status,
		esp_zb_app_signal_get_params(signal_struct->p_app_signal));
}

namespace nutt {

ZigbeeDevice *ZigbeeDevice::instance_{nullptr};

ZigbeeString::ZigbeeString(const std::string_view text, size_t max_length) {
	size_t length = std::min(text.length(), std::min(max_length, MAX_LENGTH));

	value_.reserve(1 + length);
	value_.push_back(length);
	value_.insert(value_.end(), text.cbegin(), text.cbegin() + length);
}

ZigbeeDevice::ZigbeeDevice(ZigbeeListener &listener) : listener_(listener) {
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

		endpoint.configure_cluster_list(*cluster_list);

		ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoint_list_, cluster_list,
			endpoint.id(), endpoint.profile_id(), endpoint.device_id()));

		endpoint.attach(*this);
	}
}

void ZigbeeDevice::start() {
	ESP_ERROR_CHECK(esp_zb_device_register(endpoint_list_));
	esp_zb_device_add_set_attr_value_cb(set_attr_value_cb);
	esp_zb_device_add_ota_upgrade_status_cb(ota_upgrade_status_cb);
	ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK));
	ESP_ERROR_CHECK(esp_zb_start(false));

	std::thread t;
	make_thread(t, "zigbee_main", 4096, 5, &ZigbeeDevice::run, this);
	t.detach();
}

void ZigbeeDevice::run() {
	esp_zb_main_loop_iteration();
	ESP_LOGE(TAG, "Zigbee main loop stopped");
	esp_restart();
}

esp_err_t ZigbeeDevice::set_attr_value_cb(esp_zb_zcl_set_attr_value_message_t message) {
	if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
		esp_err_t ret = instance_->set_attr_value(message.info.dst_endpoint,
			message.info.cluster, message.attribute, &message.data);

		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Rejected attribute write: endpoint %u cluster 0x%04x, attribute 0x%04x type 0x%04x size %u",
				message.info.dst_endpoint, message.info.cluster, message.attribute,
				message.data.type, message.data.size);
		}
		return ret;
	} else {
		ESP_LOGE(TAG, "Received invalid attribute write: endpoint %u cluster 0x%04x, attribute 0x%04x type 0x%04x size %u",
			message.info.dst_endpoint, message.info.cluster, message.attribute,
			message.data.type, message.data.size);
		return ESP_ERR_INVALID_ARG;
	}
}

esp_err_t ZigbeeDevice::set_attr_value(uint8_t endpoint_id, uint16_t cluster_id, uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data) {
	auto it = endpoints_.find(endpoint_id);

	if (it != endpoints_.end()) {
		return it->second.set_attr_value(cluster_id, attr_id, data);
	} else {
		ESP_LOGW(TAG, "Setting invalid attribute");
		return ESP_ERR_INVALID_ARG;
	}
}

void ZigbeeDevice::update_attr_value(uint8_t endpoint_id, uint16_t cluster_id,
		uint8_t cluster_role, uint16_t attr_id, void *value) {
	esp_zb_zcl_set_attribute_val(endpoint_id, cluster_id, cluster_role, attr_id, value, false);
}

esp_err_t ZigbeeDevice::ota_upgrade_status_cb(esp_zb_zcl_ota_update_message_t message) {
	if (message.info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
		switch (message.update_status) {
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
			ESP_LOGI(TAG, "OTA start");
			instance_->listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
			ESP_LOGI(TAG, "OTA apply");
			instance_->listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
			ESP_LOGI(TAG, "OTA receive data");
			instance_->listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
			ESP_LOGI(TAG, "OTA finished");
			instance_->listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
			ESP_LOGI(TAG, "OTA aborted");
			instance_->listener_.zigbee_ota_update(false);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
			ESP_LOGI(TAG, "OTA data complete");
			instance_->listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_OK:
			ESP_LOGI(TAG, "OTA data ok");
			instance_->listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ERROR:
			ESP_LOGI(TAG, "OTA data error");
			instance_->listener_.zigbee_ota_update(false);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_NORMAL:
			ESP_LOGI(TAG, "OTA image accepted");
			instance_->listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_BUSY:
			ESP_LOGI(TAG, "OTA busy");
			instance_->listener_.zigbee_ota_update(false);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_SERVER_NOT_FOUND:
			ESP_LOGI(TAG, "OTA server not found");
			break;
		}
	}
	return ESP_OK;
}

ZigbeeEndpoint::ZigbeeEndpoint(ep_id_t id, uint16_t profile_id, uint16_t device_id)
	: id_(id), profile_id_(profile_id), device_id_(device_id) {
}

void ZigbeeEndpoint::attach(ZigbeeDevice &device) {
	device_ = &device;
}

esp_err_t ZigbeeEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data) {
	ESP_LOGE(TAG, "set_attr_value not supported");
	return ESP_ERR_INVALID_ARG;
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
	case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
		if (status == ESP_OK) {
			if (type == ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP) {
				ESP_LOGI(TAG, "Zigbee stack initialized");
				uint16_t address = esp_zb_get_short_address();

				update_state(ZigbeeState::DISCONNECTED, address != 0xFFFF);
				ESP_LOGI(TAG, "Device address: 0x%04x (network %sconfigured)", address, network_configured_ ? "" : "not ");
			}
		} else if (type == ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP) {
			ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %d)", status);
			network_failed_ = true;
			update_state(ZigbeeState::INIT);
		} else {
			ESP_LOGI(TAG, "Failed to connect (%s, %d)", esp_zb_zdo_signal_to_string(type), status);
		}

		if (state_ != ZigbeeState::INIT) {
			if (network_configured_ || state_ == ZigbeeState::CONNECTING) {
				esp_zb_scheduler_alarm_cancel(start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING);

				if (status == ESP_OK) {
					ESP_LOGI(TAG, "Connecting (%s)", esp_zb_zdo_signal_to_string(type));
					update_state(ZigbeeState::CONNECTING);
					ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(
						type == ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP
						? ESP_ZB_BDB_MODE_INITIALIZATION
						: ESP_ZB_BDB_MODE_NETWORK_STEERING));
				} else {
					ESP_LOGI(TAG, "Retry");
					update_state(ZigbeeState::RETRY);
					esp_zb_scheduler_alarm(start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
				}
			} else {
				ESP_LOGI(TAG, "Waiting for pairing button press (%s)", esp_zb_zdo_signal_to_string(type));
				update_state(ZigbeeState::DISCONNECTED);
			}
		}
		break;

	case ESP_ZB_BDB_SIGNAL_STEERING:
		if (status == ESP_OK) {
			esp_zb_ieee_addr_t extended_pan_id;
			esp_zb_get_extended_pan_id(extended_pan_id);
			ESP_LOGI(TAG, "Joined network successfully (%s/0x%04x@%u as 0x%04x)",
					zigbee_address_string(extended_pan_id).c_str(),
					esp_zb_get_pan_id(), esp_zb_get_current_channel(),
					esp_zb_get_short_address());
			network_failed_ = false;
			update_state(ZigbeeState::CONNECTED, true);

			if (!ota_validated_) {
				esp_ota_mark_app_valid_cancel_rollback();
				ota_validated_ = true;
			}
		} else {
			ESP_LOGI(TAG, "Failed to connect (%s, %d)", esp_zb_zdo_signal_to_string(type), status);
			network_failed_ = true;
			ESP_LOGI(TAG, "Retry");
			update_state(ZigbeeState::RETRY);
			esp_zb_scheduler_alarm_cancel(start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING);
			esp_zb_scheduler_alarm(start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
		}
		break;

	case ESP_ZB_ZDO_SIGNAL_LEAVE:
		if (status == ESP_OK) {
			esp_zb_zdo_signal_leave_params_t *params = static_cast<esp_zb_zdo_signal_leave_params_t*>(data);

			network_failed_ = false;

			if (params && params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_REJOIN) {
				ESP_LOGE(TAG, "Device rejoin");
				update_state(ZigbeeState::RETRY, false);
				esp_zb_scheduler_alarm_cancel(start_top_level_commissioning, 0);
				esp_zb_scheduler_alarm(start_top_level_commissioning, 0, 1000);
			} else {
				if (!params) {
					ESP_LOGE(TAG, "Device removed");
				} else if (params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
					ESP_LOGE(TAG, "Device reset");
				} else {
					ESP_LOGE(TAG, "Device removed (type: %u)", params->leave_type);
				}

				ESP_LOGI(TAG, "Waiting for pairing button press");
				esp_zb_scheduler_alarm_cancel(start_top_level_commissioning, 0);
				update_state(ZigbeeState::DISCONNECTED, false);
			}
		}
		break;

	case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
		if (status == ESP_OK && data) {
			struct device_unavailable_params {
				esp_zb_ieee_addr_t long_addr;
				uint16_t short_addr;
			} *params = static_cast<struct device_unavailable_params*>(data);

			ESP_LOGW(TAG, "Device unavailable (%s/0x%04x)",
				zigbee_address_string(params->long_addr).c_str(),
				params->short_addr);
			listener_.zigbee_network_error();
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
			listener_.zigbee_network_error();
		}
		break;

	default:
		ESP_LOGW(TAG, "Unknown signal: %u/0x%02x, status: %d, data: %p", type, type, status, data);
		break;
	}
}

void ZigbeeDevice::start_top_level_commissioning(uint8_t mode_mask) {
	if (instance_->state_ == ZigbeeState::RETRY) {
		ESP_LOGI(TAG, "Connecting (retry)");
		instance_->update_state(ZigbeeState::CONNECTING);
		ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
	}
}

void ZigbeeDevice::network_join_or_leave() {
	if (state_ != ZigbeeState::INIT) {
		esp_zb_scheduler_alarm_cancel(start_top_level_commissioning, ESP_ZB_BDB_MODE_NETWORK_STEERING);

		network_failed_ = false;

		if (network_configured_ || state_ != ZigbeeState::DISCONNECTED) {
			ESP_LOGI(TAG, "Leave network");
			esp_zb_factory_reset();
			instance_->update_state(ZigbeeState::DISCONNECTED);
		} else {
			ESP_LOGI(TAG, "Connecting (join network)");
			instance_->update_state(ZigbeeState::CONNECTING);
			ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING));
		}
	}
}

void ZigbeeDevice::update_state(ZigbeeState state) {
	state_ = state;
	listener_.zigbee_network_state(network_configured_, state_, network_failed_);
}

void ZigbeeDevice::update_state(ZigbeeState state, bool configured) {
	network_configured_ = configured;
	update_state(state);
}

} // namespace nutt
