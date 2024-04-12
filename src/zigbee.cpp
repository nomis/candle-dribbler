/*
 * candle-dribbler - ESP32 Zigbee light controller
 * Copyright 2023-2024  Simon Arlott
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
#include <esp_ieee802154_types.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <esp_zigbee_core.h>
extern "C" {
#include <zboss_api.h>
}

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <mutex>
#include <set>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "nutt/ota.h"
#include "nutt/thread.h"

/* Check compatibility of esp-zigbee-lib and esp-zboss-lib with the current IDF */
static_assert(sizeof(esp_ieee802154_frame_info_t) == 16);
static_assert(offsetof(esp_ieee802154_frame_info_t, pending) == 0);
static_assert(offsetof(esp_ieee802154_frame_info_t, process) == 1);
static_assert(offsetof(esp_ieee802154_frame_info_t, channel) == 2);
static_assert(offsetof(esp_ieee802154_frame_info_t, rssi) == 3);
static_assert(offsetof(esp_ieee802154_frame_info_t, lqi) == 4);
static_assert(offsetof(esp_ieee802154_frame_info_t, timestamp) == 8);

extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
	nutt::ZigbeeDevice::instance_->signal_handler(
		static_cast<esp_zb_app_signal_type_t>(*signal_struct->p_app_signal),
		signal_struct->esp_err_status,
		esp_zb_app_signal_get_params(signal_struct->p_app_signal));
}

namespace nutt {

ZigbeeDevice *ZigbeeDevice::instance_{nullptr};

template<size_t Size>
static bool all_zeros(const uint8_t (&buf)[Size]) {
	return buf[0] == 0 && !std::memcmp(&buf[0], &buf[1], Size - 1);
}

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

	config.install_code_policy = false;

	if (ROUTER) {
		config.esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER;
		config.nwk_cfg.zczr_cfg.max_children = ZB_DEFAULT_MAX_CHILDREN;
	} else {
		config.esp_zb_role = ESP_ZB_DEVICE_TYPE_ED;
		config.nwk_cfg.zed_cfg.ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16MIN;
		config.nwk_cfg.zed_cfg.keep_alive = 3000; /* milliseconds */
	}

	retry_connect_ = std::make_shared<std::function<void()>>([this] { retry_connect(); });
	refresh_neighbours_ = std::make_shared<std::function<void()>>([this] { refresh_neighbours(); });

#if CONFIG_ZB_DEBUG_MODE
	esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_CRITICAL, 0);
#endif
	esp_zb_aps_src_binding_table_size_set(BINDING_TABLE_SIZE);
	esp_zb_aps_dst_binding_table_size_set(BINDING_TABLE_SIZE);
	esp_zb_init(&config);

	endpoint_list_ = esp_zb_ep_list_create();
	neighbours_ = std::make_shared<std::vector<ZigbeeNeighbour>>();
}

void ZigbeeDevice::add(ZigbeeEndpoint &endpoint) {
	if (endpoints_.emplace(endpoint.id(), endpoint).second) {
		auto *cluster_list = esp_zb_zcl_cluster_list_create();

		endpoint.configure_cluster_list(*cluster_list);

		ESP_ERROR_CHECK(esp_zb_ep_list_add_ep(endpoint_list_, cluster_list,
			{endpoint.id(), endpoint.profile_id(), endpoint.device_id(), 0}));

		endpoint.attach(*this);
	}
}

void ZigbeeDevice::start() {
	ESP_ERROR_CHECK(esp_zb_device_register(endpoint_list_));
	esp_zb_core_action_handler_register(action_handler_cb);
	ESP_ERROR_CHECK(esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK));
	ESP_ERROR_CHECK(esp_zb_start(false));

	zb_buffer_ = zb_buf_get_out();
	if (zb_buffer_ == ZB_BUF_INVALID) {
		ESP_LOGE(TAG, "ZBOSS buffer invalid");
	}

	std::thread t;
	make_thread(t, "zigbee_main", 8192, 5, &ZigbeeDevice::run_main, this);
	t.detach();
	make_thread(t, "zigbee_task", 8192, 5, &ZigbeeDevice::run_tasks, this);
	t.detach();
}

void ZigbeeDevice::run_main() {
	esp_zb_main_loop_iteration();
	assert(false);
}

void ZigbeeDevice::run_tasks() {
	std::unique_lock lock{tasks_mutex_};

	while (true) {
		auto now = clock::now();
		auto it = tasks_.begin();

		if (it == tasks_.end()) {
			tasks_cv_.wait(lock);
		} else if (it->first > now) {
			tasks_cv_.wait_until(lock, it->first);
		} else {
			auto task = it->second;

			tasks_.erase(it);

			lock.unlock();
			assert(esp_zb_lock_acquire(portMAX_DELAY));
			(*task)();
			esp_zb_lock_release();
			lock.lock();
		}
	}
}

void ZigbeeDevice::schedule_after(const std::shared_ptr<std::function<void()>> &task, uint32_t time_ms) {
	assert(task);

	std::lock_guard lock{tasks_mutex_};
	auto now = clock::now();

	tasks_.insert({now + std::chrono::milliseconds(time_ms), task});
	tasks_cv_.notify_all();
}

void ZigbeeDevice::schedule_after(std::function<void()> &&task, uint32_t time_ms) {
	schedule_after(std::make_shared<std::function<void()>>(task), time_ms);
}

void ZigbeeDevice::reschedule_after(const std::shared_ptr<std::function<void()>> &task, uint32_t time_ms) {
	assert(task);

	std::lock_guard lock{tasks_mutex_};
	auto now = clock::now();
	auto it = tasks_.begin();

	while (it != tasks_.end()) {
		if (it->second == task) {
			it = tasks_.erase(it);
		} else {
			it++;
		}
	}

	tasks_.insert({now + std::chrono::milliseconds(time_ms), task});
	tasks_cv_.notify_all();
}

void ZigbeeDevice::schedule_cancel(const std::shared_ptr<std::function<void()>> &task) {
	assert(task);

	std::lock_guard lock{tasks_mutex_};
	auto it = tasks_.begin();

	while (it != tasks_.end()) {
		if (it->second == task) {
			it = tasks_.erase(it);
		} else {
			it++;
		}
	}
}

void ZigbeeDevice::connected() {
	reschedule_after(refresh_neighbours_, REFRESH_NEIGHBOURS_MS);

	for (auto& [ep_id, ep] : endpoints_) {
		ep.configure_reporting();
	}
}

void ZigbeeDevice::disconnected() {
	schedule_cancel(refresh_neighbours_);
}

esp_err_t ZigbeeDevice::set_attr_value(const esp_zb_zcl_set_attr_value_message_t *message) {
	if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
		esp_err_t ret = ESP_ERR_INVALID_ARG;
		auto it = endpoints_.find(message->info.dst_endpoint);

		if (it != endpoints_.end()) {
			ret = it->second.set_attr_value(message->info.cluster,
				message->attribute.id, &message->attribute.data);
		}

		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "Rejected attribute write: endpoint %u cluster 0x%04x, attribute 0x%04x type 0x%04x size %u",
				message->info.dst_endpoint, message->info.cluster,
				message->attribute.id, message->attribute.data.type,
				message->attribute.data.size);
		}

		return ret;
	} else {
		ESP_LOGE(TAG, "Received invalid attribute write: endpoint %u cluster 0x%04x, attribute 0x%04x type 0x%04x size %u",
			message->info.dst_endpoint, message->info.cluster,
			message->attribute.id, message->attribute.data.type,
			message->attribute.data.size);
		return ESP_ERR_INVALID_ARG;
	}
}

esp_err_t ZigbeeDevice::ota_upgrade(const esp_zb_zcl_ota_upgrade_value_message_t *message) {
	const uint8_t *payload = message->payload;
	size_t payload_size = message->payload_size;

	if (message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS) {
		if (message->upgrade_status != ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE) {
			if (ota_receive_not_logged_) {
				ESP_LOGD(TAG, "OTA (%zu receive data messages suppressed)",
					ota_receive_not_logged_);
				ota_receive_not_logged_ = 0;
			}
			ota_last_receive_us_ = 0;
		}

		switch (message->upgrade_status) {
		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
			ESP_LOGI(TAG, "OTA start");
			ota_.reset();
			ota_header_.clear();
			ota_upgrade_subelement_ = false;
			ota_data_len_ = 0;
			ota_ = std::make_unique<CompressedOTA>();
			if (ota_->start()) {
				listener_.zigbee_ota_update(true);
			} else {
				ota_.reset();
				return ESP_FAIL;
			}
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_APPLY:
			ESP_LOGI(TAG, "OTA apply");
			listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
			/* Read and process the first sub-element, ignoring everything else */
			while (ota_header_.size() < 6 && payload_size > 0) {
				ota_header_.push_back(payload[0]);
				payload++;
				payload_size--;
			}

			if (!ota_upgrade_subelement_ && ota_header_.size() == 6) {
				if (ota_header_[0] == 0 && ota_header_[1] == 0) {
					ota_upgrade_subelement_ = true;
					ota_data_len_ =
						  (((int)ota_header_[5] & 0xFF) << 24)
						| (((int)ota_header_[4] & 0xFF) << 16)
						| (((int)ota_header_[3] & 0xFF) << 8 )
						|  ((int)ota_header_[2] & 0xFF);
					ESP_LOGD(TAG, "OTA sub-element size %zu", ota_data_len_);
				} else {
					ESP_LOGE(TAG, "OTA sub-element type %02x%02x not supported", ota_header_[0], ota_header_[1]);
					ota_.reset();
					listener_.zigbee_ota_update(false);
					return ESP_FAIL;
				}
			}

			if (ota_data_len_) {
				payload_size = std::min(ota_data_len_, payload_size);
				ota_data_len_ -= payload_size;

				if (ota_ && ota_->write(payload, payload_size)) {
					uint64_t now_us = esp_timer_get_time();

					if (!ota_last_receive_us_
							|| now_us - ota_last_receive_us_ >= 30 * 1000 * 1000) {
						ESP_LOGD(TAG, "OTA receive data (%zu messages suppressed)",
							ota_receive_not_logged_);
						ota_last_receive_us_ = now_us;
						ota_receive_not_logged_ = 0;
					} else {
						ota_receive_not_logged_++;
					}
				} else {
					ota_.reset();
					listener_.zigbee_ota_update(false);
					return ESP_FAIL;
				}
			}

			listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH:
			ESP_LOGI(TAG, "OTA finished");
			if (ota_) {
				bool ok = ota_->finish();
				ota_.reset();
				listener_.zigbee_ota_update(ok, true);
				if (ok) {
					esp_restart();
				}
			} else {
				listener_.zigbee_ota_update(false);
			}
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
			ESP_LOGI(TAG, "OTA aborted");
			listener_.zigbee_ota_update(false, true);
			ota_.reset();
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK:
			ESP_LOGI(TAG, "OTA data complete");
			listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_OK:
			ESP_LOGI(TAG, "OTA data ok");
			listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ERROR:
			ESP_LOGI(TAG, "OTA data error");
			ota_.reset();
			listener_.zigbee_ota_update(false);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_IMAGE_STATUS_NORMAL:
			ESP_LOGI(TAG, "OTA image accepted");
			listener_.zigbee_ota_update(true);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_BUSY:
			ESP_LOGI(TAG, "OTA busy");
			ota_.reset();
			listener_.zigbee_ota_update(false);
			break;

		case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_SERVER_NOT_FOUND:
			ESP_LOGI(TAG, "OTA server not found");
			ota_.reset();
			listener_.zigbee_ota_update(false);
			break;
		}
	}
	return ESP_OK;
}

esp_err_t ZigbeeDevice::action_handler_cb(esp_zb_core_action_callback_id_t callback_id,
		const void *data) {
	return instance_->action_handler(callback_id, data);
}

esp_err_t ZigbeeDevice::action_handler(esp_zb_core_action_callback_id_t callback_id,
		const void *data) {
	switch (callback_id) {
	case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
		return set_attr_value(reinterpret_cast<const esp_zb_zcl_set_attr_value_message_t*>(data));

	case ESP_ZB_CORE_OTA_UPGRADE_VALUE_CB_ID:
		return ota_upgrade(reinterpret_cast<const esp_zb_zcl_ota_upgrade_value_message_t*>(data));

	case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
		break;

	default:
		ESP_LOGW(TAG, "Unknown action: %u/0x%02x, data: %p", callback_id,
			callback_id, data);
		break;
	}

	return ESP_OK;
}

inline void ZigbeeDevice::signal_handler(esp_zb_app_signal_type_t type,
		esp_err_t status, void *data) {
	switch (type) {
	case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
	case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
		break;

	case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
	case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
	case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
		if (status == ESP_OK) {
			if (type == ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP) {
				ESP_LOGD(TAG, "Zigbee stack initialized");
				uint16_t short_address = esp_zb_get_short_address();
				esp_zb_ieee_addr_t long_address;
				esp_zb_get_long_address(long_address);

				/*
				 * If the address has not been set using the zb_fct partition it
				 * will be all zeros at startup.
				 */
				if (all_zeros(long_address)) {
					esp_read_mac(long_address, ESP_MAC_IEEE802154);
					std::reverse(std::begin(long_address), std::end(long_address));
				}

				update_state(ZigbeeState::DISCONNECTED, short_address < 0xFFF8);
				ESP_LOGI(TAG, "Device address: %s/0x%04x (network %sconfigured)",
					zigbee_address_string(long_address).c_str(),
					short_address, network_configured_ ? "" : "not ");
			}
		} else if (type == ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP) {
			ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %d)", status);
			network_failed_ = true;
			update_state(ZigbeeState::INIT);
		} else {
			ESP_LOGW(TAG, "Failed to connect (%s, %d)", esp_zb_zdo_signal_to_string(type), status);
		}

		if (state_ != ZigbeeState::INIT) {
			if (network_configured_ || state_ >= ZigbeeState::CONNECTING) {
				if (status == ESP_OK) {
					connect(esp_zb_zdo_signal_to_string(type),
						type == ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP
							? ESP_ZB_BDB_MODE_INITIALIZATION
							: ESP_ZB_BDB_MODE_NETWORK_STEERING);
				} else {
					retry();
				}
			} else {
				ESP_LOGI(TAG, "Waiting for pairing button press (%s)", esp_zb_zdo_signal_to_string(type));
				update_state(ZigbeeState::DISCONNECTED);
			}
		}
		break;

	case ESP_ZB_BDB_SIGNAL_STEERING:
		if (status == ESP_OK) {
			uint16_t short_address = esp_zb_get_short_address();
			uint16_t pan_id = esp_zb_get_pan_id();
			esp_zb_ieee_addr_t extended_pan_id;
			esp_zb_get_extended_pan_id(extended_pan_id);

			if (state_ == ZigbeeState::DISCONNECTED
					&& short_address == 0xfffe && pan_id == 0xffff
					&& all_zeros(extended_pan_id)) {
				/*
				 * Workaround false join event when leaving while connecting
				 * https://github.com/espressif/esp-zigbee-sdk/issues/66#issuecomment-1679527878
				 */
				ESP_LOGW(TAG, "Ignoring invalid join signal");
				break;
			}

			ESP_LOGI(TAG, "Joined network successfully (%s/0x%04x@%u as 0x%04x)",
					zigbee_address_string(extended_pan_id).c_str(),
					pan_id, esp_zb_get_current_channel(), short_address);
			network_failed_ = false;
			update_state(ZigbeeState::CONNECTED, true);
		} else {
			ESP_LOGW(TAG, "Failed to connect (%s, %d)", esp_zb_zdo_signal_to_string(type), status);
			network_failed_ = true;

			if (network_configured_ || state_ >= ZigbeeState::CONNECTING) {
				retry();
			}
		}
		break;

	case ESP_ZB_ZDO_SIGNAL_LEAVE:
		if (status == ESP_OK) {
			esp_zb_zdo_signal_leave_params_t *params = static_cast<esp_zb_zdo_signal_leave_params_t*>(data);

			network_failed_ = false;

			if (params && params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_REJOIN) {
				ESP_LOGE(TAG, "Device rejoin");
				update_state(ZigbeeState::RETRY, false);
				retry(true);
			} else {
				if (!params) {
					ESP_LOGE(TAG, "Device removed");
				} else if (params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
					ESP_LOGE(TAG, "Device reset");
				} else {
					ESP_LOGE(TAG, "Device removed (type: %u)", params->leave_type);
				}

				ESP_LOGI(TAG, "Waiting for pairing button press");
				cancel_retry();
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
		ESP_LOGW(TAG, "Unknown signal: %u/0x%02x, status: %d, data: %p",
			type, type, status, data);
		break;
	}
}

void ZigbeeDevice::connect(const char *why, uint8_t mode) {
	cancel_retry();

	ESP_LOGD(TAG, "Connecting (%s)", why);
	update_state(ZigbeeState::CONNECTING);

	esp_err_t err = esp_zb_bdb_start_top_level_commissioning(mode);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error connecting: %d", err);
		retry();
	}
}

void ZigbeeDevice::retry(bool quiet) {
	if (!quiet) {
		ESP_LOGD(TAG, "Retry");
	}
	update_state(ZigbeeState::RETRY);
	schedule_after(retry_connect_, 1000);
}

void ZigbeeDevice::retry_connect() {
	if (state_ == ZigbeeState::RETRY) {
		connect("retry", ESP_ZB_BDB_MODE_NETWORK_STEERING);
	}
}

void ZigbeeDevice::cancel_retry() {
	schedule_cancel(retry_connect_);
}

void ZigbeeDevice::join_network() {
	schedule([this] { join_or_leave_network(ZigbeeAction::JOIN); });
}

void ZigbeeDevice::join_or_leave_network() {
	schedule([this] { join_or_leave_network(ZigbeeAction::JOIN_OR_LEAVE); });
}

void ZigbeeDevice::leave_network() {
	schedule([this] { join_or_leave_network(ZigbeeAction::LEAVE); });
}

void ZigbeeDevice::print_bindings() {
	schedule([] {
		uint8_t buffer = zb_buf_get_out();
		if (buffer == ZB_BUF_INVALID)
			return;

		zb_zdo_mgmt_bind_param_t *args = ZB_BUF_GET_PARAM(buffer, zb_zdo_mgmt_bind_param_t);
		args->start_index = 0;
		args->dst_addr = esp_zb_get_short_address();
		zb_buf_set_status(buffer, RET_OK);

		zb_ret_t ret = zb_zdo_mgmt_bind_req(buffer, print_bindings_cb);
		if (ret == 0xFF) {
			ESP_LOGE(TAG, "Error getting bindings (start_index=%u)", args->start_index);
			zb_buf_free(buffer);
		}
	});
}

void ZigbeeDevice::print_bindings_cb(uint8_t buffer) {
	if (zb_buf_get_status(buffer) != RET_OK) {
		ESP_LOGE(TAG, "Buffer error getting bindings");
		zb_buf_free(buffer);
		return;
	}

	zb_zdo_mgmt_bind_resp_t *resp = reinterpret_cast<zb_zdo_mgmt_bind_resp_t*>(zb_buf_begin(buffer));
	zb_zdo_binding_table_record_t *entry = reinterpret_cast<zb_zdo_binding_table_record_t*>(
		reinterpret_cast<uint8_t*>(zb_buf_begin(buffer)) + sizeof(zb_zdo_mgmt_bind_resp_t));

	if (resp->status != 0) {
		ESP_LOGE(TAG, "Error getting bindings (status=%u)", resp->status);
	} else if (resp->binding_table_list_count > 0) {
		if (resp->start_index == 0) {
			ESP_LOGI(TAG, "Bindings (%u):", resp->binding_table_entries);
		}

		for (size_t i = 0; resp->start_index < resp->binding_table_entries
				&& i < resp->binding_table_list_count; i++) {
			ESP_LOGI(TAG, "[%03u] %s/%u/%04x -> %s/%u",
				resp->start_index,
				zigbee_address_string(entry->src_address).c_str(),
				entry->src_endp, entry->cluster_id,
				(entry->dst_addr_mode == ZB_BIND_DST_ADDR_MODE_16_BIT_GROUP
					? zigbee_address_string(entry->dst_address.addr_short).c_str()
					: zigbee_address_string(entry->dst_address.addr_long).c_str()),
					entry->dst_endp);
			resp->start_index++;
			entry++;
		}

		if (resp->start_index > 0
				&& resp->start_index < resp->binding_table_entries) {
			uint8_t next_index = resp->start_index;

			zb_buf_reuse(buffer);
			zb_zdo_mgmt_bind_param_t *args = ZB_BUF_GET_PARAM(buffer, zb_zdo_mgmt_bind_param_t);
			args->start_index = next_index;
			args->dst_addr = esp_zb_get_short_address();
			zb_buf_set_status(buffer, RET_OK);

			zb_ret_t ret = zb_zdo_mgmt_bind_req(buffer, print_bindings_cb);
			if (ret == ZB_ZDO_INVALID_TSN) {
				ESP_LOGE(TAG, "Error getting bindings (start_index=%u)", args->start_index);
			} else {
				return;
			}
		}
	} else if (resp->start_index == 0) {
		ESP_LOGI(TAG, "No bindings");
	}

	zb_buf_free(buffer);
}

uint16_t ZigbeeDevice::get_parent() {
	return zb_nwk_get_parent();
}

std::shared_ptr<const std::vector<ZigbeeNeighbour>> ZigbeeDevice::get_neighbours() {
	std::lock_guard lock{neighbours_mutex_};
	return neighbours_;
}

void ZigbeeDevice::join_or_leave_network(ZigbeeAction action) {
	ZigbeeAction toggle_action;

	if (state_ == ZigbeeState::INIT) {
		return;
	}

	if (network_configured_ || state_ != ZigbeeState::DISCONNECTED) {
		toggle_action = ZigbeeAction::LEAVE;
	} else {
		toggle_action = ZigbeeAction::JOIN;
	}

	if (action == ZigbeeAction::JOIN_OR_LEAVE) {
		action = toggle_action;
	} else if (action != toggle_action) {
		return;
	}

	cancel_retry();

	network_failed_ = false;

	if (action == ZigbeeAction::JOIN) {
		connect("join network", ESP_ZB_BDB_MODE_NETWORK_STEERING);
	} else if (action == ZigbeeAction::LEAVE) {
		ESP_LOGI(TAG, "Leave network");
		esp_zb_zdo_mgmt_leave_req_param_t param{};
		esp_zb_get_long_address(param.device_address);
		param.dst_nwk_addr = 0xffff;
		esp_zb_zdo_device_leave_req(&param, nullptr, nullptr);
		update_state(ZigbeeState::DISCONNECTED);
	}
}

void ZigbeeDevice::update_state(ZigbeeState state) {
	if (state_ == ZigbeeState::CONNECTED && state != ZigbeeState::CONNECTED) {
		disconnected();
	}
	state_ = state;
	if (state_ == ZigbeeState::CONNECTED) {
		connected();
	}
	listener_.zigbee_network_state(network_configured_, state_, network_failed_);
}

void ZigbeeDevice::update_state(ZigbeeState state, bool configured) {
	network_configured_ = configured;
	update_state(state);
}

void ZigbeeDevice::refresh_neighbours_cb(uint8_t buffer) {
	instance_->refresh_neighbours(buffer);
}

void ZigbeeDevice::refresh_neighbours() {
	if (zb_buffer_ == ZB_BUF_INVALID)
		return;

	zb_nwk_nbr_iterator_params_t *args = ZB_BUF_GET_PARAM(zb_buffer_, zb_nwk_nbr_iterator_params_t);
	args->update_count = 0;
	args->index = 0;
	zb_buf_set_status(zb_buffer_, RET_OK);

	new_neighbours_ = std::make_shared<std::vector<ZigbeeNeighbour>>();
	new_neighbours_->reserve(neighbours_->size());

	zb_ret_t ret = zb_nwk_nbr_iterator_next(zb_buffer_, refresh_neighbours_cb);
	if (ret != RET_OK) {
		ESP_LOGE(TAG, "Error getting neighbour table (index=%d): %d", args->index, ret);
		new_neighbours_.reset();
		schedule_after(refresh_neighbours_, 1000);
	}
}

void ZigbeeDevice::refresh_neighbours(uint8_t buffer) {
	if (zb_buf_get_status(buffer) != RET_OK) {
		ESP_LOGE(TAG, "Buffer error refreshing neighbours");
		schedule_after(refresh_neighbours_, REFRESH_NEIGHBOURS_MS);
		return;
	}

	zb_nwk_nbr_iterator_params_t *args = ZB_BUF_GET_PARAM(buffer, zb_nwk_nbr_iterator_params_t);
	zb_nwk_nbr_iterator_entry_t *entry = reinterpret_cast<zb_nwk_nbr_iterator_entry_t*>(zb_buf_begin(buffer));

	if (args->index == 0) {
		neighbour_table_update_count_ = args->update_count;
	}

	if (args->index != ZB_NWK_NBR_ITERATOR_INDEX_EOT) {
		auto type = ZigbeeDeviceType::UNKNOWN;
		auto relationship = ZigbeeNeighbourRelationship::UNKNOWN;

		if (neighbour_table_update_count_ != args->update_count) {
			ESP_LOGD(TAG, "Neighbour table updated while reading");
			new_neighbours_.reset();
			schedule_after(refresh_neighbours_, 1000);
			return;
		}

		if (entry->device_type == 0) {
			type = ZigbeeDeviceType::COORDINATOR;
		} else if (entry->device_type == 1) {
			type = ZigbeeDeviceType::ROUTER;
		} else if (entry->device_type == 2) {
			type = ZigbeeDeviceType::END_DEVICE;
		} else if (entry->device_type == 3) {
			type = ZigbeeDeviceType::UNKNOWN;
		}

		if (entry->relationship == 0) {
			relationship = ZigbeeNeighbourRelationship::PARENT;
		} else if (entry->relationship == 1) {
			relationship = ZigbeeNeighbourRelationship::CHILD;
		} else if (entry->relationship == 2) {
			relationship = ZigbeeNeighbourRelationship::SIBLING;
		} else if (entry->relationship == 3) {
			relationship = ZigbeeNeighbourRelationship::NONE;
		} else if (entry->relationship == 4) {
			relationship = ZigbeeNeighbourRelationship::FORMER_CHILD;
		} else if (entry->relationship == 5) {
			relationship = ZigbeeNeighbourRelationship::UNAUTH_CHILD;
		}

		auto addr = entry->short_addr;
		auto neighbour = ZigbeeNeighbour{ addr, type, entry->depth, relationship,
			entry->lqi, entry->rssi, {} };
		std::memcpy(neighbour.long_addr, entry->ieee_addr, sizeof(neighbour.long_addr));
		new_neighbours_->emplace_back(std::move(neighbour));

		args->index++;

		zb_ret_t ret = zb_nwk_nbr_iterator_next(buffer, refresh_neighbours_cb);
		if (ret != RET_OK) {
			ESP_LOGE(TAG, "Error getting neighbour table (index=%d): %d", args->index, ret);
			new_neighbours_.reset();
			schedule_after(refresh_neighbours_, 1000);
		}
	} else {
		if (new_neighbours_) {
			std::unique_lock lock{neighbours_mutex_};
			auto neighbours = new_neighbours_;
			neighbours_ = neighbours;
			new_neighbours_.reset();
			lock.unlock();

			listener_.zigbee_neighbours_updated(neighbours);
		}

		schedule_after(refresh_neighbours_, REFRESH_NEIGHBOURS_MS);
	}
}

ZigbeeEndpoint::ZigbeeEndpoint(ep_id_t id, esp_zb_af_profile_id_t profile_id,
		uint16_t device_id) : id_(id), profile_id_(profile_id),
		device_id_(device_id) {
}

ZigbeeEndpoint::ZigbeeEndpoint(ep_id_t id, esp_zb_af_profile_id_t profile_id,
		uint16_t device_id, ZigbeeCluster &cluster) : id_(id),
		profile_id_(profile_id), device_id_(device_id) {
	add(cluster);
}

ZigbeeEndpoint::ZigbeeEndpoint(ep_id_t id, esp_zb_af_profile_id_t profile_id,
		uint16_t device_id,
		std::vector<std::reference_wrapper<ZigbeeCluster>> &&clusters)
		: id_(id), profile_id_(profile_id), device_id_(device_id) {
	for (auto cluster : clusters)
		add(cluster);
}

void ZigbeeEndpoint::add(ZigbeeCluster &cluster) {
	if (clusters_.emplace(cluster.id(), cluster).second)
		cluster.attach(*this);
}

void ZigbeeEndpoint::attach(ZigbeeDevice &device) {
	device_ = &device;
}

void ZigbeeEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	std::set<uint16_t> cluster_ids;

	for (auto &cluster : clusters_)
		cluster_ids.emplace(cluster.first);

	for (auto &cluster_id : cluster_ids)
		clusters_.at(cluster_id).configure_cluster_list(cluster_list);
}

esp_err_t ZigbeeEndpoint::set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data) {
	auto it = clusters_.find(cluster_id);

	if (it != clusters_.end()) {
		return it->second.set_attr_value(attr_id, data);
	} else {
		return ESP_ERR_INVALID_ARG;
	}
}

void ZigbeeEndpoint::configure_reporting() {
	for (auto& [cluster_id, cluster] : clusters_) {
		cluster.configure_reporting();
	}
}

void ZigbeeCluster::configure_reporting() {
	if (role_ != ESP_ZB_ZCL_CLUSTER_SERVER_ROLE)
		return;

	for (auto attr : attrs_) {
		auto *rep_info = zb_zcl_find_reporting_info(ep_->id(), id_, role(), attr);

		if (!rep_info)
			continue;

		rep_info->u.send_info.min_interval = MIN_INTERVAL;
		rep_info->u.send_info.max_interval = MAX_INTERVAL;
		rep_info->u.send_info.def_min_interval = MIN_INTERVAL;
		rep_info->u.send_info.def_max_interval = MAX_INTERVAL;

		zb_ret_t ret = zb_zcl_put_reporting_info(rep_info, ZB_TRUE);

		if (ret != RET_OK) {
			ESP_LOGE(TAG, "%s: Unable to configure reporting for %u/%04x/%04x: %d",
				__func__, ep_->id(), id_, attr, ret);
		}
	}
}

ZigbeeCluster::ZigbeeCluster(uint16_t id, esp_zb_zcl_cluster_role_t role)
		: id_(id), role_(role) {
}

ZigbeeCluster::ZigbeeCluster(uint16_t id, esp_zb_zcl_cluster_role_t role, const std::vector<uint16_t> &attrs)
		: id_(id), role_(role), attrs_(attrs) {
}

void ZigbeeCluster::attach(ZigbeeEndpoint &ep) {
	ep_ = &ep;
}

esp_err_t ZigbeeCluster::set_attr_value(uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data) {
	return ESP_ERR_INVALID_ARG;
}

void ZigbeeCluster::update_attr_value(uint16_t attr_id, void *value) {
	esp_zb_zcl_set_attribute_val(ep_->id(), id_, role_, attr_id, value, false);
}

} // namespace nutt

using namespace nutt;

/*
 * Workaround for zb_zcl_send_report_attr_command() being called with
 * rep_info->manuf_code == 0 which doesn't exist calling
 * zb_zcl_get_attr_desc_manuf_a() result in in a null pointer dereference
 * https://github.com/espressif/esp-zigbee-sdk/issues/86
 */
extern "C" void __real_zb_zcl_send_report_attr_command(
	struct zb_zcl_reporting_info_s *rep_info, zb_uint8_t param);

extern "C" void __wrap_zb_zcl_send_report_attr_command(
		struct zb_zcl_reporting_info_s *rep_info, zb_uint8_t param) {
retry:
	zb_zcl_attr_t *attr = zb_zcl_get_attr_desc_manuf_a(rep_info->ep,
		rep_info->cluster_id, rep_info->cluster_role, rep_info->attr_id,
		rep_info->manuf_code);
	if (attr) {
		ESP_LOGV(ZigbeeDevice::TAG, "zb_zcl_send_report_attr_command: ep=%u cluster_id=%u cluster_role=%u attr_id=%u manuf_code=%u attr=%p",
			rep_info->ep, rep_info->cluster_id, rep_info->cluster_role,
			rep_info->attr_id, rep_info->manuf_code, attr);
	} else if (rep_info->manuf_code == 0) {
		ESP_LOGW(ZigbeeDevice::TAG, "zb_zcl_send_report_attr_command: fixing manuf_code for ep=%u cluster_id=%u cluster_role=%u attr_id=%u manuf_code=%u",
			rep_info->ep, rep_info->cluster_id, rep_info->cluster_role,
			rep_info->attr_id, rep_info->manuf_code);
		rep_info->manuf_code = ZB_ZCL_MANUFACTURER_WILDCARD_ID;
		goto retry;
	} else {
		ESP_LOGE(ZigbeeDevice::TAG, "zb_zcl_send_report_attr_command: invalid ep=%u cluster_id=%u cluster_role=%u attr_id=%u manuf_code=%u",
			rep_info->ep, rep_info->cluster_id, rep_info->cluster_role,
			rep_info->attr_id, rep_info->manuf_code);
		return;
	}

	__real_zb_zcl_send_report_attr_command(rep_info, param);
}
