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

#include "nutt/device.h"

#include <esp_app_desc.h>
#include <esp_chip_info.h>
#include <esp_core_dump.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_task_wdt.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nutt/base64.h"
#include "nutt/main.h"
#include "nutt/light.h"
#include "nutt/thread.h"
#include "nutt/ui.h"
#include "nutt/util.h"
#include "nutt/zigbee.h"

using namespace std::chrono_literals;

namespace nutt {

Device::Device(UserInterface &ui) : WakeupThread("Device", true), ui_(ui),
		zigbee_(*new ZigbeeDevice{*this}),
		basic_cl_(*this, "uuid.uk",
			(MAX_LIGHTS > 0 || !ZigbeeDevice::ROUTER)
				? "candle-dribbler" : "router",
			"https://github.com/nomis/candle-dribbler"),
		identify_cl_(ui_) {
	uptime_task_ = std::make_shared<std::function<void()>>([this] {
		uint32_t next_ms = uptime_cl_.update(core_dump_present_);
		zigbee_.reschedule_after(uptime_task_, next_ms);
	});

	connected_task_ = std::make_shared<std::function<void()>>([this] {
		uint32_t next_ms = connected_cl_.update();
		zigbee_.reschedule_after(connected_task_, next_ms);
	});

	part_current_ = esp_ota_get_running_partition();
	part_next_ = esp_ota_get_next_update_partition(nullptr);
	part_boot_ = esp_ota_get_boot_partition();

	esp_ota_img_states_t state;

	if (!esp_ota_get_state_partition(part_current_, &state)) {
		if (state == ESP_OTA_IMG_VALID) {
			ota_validated_ = true;
		}
	}

	auto &main_ep = *new ZigbeeEndpoint{MAIN_EP_ID, ESP_ZB_AF_HA_PROFILE_ID,
		ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, {basic_cl_, identify_cl_, uptime_cl_}};

	if (OTA_SUPPORTED) {
		ESP_LOGD(TAG, "OTA supported");
		main_ep.add(*new device::UpgradeCluster{});
	}

	zigbee_.add(main_ep);

	auto part_count = esp_ota_get_app_partition_count();
	const esp_partition_t *part = part_current_;

	if (part->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY) {
		ota_validated_ = true;
		part = esp_ota_get_next_update_partition(part);
	}

	for (size_t i = 0; i < part_count; i++, part = esp_ota_get_next_update_partition(part)) {
		parts_.push_back(part);

		auto &software_cl = *new device::SoftwareCluster{*this, i};

		software_cls_.emplace_back(software_cl);
		zigbee_.add(*new ZigbeeEndpoint{
			static_cast<ep_id_t>(SOFTWARE_BASE_EP_ID + i),
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
			software_cl});
	}

	zigbee_.add(*new ZigbeeEndpoint{CONNECTED_EP_ID, ESP_ZB_AF_HA_PROFILE_ID,
			ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, {connected_cl_}});

	zigbee_.add(*new ZigbeeEndpoint{UPLINK_PARENT_EP_ID, ESP_ZB_AF_HA_PROFILE_ID,
			ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, {uplink_cl_}});

	zigbee_.add(*new ZigbeeEndpoint{UPLINK_RSSI_EP_ID, ESP_ZB_AF_HA_PROFILE_ID,
			ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, {rssi_cl_}});

	reload_core_dump_status();
	ui_.core_dump(core_dump_present_);
}

void Device::add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints) {
	lights_.emplace(light.index(), light);
	light_tasks_.emplace(light.index(),
			std::make_shared<std::function<void()>>([&light] {
		ESP_LOGD(TAG, "Refresh light %u", light.index());
		light.refresh();
	}));

	for (auto ep : endpoints)
		zigbee_.add(ep);
}

void Device::start() {
	zigbee_.start();
	uptime_cl_.update(core_dump_present_);
	zigbee_.schedule_after(uptime_task_, std::chrono::milliseconds(1min).count());

	std::thread t;
	make_thread(t, "device_main", 4096, 19, &Device::run_loop, this);
	t.detach();
}

void Device::request_refresh(const Light &light) {
	zigbee_.reschedule(light_tasks_.at(light.index()));
}

void Device::join_network() {
	zigbee_.join_network();
}

void Device::join_or_leave_network() {
	zigbee_.join_or_leave_network();
}

void Device::leave_network() {
	zigbee_.leave_network();
}

void Device::print_bindings() {
	zigbee_.print_bindings();
}

void Device::print_neighbours() {
	auto neighbours = zigbee_.get_neighbours();
	auto parent = zigbee_.get_parent();

	ESP_LOGI(TAG, "Neighbours (%zu):", neighbours->size());

	for (auto &neighbour : *neighbours) {
		char type = '_';
		char relationship = '_';

		switch (neighbour.type) {
		case ZigbeeDeviceType::COORDINATOR:
			type = 'C';
			break;

		case ZigbeeDeviceType::ROUTER:
			type = 'R';
			break;

		case ZigbeeDeviceType::END_DEVICE:
			type = 'E';
			break;

		case ZigbeeDeviceType::UNKNOWN:
			type = '_';
			break;
		}

		switch (neighbour.relationship) {
		case ZigbeeNeighbourRelationship::PARENT:
			relationship = 'P';
			break;

		case ZigbeeNeighbourRelationship::CHILD:
			relationship = 'C';
			break;

		case ZigbeeNeighbourRelationship::SIBLING:
			relationship = 'S';
			break;

		case ZigbeeNeighbourRelationship::NONE:
			relationship = '_';
			break;

		case ZigbeeNeighbourRelationship::FORMER_CHILD:
			relationship = 'F';
			break;

		case ZigbeeNeighbourRelationship::UNAUTH_CHILD:
			relationship = 'U';
			break;

		case ZigbeeNeighbourRelationship::UNKNOWN:
			relationship = '?';
			break;
		}

		ESP_LOGI(TAG, "%s/%04x %c %3u %c%c LQI %3u RSSI %4d",
			zigbee_address_string(neighbour.long_addr).c_str(),
			neighbour.short_addr, type, neighbour.depth, relationship,
			neighbour.short_addr == parent ? '*' : ' ',
			neighbour.lqi, neighbour.rssi);
	}
}

void Device::reload_core_dump_status() {
	const esp_partition_t *part = esp_partition_find_first(
		ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP,
		nullptr);
	uint32_t size;

	if (!part || esp_partition_read(part, 0, &size, sizeof(size)))
		return;

	core_dump_present_ = size != UINT32_MAX;
}

void Device::print_core_dump(bool full) {
	const esp_partition_t *part = esp_partition_find_first(
		ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP,
		nullptr);
	esp_err_t err;
	uint32_t size;

	if (!part) {
		ESP_LOGI(TAG, "Core dump partition not found");
	} else if ((err = esp_partition_read(part, 0, &size, sizeof(size)))) {
		ESP_LOGI(TAG, "Unable to read core dump partition: %d", err);
	} else if (size == UINT32_MAX) {
		ESP_LOGI(TAG, "No core dump");
	} else if (full) {
		if (size > part->size) {
			ESP_LOGE(TAG, "Core dump too large: %" PRIu32 " > %" PRIu32, size, part->size);
			size = part->size;
		}
		if ((err = esp_core_dump_image_check())) {
			ESP_LOGE(TAG, "Core dump invalid: %d", err);
		}

		// Increase watchdog timeout
		esp_task_wdt_config_t twdt_config{};

		twdt_config.timeout_ms = 60 * 1000;
		twdt_config.idle_core_mask = 0;
		twdt_config.trigger_panic = true;

		ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));

		printf("================= CORE DUMP START =================\n");
		for (size_t offset = 0; offset < size; ) {
			char data[48] = { 0 };
			size_t remaining = std::min(static_cast<uint32_t>(sizeof(data)), size - offset);

			if (esp_partition_read(part, offset, data, remaining))
				break;

			std::shared_ptr<char> buf(base64_encode(data, remaining, nullptr), free);
			printf("%s", buf.get());

			offset += sizeof(data);
		}
		printf("================= CORE DUMP END ===================\n");

		// Restore watchdog timeout
		twdt_config.timeout_ms = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000;
		ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&twdt_config));
	} else {
		std::shared_ptr<esp_core_dump_summary_t> summary(
			static_cast<decltype(summary)::element_type*>(
				calloc(1, sizeof(decltype(summary)::element_type))), free);

		err = esp_core_dump_get_summary(summary.get());
		if (err == ESP_OK) {
			ESP_LOGI(TAG, "PC: %08" PRIx32, summary->exc_pc);
			ESP_LOGI(TAG, "Task: %s", summary->exc_task);
		}
	}
}

void Device::erase_core_dump() {
	esp_err_t err = esp_core_dump_image_erase();

	if (err == ESP_OK) {
		ESP_LOGI(TAG, "Core dump erased");
	} else if (err == ESP_ERR_NOT_FOUND) {
		ESP_LOGI(TAG, "Core dump partition not found");
	} else {
		ESP_LOGI(TAG, "Core dump erase error: %d", err);
	}

	reload_core_dump_status();
	ui_.core_dump(core_dump_present_);

	zigbee_.reschedule(uptime_task_);
}

unsigned long Device::run_tasks() {
	unsigned long wait_ms = WATCHDOG_INTERVAL_MS;

	esp_task_wdt_reset();

	for (auto &light : lights_)
		wait_ms = std::min(wait_ms, light.second.run());

	return wait_ms;
}

void Device::make_app_info(int app_index, std::string &label, std::string &date_code, std::string &version) {
	const esp_partition_t *part = app_index < 0 ? part_current_ : parts_.at(app_index);
	const esp_app_desc_t *desc{nullptr};
	esp_app_desc_t part_desc{};

	label.clear();
	date_code.clear();
	version.clear();

	if (app_index < 0) {
		desc = esp_app_get_description();
	} else if (!esp_ota_get_partition_description(part, &part_desc)) {
		desc = &part_desc;
	}

	if (desc) {
		if (desc->date[0] != '\0') {
			std::string time = null_terminated_string(desc->time);

			for (char c : null_terminated_string(desc->date)) {
				if (c >= '0' && c <= '9')
					date_code += c;
			}

			date_code += 'T';

			for (char c : time) {
				if (c >= '0' && c <= '9') {
					date_code += c;
				} else if (c == ' ') {
					break;
				}
			}

			if (time.rfind(" +0000", std::string::npos) != std::string::npos)
				date_code += 'Z';
		}

		label += null_terminated_string(desc->project_name);
		label += " | ";

		version = null_terminated_string(desc->version);
		version += " | ";
		version += null_terminated_string(desc->idf_ver);
	}

	label += part->label;

	if (part == part_current_) {
		label += " [current]";
	}
	if (part == part_next_) {
		label += " [next]";
	}
	if (part == part_boot_) {
		label += " [boot]";
	}

	if (app_index >= 0) {
		esp_ota_img_states_t state;

		if (esp_ota_get_state_partition(part, &state))
			state = ESP_OTA_IMG_UNDEFINED;

		label += ' ';
		switch (state) {
		case ESP_OTA_IMG_NEW: label += "new"; break;
		case ESP_OTA_IMG_PENDING_VERIFY: label += "pending-verify"; break;
		case ESP_OTA_IMG_VALID: label += "valid"; break;
		case ESP_OTA_IMG_INVALID: label += "invalid"; break;
		case ESP_OTA_IMG_ABORTED: label += "aborted"; break;
		default: label += "undefined"; break;
		}
	}

	if (date_code.empty())
		date_code.append(8, '0');
}

void Device::configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster,
		int app_index) {
	std::string label, date_code, version;

	make_app_info(app_index, label, date_code, version);

	ESP_LOGD(TAG, "Date code: %s", date_code.c_str());
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
		ZigbeeString{date_code, MAX_DATE_CODE_LENGTH}.data()));

	ESP_LOGD(TAG, "Label: %s", label.c_str());
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_LABEL_ID,
		ZigbeeString{label, MAX_STRING_LENGTH}.data()));

	ESP_LOGD(TAG, "Version: %s", version.c_str());
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_VERSION_DETAILS_ID,
		ZigbeeString{version, MAX_STRING_LENGTH}.data()));
}

void Device::reload_app_info(bool full) {
	part_next_ = esp_ota_get_next_update_partition(nullptr);
	part_boot_ = esp_ota_get_boot_partition();

	basic_cl_.reload_app_info();

	for (device::SoftwareCluster &software_cl : software_cls_)
		software_cl.reload_app_info(full);
}

void Device::zigbee_network_state(bool configured, ZigbeeState state,
		bool failed) {
	auto ui_state = ui::NetworkState::FAILED;

	switch (state) {
	case ZigbeeState::INIT:
	case ZigbeeState::DISCONNECTED:
		ui_state = ui::NetworkState::DISCONNECTED;
		break;

	case ZigbeeState::RETRY:
	case ZigbeeState::CONNECTING:
		ui_state = ui::NetworkState::CONNECTING;
		break;

	case ZigbeeState::CONNECTED:
		ui_state = ui::NetworkState::CONNECTED;
		break;
	}

	if (failed) {
		ui_state = ui::NetworkState::FAILED;
	}

	ui_.network_state(configured, ui_state);

	if (state == ZigbeeState::CONNECTED) {
		if (!ota_validated_) {
			esp_ota_mark_app_valid_cancel_rollback();
			ota_validated_ = true;
			reload_app_info(false);
		}

		connected_cl_.connected();
		zigbee_.reschedule_after(connected_task_, 1000);
	} else {
		connected_cl_.disconnected();
		zigbee_.reschedule_after(connected_task_, 1000);
	}
}

void Device::zigbee_network_error() {
	ui_.network_error();
}

void Device::zigbee_ota_update(bool ok, bool app_changed) {
	ui_.ota_update(ok);

	if (app_changed) {
		reload_app_info(true);
	}
}

void Device::zigbee_neighbours_updated(const std::shared_ptr<const std::vector<ZigbeeNeighbour>> &neighbours) {
	uint16_t uplink = zigbee_.get_parent();
	int8_t rssi = -128;

	for (const auto &neighbour : *neighbours) {
		if (neighbour.short_addr == uplink) {
			rssi = neighbour.rssi;
		}
	}

	uplink_cl_.update(uplink);
	rssi_cl_.update(rssi);
}

namespace device {

uint8_t BasicCluster::power_source_{0x04}; /* DC */
uint8_t BasicCluster::device_class_{0x00}; /* Lighting */
uint8_t BasicCluster::device_type_{0xf0}; /* Generic actuator */

BasicCluster::BasicCluster(Device &device,
		const std::string_view manufacturer, const std::string_view model,
		const std::string_view url) : ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_BASIC,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE), device_(device),
		manufacturer_(manufacturer), model_(model), url_(url) {
}

void BasicCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(nullptr);

	/* Integers are used by reference but strings are immediately [copied] by value 🤷 */
	ESP_ERROR_CHECK(esp_zb_cluster_update_attr(basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source_));

	if (!manufacturer_.empty())
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ZigbeeString{manufacturer_, 32}.data()));

	if (!model_.empty())
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ZigbeeString{model_, 32}.data()));

	if (!url_.empty())
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_URL_ID, ZigbeeString{url_, 50}.data()));

	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_GENERIC_DEVICE_CLASS_ID, &device_class_));

	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_GENERIC_DEVICE_TYPE_ID, &device_type_));

	std::string serial_number;
	esp_chip_info_t chip{};
	uint8_t mac[8] = { 0 };
	std::vector<char> rev_str(6);
	std::vector<char> mac_str(18);

	esp_chip_info(&chip);
	switch (chip.model) {
	case CHIP_ESP32C6: serial_number = "ESP32-C6/"; break;
	case CHIP_ESP32H2: serial_number = "ESP32-H2/"; break;
	default: serial_number = "Unknown/"; break;
	}

	snprintf(rev_str.data(), rev_str.size(), "%02x.%02x",
		(chip.revision >> 8) & 0xFF, chip.revision & 0xFF);
	serial_number.append(rev_str.data());
	serial_number += '/';

	ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_BASE));
	snprintf(mac_str.data(), mac_str.size(), "%02x:%02x:%02x:%02x:%02x:%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	serial_number.append(mac_str.data());

	ESP_LOGD(TAG, "Serial Number: %s", serial_number.c_str());

	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_SERIAL_NUMBER_ID, ZigbeeString{serial_number, 50}.data()));

	device_.configure_basic_cluster(*basic_cluster, -1);

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(&cluster_list,
		basic_cluster, role()));
}

void BasicCluster::reload_app_info() {
	std::string label, date_code, version;

	device_.make_app_info(-1, label, date_code, version);

	ESP_LOGD(TAG, "Label: %s", label.c_str());
	update_attr_value(ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_LABEL_ID,
		ZigbeeString{label, Device::MAX_STRING_LENGTH}.data());
}

IdentifyCluster::IdentifyCluster(UserInterface &ui)
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE), ui_(ui) {
}

void IdentifyCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(&cluster_list,
		esp_zb_identify_cluster_create(nullptr), role()));
}

esp_err_t IdentifyCluster::set_attr_value(uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data) {
	if (attr_id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
		if (data->type == ESP_ZB_ZCL_ATTR_TYPE_U16
				&& data->size == sizeof(uint16_t)) {
			uint16_t identify_time = *(uint16_t *)data->value;
			ui_.identify(identify_time);
			return ESP_OK;
		}
	}
	return ESP_ERR_INVALID_ARG;
}

UpgradeCluster::UpgradeCluster()
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE,
			ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE) {
}

void UpgradeCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_ota_cluster_cfg_t ota_config{};
	ota_config.ota_upgrade_manufacturer = OTA_MANUFACTURER_ID;
	ota_config.ota_upgrade_image_type = OTA_IMAGE_TYPE_ID;
	ota_config.ota_upgrade_file_version = OTA_FILE_VERSION;

	esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_config);

	esp_zb_zcl_ota_upgrade_client_variable_t ota_client_parameters{};
	ota_client_parameters.max_data_size = UINT8_MAX;
	ota_client_parameters.timer_query = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF;

	esp_zb_ota_cluster_add_attr(ota_cluster,
		ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID,
		&ota_client_parameters);

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(&cluster_list,
		ota_cluster, role()));
}

uint32_t UptimeCluster::app_type_{
	  (  0x00 << 24)  /* Group: Analog Input    */
	| (  0x0E << 16)  /* Type:  Time in Seconds */
	|  0x0000         /* Index: Relative time   */
};

uint16_t UptimeCluster::units_{70}; /* Time - Days */

UptimeCluster::UptimeCluster()
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			{
				ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID,
				ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RELIABILITY_ID,
				ESP_ZB_ZCL_ATTR_ANALOG_INPUT_STATUS_FLAGS_ID,
			}) {
}

void UptimeCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *input_cluster = esp_zb_analog_input_cluster_create(nullptr);

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_APPLICATION_TYPE_ID, &app_type_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &units_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
			ZigbeeString("Uptime").data()));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RELIABILITY_ID, &reliability_));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(&cluster_list,
		input_cluster, role()));
}

uint32_t UptimeCluster::update(bool fault) {
	uint64_t uptime_us = esp_timer_get_time();

	uptime_ = uptime_us / static_cast<float>(std::chrono::microseconds(24h).count());
	update_attr_value(ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &uptime_);

	if (fault) {
		reliability_ = ESP_ZB_ZCL_ANALOG_INPUT_RELIABILITY_UNRELIABLE_OTHER;
		status_flags_ = ESP_ZB_ZCL_ANALOG_INPUT_STATUS_FLAG_FAULT;
	} else {
		reliability_ = ESP_ZB_ZCL_ANALOG_INPUT_RELIABILITY_NO_FAULT_DETECTED;
		status_flags_ = ESP_ZB_ZCL_ANALOG_INPUT_STATUS_FLAG_NORMAL;
	}
	update_attr_value(ESP_ZB_ZCL_ATTR_ANALOG_INPUT_RELIABILITY_ID, &reliability_);
	update_attr_value(ESP_ZB_ZCL_ATTR_ANALOG_INPUT_STATUS_FLAGS_ID, &status_flags_);

	return uptime_us < std::chrono::microseconds(1h).count()
		? std::chrono::milliseconds(1min).count()
		: std::chrono::milliseconds(1h).count();
}

uint32_t ConnectedCluster::app_type_{
	  (  0x00 << 24)  /* Group: Analog Input    */
	| (  0x0E << 16)  /* Type:  Time in Seconds */
	|  0x0000         /* Index: Relative time   */
};

uint16_t ConnectedCluster::units_{70}; /* Time - Days */

ConnectedCluster::ConnectedCluster()
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			{ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID}) {
}

void ConnectedCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *input_cluster = esp_zb_analog_input_cluster_create(nullptr);

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_APPLICATION_TYPE_ID, &app_type_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &units_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
			ZigbeeString("Connected").data()));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(&cluster_list,
		input_cluster, role()));
}

void ConnectedCluster::connected() {
	connect_time_us_ = esp_timer_get_time();
}

void ConnectedCluster::disconnected() {
	connect_time_us_ = 0;
}

uint32_t ConnectedCluster::update() {
	if (connect_time_us_) {
		uint64_t connected_us = esp_timer_get_time() - connect_time_us_;

		connected_ = connected_us / static_cast<float>(std::chrono::microseconds(24h).count());
		update_attr_value(ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &connected_);

		return connected_us < std::chrono::microseconds(1h).count()
			? std::chrono::milliseconds(1min).count()
			: std::chrono::milliseconds(1h).count();
	} else {
		connected_ = 0;
		return std::chrono::milliseconds(1h).count();
	}
}

uint32_t UplinkCluster::app_type_{
	  (  0x00 << 24)  /* Group: Analog Input */
	| (  0xFF << 16)  /* Type:  Other            */
	|  0x0000         /* Index: N/A              */
};

uint16_t UplinkCluster::units_{0x0100}; /* Proprietary */

UplinkCluster::UplinkCluster()
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			{ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID}) {
}

void UplinkCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *input_cluster = esp_zb_analog_input_cluster_create(nullptr);

	ESP_ERROR_CHECK(esp_zb_cluster_update_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &uplink_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_APPLICATION_TYPE_ID, &app_type_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &units_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
			ZigbeeString("Uplink").data()));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(&cluster_list,
		input_cluster, role()));
}

void UplinkCluster::update(uint16_t uplink) {
	float uplinkf = uplink;

	if (uplink_ != uplinkf) {
		uplink_ = uplinkf;
		update_attr_value(ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &uplink_);
	}
}

uint32_t RSSICluster::app_type_{
	  (  0x00 << 24)  /* Group: Analog Input */
	| (  0xFF << 16)  /* Type:  Other        */
	|  0x0000         /* Index: N/A          */
};

uint16_t RSSICluster::units_{199}; /* Decibels */

RSSICluster::RSSICluster()
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_ANALOG_INPUT,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
			{ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID}) {
}

void RSSICluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *input_cluster = esp_zb_analog_input_cluster_create(nullptr);

	ESP_ERROR_CHECK(esp_zb_cluster_update_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &rssi_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_APPLICATION_TYPE_ID, &app_type_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_ENGINEERING_UNITS_ID, &units_));

	ESP_ERROR_CHECK(esp_zb_analog_input_cluster_add_attr(input_cluster,
			ESP_ZB_ZCL_ATTR_ANALOG_INPUT_DESCRIPTION_ID,
			ZigbeeString("RSSI").data()));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_analog_input_cluster(&cluster_list,
		input_cluster, role()));
}

void RSSICluster::update(int8_t rssi) {
	if (rssi_ != rssi) {
		rssi_ = rssi;
		update_attr_value(ESP_ZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &rssi_);
	}
}

SoftwareCluster::SoftwareCluster(Device &device, size_t index)
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_BASIC,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE), device_(device), index_(index) {
}

void SoftwareCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(nullptr);

	ESP_LOGD(TAG, "App Partition: %zu", index_);

	device_.configure_basic_cluster(*basic_cluster, index_);

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(&cluster_list,
		basic_cluster, role()));
}

void SoftwareCluster::reload_app_info(bool full) {
	std::string label, date_code, version;

	device_.make_app_info(index_, label, date_code, version);

	ESP_LOGD(TAG, "App Partition: %zu", index_);

	if (full) {
		ESP_LOGD(TAG, "Date code: %s", date_code.c_str());
		update_attr_value(ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
			ZigbeeString{date_code, Device::MAX_DATE_CODE_LENGTH}.data());
	}

	ESP_LOGD(TAG, "Label: %s", label.c_str());
	update_attr_value(ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_LABEL_ID,
		ZigbeeString{label, Device::MAX_STRING_LENGTH}.data());

	if (full) {
		ESP_LOGD(TAG, "Version: %s", version.c_str());
		update_attr_value(ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_VERSION_DETAILS_ID,
			ZigbeeString{version, Device::MAX_STRING_LENGTH}.data());
	}
}

} // namespace device

} // namespace nutt
