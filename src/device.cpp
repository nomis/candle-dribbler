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

#include "nutt/device.h"

#include <esp_app_desc.h>
#include <esp_chip_info.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nutt/light.h"
#include "nutt/thread.h"
#include "nutt/ui.h"
#include "nutt/util.h"
#include "nutt/zigbee.h"

namespace nutt {

Device *Device::instance_{nullptr};

Device::Device(UserInterface &ui) : WakeupThread("Device"), ui_(ui),
		zigbee_(*new ZigbeeDevice{*this}),
		basic_cl_(*this, "uuid.uk", "candle-dribbler",
			"https://github.com/nomis/candle-dribbler"),
		identify_cl_(ui_) {
	assert(!instance_);
	instance_ = this;

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
		ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID, {basic_cl_, identify_cl_}};

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
}

void Device::add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints) {
	lights_.emplace(light.index(), light);

	for (auto ep : endpoints)
		zigbee_.add(ep);
}

void Device::start() {
	zigbee_.start();
	esp_zb_scheduler_alarm(&Device::scheduled_uptime, 0, 0);

	std::thread t;
	make_thread(t, "device_main", 4096, 19, &Device::run_loop, this);
	t.detach();
}

void Device::request_refresh(Light &light) {
	esp_zb_scheduler_alarm(&Device::scheduled_refresh, light.index(), 1);
}

void Device::do_refresh(uint8_t light) {
	ESP_LOGD(TAG, "Refresh light %u", light);
	lights_.at(light).refresh();
}

void Device::scheduled_refresh(uint8_t param) {
	instance_->do_refresh(param);
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

void Device::print_neighbours() {
	auto neighbours = zigbee_.get_neighbours();

	ESP_LOGI(TAG, "Neighbours (%zu):", neighbours->size());

	for (auto& [short_addr, neighbour] : *neighbours) {
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

		case ZigbeeNeighbourRelationship::OTHER:
			relationship = 'O';
			break;

		case ZigbeeNeighbourRelationship::FORMER_CHILD:
			relationship = 'F';
			break;

		case ZigbeeNeighbourRelationship::UNAUTH_CHILD:
			relationship = 'U';
			break;

		case ZigbeeNeighbourRelationship::UNKNOWN:
			relationship = '_';
			break;
		}

		ESP_LOGI(TAG, "%s/%04x %c %u %c LQI %u RSSI %d",
			neighbour.long_addr.c_str(), short_addr, type, neighbour.depth,
			relationship, neighbour.lqi, neighbour.rssi);
	}
}

void Device::scheduled_uptime(uint8_t param) {
	instance_->basic_cl_.update_uptime();
	esp_zb_scheduler_alarm(&Device::scheduled_uptime, 0, 60000);
}

unsigned long Device::run_tasks() {
	unsigned long wait_ms = ULONG_MAX;

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
		if (std::strlen(desc->date)) {
			std::string time = desc->time;

			for (char c : desc->date) {
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

		label += desc->project_name;
		label += " | ";

		version = desc->version;
		version += " | ";
		version += desc->idf_ver;
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

	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_LOCATION_DESCRIPTION_ID, ZigbeeString{std::string{16, ' '}}.data()));

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

void BasicCluster::update_uptime() {
	auto uptime = duration_us_to_string(esp_timer_get_time());

	update_attr_value(ESP_ZB_ZCL_ATTR_BASIC_LOCATION_DESCRIPTION_ID,
		ZigbeeString{uptime, Device::MAX_STRING_LENGTH}.data());
}

IdentifyCluster::IdentifyCluster(UserInterface &ui)
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE), ui_(ui) {
}

void IdentifyCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(&cluster_list,
		esp_zb_identify_cluster_create(nullptr), role()));
}

void UpgradeCluster::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_ota_cluster_cfg_t ota_config{};
	ota_config.ota_upgrade_manufacturer = OTA_MANUFACTURER_ID;
	ota_config.ota_upgrade_image_type = OTA_IMAGE_TYPE_ID;
	ota_config.ota_upgrade_file_version = OTA_FILE_VERSION;

	esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_config);

	esp_zb_ota_upgrade_client_parameter_t ota_client_parameters{};
	ota_client_parameters.max_data_size = UINT8_MAX;
	ota_client_parameters.query_timer = ESP_ZB_ZCL_OTA_UPGRADE_QUERY_TIMER_COUNT_DEF;

	esp_zb_ota_cluster_add_attr(ota_cluster,
		ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_PARAMETER_ID,
		esp_zb_ota_client_parameter(&ota_client_parameters));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(&cluster_list,
		ota_cluster, role()));
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
		: ZigbeeCluster(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY,
			ESP_ZB_ZCL_CLUSTER_SERVER_ROLE) {
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
