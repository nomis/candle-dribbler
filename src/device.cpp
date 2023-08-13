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
#include <esp_ota_ops.h>

#include <algorithm>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "nutt/light.h"
#include "nutt/thread.h"
#include "nutt/ui.h"

namespace nutt {

Device *Device::instance_{nullptr};

Device::Device(UserInterface &ui) : WakeupThread("Device"), ui_(ui),
		zigbee_(*new ZigbeeDevice{ui}) {
	assert(!instance_);
	instance_ = this;

	zigbee_.add(*new device::MainEndpoint{*this, "uuid.uk", "candle-dribbler",
		"https://github.com/nomis/candle-dribbler"});

	for (size_t i = 0; i < esp_ota_get_app_partition_count(); i++)
		zigbee_.add(*new device::SoftwareEndpoint{i});
}

void Device::add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints) {
	lights_.emplace_back(light);

	for (auto ep : endpoints)
		zigbee_.add(ep);
}

void Device::start() {
	zigbee_.start();

	std::thread t;
	make_thread(t, "device_main", 4096, 19, &Device::run_loop, this);
	t.detach();
}

void Device::request_refresh() {
	esp_zb_scheduler_alarm(&Device::scheduled_refresh, 0, 0);
}

void Device::do_refresh() {
	ESP_LOGI(TAG, "Refresh");

	for (Light &light : lights_)
		light.refresh();
}

void Device::scheduled_refresh(uint8_t param) {
	instance_->do_refresh();
}

void Device::network_join_or_leave() {
	esp_zb_scheduler_alarm(&Device::scheduled_network_join_or_leave, 0, 0);
}

void Device::scheduled_network_join_or_leave(uint8_t param) {
	instance_->zigbee_.network_join_or_leave();
}

unsigned long Device::run_tasks() {
	unsigned long wait_ms = ULONG_MAX;

	for (Light &light : lights_)
		wait_ms = std::min(wait_ms, light.run());

	return wait_ms;
}

void Device::configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster,
		std::string label, const esp_app_desc_t *desc) {
	std::string date_code;
	std::string version;

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

		if (!label.empty())
			label += " | ";

		label += desc->project_name;

		version = desc->version;
		version += " | ";
		version += desc->idf_ver;
	}

	if (date_code.empty())
		date_code.append(8, '0');

	ESP_LOGI(TAG, "Date code: %s", date_code.c_str());
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
		ZigbeeString{date_code, 16}.data()));

	if (!label.empty()) {
		ESP_LOGI(TAG, "Label: %s", label.c_str());
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_LABEL_ID,
			ZigbeeString{label, 70}.data()));
	}

	if (!version.empty()) {
		ESP_LOGI(TAG, "Version: %s", version.c_str());
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_VERSION_DETAILS_ID,
			ZigbeeString{version, 70}.data()));
	}
}

namespace device {

uint8_t MainEndpoint::power_source_{0x04}; /* DC */
uint8_t MainEndpoint::device_class_{0x00}; /* Lighting */
uint8_t MainEndpoint::device_type_{0xf0}; /* Generic actuator */

MainEndpoint::MainEndpoint(Device &device,
		const std::string_view manufacturer, const std::string_view model,
		const std::string_view url) : ZigbeeEndpoint(EP_ID,
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		device_(device), manufacturer_(manufacturer), model_(model), url_(url) {
}

void MainEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
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

	ESP_LOGI(TAG, "Serial Number: %s", serial_number.c_str());

	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_SERIAL_NUMBER_ID, ZigbeeString{serial_number, 50}.data()));

	Device::configure_basic_cluster(*basic_cluster, "", esp_app_get_description());

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(&cluster_list,
		basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(&cluster_list,
		esp_zb_identify_cluster_create(nullptr), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

	esp_zb_ota_cluster_cfg_t ota_config{};
	ota_config.ota_upgrade_manufacturer = MANUFACTURER_ID;
	ota_config.ota_upgrade_image_type = IMAGE_TYPE_ID;

	esp_zb_attribute_list_t *ota_cluster = esp_zb_ota_cluster_create(&ota_config);

	esp_zb_ota_upgrade_client_parameter_t ota_client_parameters{};
	ota_client_parameters.max_data_size = UINT8_MAX;

	esp_zb_ota_cluster_add_attr(ota_cluster,
		ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_PARAMETER_ID,
		esp_zb_ota_client_parameter(&ota_client_parameters));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_ota_cluster(&cluster_list,
		ota_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
}

esp_err_t MainEndpoint::set_attr_value(uint16_t cluster_id,
		uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data) {
	if (cluster_id == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY) {
		if (attr_id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
			if (data->type == ESP_ZB_ZCL_ATTR_TYPE_U16
					&& data->size == sizeof(uint16_t)) {
				uint16_t identify_time = *(uint16_t *)data->value;
				device_.ui().identify(identify_time);
				return ESP_OK;
			}
		}
	}
	return ESP_ERR_INVALID_ARG;
}

SoftwareEndpoint::SoftwareEndpoint(size_t index)
		: ZigbeeEndpoint(BASE_EP_ID + index,
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		index_(index) {
}

void SoftwareEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(nullptr);

	const esp_partition_t *current = esp_ota_get_running_partition();
	const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
	const esp_partition_t *boot = esp_ota_get_boot_partition();
	const esp_partition_t *part = current;
	esp_app_desc_t desc;
	std::string label;

	for (int i = 0; i < esp_ota_get_app_partition_count() && i <= index_;
			i++, part = esp_ota_get_next_update_partition(part)) {}

	label = part->label;

	if (part == current) {
		label += " [current]";
	}
	if (part == next) {
		label += " [next]";
	}
	if (part == boot) {
		label += " [boot]";
	}

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

	ESP_LOGI(TAG, "App Partition: %zu", index_);

	Device::configure_basic_cluster(*basic_cluster, label,
		!esp_ota_get_partition_description(part, &desc) ? &desc : nullptr);

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(&cluster_list,
		basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

} // namespace device

} // namespace nutt
