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

#include <esp_app_desc.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_ota_ops.h>

#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace nutt {

Device *Device::instance_{nullptr};

Device::Device() : zigbee_(*new ZigbeeDevice{}) {
	assert(!instance_);
	instance_ = this;

	zigbee_.add(*new IdentifyEndpoint{"uuid.uk", "candle-dribbler",
		"https://github.com/nomis/candle-dribbler"});

	for (size_t i = 0; i < esp_ota_get_app_partition_count(); i++)
		zigbee_.add(*new SoftwareEndpoint{i});
}

void Device::add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints) {
	lights_.emplace_back(light);

	for (auto ep : endpoints)
		zigbee_.add(ep);
}

void Device::start() {
	zigbee_.start();
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

IdentifyEndpoint::IdentifyEndpoint(const std::string_view manufacturer,
		const std::string_view model, const std::string_view url)
		: ZigbeeEndpoint(EP_ID,
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		manufacturer_(manufacturer), model_(model), url_(url) {
}

void Device::configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster,
		std::string version, const esp_app_desc_t *desc) {
	std::string date_code;

	if (desc) {
		if (std::strlen(desc->date)) {
			std::string month{desc->date};

			month.resize(3);

			date_code += desc->date[7];
			date_code += desc->date[8];
			date_code += desc->date[9];
			date_code += desc->date[10];
			date_code += desc->date[4] == ' ' ? '0' : desc->date[4];
			date_code += desc->date[5];

			if (month == "Jan") { date_code += "01"; }
			else if (month == "Feb") { date_code += "02"; }
			else if (month == "Mar") { date_code += "03"; }
			else if (month == "Apr") { date_code += "04"; }
			else if (month == "May") { date_code += "05"; }
			else if (month == "Jun") { date_code += "06"; }
			else if (month == "Jul") { date_code += "07"; }
			else if (month == "Aug") { date_code += "08"; }
			else if (month == "Sep") { date_code += "09"; }
			else if (month == "Oct") { date_code += "10"; }
			else if (month == "Nov") { date_code += "11"; }
			else if (month == "Dec") { date_code += "12"; }

			for (char c : desc->time) {
				if (c >= '0' && c <= '9')
					date_code += c;

				if (date_code.length() >= 16)
					break;
			}
		}

		if (!version.empty())
			version += ' ';

		version += desc->version;
	}

	if (date_code.empty())
		date_code.append(8, '0');

	ESP_LOGI(TAG, "Date code: %s", date_code.c_str());
	ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
		ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
		ZigbeeString{date_code}.data()));

	if (!version.empty()) {
		ESP_LOGI(TAG, "Version: %s", version.c_str());
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_VERSION_DETAILS_ID,
			ZigbeeString{version}.data()));
	}
}

void IdentifyEndpoint::configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster) {
	if (!manufacturer_.empty())
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, ZigbeeString{manufacturer_}.data()));

	if (!model_.empty())
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, ZigbeeString{model_}.data()));

	if (!url_.empty())
		ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(&basic_cluster,
			ESP_ZB_ZCL_ATTR_BASIC_PRODUCT_URL_ID, ZigbeeString{url_}.data()));

	Device::configure_basic_cluster(basic_cluster, "", esp_app_get_description());
}

void IdentifyEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(&cluster_list,
		esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

SoftwareEndpoint::SoftwareEndpoint(size_t index)
		: ZigbeeEndpoint(BASE_EP_ID + index,
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		index_(index) {
}

void SoftwareEndpoint::configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster) {
	const esp_partition_t *current = esp_ota_get_running_partition();
	const esp_partition_t *next = esp_ota_get_next_update_partition(nullptr);
	const esp_partition_t *boot = esp_ota_get_boot_partition();
	const esp_partition_t *part = current;
	esp_app_desc_t desc;
	std::string version;

	for (int i = 0; i < esp_ota_get_app_partition_count() && i <= index_;
			i++, part = esp_ota_get_next_update_partition(part)) {}

	version = part->label;

	if (part == current) {
		version += " [current]";
	}
	if (part == next) {
		version += " [next]";
	}
	if (part == boot) {
		version += " [boot]";
	}

	esp_ota_img_states_t state;

	if (esp_ota_get_state_partition(part, &state))
		state = ESP_OTA_IMG_UNDEFINED;

	version += ' ';
	switch (state) {
	case ESP_OTA_IMG_NEW: version += "new"; break;
	case ESP_OTA_IMG_PENDING_VERIFY: version += "pending-verify"; break;
	case ESP_OTA_IMG_VALID: version += "valid"; break;
	case ESP_OTA_IMG_INVALID: version += "invalid"; break;
	case ESP_OTA_IMG_ABORTED: version += "aborted"; break;
	default: version += "undefined"; break;
	}

	ESP_LOGI(TAG, "App Partition: %zu", index_);

	Device::configure_basic_cluster(basic_cluster, version,
		!esp_ota_get_partition_description(part, &desc) ? &desc : nullptr);
}

} // namespace nutt
