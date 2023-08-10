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
#include <esp_err.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include "nutt/light.h"
#include "nutt/thread.h"

namespace nutt {

Device *Device::instance_{nullptr};
uint8_t IdentifyEndpoint::power_source_{4}; /* DC */

Device::Device() : zigbee_(*new ZigbeeDevice{}) {
	assert(!instance_);
	instance_ = this;

	semaphore_ = xSemaphoreCreateBinary();
	if (!semaphore_) {
		ESP_LOGE(TAG, "Semaphore create failed");
		esp_restart();
	}

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

	std::thread t;
	make_thread(t, "device_main", 4096, 19, &Device::run, this);
	t.detach();
}

void Device::request_refresh() {
	esp_zb_scheduler_alarm(&Device::scheduled_refresh, 0, 0);
}

void Device::wake_up_isr() {
	BaseType_t xHigherPriorityTaskWoken{pdFALSE};

	xSemaphoreGiveFromISR(semaphore_, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

void Device::run() {
	while (true) {
		TickType_t wait = portMAX_DELAY;

		for (Light &light : lights_)
			wait = std::min(wait, std::max(static_cast<TickType_t>(1U), light.run()));

		xSemaphoreTake(semaphore_, wait);
	}

	ESP_LOGE(TAG, "Device main loop stopped");
	esp_restart();
}

IdentifyEndpoint::IdentifyEndpoint(const std::string_view manufacturer,
		const std::string_view model, const std::string_view url)
		: ZigbeeEndpoint(EP_ID,
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID),
		manufacturer_(manufacturer), model_(model), url_(url) {
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

void IdentifyEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
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

	Device::configure_basic_cluster(*basic_cluster, "", esp_app_get_description());

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(&cluster_list,
		basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(&cluster_list,
		esp_zb_identify_cluster_create(nullptr), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
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

} // namespace nutt
