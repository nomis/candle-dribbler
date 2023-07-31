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

#include <functional>
#include <vector>

namespace nutt {

Device *Device::instance_{nullptr};

Device::Device() : zigbee_(*new ZigbeeDevice{"uuid.uk", "candle-dribbler"}) {
	assert(!instance_);
	instance_ = this;

	zigbee_.add(*new IdentifyEndpoint{});
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

IdentifyEndpoint::IdentifyEndpoint()
		: ZigbeeEndpoint(EP_ID,
			ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID) {
}

void IdentifyEndpoint::configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {
	ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(&cluster_list,
		esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
}

} // namespace nutt
