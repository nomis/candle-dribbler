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

#pragma once

#include <esp_app_desc.h>

#include <functional>
#include <vector>

#include "zigbee.h"

namespace nutt {

class Light;

class Device {
public:
	Device();
	~Device() = delete;

	void add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints);
	void start();
	void request_refresh();

	static void configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster,
		std::string version, const esp_app_desc_t *desc);

private:
	static constexpr const char *TAG = "nutt.Device";

	static void scheduled_refresh(uint8_t param);

	void do_refresh();

	static Device *instance_;

	ZigbeeDevice &zigbee_;
	std::vector<std::reference_wrapper<Light>> lights_;
};

class IdentifyEndpoint: public ZigbeeEndpoint {
public:
	IdentifyEndpoint(const std::string_view manufacturer,
		const std::string_view model, const std::string_view url);
	~IdentifyEndpoint() = delete;

	void configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster) override;
	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

private:
	static constexpr const ep_id_t EP_ID = 1;

	const std::string_view manufacturer_;
	const std::string_view model_;
	const std::string_view url_;
};

class SoftwareEndpoint: public ZigbeeEndpoint {
public:
	SoftwareEndpoint(size_t index);
	~SoftwareEndpoint() = delete;

	void configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster) override;

private:
	static constexpr const char *TAG = "nutt.Device";
	static constexpr const ep_id_t BASE_EP_ID = 200;
	size_t index_;
};

} // namespace nutt
