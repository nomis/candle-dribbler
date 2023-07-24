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

#include <memory>
#include <vector>

#include "zigbee.h"

namespace nutt {

class Light;

class Device {
public:
	Device();
	~Device() = default;

	void add(std::shared_ptr<Light> light, std::vector<std::shared_ptr<ZigbeeEndpoint>> &&endpoints);
	void start();

private:
	ZigbeeDevice zigbee_;
	std::vector<std::shared_ptr<Light>> lights_;
};

class IdentifyEndpoint: public ZigbeeEndpoint {
public:
	IdentifyEndpoint();
	~IdentifyEndpoint() = default;

	esp_zb_cluster_list_t* cluster_list() override;

private:
	static constexpr const ep_id_t EP_ID = 1;
};

} // namespace nutt
