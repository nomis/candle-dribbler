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

#include "device.h"
#include "zigbee.h"

namespace nutt {

class Light;

namespace light {

class PrimaryEndpoint: public ZigbeeEndpoint {
public:
	PrimaryEndpoint(Light &light);
	~PrimaryEndpoint() = default;

	esp_zb_cluster_list_t* cluster_list() override;

	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 10;

	Light &light_;
};

class SecondaryEndpoint: public ZigbeeEndpoint {
public:
	SecondaryEndpoint(Light &light);
	~SecondaryEndpoint() = default;

	esp_zb_cluster_list_t* cluster_list() override;

	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 20;

	Light &light_;
};

class StatusEndpoint: public ZigbeeEndpoint {
public:
	StatusEndpoint(Light &light);
	~StatusEndpoint() = default;

	esp_zb_cluster_list_t* cluster_list() override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 30;

	Light &light_;
};

class TemporaryEnableEndpoint: public ZigbeeEndpoint {
public:
	TemporaryEnableEndpoint(Light &light);
	~TemporaryEnableEndpoint() = default;

	esp_zb_cluster_list_t* cluster_list() override;

	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 40;

	Light &light_;
};

class PersistentEnableEndpoint: public ZigbeeEndpoint {
public:
	PersistentEnableEndpoint(Light &light);
	~PersistentEnableEndpoint() = default;

	esp_zb_cluster_list_t* cluster_list() override;

	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 50;

	Light &light_;
};

} // namespace light

class Light: public std::enable_shared_from_this<Light> {
public:
	Light(size_t index, int switch_pin_, int relay_pin_);
	~Light() = default;

	inline size_t index() const { return index_; }

	void attach(Device &device);

	void primary_switch(bool state);
	void secondary_switch(bool state);
	void temporary_enable(bool state);
	void persistent_enable(bool state);

private:
	static constexpr const char *TAG = "nutt.Light";

	size_t index_;
	int switch_pin_;
	int relay_pin_;
	std::shared_ptr<light::PrimaryEndpoint> primary_ep_;
	std::shared_ptr<light::StatusEndpoint> status_ep_;
};

} // namespace nutt
