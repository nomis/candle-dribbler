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

#include <nvs.h>
#include <nvs_handle.hpp>

#include <vector>

#include "device.h"
#include "zigbee.h"

namespace nutt {

class Light;

namespace light {

class PrimaryEndpoint: public ZigbeeEndpoint {
public:
	PrimaryEndpoint(Light &light);
	~PrimaryEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 10;

	Light &light_;
	bool state_;
};

class SecondaryEndpoint: public ZigbeeEndpoint {
public:
	SecondaryEndpoint(Light &light);
	~SecondaryEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 20;

	Light &light_;
	bool state_;
};

class StatusEndpoint: public ZigbeeEndpoint {
public:
	StatusEndpoint(Light &light);
	~StatusEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();

private:
	static constexpr const ep_id_t BASE_EP_ID = 30;

	Light &light_;
	bool state_;
};

class TemporaryEnableEndpoint: public ZigbeeEndpoint {
public:
	TemporaryEnableEndpoint(Light &light);
	~TemporaryEnableEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 40;

	Light &light_;
	bool state_;
};

class PersistentEnableEndpoint: public ZigbeeEndpoint {
public:
	PersistentEnableEndpoint(Light &light);
	~PersistentEnableEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 50;

	Light &light_;
	bool state_;
};

} // namespace light

class Light {
public:
	Light(size_t index, int switch_pin_, int relay_pin_);
	~Light() = delete;

	inline size_t index() const { return index_; }

	void attach(Device &device);

	bool primary_on() const { return primary_on_; }
	bool secondary_on() const { return secondary_on_; }
	bool temporary_enable() const { return temporary_enable_; }
	bool persistent_enable() const { return persistent_enable_; }
	bool on() const { return on_; }

	void primary_switch(bool state);
	void secondary_switch(bool state);
	void temporary_enable(bool state);
	void persistent_enable(bool state);

private:
	static constexpr const char *TAG = "nutt.Light";
	static std::unique_ptr<nvs::NVSHandle> nvs_;

	bool open_nvs();
	bool persistent_enable_nvs();
	void persistent_enable_nvs(bool state);
	void update_state();

	const size_t index_;
	const int switch_pin_;
	const int relay_pin_;

	bool primary_on_{false};
	bool secondary_on_{false};
	bool persistent_enable_{true};
	bool temporary_enable_{true};
	bool on_{false};

	light::PrimaryEndpoint &primary_ep_;
	light::SecondaryEndpoint &secondary_ep_;
	light::StatusEndpoint &status_ep_;
	light::TemporaryEnableEndpoint &temporary_enable_ep_;
};

} // namespace nutt
