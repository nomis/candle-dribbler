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

#include <esp_zigbee_cluster.h>
#include <esp_zigbee_type.h>
#include <nvs.h>
#include <nvs_handle.hpp>
#include <driver/gpio.h>

#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "debounce.h"
#include "device.h"
#include "zigbee.h"

namespace nutt {

class Light;

namespace light {

class OnOffEndpoint: public ZigbeeEndpoint {
protected:
	OnOffEndpoint(Light &light, ep_id_t base_ep_id,
		esp_zb_ha_standard_devices_t type, bool state);
	~OnOffEndpoint() = default;

	static constexpr const char *TAG = "nutt.Light";

	void configure_light_cluster_list(esp_zb_cluster_list_t &cluster_list);
	void configure_switch_cluster_list(esp_zb_cluster_list_t &cluster_list);

	Light &light_;
	bool state_;

private:
	void configure_common_cluster_list(esp_zb_cluster_list_t &cluster_list);
};

class PrimaryEndpoint: public OnOffEndpoint {
public:
	explicit PrimaryEndpoint(Light &light);
	~PrimaryEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	esp_err_t set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 10;
};

class SecondaryEndpoint: public OnOffEndpoint {
public:
	explicit SecondaryEndpoint(Light &light);
	~SecondaryEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	esp_err_t set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 20;
};

class TertiaryEndpoint: public OnOffEndpoint {
public:
	explicit TertiaryEndpoint(Light &light);
	~TertiaryEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	esp_err_t set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 30;
};

class SwitchStatusEndpoint: public ZigbeeEndpoint {
public:
	explicit SwitchStatusEndpoint(Light &light);
	~SwitchStatusEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	esp_err_t set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *value) override;

private:
	static constexpr const char *TAG = "nutt.Light";
	static constexpr const ep_id_t BASE_EP_ID = 60;
	static uint32_t type_;

	Light &light_;
	uint8_t state_;
};

class EnableEndpoint: public OnOffEndpoint {
public:
	explicit EnableEndpoint(Light &light);
	~EnableEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	esp_err_t set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *value) override;

private:
	static constexpr const ep_id_t BASE_EP_ID = 70;
};

} // namespace light

class Light {
public:
	Light(size_t index, gpio_num_t switch_pin, bool switch_active_low,
		gpio_num_t relay_pin, bool relay_active_low);
	~Light() = delete;

	static constexpr const char *TAG = "nutt.Light";
	static constexpr const size_t NUM_EP_PER_LIGHT = 5;

	inline size_t index() const { return index_; }

	void attach(Device &device);

	unsigned long run();

	bool primary_on() const;
	bool secondary_on() const;
	bool tertiary_on() const;
	bool switch_on() const;
	bool enable() const;
	bool on() const;

	void primary_switch(bool state, bool local);
	void secondary_switch(bool state, bool local);
	void tertiary_switch(bool state);
	void enable(bool state);

	void request_refresh();
	void refresh();

private:
	static constexpr const unsigned long DEBOUNCE_US = 20 * 1000;
	static std::unique_ptr<nvs::NVSHandle> nvs_;

	bool open_nvs();
	bool enable_nvs();
	void enable_nvs(bool state);

	void secondary_switch_locked(bool state, bool local);
	void update_state();

	inline int relay_active() const { return relay_active_low_ ? 0 : 1; }
	inline int relay_inactive() const { return relay_active_low_ ? 1 : 0; }

	const size_t index_;
	Debounce switch_debounce_;
	const gpio_num_t relay_pin_;
	const bool relay_active_low_;

	mutable std::mutex mutex_;
	bool primary_on_{false};
	bool secondary_on_{false};
	bool tertiary_on_{false};
	bool switch_active_;
	uint64_t switch_change_us_{0};
	bool enable_{true};
	bool on_{false};

	light::PrimaryEndpoint &primary_ep_;
	light::SecondaryEndpoint &secondary_ep_;
	light::TertiaryEndpoint &tertiary_ep_;
	light::SwitchStatusEndpoint &switch_status_ep_;

	Device *device_{nullptr};
};

} // namespace nutt
