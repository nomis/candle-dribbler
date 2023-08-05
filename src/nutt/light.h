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
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>

#include <atomic>
#include <mutex>
#include <thread>
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
	static constexpr const char *TAG = "nutt.Light";
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
	static constexpr const char *TAG = "nutt.Light";
	static constexpr const ep_id_t BASE_EP_ID = 20;

	Light &light_;
	bool state_;
};

class SwitchStatusEndpoint: public ZigbeeEndpoint {
public:
	SwitchStatusEndpoint(Light &light);
	~SwitchStatusEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	void refresh();
	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const char *TAG = "nutt.Light";
	static constexpr const ep_id_t BASE_EP_ID = 30;

	Light &light_;
	bool state_;
};

class EnableEndpoint: public ZigbeeEndpoint {
public:
	EnableEndpoint(Light &light);
	~EnableEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

	uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value) override;

private:
	static constexpr const char *TAG = "nutt.Light";
	static constexpr const ep_id_t BASE_EP_ID = 50;

	Light &light_;
	bool state_;
};

} // namespace light

IRAM_ATTR void light_interrupt_handler(void *arg);

class Light {
	friend void light_interrupt_handler(void *arg);

public:
	Light(size_t index, gpio_num_t switch_pin, bool switch_active_low,
		gpio_num_t relay_pin, bool relay_active_low);
	~Light() = delete;

	inline size_t index() const { return index_; }

	void attach(Device &device);

	TickType_t run();

	bool primary_on() const;
	bool secondary_on() const;
	bool switch_on() const;
	bool enable() const;
	bool on() const;

	void primary_switch(bool state, bool local);
	void secondary_switch(bool state);
	void enable(bool state);

	void request_refresh();
	void refresh();

private:
	static constexpr const char *TAG = "nutt.Light";
	static constexpr const uint64_t DEBOUNCE_US = 20 * 1000;
	static std::unique_ptr<nvs::NVSHandle> nvs_;

	bool open_nvs();
	bool enable_nvs();
	void enable_nvs(bool state);
	IRAM_ATTR void interrupt_handler();
	void update_state();

	inline int switch_active() const { return switch_active_low_ ? 0 : 1; }
	inline int relay_active() const { return relay_active_low_ ? 0 : 1; }
	inline int relay_inactive() const { return relay_active_low_ ? 1 : 0; }

	const size_t index_;
	const gpio_num_t switch_pin_;
	const bool switch_active_low_;
	const gpio_num_t relay_pin_;
	const bool relay_active_low_;

	mutable std::mutex mutex_;
	bool primary_on_{false};
	bool secondary_on_{false};
	bool enable_{true};
	bool on_{false};

	int switch_change_state_;
	uint64_t switch_change_us_{0};
	int switch_state_;
	unsigned long switch_change_count_{0};
	std::atomic<unsigned long> switch_change_count_irq_{0};
	bool switch_active_;

	light::PrimaryEndpoint &primary_ep_;
	light::SecondaryEndpoint &secondary_ep_;
	light::SwitchStatusEndpoint &switch_status_ep_;

	Device *device_{nullptr};
};

} // namespace nutt
