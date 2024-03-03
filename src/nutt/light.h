/*
 * candle-dribbler - ESP32 Zigbee light controller
 * Copyright 2023-2024  Simon Arlott
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
#include "main.h"
#include "zigbee.h"

namespace nutt {

class Light;

namespace light {

class GroupsCluster: public ZigbeeCluster {
public:
	GroupsCluster();
	~GroupsCluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
};

class ScenesCluster: public ZigbeeCluster {
public:
	ScenesCluster();
	~ScenesCluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
};

class BooleanCluster: public ZigbeeCluster {
protected:
	BooleanCluster(Light &light, const char *name, uint16_t cluster_id,
		uint16_t attr_id);
	~BooleanCluster() = default;

public:
	void refresh();
	esp_err_t set_attr_value(uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *value) override;

protected:
	static constexpr const char *TAG = "nutt.Light";

	void configure_light_cluster_list(esp_zb_cluster_list_t &cluster_list);
	void configure_switch_cluster_list(esp_zb_cluster_list_t &cluster_list);
	void configure_binary_input_cluster_list(esp_zb_cluster_list_t &cluster_list);
	virtual bool refresh_value() = 0;
	virtual void updated_value(bool value) = 0;

	Light &light_;

private:
	static uint32_t app_type_;

	const char *name_;
	const uint16_t attr_id_;
	uint8_t state_;
};

class PrimaryCluster: public BooleanCluster {
public:
	explicit PrimaryCluster(Light &light);
	~PrimaryCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

class SecondaryCluster: public BooleanCluster {
public:
	explicit SecondaryCluster(Light &light);
	~SecondaryCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

class TertiaryCluster: public BooleanCluster {
public:
	explicit TertiaryCluster(Light &light);
	~TertiaryCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

class SwitchStatusCluster: public BooleanCluster {
public:
	explicit SwitchStatusCluster(Light &light);
	~SwitchStatusCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

class TemporaryEnableCluster: public BooleanCluster {
public:
	explicit TemporaryEnableCluster(Light &light);
	~TemporaryEnableCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

class PersistentEnableCluster: public BooleanCluster {
public:
	explicit PersistentEnableCluster(Light &light);
	~PersistentEnableCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

protected:
	bool refresh_value() override;
	void updated_value(bool value) override;
};

} // namespace light

class Light {
public:
	Light(uint8_t index, gpio_num_t switch_pin, bool switch_active_low,
		gpio_num_t relay_pin, bool relay_active_low);
	~Light() = delete;

	static constexpr const char *TAG = "nutt.Light";
	static constexpr const size_t NUM_EP_PER_LIGHT = 6;

	inline uint8_t index() const { return index_; }

	void attach(Device &device);

	unsigned long run();

	bool primary_on() const;
	bool secondary_on() const;
	bool tertiary_on() const;
	bool switch_on() const;
	bool temporary_enable() const;
	bool persistent_enable() const;
	bool on() const;

	void primary_switch(bool state, bool local);
	void secondary_switch(bool state, bool local);
	void tertiary_switch(bool state);
	void temporary_enable(bool state);
	void persistent_enable(bool state);

	void request_refresh();
	void refresh();

private:
	enum RtcStateBit {
		RTC_STATE_BIT_PRIMARY_ON = 0,
		RTC_STATE_BIT_SECONDARY_ON = 1,
		RTC_STATE_BIT_TERTIARY_ON = 2,
		RTC_STATE_BIT_SWITCH_ACTIVE = 4,
		RTC_STATE_BIT_TEMPORARY_ENABLE = 8,
		RTC_STATE_BIT_ON = 12,
	};

	static constexpr const ep_id_t PRIMARY_BASE_EP_ID = 10;
	static constexpr const ep_id_t SECONDARY_BASE_EP_ID = 20;
	static constexpr const ep_id_t TERTIARY_BASE_EP_ID = 30;
	static constexpr const ep_id_t SWITCH_STATUS_BASE_EP_ID = 60;
	static constexpr const ep_id_t TEMPORARY_ENABLE_BASE_EP_ID = 70;
	static constexpr const ep_id_t PERSISTENT_ENABLE_BASE_EP_ID = 80;
	static constexpr const unsigned long DEBOUNCE_US = 20 * 1000;
	static std::unique_ptr<nvs::NVSHandle> nvs_;
	static uint32_t rtc_state_[MAX_LIGHTS];

	inline uint32_t read_rtc_state() { return rtc_state_[index_ - 1]; }
	uint32_t rtc_checksum(uint16_t value);
	bool valid_rtc_state();
	void load_rtc_state();
	uint32_t create_rtc_state();
	inline void save_rtc_state(uint32_t value) { rtc_state_[index_ - 1] = value; }

	bool open_nvs();
	bool enable_nvs();
	void enable_nvs(bool state);

	void secondary_switch_locked(bool state, bool local);
	void update_state();

	inline int relay_active() const { return relay_active_low_ ? 0 : 1; }
	inline int relay_inactive() const { return relay_active_low_ ? 1 : 0; }

	const uint8_t index_;
	Debounce switch_debounce_;
	const gpio_num_t relay_pin_;
	const bool relay_active_low_;

	mutable std::mutex mutex_;
	bool primary_on_{false};
	bool secondary_on_{false};
	bool tertiary_on_{false};
	bool switch_active_;
	uint64_t switch_change_us_{0};
	bool persistent_enable_{true};
	bool temporary_enable_{true};
	bool on_{false};

	light::PrimaryCluster &primary_cl_;
	light::SecondaryCluster &secondary_cl_;
	light::SwitchStatusCluster &switch_status_cl_;
	light::TemporaryEnableCluster &temporary_enable_cl_;

	Device *device_{nullptr};
};

} // namespace nutt
