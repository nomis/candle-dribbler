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

#include <esp_zigbee_core.h>

#include <algorithm>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace nutt {

using ep_id_t = uint8_t;

class ZigbeeDevice;
class ZigbeeEndpoint;

class ZigbeeString {
public:
	explicit ZigbeeString(const std::string_view text, size_t max_length = MAX_LENGTH);
	~ZigbeeString() = default;

	inline char* data() { return value_.data(); }
	inline bool empty() const { return value_.size() == 1; }

private:
	static constexpr const size_t MAX_LENGTH = UINT8_MAX - 1;

	std::vector<char> value_;
};

static inline std::string zigbee_address_string(esp_zb_ieee_addr_t address) {
	std::vector<char> data(24);

	snprintf(data.data(), data.size(),
		"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		address[7], address[6], address[5], address[4],
		address[3], address[2], address[1], address[0]);

	return {data.data(), data.size()};
}

class ZigbeeEndpoint {
protected:
	ZigbeeEndpoint(ep_id_t id, uint16_t profile_id, uint16_t device_id);
	~ZigbeeEndpoint() = default;

public:
	inline ep_id_t id() const { return id_; };
	virtual void configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster) {};
	virtual void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) {};
	inline uint16_t profile_id() const { return profile_id_; };
	inline uint16_t device_id() const { return device_id_; };

	void attach(ZigbeeDevice &device);

	virtual uint8_t set_attr_value(uint16_t cluster_id, uint16_t attr_id, void *value);
	void update_attr_value(uint16_t cluster_id, uint8_t cluster_role, uint16_t attr_id, void *value);

private:
	const ep_id_t id_;
	const uint16_t profile_id_;
	const uint16_t device_id_;
	ZigbeeDevice *device_{nullptr};
};

class ZigbeeDevice {
	friend void ::esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
	friend void ZigbeeEndpoint::update_attr_value(uint16_t cluster_id, uint8_t cluster_role, uint16_t attr_id, void *value);

public:
	ZigbeeDevice();
	~ZigbeeDevice() = delete;

	void add(ZigbeeEndpoint &endpoint);
	void start();

private:
	static constexpr const char *TAG = "nutt.ZigbeeDevice";

	static void start_top_level_commissioning(uint8_t mode_mask);
	static void attr_value_cb(uint8_t status, uint8_t endpoint_id, uint16_t cluster_id, uint16_t attr_id, void *value);

	void run();
	void signal_handler(esp_zb_app_signal_type_t type, esp_err_t status, void *data);
	uint8_t set_attr_value(uint8_t endpoint_id, uint16_t cluster_id, uint16_t attr_id, void *value);
	void update_attr_value(uint8_t endpoint_id, uint16_t cluster_id, uint8_t cluster_role, uint16_t attr_id, void *value);

	static ZigbeeDevice *instance_;
	static uint8_t power_source_;

	esp_zb_ep_list_t *endpoint_list_{nullptr};
	std::unordered_map<ep_id_t,ZigbeeEndpoint&> endpoints_;
};

} // namespace nutt
