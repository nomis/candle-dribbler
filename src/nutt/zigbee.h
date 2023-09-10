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
#include <sdkconfig.h>

#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nutt {

using ep_id_t = uint8_t;

class UserInterface;
class ZigbeeDevice;
class ZigbeeEndpoint;
class ZigbeeListener;

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

	return {data.data()};
}

class ZigbeeCluster {
protected:
	ZigbeeCluster(uint16_t id, esp_zb_zcl_cluster_role_t role);
	~ZigbeeCluster() = default;

public:
	inline uint16_t id() const { return id_; }
	inline esp_zb_zcl_cluster_role_t role() const { return role_; }
	void attach(ZigbeeEndpoint &endpoint);
	virtual void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) = 0;

	virtual esp_err_t set_attr_value(uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data);
	void update_attr_value(uint16_t attr_id, void *value);

private:
	const uint16_t id_;
	const esp_zb_zcl_cluster_role_t role_;
	ZigbeeEndpoint *ep_{nullptr};
};

class ZigbeeEndpoint {
public:
	ZigbeeEndpoint(ep_id_t id, esp_zb_af_profile_id_t profile_id,
		uint16_t device_id);
	ZigbeeEndpoint(ep_id_t id, esp_zb_af_profile_id_t profile_id,
		uint16_t device_id, ZigbeeCluster &cluster);
	ZigbeeEndpoint(ep_id_t id, esp_zb_af_profile_id_t profile_id,
		uint16_t device_id,
		std::vector<std::reference_wrapper<ZigbeeCluster>> &&clusters);
	~ZigbeeEndpoint() = delete;

	inline ep_id_t id() const { return id_; }
	inline esp_zb_af_profile_id_t profile_id() const { return profile_id_; }
	inline uint16_t device_id() const { return device_id_; }

	void add(ZigbeeCluster &cluster);
	void attach(ZigbeeDevice &device);
	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list);
	esp_err_t set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data);

private:
	const ep_id_t id_;
	const esp_zb_af_profile_id_t profile_id_;
	const uint16_t device_id_;
	std::unordered_map<uint16_t,ZigbeeCluster&> clusters_;
	ZigbeeDevice *device_{nullptr};
};

enum class ZigbeeState {
	INIT,
	DISCONNECTED,
	RETRY,
	CONNECTING,
	CONNECTED,
};

enum class ZigbeeAction : uint8_t {
	JOIN,
	LEAVE,
	JOIN_OR_LEAVE,
};

class ZigbeeDevice {
	friend void ::esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);

public:
	explicit ZigbeeDevice(ZigbeeListener &listener);
	~ZigbeeDevice() = delete;

	static constexpr const char *TAG = "nutt.ZigbeeDevice";

	void add(ZigbeeEndpoint &endpoint);
	void start();
	void network_do(ZigbeeAction action);

private:
#ifdef CONFIG_NUTT_ZIGBEE_ROLE_ROUTER
	static constexpr const bool ROUTER = true;
#else
	static constexpr const bool ROUTER = false;
#endif

	static void start_top_level_commissioning(uint8_t mode_mask);
	static esp_err_t action_handler(esp_zb_core_action_callback_id_t callback_id, const void *data);

	void run();
	void signal_handler(esp_zb_app_signal_type_t type, esp_err_t status, void *data);

	esp_err_t set_attr_value(const esp_zb_zcl_set_attr_value_message_t *message);
	esp_err_t ota_upgrade(const esp_zb_zcl_ota_update_message_t *message);

	void update_state(ZigbeeState state);
	void update_state(ZigbeeState state, bool configured);

	static ZigbeeDevice *instance_;

	ZigbeeListener &listener_;
	esp_zb_ep_list_t *endpoint_list_{nullptr};
	std::unordered_map<ep_id_t,ZigbeeEndpoint&> endpoints_;

	bool network_configured_{false};
	bool network_failed_{false};
	ZigbeeState state_{ZigbeeState::INIT};
	uint64_t ota_last_receive_us_{0};
	size_t ota_receive_not_logged_{0};
};

class ZigbeeListener {
protected:
	ZigbeeListener() = default;
	~ZigbeeListener() = default;

public:
	virtual void zigbee_network_state(bool configured, ZigbeeState state, bool failed) = 0;
	virtual void zigbee_network_error() = 0;
	virtual void zigbee_ota_update(bool ok, bool app_changed = false) = 0;
};

} // namespace nutt
