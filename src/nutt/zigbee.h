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

#include <esp_zigbee_core.h>
#include <sdkconfig.h>

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace nutt {

using ep_id_t = uint8_t;

class CompressedOTA;
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

static inline std::string zigbee_address_string(const esp_zb_ieee_addr_t address) {
	std::vector<char> data(24);

	snprintf(data.data(), data.size(),
		"%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
		address[7], address[6], address[5], address[4],
		address[3], address[2], address[1], address[0]);

	return {data.data()};
}

static inline std::string zigbee_address_string(const uint16_t address) {
	std::vector<char> data(5);

	snprintf(data.data(), data.size(), "%04x", address);

	return {data.data()};
}

class ZigbeeCluster {
protected:
	ZigbeeCluster(uint16_t id, esp_zb_zcl_cluster_role_t role);
	ZigbeeCluster(uint16_t id, esp_zb_zcl_cluster_role_t role, const std::vector<uint16_t> &attrs);
	~ZigbeeCluster() = default;

public:
	static constexpr const char *TAG = "nutt.ZigbeeCluster";

	inline uint16_t id() const { return id_; }
	inline esp_zb_zcl_cluster_role_t role() const { return role_; }
	inline const std::vector<uint16_t>& attrs() const { return attrs_; }

	void attach(ZigbeeEndpoint &endpoint);
	virtual void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) = 0;
	void configure_reporting();

	virtual esp_err_t set_attr_value(uint16_t attr_id, const esp_zb_zcl_attribute_data_t *data);
	void update_attr_value(uint16_t attr_id, void *value);

private:
	static constexpr uint16_t MIN_INTERVAL = 0;
	static constexpr uint16_t MAX_INTERVAL = 900;

	const uint16_t id_;
	const esp_zb_zcl_cluster_role_t role_;
	const std::vector<uint16_t> attrs_;
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
	void configure_reporting();
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

enum class ZigbeeDeviceType : uint8_t {
	COORDINATOR,
	ROUTER,
	END_DEVICE,
	UNKNOWN,
};

enum class ZigbeeNeighbourRelationship : uint8_t {
	PARENT,
	CHILD,
	SIBLING,
	NONE,
	FORMER_CHILD,
	UNAUTH_CHILD,
	UNKNOWN,
};

struct ZigbeeNeighbour {
	uint16_t short_addr;
	ZigbeeDeviceType type;

	uint8_t depth;
	ZigbeeNeighbourRelationship relationship;

	uint8_t lqi;
	int8_t rssi;

	esp_zb_ieee_addr_t long_addr;
};

class ZigbeeDevice {
	friend void ::esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);

public:
#ifndef CONFIG_NUTT_ZIGBEE_ROLE_ROUTER
#define CONFIG_NUTT_ZIGBEE_ROLE_ROUTER 0
#endif
	static constexpr const bool ROUTER = CONFIG_NUTT_ZIGBEE_ROLE_ROUTER;

	explicit ZigbeeDevice(ZigbeeListener &listener);
	~ZigbeeDevice() = delete;

	static constexpr const char *TAG = "nutt.ZigbeeDevice";

	void add(ZigbeeEndpoint &endpoint);
	void start();
	void join_network();
	void join_or_leave_network();
	void leave_network();
	uint16_t get_parent();
	std::shared_ptr<const std::vector<ZigbeeNeighbour>> get_neighbours();
	void print_bindings();

	inline void schedule(const std::shared_ptr<std::function<void()>> &task) { schedule_after(task, 0); }
	inline void schedule(std::function<void()> &&task) { schedule_after(std::move(task), 0); }
	void schedule_after(const std::shared_ptr<std::function<void()>> &task, uint32_t time_ms);
	void schedule_after(std::function<void()> &&task, uint32_t time_ms);
	inline void reschedule(const std::shared_ptr<std::function<void()>> &task) { reschedule_after(task, 0); }
	void reschedule_after(const std::shared_ptr<std::function<void()>> &task, uint32_t time_ms);
	void schedule_cancel(const std::shared_ptr<std::function<void()>> &task);

private:
	static constexpr uint32_t REFRESH_NEIGHBOURS_MS = 60000;

	static void start_top_level_commissioning(uint8_t mode_mask);
	static esp_err_t action_handler(esp_zb_core_action_callback_id_t callback_id, const void *data);
	static void refresh_neighbours_cb(uint8_t buffer);
	static void scheduled_print_bindings(uint8_t param);
	static void print_bindings_cb(uint8_t buffer);

	void run();
	void connect(const char *why, uint8_t mode);
	void retry(bool quiet = false);
	void cancel_retry();
	void connected();
	void disconnected();
	void signal_handler(esp_zb_app_signal_type_t type, esp_err_t status, void *data);

	esp_err_t set_attr_value(const esp_zb_zcl_set_attr_value_message_t *message);
	esp_err_t ota_upgrade(const esp_zb_zcl_ota_upgrade_value_message_t *message);

	void update_state(ZigbeeState state);
	void update_state(ZigbeeState state, bool configured);

	void join_or_leave_network(ZigbeeAction action);

	static ZigbeeDevice *instance_;

	std::mutex tasks_mutex_;
	std::multimap<uint64_t,std::shared_ptr<std::function<void()>>> tasks_;
	std::shared_ptr<std::function<void()>> start_top_level_commissioning_;
	std::shared_ptr<std::function<void()>> refresh_neighbours_;

	ZigbeeListener &listener_;
	esp_zb_ep_list_t *endpoint_list_{nullptr};
	std::unordered_map<ep_id_t,ZigbeeEndpoint&> endpoints_;

	bool network_configured_{false};
	bool network_failed_{false};
	ZigbeeState state_{ZigbeeState::INIT};

	std::unique_ptr<CompressedOTA> ota_;
	std::vector<uint8_t> ota_header_;
	bool ota_upgrade_subelement_{false};
	size_t ota_data_len_{0};
	uint64_t ota_last_receive_us_{0};
	size_t ota_receive_not_logged_{0};

	uint8_t zb_buffer_;
	uint16_t neighbour_table_update_count_;
	std::mutex neighbours_mutex_;
	std::shared_ptr<std::vector<ZigbeeNeighbour>> neighbours_;
	std::shared_ptr<std::vector<ZigbeeNeighbour>> new_neighbours_;
};

class ZigbeeListener {
protected:
	ZigbeeListener() = default;
	~ZigbeeListener() = default;

public:
	virtual void zigbee_network_state(bool configured, ZigbeeState state, bool failed) {};
	virtual void zigbee_network_error() {};
	virtual void zigbee_ota_update(bool ok, bool app_changed = false) {};
	virtual void zigbee_neighbours_updated(const std::shared_ptr<const std::vector<ZigbeeNeighbour>> &neighbours) {};
};

} // namespace nutt
