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
#include <esp_attr.h>
#include <esp_partition.h>

#include <memory>
#include <functional>
#include <vector>

#include "thread.h"
#include "zigbee.h"

namespace nutt {

class Device;
class Light;
class UserInterface;

namespace device {

class BasicCluster: public ZigbeeCluster {
public:
	BasicCluster(Device &device, const std::string_view manufacturer,
		const std::string_view model, const std::string_view url);
	~BasicCluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	void reload_app_info();

private:
	static constexpr const char *TAG = "nutt.Device";
	static uint8_t power_source_;
	static uint8_t device_class_;
	static uint8_t device_type_;

	Device &device_;
	const std::string_view manufacturer_;
	const std::string_view model_;
	const std::string_view url_;
};

class IdentifyCluster: public ZigbeeCluster {
public:
	IdentifyCluster(UserInterface &ui);
	~IdentifyCluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	esp_err_t set_attr_value(uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data)  override;

private:
	UserInterface &ui_;
};

class UpgradeCluster: public ZigbeeCluster {
public:
	UpgradeCluster();
	~UpgradeCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

private:
#ifdef CONFIG_NUTT_SUPPORT_OTA
	static constexpr const uint16_t OTA_MANUFACTURER_ID = CONFIG_NUTT_OTA_MANUFACTURER_ID;
	static constexpr const uint16_t OTA_IMAGE_TYPE_ID = CONFIG_NUTT_OTA_IMAGE_TYPE_ID;
# ifdef CONFIG_NUTT_OTA_FILE_VERSION_FROM_GIT_COMMIT
	static constexpr const uint32_t OTA_FILE_VERSION = static_cast<uint32_t>(NUTT_COMMIT_TIME);
# else
	static constexpr const uint32_t OTA_FILE_VERSION = CONFIG_NUTT_OTA_FILE_VERSION;
# endif
#else
	static constexpr const uint16_t OTA_MANUFACTURER_ID = 0;
	static constexpr const uint16_t OTA_IMAGE_TYPE_ID = 0;
	static constexpr const uint32_t OTA_FILE_VERSION = 0;
#endif
};

class UptimeCluster: public ZigbeeCluster {
public:
	UptimeCluster();
	~UptimeCluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	uint32_t update(bool fault);

private:
	static uint32_t app_type_;
	static uint16_t units_;

	float uptime_{0};
	uint8_t reliability_{ESP_ZB_ZCL_ANALOG_INPUT_RELIABILITY_NO_FAULT_DETECTED};
	uint8_t status_flags_{ESP_ZB_ZCL_ANALOG_INPUT_STATUS_FLAG_NORMAL};
};

class ConnectedCluster: public ZigbeeCluster {
public:
	ConnectedCluster();
	~ConnectedCluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	void connected();
	void disconnected();
	uint32_t update();

private:
	static uint32_t app_type_;
	static uint16_t units_;

	uint64_t connect_time_us_{0};
	float connected_{0};
};

class UplinkCluster: public ZigbeeCluster {
public:
	UplinkCluster();
	~UplinkCluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	void update(uint16_t uplink);

private:
	static uint32_t app_type_;
	static uint16_t units_;

	float uplink_{0xffff};
};

class RSSICluster: public ZigbeeCluster {
public:
	RSSICluster();
	~RSSICluster() = default;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	void update(int8_t rssi);

private:
	static uint32_t app_type_;
	static uint16_t units_;

	float rssi_{-128};
};

class SoftwareCluster: public ZigbeeCluster {
public:
	SoftwareCluster(Device &device, size_t index);
	~SoftwareCluster() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	void reload_app_info(bool full);

private:
	static constexpr const char *TAG = "nutt.Device";

	Device &device_;
	size_t index_;
};

} // namespace device

class Device: public WakeupThread, public ZigbeeListener {
public:
	explicit Device(UserInterface &ui);
	~Device() = delete;

	// cppcheck-suppress duplInheritedMember
	static constexpr const char *TAG = "nutt.Device";
	/* Assumes 2 OTA partitions are configured */
	static constexpr const size_t NUM_EP_PER_DEVICE = 5;
	static constexpr const size_t MAX_DATE_CODE_LENGTH = 16;
	static constexpr const size_t MAX_STRING_LENGTH = 70;

	void add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints);
	void start();
	void request_refresh(Light &light);

	inline UserInterface& ui() { return ui_; };
	void join_network();
	void join_or_leave_network();
	void leave_network();
	void print_bindings();
	void print_neighbours();
	void print_core_dump(bool full);
	void erase_core_dump();

	void configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster, int app_index);
	void make_app_info(int index, std::string &label, std::string &date_code, std::string &version);

	void zigbee_network_state(bool configured, ZigbeeState state, bool failed) override;
	void zigbee_network_error() override;
	void zigbee_ota_update(bool ok, bool app_changed) override;
	void zigbee_neighbours_updated(const std::shared_ptr<const std::vector<ZigbeeNeighbour>> &neighbours) override;

private:
	static constexpr const ep_id_t MAIN_EP_ID = 1;
	static constexpr const ep_id_t SOFTWARE_BASE_EP_ID = 200;
	static constexpr const ep_id_t CONNECTED_EP_ID = 210;
	static constexpr const ep_id_t UPLINK_PARENT_EP_ID = 211;
	static constexpr const ep_id_t UPLINK_RSSI_EP_ID = 212;
#ifdef CONFIG_NUTT_SUPPORT_OTA
	static constexpr const bool OTA_SUPPORTED = true;
#else
	static constexpr const bool OTA_SUPPORTED = false;
#endif

	static void scheduled_refresh(uint8_t param);
	static void scheduled_uptime(uint8_t param);
	static void scheduled_connected(uint8_t param);

	void reload_app_info(bool full);
	void do_refresh(uint8_t light);
	unsigned long run_tasks() override;

	static Device *instance_;

	UserInterface &ui_;
	ZigbeeDevice &zigbee_;
	const esp_partition_t *part_current_;
	const esp_partition_t *part_next_;
	const esp_partition_t *part_boot_;
	std::vector<const esp_partition_t*> parts_;
	device::BasicCluster basic_cl_;
	device::IdentifyCluster identify_cl_;
	device::UptimeCluster uptime_cl_;
	device::ConnectedCluster connected_cl_;
	device::UplinkCluster uplink_cl_;
	device::RSSICluster rssi_cl_;
	std::vector<std::reference_wrapper<device::SoftwareCluster>> software_cls_;
	std::unordered_map<uint8_t,Light&> lights_;
	bool ota_validated_{false};
	std::atomic<bool> core_dump_present_{false};
};

} // namespace nutt
