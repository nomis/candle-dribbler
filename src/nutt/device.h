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

#include <functional>
#include <vector>

#include "thread.h"
#include "zigbee.h"

namespace nutt {

class Light;
class UserInterface;

namespace device {

class MainEndpoint;
class SoftwareEndpoint;

} // namespace device

class Device: public WakeupThread, public ZigbeeListener {
public:
	Device(UserInterface &ui);
	~Device() = delete;

	static constexpr const char *TAG = "nutt.Device";
	/* Assumes 2 OTA partitions are configured */
	static constexpr const size_t NUM_EP_PER_DEVICE = 3;
	static constexpr const size_t MAX_DATE_CODE_LENGTH = 16;
	static constexpr const size_t MAX_STRING_LENGTH = 70;

	void add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints);
	void start();
	void request_refresh();
	using WakeupThread::wake_up_isr;

	inline UserInterface& ui() { return ui_; };
	void network_join_or_leave();

	void configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster, int app_index);
	void make_app_info(int index, std::string &label, std::string &date_code, std::string &version);

	void zigbee_network_state(bool configured, ZigbeeState state, bool failed) override;
	void zigbee_network_error() override;
	void zigbee_ota_update(bool ok, bool app_changed) override;

private:
	static void scheduled_refresh(uint8_t param);
	static void scheduled_network_join_or_leave(uint8_t param);

	void reload_app_info(bool full);
	void do_refresh();
	unsigned long run_tasks() override;

	static Device *instance_;

	UserInterface &ui_;
	ZigbeeDevice &zigbee_;
	const esp_partition_t *part_current_;
	const esp_partition_t *part_next_;
	const esp_partition_t *part_boot_;
	std::vector<const esp_partition_t*> parts_;
	device::MainEndpoint &main_ep_;
	std::vector<std::reference_wrapper<device::SoftwareEndpoint>> software_eps_;
	std::vector<std::reference_wrapper<Light>> lights_;
	bool ota_validated_{false};
};

namespace device {

class MainEndpoint: public ZigbeeEndpoint {
public:
	MainEndpoint(Device &device, const std::string_view manufacturer,
		const std::string_view model, const std::string_view url);
	~MainEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	void reload_app_info();
	esp_err_t set_attr_value(uint16_t cluster_id, uint16_t attr_id,
		const esp_zb_zcl_attribute_data_t *data)  override;

private:
	static constexpr const char *TAG = "nutt.Device";
	static constexpr const ep_id_t EP_ID = 1;
#ifdef CONFIG_NUTT_SUPPORT_OTA
	static constexpr const bool OTA_SUPPORTED = true;
	static constexpr const uint16_t OTA_MANUFACTURER_ID = CONFIG_NUTT_OTA_MANUFACTURER_ID;
	static constexpr const uint16_t OTA_IMAGE_TYPE_ID = CONFIG_NUTT_OTA_IMAGE_TYPE_ID;
	static constexpr const uint32_t OTA_FILE_VERSION = CONFIG_NUTT_OTA_FILE_VERSION;
#else
	static constexpr const bool OTA_SUPPORTED = false;
	static constexpr const uint16_t OTA_MANUFACTURER_ID = 0;
	static constexpr const uint16_t OTA_IMAGE_TYPE_ID = 0;
	static constexpr const uint32_t OTA_FILE_VERSION = 0;
#endif
	static uint8_t power_source_;
	static uint8_t device_class_;
	static uint8_t device_type_;

	Device &device_;
	const std::string_view manufacturer_;
	const std::string_view model_;
	const std::string_view url_;
};

class SoftwareEndpoint: public ZigbeeEndpoint {
public:
	SoftwareEndpoint(Device &device, size_t index);
	~SoftwareEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
	void reload_app_info(bool full);

private:
	static constexpr const char *TAG = "nutt.Device";
	static constexpr const ep_id_t BASE_EP_ID = 200;

	Device &device_;
	size_t index_;
};

} // namespace device

} // namespace nutt
