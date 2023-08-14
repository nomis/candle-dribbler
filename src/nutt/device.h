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

#include <functional>
#include <vector>

#include "thread.h"
#include "zigbee.h"

namespace nutt {

class Light;
class UserInterface;

class Device: public WakeupThread, public ZigbeeListener {
public:
	Device(UserInterface &ui);
	~Device() = delete;

	/* Assumes 2 OTA partitions are configured */
	static constexpr const size_t NUM_EP_PER_DEVICE = 3;

	void add(Light &light, std::vector<std::reference_wrapper<ZigbeeEndpoint>> &&endpoints);
	void start();
	void request_refresh();
	using WakeupThread::wake_up_isr;

	inline UserInterface& ui() { return ui_; };
	void network_join_or_leave();

	static void configure_basic_cluster(esp_zb_attribute_list_t &basic_cluster,
		std::string label, const esp_app_desc_t *desc);

	void zigbee_network_state(bool configured, ZigbeeState state, bool failed) override;
	void zigbee_network_error() override;
	void zigbee_ota_update(bool ok) override;

private:
	static constexpr const char *TAG = "nutt.Device";

	static void scheduled_refresh(uint8_t param);
	static void scheduled_network_join_or_leave(uint8_t param);

	void do_refresh();
	unsigned long run_tasks() override;

	static Device *instance_;

	UserInterface &ui_;
	ZigbeeDevice &zigbee_;
	std::vector<std::reference_wrapper<Light>> lights_;
};

namespace device {

class MainEndpoint: public ZigbeeEndpoint {
public:
	MainEndpoint(Device &device, const std::string_view manufacturer,
		const std::string_view model, const std::string_view url);
	~MainEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;
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
	SoftwareEndpoint(size_t index);
	~SoftwareEndpoint() = delete;

	void configure_cluster_list(esp_zb_cluster_list_t &cluster_list) override;

private:
	static constexpr const char *TAG = "nutt.Device";
	static constexpr const ep_id_t BASE_EP_ID = 200;
	size_t index_;
};

} // namespace device

} // namespace nutt
