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

#include "nutt/log.h"

#include <esp_log.h>
#include <nvs.h>
#include <nvs_handle.hpp>

#include <memory>

#include "nutt/main.h"
#include "nutt/device.h"
#include "nutt/light.h"
#include "nutt/thread.h"
#include "nutt/ui.h"
#include "nutt/zigbee.h"

namespace nutt {

std::unique_ptr<nvs::NVSHandle> Logging::nvs_;

Logging::Logging() {
	esp_log_level_set(TAG, ESP_LOG_INFO);
	configure_sys(sys_level_nvs());
	configure_app(app_level_nvs());
}

const char* Logging::to_string(esp_log_level_t level) {
	const char *level_str = "UNKNOWN";

	switch (level) {
	case ESP_LOG_NONE: level_str = "OFF"; break;
	case ESP_LOG_ERROR: level_str = "ERROR"; break;
	case ESP_LOG_WARN: level_str = "WARN"; break;
	case ESP_LOG_INFO: level_str = "INFO"; break;
	case ESP_LOG_DEBUG: level_str = "DEBUG"; break;
	case ESP_LOG_VERBOSE: level_str = "VERBOSE"; break;
	}

	return level_str;
}

void Logging::set_app_level(esp_log_level_t level) {
	configure_app(level);
	app_level_nvs(level);
	ESP_LOGI(TAG, "Application log level set to %s", to_string(level));
}

void Logging::configure_app(esp_log_level_t level) {
	esp_log_level_set(nutt::TAG, level);
	esp_log_level_set(nutt::Device::TAG, level);
	esp_log_level_set(nutt::Light::TAG, level);
	esp_log_level_set(nutt::UserInterface::TAG, level);
	esp_log_level_set(nutt::ZigbeeDevice::TAG, level);
	esp_log_level_set(nutt::WakeupThread::TAG, level);
}

void Logging::set_sys_level(esp_log_level_t level) {
	configure_sys(level);
	sys_level_nvs(level);
	ESP_LOGI(TAG, "System log level set to %s", to_string(level));
}

void Logging::configure_sys(esp_log_level_t level) {
	esp_log_default_level = level;
}

bool Logging::open_nvs() {
	if (!nvs_) {
		nvs_ = nvs::open_nvs_handle("log", NVS_READWRITE, nullptr);
	}

	return !!nvs_;
}

esp_log_level_t Logging::app_level_nvs() {
	if (open_nvs()) {
		uint8_t value;

		if (nvs_->get_item("app_level", value) == ESP_OK) {
			return static_cast<esp_log_level_t>(value);
		}
	}

	return ESP_LOG_INFO;
}

void Logging::app_level_nvs(esp_log_level_t level) {
	if (open_nvs()) {
		uint8_t value = static_cast<uint8_t>(level);

		nvs_->set_item("app_level", value);
		nvs_->commit();
	}
}

esp_log_level_t Logging::sys_level_nvs() {
	if (open_nvs()) {
		uint8_t value;

		if (nvs_->get_item("sys_level", value) == ESP_OK) {
			return static_cast<esp_log_level_t>(value);
		}
	}

	return default_level_;
}

void Logging::sys_level_nvs(esp_log_level_t level) {
	if (open_nvs()) {
		uint8_t value = static_cast<uint8_t>(level);

		nvs_->set_item("sys_level", value);
		nvs_->commit();
	}
}

} // namespace nutt
