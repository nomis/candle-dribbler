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

#include <esp_log.h>
#include <nvs.h>
#include <nvs_handle.hpp>

#include <memory>

namespace nutt {

class Logging {
public:
	Logging();
	~Logging() = delete;

	void set_app_level(esp_log_level_t level);
	void set_sys_level(esp_log_level_t level);

private:
	static constexpr const char *TAG = "nutt.Logging";

	static const char* to_string(esp_log_level_t level);

	static std::unique_ptr<nvs::NVSHandle> nvs_;

	void configure_app(esp_log_level_t level);
	void configure_sys(esp_log_level_t level);

	bool open_nvs();
	esp_log_level_t app_level_nvs();
	void app_level_nvs(esp_log_level_t state);
	esp_log_level_t sys_level_nvs();
	void sys_level_nvs(esp_log_level_t state);

	esp_log_level_t default_level_{esp_log_default_level};
};

} // namespace nutt
