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

#include <esp_ota_ops.h>
#include <zlib.h>

#include <memory>

namespace nutt {

class CompressedOTA {
public:
	static constexpr const char *TAG = "nutt.OTA";

	CompressedOTA() = default;
	~CompressedOTA();

	bool start();
	bool write(const uint8_t *data, size_t size);
	bool finish();

private:
	bool write(const uint8_t *data, size_t size, bool flush);

	bool zlib_init_{false};
	z_stream zlib_stream_;
	const esp_partition_t *part_{nullptr};
	esp_ota_handle_t handle_{0};
};

} // namespace nutt
