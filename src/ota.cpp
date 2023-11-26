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

#include "nutt/ota.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <zlib.h>

namespace nutt {

CompressedOTA::~CompressedOTA() {
	if (part_) {
		esp_ota_abort(handle_);
	}

	if (zlib_init_) {
		inflateEnd(&zlib_stream_);
	}
}

bool CompressedOTA::start() {
	if (zlib_init_) {
		inflateEnd(&zlib_stream_);
		zlib_init_ = false;
	}

	zlib_stream_ = {};
	zlib_stream_.zalloc = Z_NULL;
	zlib_stream_.zfree = Z_NULL;
	zlib_stream_.opaque = Z_NULL;
	zlib_stream_.next_in = nullptr;
	zlib_stream_.avail_in = 0;

	int ret = inflateInit(&zlib_stream_);

	if (ret == Z_OK) {
		zlib_init_ = true;
	} else {
		ESP_LOGE(TAG, "zlib init failed: %d", ret);
		return false;
	}

	if (part_) {
		ESP_LOGE(TAG, "OTA already started");
		part_ = nullptr;
		esp_ota_abort(handle_);
		return false;
	}

	part_ = esp_ota_get_next_update_partition(nullptr);
	if (!part_) {
		ESP_LOGE(TAG, "No next OTA partition");
		return false;
	}

	esp_err_t err = esp_ota_begin(part_, OTA_WITH_SEQUENTIAL_WRITES, &handle_);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error starting OTA: %d", err);
		part_ = nullptr;
		return false;
	}

	return true;
}

bool CompressedOTA::write(const uint8_t *data, size_t size) {
	return write(data, size, false);
}

bool CompressedOTA::write(const uint8_t *data, size_t size, bool flush) {
	uint8_t buf[256];

	if (!part_) {
		return false;
	}

	zlib_stream_.avail_in = size;
	zlib_stream_.next_in = data;

	do {
		zlib_stream_.avail_out = sizeof(buf);
		zlib_stream_.next_out = buf;

		int ret = inflate(&zlib_stream_, flush ? Z_FINISH : Z_NO_FLUSH);
		if (ret == Z_STREAM_ERROR || ret == Z_NEED_DICT || ret == Z_DATA_ERROR || ret == Z_MEM_ERROR) {
			ESP_LOGE(TAG, "zlib error: %d", ret);
			esp_ota_abort(handle_);
			part_ = nullptr;
			return false;
		}

		size_t available = sizeof(buf) - zlib_stream_.avail_out;

		if (available > 0) {
			esp_err_t err = esp_ota_write(handle_, buf, available);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "Error writing OTA: %d", err);
				esp_ota_abort(handle_);
				part_ = nullptr;
				return false;
			}
		}
	} while (zlib_stream_.avail_in > 0 || zlib_stream_.avail_out == 0);

	return true;
}

bool CompressedOTA::finish() {
	if (!part_) {
		ESP_LOGE(TAG, "OTA not running");
		return false;
	}

	if (!write(nullptr, 0, true)) {
		return false;
	}

	esp_err_t err = esp_ota_end(handle_);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error ending OTA: %d", err);
		part_ = nullptr;
		return false;
	}

	inflateEnd(&zlib_stream_);
	zlib_init_ = false;

	err = esp_ota_set_boot_partition(part_);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Error setting boot partition: %d", err);
		part_ = nullptr;
		return false;
	}

	part_ = nullptr;
	return true;
}

} // namespace nutt
