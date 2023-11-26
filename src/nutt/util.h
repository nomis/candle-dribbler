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

#include <cstring>
#include <string>
#include <string_view>

namespace nutt {

std::string duration_us_to_string(uint64_t duration_us);

template<typename T, size_t size>
static inline std::string null_terminated_string(T(&data)[size]) {
	T *found = reinterpret_cast<T*>(std::memchr(&data[0], '\0', size));
	return std::string{&data[0], found ? (found - &data[0]) : size};
};

} // namespace nutt
