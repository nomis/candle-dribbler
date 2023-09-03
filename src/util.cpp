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

#include "nutt/util.h"

#include <stdio.h>

#include <string>
#include <vector>

namespace nutt {

std::string duration_us_to_string(uint64_t duration_us) {
	uint64_t duration_ms = duration_us / 1000U;
	unsigned long days = 0;
	unsigned int hours, minutes, seconds, milliseconds;

	days = duration_ms / (86400U * 1000U);
	duration_ms %= 86400U * 1000U;

	hours = duration_ms / (3600U * 1000U);
	duration_ms %= 3600U * 1000U;

	minutes = duration_ms / (60U * 1000U);
	duration_ms %= 60U * 1000U;

	seconds = duration_ms / 1000U;
	duration_ms %= 1000U;

	milliseconds = duration_ms;

	std::vector<char> data(32);

	snprintf(data.data(), data.size(), "%03lu+%02u:%02u:%02u.%03u",
		days, hours, minutes, seconds, milliseconds);

	return {data.data()};
}

} // namespace nutt
