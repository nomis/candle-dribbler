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

#include <cstddef>
#include <sdkconfig.h>

namespace nutt {

static constexpr const char *TAG = "nutt";
static constexpr const size_t MAX_LIGHTS = CONFIG_NUTT_MAX_LIGHTS;
static constexpr const bool SWITCH_ACTIVE_LOW = CONFIG_NUTT_SWITCH_ACTIVE_LOW;
static constexpr const bool RELAY_ACTIVE_LOW = CONFIG_NUTT_RELAY_ACTIVE_LOW;

} // namespace nutt
