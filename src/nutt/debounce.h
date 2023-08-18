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

#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>

#include <atomic>

namespace nutt {

class WakeupThread;

IRAM_ATTR void debounce_interrupt_handler(void *arg);

struct DebounceResult {
	unsigned long wait_ms;
	bool changed;
};

class Debounce {
	friend void debounce_interrupt_handler(void *arg);

public:
	Debounce(gpio_num_t pin, bool active_low, unsigned long duration_us);
	Debounce(gpio_num_t pin, bool active_low, unsigned long press_duration_us,
		unsigned long release_duration_us);
	~Debounce();

	void start(WakeupThread &wakeup);
	DebounceResult run();
	inline bool value() const { return state_ == active(); }
	inline bool first() const { return first_ == 1; }

private:
	inline int active() const { return active_low_ ? 0 : 1; }
	inline int inactive() const { return active_low_ ? 1 : 0; }

	IRAM_ATTR void interrupt_handler();

	WakeupThread *wakeup_{nullptr};
	const unsigned long press_duration_us_;
	const unsigned long release_duration_us_;
	const gpio_num_t pin_;
	const bool active_low_;

	uint8_t first_{0};
	int change_state_;
	uint64_t change_us_{0};
	int state_;
	unsigned long change_count_{0};
	std::atomic<unsigned long> change_count_irq_{0};
};

} // namespace nutt
