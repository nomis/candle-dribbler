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

#include "nutt/debounce.h"

#include <driver/gpio.h>

#include "nutt/thread.h"

namespace nutt {

Debounce::Debounce(gpio_num_t pin, bool active_low, unsigned long duration_us)
		: Debounce(pin, active_low, duration_us, duration_us) {
}

Debounce::Debounce(gpio_num_t pin, bool active_low,
		unsigned long press_duration_us, unsigned long release_duration_us)
		: press_duration_us_(press_duration_us),
		release_duration_us_(release_duration_us),
		pin_(pin), active_low_(active_low) {
	gpio_config_t config = {
		.pin_bit_mask = 1ULL << pin_,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = active_low_ ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
		.pull_down_en = active_low_ ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	ESP_ERROR_CHECK(gpio_config(&config));
	change_state_ = state_ = gpio_get_level(pin_);
	ESP_ERROR_CHECK(gpio_isr_handler_add(pin_, debounce_interrupt_handler, this));
}

Debounce::~Debounce() {
	ESP_ERROR_CHECK(gpio_intr_disable(pin_));
}

void Debounce::start(WakeupThread &wakeup) {
	wakeup_ = &wakeup;
	ESP_ERROR_CHECK(gpio_set_intr_type(pin_, GPIO_INTR_ANYEDGE));
	ESP_ERROR_CHECK(gpio_intr_enable(pin_));
}

DebounceResult Debounce::run() {
	unsigned long wait_ms = ULONG_MAX;
	unsigned long change_count_copy = change_count_irq_;
	uint64_t now_us = esp_timer_get_time();
	int level = gpio_get_level(pin_);
	bool changed = false;

	if (change_count_ != change_count_copy) {
		change_count_ = change_count_copy;
		change_us_ = now_us;
	}

	if (change_state_ != level) {
		change_state_ = level;
		change_us_ = now_us;
	}

	if (first_ == 0 && change_us_ == 0) {
		change_us_ = now_us;
	}

	auto debounce_us = change_state_ == active() ? press_duration_us_ : release_duration_us_;

	if (first_ == 0 || state_ != change_state_) {
		if (now_us - change_us_ >= debounce_us) {
			state_ = change_state_;
			changed = true;

			if (first_ < 2) {
				first_++;
			}
		} else {
			wait_ms = (debounce_us - (now_us - change_us_)) / 1000UL;
		}
	}

	return {wait_ms, changed};
}

void debounce_interrupt_handler(void *arg) {
	static_cast<Debounce*>(arg)->interrupt_handler();
}

void Debounce::interrupt_handler() {
	change_count_irq_++;
	wakeup_->wake_up_isr();
}

} // namespace nutt
