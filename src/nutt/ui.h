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
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <led_strip.h>
#include <sdkconfig.h>

#include <atomic>

namespace nutt {

class Device;

IRAM_ATTR void ui_network_join_interrupt_handler(void *arg);

class UserInterface {
	friend void ui_network_join_interrupt_handler(void *arg);

public:
	UserInterface(gpio_num_t network_join_pin);
	~UserInterface() = delete;

	void attach(Device &device);
	void run();

	void identify(uint16_t seconds);

private:
	static constexpr const char *TAG = "nutt.UI";
	static constexpr const uint8_t LED_LEVEL = CONFIG_NUTT_UI_LED_BRIGHTNESS;

	IRAM_ATTR void network_join_interrupt_handler();
	void set_led(uint8_t red, uint8_t green, uint8_t blue);

	SemaphoreHandle_t semaphore_{nullptr};
	led_strip_handle_t led_strip_{nullptr};
	unsigned long button_press_count_{0};
	std::atomic<unsigned long> button_press_count_irq_{0};
	Device *device_{nullptr};
};

} // namespace nutt
