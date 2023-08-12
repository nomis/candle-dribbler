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

#include "nutt/ui.h"

#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <led_strip.h>

#include "nutt/device.h"

namespace nutt {

UserInterface::UserInterface(gpio_num_t network_join_pin): WakeupThread("UI") {
	led_strip_config_t led_strip_config{};
	led_strip_rmt_config_t rmt_config{};

	led_strip_config.max_leds = 1;
	led_strip_config.strip_gpio_num = 8;
	led_strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
	led_strip_config.led_model = LED_MODEL_WS2812;
	rmt_config.resolution_hz = 10 * 1000 * 1000;

	ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_config, &rmt_config, &led_strip_));
	set_led(0, 0, 0);

	gpio_config_t network_join_config = {
		.pin_bit_mask = 1ULL << network_join_pin,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_NEGEDGE,
	};

	ESP_ERROR_CHECK(gpio_config(&network_join_config));
	ESP_ERROR_CHECK(gpio_isr_handler_add(network_join_pin, ui_network_join_interrupt_handler, this));
}

void ui_network_join_interrupt_handler(void *arg) {
	static_cast<UserInterface*>(arg)->network_join_interrupt_handler();
}

void UserInterface::network_join_interrupt_handler() {
	button_press_count_irq_++;
	wake_up_isr();
}

void UserInterface::set_led(uint8_t red, uint8_t green, uint8_t blue) {
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip_, 0, red * LED_LEVEL / 255,
		green * LED_LEVEL / 255, blue * LED_LEVEL / 255));
	ESP_ERROR_CHECK(led_strip_refresh(led_strip_));
}

void UserInterface::attach(Device &device) {
	device_ = &device;
}

unsigned long UserInterface::run_tasks() {
	unsigned long button_press_count_copy = button_press_count_irq_;

	if (button_press_count_copy != button_press_count_) {
		button_press_count_ = button_press_count_copy;
		ESP_LOGI(TAG, "Network join/leave button pressed");
		Device *device = device_;

		if (device)
			device->network_join_or_leave();
	}

	return ULONG_MAX;
}

void UserInterface::identify(uint16_t seconds) {
	ESP_LOGI(TAG, "Identify for %us", seconds);
	if (seconds) {
		set_led(255, 0, 255);
	} else {
		set_led(0, 0, 0);
	}
}

} // namespace nutt
