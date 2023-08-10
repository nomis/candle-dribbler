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
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "nutt/device.h"

namespace nutt {

UserInterface::UserInterface(gpio_num_t network_join_pin) {
	semaphore_ = xSemaphoreCreateBinary();
	if (!semaphore_) {
		ESP_LOGE(TAG, "Semaphore create failed");
		esp_restart();
	}

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
	BaseType_t xHigherPriorityTaskWoken{pdFALSE};

	button_press_count_irq_++;

	xSemaphoreGiveFromISR(semaphore_, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UserInterface::attach(Device &device) {
	device_ = &device;
}

void UserInterface::run() {
	while (true) {
		xSemaphoreTake(semaphore_, portMAX_DELAY);

		unsigned long button_press_count_copy = button_press_count_irq_;

		if (button_press_count_copy != button_press_count_) {
			button_press_count_ = button_press_count_copy;
			ESP_LOGI(TAG, "Network join/leave button pressed");
			Device *device = device_;

			if (device)
				device->network_join_or_leave();
		}
	}

	ESP_LOGE(TAG, "UI loop stopped");
	esp_restart();
}

} // namespace nutt
