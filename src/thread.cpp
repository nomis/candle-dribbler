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

#include "nutt/thread.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace nutt {

WakeupThread::WakeupThread(const char *name) : name_(name) {
	semaphore_ = xSemaphoreCreateBinary();
	if (!semaphore_) {
		ESP_LOGE(TAG, "Semaphore create for %s failed", name_);
		esp_restart();
	}
}

void WakeupThread::run_loop() {
	while (true) {
		TickType_t wait_ticks = portMAX_DELAY;
		unsigned long wait_ms = run_tasks();

		static_assert(static_cast<uintmax_t>(ULONG_MAX)
			<= static_cast<uintmax_t>(portMAX_DELAY)
				* static_cast<uintmax_t>(portTICK_PERIOD_MS));

		if (wait_ms < ULONG_MAX) {
			wait_ticks = std::max(static_cast<TickType_t>(1U),
				wait_ms / portTICK_PERIOD_MS);
		}

		xSemaphoreTake(semaphore_, wait_ticks);
	}

	ESP_LOGE(TAG, "%s loop stopped", name_);
	esp_restart();
}

void WakeupThread::wake_up_isr() {
	BaseType_t xHigherPriorityTaskWoken{pdFALSE};

	xSemaphoreGiveFromISR(semaphore_, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

} // namespace nutt
