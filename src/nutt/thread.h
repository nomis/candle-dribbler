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

#include <esp_pthread.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <thread>
#include <utility>

namespace nutt {

template<class Function, class... Args>
void make_thread(std::thread &t, const char *name, size_t stack_size,
		size_t prio, Function&& f, Args&&... args ) {
	auto cfg = esp_pthread_get_default_config();
	cfg.stack_size = stack_size;
	cfg.prio = prio;
	cfg.thread_name = name;
	esp_pthread_set_cfg(&cfg);

	t = std::thread{std::forward<Function>(f), std::forward<Args>(args)...};
}

class WakeupThread {
public:
	static constexpr const char *TAG = "nutt.WakeupThread";

	IRAM_ATTR void wake_up_isr();

protected:
	WakeupThread(const char *name);
	~WakeupThread() = default;

	virtual unsigned long run_tasks() = 0;
	void run_loop();
	void wake_up();

private:
	static void wake_up_timer(void *arg);

	const char *name_;
	SemaphoreHandle_t semaphore_{nullptr};
	esp_timer_handle_t timer_{nullptr};
};

} // namespace nutt
