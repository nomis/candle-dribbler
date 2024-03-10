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

#include "nutt/main.h"

#include <esp_err.h>
#include <esp_log.h>
#include <esp_task_wdt.h>
#include <nvs_flash.h>
#include <driver/gpio.h>

#include "nutt/device.h"
#include "nutt/log.h"
#include "nutt/light.h"
#include "nutt/ui.h"

using namespace nutt;

static_assert(nutt::Device::NUM_EP_PER_DEVICE + MAX_LIGHTS * nutt::Light::NUM_EP_PER_LIGHT <= ZB_MAX_EP_NUMBER,
	"You'll need to ask Espressif to let you use more endpoints");

extern "C" void app_main() {
	ESP_ERROR_CHECK(esp_task_wdt_add(nullptr));

	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);
	ESP_ERROR_CHECK(esp_task_wdt_reset());

	auto &logging = *new Logging{};

	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2));

	auto &ui = *new UserInterface{logging, GPIO_NUM_4, true};
	auto &device = *new Device{ui};

	/*                                 Switch       Active Low          Relay        Active Low
	 *                                 -----------  -----------------   -----------  ----------------
	 */
	if (MAX_LIGHTS >= 1) (new Light{1, GPIO_NUM_3,  SWITCH_ACTIVE_LOW,  GPIO_NUM_18, RELAY_ACTIVE_LOW })->attach(device);
	if (MAX_LIGHTS >= 2) (new Light{2, GPIO_NUM_2,  SWITCH_ACTIVE_LOW,  GPIO_NUM_19, RELAY_ACTIVE_LOW })->attach(device);
	if (MAX_LIGHTS >= 3) (new Light{3, GPIO_NUM_11, SWITCH_ACTIVE_LOW,  GPIO_NUM_20, RELAY_ACTIVE_LOW })->attach(device);
	if (MAX_LIGHTS >= 4) (new Light{4, GPIO_NUM_10, SWITCH_ACTIVE_LOW,  GPIO_NUM_21, RELAY_ACTIVE_LOW })->attach(device);
	if (MAX_LIGHTS >= 5) (new Light{5, GPIO_NUM_1,  SWITCH_ACTIVE_LOW,  GPIO_NUM_22, RELAY_ACTIVE_LOW })->attach(device);
	if (MAX_LIGHTS >= 6) (new Light{6, GPIO_NUM_0,  SWITCH_ACTIVE_LOW,  GPIO_NUM_23, RELAY_ACTIVE_LOW })->attach(device);
	if (MAX_LIGHTS >= 7) (new Light{7, GPIO_NUM_7,  SWITCH_ACTIVE_LOW,  GPIO_NUM_15, RELAY_ACTIVE_LOW })->attach(device);
	if (MAX_LIGHTS >= 8) (new Light{8, GPIO_NUM_6,  SWITCH_ACTIVE_LOW,  GPIO_NUM_5,  RELAY_ACTIVE_LOW })->attach(device);

	ESP_ERROR_CHECK(esp_task_wdt_reset());
	device.start();

	ui.attach(device);
	ui.start();
	ESP_ERROR_CHECK(esp_task_wdt_reset());

	TaskStatus_t status;

	vTaskGetInfo(nullptr, &status, pdTRUE, eRunning);
	ESP_LOGD(TAG, "Free stack: %lu", status.usStackHighWaterMark);

	ESP_ERROR_CHECK(esp_task_wdt_delete(nullptr));
}
