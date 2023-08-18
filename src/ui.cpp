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
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <hal/uart_ll.h>
#include <led_strip.h>
#include <soc/uart_pins.h>
#include <string.h>

#include <bitset>
#include <thread>
#include <unordered_map>

#include "nutt/debounce.h"
#include "nutt/device.h"
#include "nutt/log.h"
#include "nutt/zigbee.h"

namespace nutt {

using namespace ui;
using namespace ui::colour;

const std::unordered_map<Event,LEDSequence> UserInterface::led_sequences_{
	{ Event::IDLE,                                 {    0, { { OFF, 0 }                     } } },
	{ Event::NETWORK_CONNECT,                      { 8000, { { GREEN, 5000 }, { OFF, 0 }    } } },
	{ Event::NETWORK_CONNECTED,                    {    0, { { GREEN, 250 }, { OFF, 2750 }  } } },
	{ Event::OTA_UPDATE_OK,                        {  500, { { CYAN, 0 }                    } } },
	{ Event::LIGHT_SWITCHED_LOCAL,                 { 2000, { { ORANGE, 0 }                  } } },
	{ Event::LIGHT_SWITCHED_REMOTE,                { 2000, { { BLUE, 0 }                    } } },
	{ Event::IDENTIFY,                             { 3000, { { MAGENTA, 0 }                 } } },
	{ Event::OTA_UPDATE_ERROR,                     { 3000, { { RED, 200 }, { OFF, 200 }     } } },
	{ Event::NETWORK_UNCONFIGURED_DISCONNECTED,    {    0, { { WHITE, 0 }                   } } },
	{ Event::NETWORK_UNCONFIGURED_CONNECTING,      {    0, { { WHITE, 250 }, { OFF, 250 }   } } },
	{ Event::NETWORK_CONFIGURED_DISCONNECTED,      {    0, { { YELLOW, 0 }                  } } },
	{ Event::NETWORK_CONFIGURED_CONNECTING,        {    0, { { YELLOW, 250 }, { OFF, 250 }  } } },
	{ Event::NETWORK_ERROR,                        { 1000, { { RED, 250 }, { OFF, 250 }     } } },
	{ Event::NETWORK_CONFIGURED_FAILED,            {    0, { { RED, 0 }                     } } },
	{ Event::NETWORK_UNCONFIGURED_FAILED,          {    0, { { RED, 0 }, { OFF, 500 }       } } },
};

} // namespace nutt

namespace nutt {

namespace colour = ui::colour;
using ui::Event;
using ui::NetworkState;
using ui::RGBColour;

UserInterface::UserInterface(Logging &logging, gpio_num_t network_join_pin,
		bool active_low) : WakeupThread("UI"), logging_(logging),
		button_debounce_(network_join_pin, active_low, DEBOUNCE_PRESS_US,
			DEBOUNCE_RELEASE_US) {
	led_strip_config_t led_strip_config{};
	led_strip_rmt_config_t rmt_config{};

	led_strip_config.max_leds = 1;
	led_strip_config.strip_gpio_num = 8;
	led_strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
	led_strip_config.led_model = LED_MODEL_WS2812;
	rmt_config.resolution_hz = 10 * 1000 * 1000;

	ESP_ERROR_CHECK(led_strip_new_rmt_device(&led_strip_config, &rmt_config, &led_strip_));
	set_led(colour::OFF);

	ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, U0TXD_GPIO_NUM, U0RXD_GPIO_NUM,
		UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	uart_config_t uart_config{};
	uart_config.baud_rate = 115200;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

	ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
	ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, SOC_UART_FIFO_LEN + 1,
		0, 0, nullptr, ESP_INTR_FLAG_LEVEL1));

	uart_intr_config_t uart_int_config{};
	uart_int_config.intr_enable_mask = UART_INTR_RXFIFO_FULL;
	uart_int_config.rxfifo_full_thresh = 1;

	ESP_ERROR_CHECK(uart_intr_config(UART_NUM_0, &uart_int_config));
	ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_0));
}

void UserInterface::set_led(RGBColour colour) {
	ESP_ERROR_CHECK(led_strip_set_pixel(led_strip_, 0,
		colour.red * LED_LEVEL / 255,
		colour.green * LED_LEVEL / 255,
		colour.blue * LED_LEVEL / 255));
	ESP_ERROR_CHECK(led_strip_refresh(led_strip_));
}

void UserInterface::attach(Device &device) {
	device_ = &device;
}

void UserInterface::start() {
	std::thread t;

	button_debounce_.start(*this);

	make_thread(t, "ui_main", 4096, 1, &UserInterface::run_loop, this);
	t.detach();

	make_thread(t, "ui_uart", 4096, 1, &UserInterface::uart_handler, this);
	t.detach();
}

unsigned long UserInterface::run_tasks() {
	DebounceResult debounce = button_debounce_.run();

	if (debounce.changed
			&& button_debounce_.value()
			&& !button_debounce_.first()) {
		Device *device = device_;

		ESP_LOGI(TAG, "Network join/leave button pressed");

		if (device) {
			device->network_do(ZigbeeAction::JOIN_OR_LEAVE);
		}
	}

	return std::min(debounce.wait_ms, update_led());
}

void UserInterface::uart_handler() {
	char buf[1];

	while (true) {
		if (uart_read_bytes(UART_NUM_0, buf, sizeof(buf), portMAX_DELAY) == 1) {
			Device *device = device_;

			if (buf[0] == '0') {
				logging_.set_app_level(ESP_LOG_NONE);
				logging_.set_sys_level(ESP_LOG_NONE);
			} else if (buf[0] >= '1' && buf[0] <= '5') {
				logging_.set_app_level(static_cast<esp_log_level_t>(buf[0] - '1' + 1));
			} else if (buf[0] >= '6' && buf[0] <= '9') {
				logging_.set_sys_level(static_cast<esp_log_level_t>(buf[0] - '6' + 1));
			} else if (buf[0] == 'R') {
				esp_restart();
			} else if (device && buf[0] == 'j') {
				device->network_do(ZigbeeAction::JOIN);
			} else if (device && buf[0] == 'l') {
				device->network_do(ZigbeeAction::LEAVE);
			}
		}
	}
}

void UserInterface::start_event(Event event) {
	unsigned long value = static_cast<unsigned long>(event);

	active_events_.set(value);
	active_sequence_.insert_or_assign(event, led_sequences_.at(event));
}

void UserInterface::restart_event(Event event) {
	stop_event(event);
	start_event(event);
}

bool UserInterface::event_active(Event event) {
	unsigned long value = static_cast<unsigned long>(event);

	return active_events_.test(value);
}

void UserInterface::stop_event(Event event) {
	if (event_active(event)) {
		unsigned long value = static_cast<unsigned long>(event);

		active_events_.reset(value);
		active_sequence_.erase(event);

		if (render_time_us_ && render_event_ == event) {
			render_time_us_ = 0;
		}
	}
}

inline void UserInterface::stop_events(std::initializer_list<Event> events) {
	for (Event event : events)
		stop_event(event);
}

unsigned long UserInterface::update_led() {
	uint64_t now_us_ = esp_timer_get_time();

	if (render_time_us_ && event_active(render_event_)) {
		uint64_t elapsed_us = now_us_ - render_time_us_;
		auto &sequence = active_sequence_[render_event_];

		if (sequence.states[0].duration_ms) {
			if (elapsed_us >= sequence.states[0].remaining_us) {
				sequence.states[0].remaining_us = sequence.states[0].duration_ms * 1000UL;
				auto state_copy = sequence.states[0];
				sequence.states.erase(sequence.states.begin());
				sequence.states.emplace_back(std::move(state_copy));
			} else {
				sequence.states[0].remaining_us -= elapsed_us;
			}
		}

		if (sequence.duration_ms) {
			if (elapsed_us >= sequence.remaining_us) {
				stop_event(render_event_);
			} else {
				sequence.remaining_us -= elapsed_us;
			}
		}
	}

	unsigned long wait_ms;
	RGBColour colour;
	Event event;

	std::unique_lock lock{mutex_};
	unsigned long value = ffsl(active_events_.to_ulong());

	if (value) {
		event = static_cast<Event>(value - 1);
	} else {
		event = Event::IDLE;
		start_event(event);
	}

	auto &sequence = active_sequence_[event];

	if (sequence.states[0].duration_ms) {
		wait_ms = std::min(static_cast<unsigned long>(sequence.states[0].remaining_us / 1000UL), ULONG_MAX - 1);
	} else {
		wait_ms = ULONG_MAX;
	}

	if (sequence.duration_ms) {
		wait_ms = std::min(wait_ms, std::min(static_cast<unsigned long>(sequence.remaining_us / 1000UL), ULONG_MAX - 1));
	} else {
		wait_ms = std::min(wait_ms, ULONG_MAX);
	}

	colour = sequence.states[0].colour;

	render_time_us_ = esp_timer_get_time();
	render_event_ = event;
	lock.unlock();

	set_led(colour);

	return wait_ms;
}

void UserInterface::network_state(bool configured, NetworkState state) {
	std::lock_guard lock{mutex_};
	auto event = Event::IDLE;

	switch (state) {
	case NetworkState::DISCONNECTED:
		event = configured ? Event::NETWORK_CONFIGURED_DISCONNECTED
			: Event::NETWORK_UNCONFIGURED_DISCONNECTED;
		break;

	case NetworkState::CONNECTING:
		event = configured ? Event::NETWORK_CONFIGURED_CONNECTING
			: Event::NETWORK_UNCONFIGURED_CONNECTING;
		break;

	case NetworkState::CONNECTED:
		event = Event::NETWORK_CONNECTED;
		break;

	case NetworkState::FAILED:
		event = configured ? Event::NETWORK_CONFIGURED_FAILED
			: Event::NETWORK_UNCONFIGURED_FAILED;
		stop_event(Event::NETWORK_ERROR);
		break;
	}

	if (state == NetworkState::CONNECTED) {
		if (!event_active(event)) {
			restart_event(Event::NETWORK_CONNECT);
		}
	} else {
		stop_event(Event::NETWORK_CONNECT);
	}

	stop_events({
		Event::NETWORK_CONFIGURED_CONNECTING,
		Event::NETWORK_CONFIGURED_DISCONNECTED,
		Event::NETWORK_CONFIGURED_FAILED,
		Event::NETWORK_CONNECTED,
		Event::NETWORK_UNCONFIGURED_CONNECTING,
		Event::NETWORK_UNCONFIGURED_DISCONNECTED,
		Event::NETWORK_UNCONFIGURED_FAILED,
	});

	restart_event(event);
	wake_up();
}

void UserInterface::network_error() {
	std::lock_guard lock{mutex_};

	restart_event(Event::NETWORK_ERROR);
	wake_up();
}

void UserInterface::identify(uint16_t seconds) {
	ESP_LOGI(TAG, "Identify for %us", seconds);

	std::lock_guard lock{mutex_};

	stop_event(Event::IDENTIFY);
	if (seconds) {
		start_event(Event::IDENTIFY);
	}
	wake_up();
}

void UserInterface::ota_update(bool ok) {
	std::lock_guard lock{mutex_};

	restart_event(ok ? Event::OTA_UPDATE_OK : Event::OTA_UPDATE_ERROR);
	wake_up();
}

void UserInterface::light_switched(bool local) {
	std::lock_guard lock{mutex_};

	restart_event(local ? Event::LIGHT_SWITCHED_LOCAL
		: Event::LIGHT_SWITCHED_REMOTE);
	wake_up();
}

} // namespace nutt
