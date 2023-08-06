Candle Dribbler
===============

ESP32 Zigbee multi-channel light controller and 433 MHz RF receiver.

Supports up to 6 lights (switch and relay) as well as the SONOFF RM433 and
Griffin AirClick RF receivers.

Requires an ESP32-H2 or ESP32-C6.

Work in progress, doesn't do any RF or OTA updates yet.

	...some very big candles that were just lava streams of wax, and a raven on
	a skull.

	"They get it all out of a catalogue," said the raven. "Believe me. It all
	comes in a big box. You think candles get dribbly like that by themselves?
	That's three days' work for a skilled candle dribbler."

	"You're just making that up," said Susan.
	"Anyway, you can't just buy a skull."

	-- `Terry Pratchett <https://en.wikipedia.org/wiki/Terry_Pratchett>`_
	(`Soul Music, 1994 <https://en.wikipedia.org/wiki/Soul_Music_(novel)>`_)

Usage
-----

Each physical light has a switch GPIO and a relay GPIO and presents the
following Zigbee endpoints:

* Primary Light - *physical switch*
* Secondary Light - *virtual switch*
* Tertiary Light - *virtual switch*
* Switch Status (Binary Input) - *physical switch*
* Enable Switch

The **Primary Light** will be set on/off whenever the switch GPIO is activated
or deactivated. It does not function as a two-way switch.

The relay will be activated under the following conditions:

* The **Primary Light** is on *and* the **Enable Switch** is on
* The **Secondary Light** is on
* The **Tertiary Light** is on

The reason for having so many light endpoints is that it provides the unmodified
state of the physical switch when implementing automation on the virtual
switches. If a timer is used to turn on/off the light using the secondary or
tertiary endpoints, it will remain separate from any use of the primary switch
instead of being unaware that the light should now stay on.

Whenever the **Primary Light** is turned off the **Secondary Light** will also
be turned off. This can be used to implement "turn on now, but cancel when the
light is switched off" behaviour locally without relying on remote communication
(after turning on the light). Useful for preempting motion sensors or
automatically turning the light on for a period of time but then having the
light turn off normally *without* having a conflict between tracking the
physical and virtual switch states.

The **Enable Switch** can be used to prevent the **Primary Light** from
activating the relay, e.g. to ignore motion sensor activations when changing
bulbs.

All of the **Light** endpoints can be modified remotely, with the **Switch
Status** being a read-only representation of the current switch state (which is
a useful record of activity if that is a motion detector).

Build
-----

This project can be built with the `ESP-IDF build system
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html`_.

Configure::

	idf.py set-target esp32c6
	idf.py menuconfig

Under "Component config" you'll find "Candle Dribbler" where you can configure
the number of lights supported and whether switches/relays are active low or not.

The GPIO configuration assumes you're using an `ESP32-C6-DevKitC-1
<https://docs.espressif.com/projects/espressif-esp-dev-kits/en/latest/esp32c6/esp32-c6-devkitc-1/>`_.

Build::

	idf.py build

Flash::

	idf.py flash
