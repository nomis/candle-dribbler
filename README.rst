Candle Dribbler
===============

ESP32 Zigbee multi-channel light controller.

Supports up to 6 lights (switch and relay).

Requires an ESP32-H2 or ESP32-C6.

Work in progress.

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

Use a push button on GPIO4 to GND (or use the UART command) to join/leave the
Zigbee network. Joining a new network is not performed automatically. Leaving
the network currently requires a restart.

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

The power on state is all lights off. The **Enable Switch** setting is stored in
flash so it's persistent.

LED Events
~~~~~~~~~~

.. list-table::
   :widths: 50 50
   :header-rows: 1

   * - Colours
     - Description
   * - White
     - Disconnected (network not configured)
   * - White (blinking)
     - Connecting (network not configured)
   * - Yellow
     - Disconnected (network configured)
   * - Yellow (blinking)
     - Connecting (network configured)
   * - Green (constant then blinking every 3 seconds)
     - Network connected
   * - Red (blinking 2 times for 1 second)
     - Network error
   * - Red
     - Network failed (network configured)
   * - Red (blinking)
     - Network failed (network not configured)
   * - Orange (for 2 seconds)
     - Light switched locally
   * - Blue (for 2 seconds)
     - Light switched remotely
   * - Magenta
     - Identify request received
   * - Cyan
     - OTA update in progress
   * - Red (blinking 8 times in 3 seconds)
     - OTA update error
   * - Rainbow (cycling)
     - Core dump preset

UART Commands
~~~~~~~~~~~~~

.. list-table::
   :widths: 15 85
   :header-rows: 1

   * - Keys
     - Description
   * - ``0``
     - Disable logging (persistent)
   * - ``1``\ ..\ ``5``
     - Set application log level to ERROR..VERBOSE (persistent)
   * - ``6``\ ..\ ``9``
     - Set system log level to ERROR..DEBUG (persistent)
   * - ``j``
     - Join Zigbee network (no effect if already joined/joining)
   * - ``l``
     - Leave Zigbee network (no effect if already left)
   * - ``m``
     - Print memory information
   * - ``t``
     - Print task list and stats
   * - ``R``
     - Restart
   * - ``C``
     - Crash (used for testing to generate a core dump)
   * - ``d``
     - Print brief core dump summary
   * - ``D``
     - Print whole core dump
   * - ``E``
     - Erase saved core dump

Build
-----

This project can be built with the `ESP-IDF build system
<https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html>`_.

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


Help
----

I have 15 "light" entities in Home Assistant, which is which?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Zigbee specifications are thousands of pages long and it supports 240
endpoints per device but there's no attribute to describe on/off endpoints if
you have more than one of the same type!

The endpoints should be in some kind of logical order, which means you'll have
all of the **Primary Light**\ s first, then the **Secondary Light**\ s followed
by the **Tertiary Light**\ s. Each of these groupings should be in the order of
the physical light GPIOs.

If it's still not clear, you can identify each endpoint with the following
steps:

#. Turn all of the switch endpoints on and all of the light endpoints off.
#. Turn each physical light switch on one by one. The light and binary input
   endpoints that turn on will identify the corresponding **Primary Light** and
   **Switch Status** endpoints.
#. Turn all the other light endpoints on.
#. Turn each **Primary Light** off one by one. That will identify the
   corresponding **Secondary Light** endpoint because it will turn off too.
#. Turn each of the remaining light endpoints off one by one. That will identify
   the corresponding **Tertiary Light** endpoints because the light will go off.
#. Turn all the switch endpoints off and all of the **Primary Light** endpoints
   on.
#. Turn the switch endpoints on one by one. That will identify the corresponding
   **Enable Switch** endpoint because the light will come on.
