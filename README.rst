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
following Zigbee on/off clusters:

* Primary Light - *physical switch*
* Secondary Light - *virtual switch*
* Tertiary Light - *virtual switch*
* Switch Status (Binary Input) - *physical switch*
* Temporary Enable Switch
* Persistent Enable Switch

The **Primary Light** will be set on/off whenever the switch GPIO is activated
or deactivated. It does not function as a two-way switch.

The relay will be activated under the following conditions:

* The **Primary Light** is on *and* the **Temporary Enable Switch** is on
* The **Secondary Light** is on
* The **Tertiary Light** is on

The reason for having so many light clusters is that it provides the unmodified
state of the physical switch when implementing automation on the virtual
switches. If a timer is used to turn on/off the light using the secondary or
tertiary cluster, it will remain separate from any use of the primary switch
instead of being unaware that the light should now stay on.

Whenever the **Primary Light** is turned off the **Secondary Light** will also
be turned off. This can be used to implement "turn on now, but cancel when the
light is switched off" behaviour locally without relying on remote communication
(after turning on the light). Useful for preempting motion detector or
automatically turning the light on for a period of time but then having the
light turn off normally *without* having a conflict between tracking the
physical and virtual switch states.

The **Temporary Enable Switch** can be used to prevent the **Primary Light**
from activating the relay, e.g. to ignore motion detector activations when
changing bulbs or during specific time periods or conditions. When changed, the
**Persistent Enable Switch** will update the **Temporary Enable Switch** and
store the selection in flash to be used on startup.

All of the **Light** clusters can be modified remotely, with the **Switch
Status** being a read-only representation of the current switch state (which is
a useful record of activity if that is a motion detector).

The power on state is all lights off. The **Persistent Enable Switch** setting
is stored in flash so it's persistent (it will also be used to set the initial
value of the **Temporary Enable Switch**).

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

What order are all the entities shown in Home Assistant?
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Zigbee specifications are thousands of pages long and it supports 240
endpoints per device but there's no attribute to describe on/off clusters if
you have more than one of the same type!

Using `this version of homeassistant-entity-renamer
<https://github.com/nomis/homeassistant-entity-renamer>`_ that can update
the friendly names (so that they're not all "Light" and "Switch") and the
`hass-rename-entities.sh script <hass-rename-entities.sh>`_ you can rename
all of the entities automatically.

The control cluster endpoints are in the following order:

.. list-table::
   :widths: 20 10 70
   :header-rows: 1

   * - Type
     - Endpoint
     - Name
   * - Light
     - 11
     - Light 1 (Primary)
   * - Light
     - 12
     - Light 2 (Primary)
   * - Light
     - 13
     - Light 3 (Primary)
   * - ⋮
     - ⋮
     - ⋮
   * - Light
     - 1n
     - Light N (Primary)
   * - Light
     - 21
     - Light 1 (Secondary)
   * - Light
     - 22
     - Light 2 (Secondary)
   * - Light
     - 23
     - Light 3 (Secondary)
   * - ⋮
     - ⋮
     - ⋮
   * - Light
     - 2n
     - Light N (Secondary)
   * - Light
     - 31
     - Light 1 (Tertiary)
   * - Light
     - 32
     - Light 2 (Tertiary)
   * - Light
     - 33
     - Light 3 (Tertiary)
   * - ⋮
     - ⋮
     - ⋮
   * - Light
     - 3n
     - Light N (Tertiary)
   * - Switch
     - 71
     - Enable 1 (Temporary)
   * - Switch
     - 72
     - Enable 2 (Temporary)
   * - Switch
     - 73
     - Enable 3 (Temporary)
   * - ⋮
     - ⋮
     - ⋮
   * - Switch
     - 7n
     - Enable N (Temporary)
   * - Switch
     - 81
     - Enable 1 (Persistent)
   * - Switch
     - 82
     - Enable 2 (Persistent)
   * - Switch
     - 83
     - Enable 3 (Persistent)
   * - ⋮
     - ⋮
     - ⋮
   * - Switch
     - 8n
     - Enable N (Persistent)

The sensor cluster endpoints are in the following order:

.. list-table::
   :widths: 20 10 70
   :header-rows: 1

   * - Type
     - Endpoint
     - Name
   * - Binaryinput
     - 11
     - Switch 1
   * - Binaryinput
     - 12
     - Switch 2
   * - Binaryinput
     - 13
     - Switch 3
   * - ⋮
     - ⋮
     - ⋮
   * - Binaryinput
     - 1n
     - Switch N
