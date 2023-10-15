#!/bin/bash
if [ -z "$2" ] || [ -z "$3" ] || [ -z "$4" ]; then
	echo "Usage: $0 <old suffix|-> <new id> <new name> <light> [light] [light] [light]"
	echo
	echo "Example: $0 uuid_uk_candle_dribbler candle_dribbler_1 \"Candle Dribbler 1\" A B C D"
	exit 1
fi
OLD="$1"
NEW="$2"
NAME="$3"
[ "$OLD" = "-" ] && OLD=""
shift 3
LIGHTS=("$@")

function generate_file() {
	echo "{"

	light=1
	switch=1
	binary_sensor=1

	i=0
	while [ $i -lt ${#LIGHTS[@]} ]; do
		n=$(($i + 1))

		id="_${light}"
		[ $light -eq 1 ] && id=""
		old="light.${OLD}_light${id}"
		new="light.${NEW}_light_${n}p"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Light ${LIGHTS[$i]} (Primary)\"],"
		light=$(($light + 1))

		id="_${switch}"
		[ $switch -eq 1 ] && id=""
		old="switch.${OLD}_switch${id}"
		new="switch.${NEW}_enable_${n}t"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Enable ${LIGHTS[$i]} (Temporary)\"],"
		switch=$(($switch + 1))

		id="_${binary_sensor}"
		[ $binary_sensor -eq 1 ] && id=""
		old="binary_sensor.${OLD}_binaryinput${id}"
		new="binary_sensor.${NEW}_switch_${n}"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Switch ${LIGHTS[$i]}\"],"
		binary_sensor=$(($binary_sensor + 1))

		i=$n
	done

	i=0
	while [ $i -lt ${#LIGHTS[@]} ]; do
		n=$(($i + 1))

		old="light.${OLD}_light_${light}"
		new="light.${NEW}_light_${n}s"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Light ${LIGHTS[$i]} (Secondary)\"],"
		light=$(($light + 1))

		old="switch.${OLD}_switch_${switch}"
		new="switch.${NEW}_enable_${n}p"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Enable ${LIGHTS[$i]} (Persistent)\"],"
		switch=$(($switch + 1))

		i=$n
	done

	i=0
	while [ $i -lt ${#LIGHTS[@]} ]; do
		n=$(($i + 1))

		old="light.${OLD}_light_${light}"
		new="light.${NEW}_light_${n}t"
		[ -n "$OLD" ] || old="$new"
		echo "  \"${old}\": [\"${new}\", \"${NAME} Light ${LIGHTS[$i]} (Tertiary)\"],"
		light=$(($light + 1))

		i=$n
	done

	old="button.${OLD}_identify"
	new="button.${NEW}_identify"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Identify\"],"

	old="sensor.${OLD}_analoginput"
	new="sensor.${NEW}_uptime"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Uptime\"],"

	old="sensor.${OLD}_analoginput_2"
	new="sensor.${NEW}_connected"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Connected\"],"

	old="sensor.${OLD}_analoginput_3"
	new="sensor.${NEW}_uplink"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} Uplink\"],"

	old="sensor.${OLD}_analoginput_4"
	new="sensor.${NEW}_uplink_rssi"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} RSSI\"],"

	old="sensor.${OLD}_lqi"
	new="sensor.${NEW}_lqi"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} LQI\"],"

	old="sensor.${OLD}_rssi"
	new="sensor.${NEW}_rssi"
	[ -n "$OLD" ] || old="$new"
	echo "  \"${old}\": [\"${new}\", \"${NAME} RSSI\"]"

	echo "}"
}

pipenv run ./homeassistant-entity-renamer.py --file <(generate_file)
