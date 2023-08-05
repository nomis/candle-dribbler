#!/usr/bin/env python3
# esp32-app-set-desc - Update ESP32 app descriptor
# Copyright 2022  Simon Arlott

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# PlatformIO usage:
#
# [env:...]
# extra_scripts = post:esp32-app-set-desc.py
# custom_app_name = Hello World
# custom_app_version = 42
#
# [env:...]
# extra_scripts = post:esp32-app-set-desc.py
# custom_app_name = Hello World
# custom_app_version = !git describe --dirty=+ --always

from datetime import datetime, timezone
import argparse
import hashlib
import os
import struct
import subprocess
import sys

def rewrite_app_desc(fw_elf, fw_bin, timestamp=datetime.now(tz=timezone.utc), name=None, version=None):
	with open(fw_elf, "rb") as f:
		hash_elf = hashlib.sha256(f.read()).digest()

	# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/app_image_format.html
	sizeof_esp_image_header_t = 24
	sizeof_esp_image_segment_header_t = 8
	sizeof_esp_app_desc_t = 256
	header_len = sizeof_esp_image_header_t + sizeof_esp_image_segment_header_t + sizeof_esp_app_desc_t
	image_header_fmt = "<4B L B 3B H B 8B B"
	image_segment_header_off = sizeof_esp_image_header_t
	image_segment_header_fmt = "<2L"
	app_desc_fmt = "<2L 2L 32s 32s 16s 16s 32s 32s 20L"

	segments = 0
	data = b""
	with open(fw_bin, "rb") as f:
		data = f.read(sizeof_esp_image_header_t)
		assert len(data) == sizeof_esp_image_header_t

		image_header = list(struct.unpack(image_header_fmt, data))
		assert image_header[0] == 0xE9, f"ESP_IMAGE_HEADER_MAGIC {image_header[0]:02X}"

		segments = image_header[1]
		hash_appended = image_header[19]
		if hash_appended == 0:
			hash_len = 0
		elif hash_appended == 1:
			hash_len = 32
		else:
			raise ValueError(f"Unsupported hash type {hash_appended}")
		checksum_act = 0xEF

		segment_offs = []
		segment_lens = []
		for i in range(0, segments):
			segment_offs.append(len(data))
			data += f.read(sizeof_esp_image_segment_header_t)
			assert len(data) == segment_offs[i] + sizeof_esp_image_segment_header_t
			image_segment_header = list(struct.unpack(image_segment_header_fmt, data[segment_offs[i]:segment_offs[i] + sizeof_esp_image_segment_header_t]))

			segment_lens.append(image_segment_header[1])
			segment = f.read(segment_lens[i])
			for b in segment:
				checksum_act ^= b
			assert len(segment) == segment_lens[i]
			data += segment

		remaining_len = (16 - (len(data) % 16)) + hash_len
		remaining = f.read(remaining_len)
		assert len(remaining) == remaining_len
		data += remaining
		assert len(f.read()) == 0, "unexpected/unsupported trailing data in file"

		checksum_exp = data[-hash_len - 1]
		checksum_act = checksum_act

		assert checksum_act == checksum_exp, f"checksum mismatch: {checksum_act:02X} {checksum_exp:02X}"

		if hash_appended == 1:
			hash_image_exp = data[-hash_len:].hex()
			hash_image_act = hashlib.sha256(data[:-hash_len]).hexdigest()

			assert hash_image_act == hash_image_exp, f"hash mismatch: {hash_image_act} {hash_image_exp}"

	assert(segments > 0), "Image has no segments"

	orig_app_desc = data[segment_offs[0] + sizeof_esp_image_segment_header_t:segment_offs[0] + sizeof_esp_image_segment_header_t + sizeof_esp_app_desc_t]
	app_desc = list(struct.unpack(app_desc_fmt, orig_app_desc))
	app_desc[0] == 0xABCD5432, f"ESP_APP_DESC_MAGIC_WORD {app_desc[0]:04X}"
	if version is not None:
		app_desc[4] = _utf8_truncate(version, 31)
	if name is not None:
		app_desc[5] = _utf8_truncate(name, 31)
	app_desc[6] = _utf8_truncate(_time_format(timestamp), 15)
	app_desc[7] = _utf8_truncate(_date_format(timestamp), 15)
	app_desc[9] = hash_elf[0:32]
	new_app_desc = struct.pack(app_desc_fmt, *app_desc)

	new_checksum = checksum_act
	for b in orig_app_desc:
		new_checksum ^= b
	for b in new_app_desc:
		new_checksum ^= b

	new_data = data[0:segment_offs[0] + sizeof_esp_image_segment_header_t] + new_app_desc + data[segment_offs[0] + sizeof_esp_image_segment_header_t + sizeof_esp_app_desc_t:-hash_len - 1]
	new_data += bytes([new_checksum])
	if hash_appended == 1:
		new_data += hashlib.sha256(new_data).digest()

	with open(f"{fw_bin}~", "wb") as f:
		f.write(new_data)
	os.rename(f"{fw_bin}~", fw_bin)

def after_fw_bin(source, target, env):
	fw_elf = str(source[0])
	fw_bin = str(target[0])

	desc = {
		"timestamp": datetime.fromtimestamp(float(env["UNIX_TIME"]), tz=timezone.utc),
		"name": env.GetProjectOption("custom_app_name", None),
		"version": env.GetProjectOption("custom_app_version", None),
	}

	if desc["version"] and desc["version"].startswith("!"):
		proc = subprocess.run(desc["version"][1:], shell=True, check=True, universal_newlines=True, stdout=subprocess.PIPE).stdout
		desc["version"] = proc.strip()

	str_desc = desc.copy()
	str_desc["date"] = _date_format(desc["timestamp"])
	str_desc["time"] = _time_format(desc["timestamp"])
	del str_desc["timestamp"]

	print(f"Update ESP32 app descriptor in {fw_bin}: {str_desc}")
	rewrite_app_desc(fw_elf, fw_bin, **desc)

def _utf8_truncate(text, max_len):
	text = text[0:max_len]
	data = text.encode("utf-8")
	while len(data) > max_len:
		text = text[:-1]
		data = text.encode("utf-8")
	return data

def _date_format(dt):
	return str(dt.date())

def _time_format(dt):
	dt = dt.replace(microsecond=0)
	if dt.tzinfo:
		return dt.strftime("%H:%M:%S %z")
	else:
		return dt.strftime("%H:%M:%S")

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description="Update ESP32 app descriptor")
	parser.add_argument("fw_elf", metavar="ELF", type=str, help="Firmware ELF filename")
	parser.add_argument("fw_bin", metavar="IMAGE", type=str, help="Firmware image filename")
	parser.add_argument("-n", "--name", metavar="NAME", type=str, help="Application name")
	parser.add_argument("-v", "--version", metavar="VERSION", type=str, help="Application version")

	args = parser.parse_args()
	rewrite_app_desc(**vars(args))
elif __name__ == "SCons.Script":
	Import("env")

	env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_fw_bin)

