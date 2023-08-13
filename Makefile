.PHONY: all target config build clean flash erase-ota app-flash monitor

all: build

target:
	idf.py set-target esp32c6

config:
	idf.py menuconfig

build:
	idf.py build

clean:
	idf.py clean

flash: build
	idf.py flash

erase-ota:
	idf.py erase-otadata

app-flash: build
	idf.py app-flash

monitor:
	idf.py monitor --timestamps --timestamp-format "%Y-%m-%d %H:%M:%S.%f" --no-reset
