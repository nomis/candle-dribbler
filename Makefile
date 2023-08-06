.PHONY: all target config build clean flash app-flash monitor

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

app-flash: build
	idf.py app-flash

monitor:
	idf.py monitor --timestamps --no-reset
