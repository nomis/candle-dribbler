.PHONY: all conf build clean flash app-flash monitor

all: build

conf:
	idf.py set-target esp32c6

build:
	idf.py build

clean:
	idf.py clean

flash: build
	idf.py flash

app-flash: build
	idf.py app-flash

monitor:
	idf.py monitor --timestamps
