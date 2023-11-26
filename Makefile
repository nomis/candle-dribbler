.PHONY: all target config build clean flash erase-ota app-flash monitor cppcheck pipenv

PIPENV=$(CURDIR)/pipenv
PYTHON=$(PIPENV)/.venv/bin/python

all: build

target:
	idf.py set-target esp32c6

config:
	idf.py menuconfig

build:
	idf.py build

clean:
	idf.py clean
	+$(MAKE) -C $(PIPENV) -L clean

flash: build
	idf.py flash

erase-ota:
	idf.py erase-otadata

app-flash: build
	idf.py app-flash

monitor:
	idf.py monitor --timestamps --timestamp-format "%Y-%m-%d %H:%M:%S.%f" --no-reset

cppcheck:
	cppcheck --enable=all --suppress=unusedFunction --suppress=useStlAlgorithm \
		--suppress=knownConditionTrueFalse --suppress=missingIncludeSystem \
		--suppress=internalAstError --inline-suppr -I src/ src/*.cpp

pipenv:
	+$(MAKE) -C $(PIPENV) -L

build/candle-dribbler.ota: build/candle-dribbler.bin build/config/sdkconfig.h bin/create-ota.py Makefile | pipenv
	rm -f $@~
	$(PYTHON) bin/create-ota.py \
		-m $(shell grep -F CONFIG_NUTT_OTA_MANUFACTURER_ID build/config/sdkconfig.h | cut -d ' ' -f 3) \
		-i $(shell grep -F CONFIG_NUTT_OTA_IMAGE_TYPE_ID build/config/sdkconfig.h | cut -d ' ' -f 3) \
		-v $(shell grep -F CONFIG_NUTT_OTA_FILE_VERSION build/config/sdkconfig.h | cut -d ' ' -f 3) \
		-- $< > $@~
	mv $@~ $@
