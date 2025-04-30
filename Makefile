.PHONY: app target config clean flash erase-ota app-flash monitor cppcheck pipenv

PIPENV=$(CURDIR)/pipenv
PYTHON=$(PIPENV)/.venv/bin/python

app: | build
	idf.py build

build:
	-btrfs subvolume create build
	mkdir -p build

target: | build
	idf.py set-target esp32c6

config: | build
	idf.py menuconfig

clean: | build
	idf.py clean
	+$(MAKE) -C $(PIPENV) -L clean

flash: app
	idf.py flash

erase-ota: | build
	idf.py erase-otadata

app-flash: app
	idf.py app-flash

monitor: | build
	idf.py monitor --timestamps --timestamp-format "%Y-%m-%d %H:%M:%S.%f" --no-reset

cppcheck:
	cppcheck --enable=all --suppress=unusedFunction --suppress=useStlAlgorithm \
		--suppress=knownConditionTrueFalse --suppress=missingIncludeSystem \
		--suppress=internalAstError --inline-suppr -I src/ src/*.cpp

pipenv:
	+$(MAKE) -C $(PIPENV) -L

build/candle-dribbler.ota: build/candle-dribbler.bin build/config/sdkconfig.h bin/create-ota.py Makefile | build pipenv
	rm -f $@~
	$(PYTHON) bin/create-ota.py \
		-m $(shell grep -F CONFIG_NUTT_OTA_MANUFACTURER_ID build/config/sdkconfig.h | cut -d ' ' -f 3) \
		-i $(shell grep -F CONFIG_NUTT_OTA_IMAGE_TYPE_ID build/config/sdkconfig.h | cut -d ' ' -f 3) \
		-v $(shell grep -F CONFIG_NUTT_OTA_FILE_VERSION build/config/sdkconfig.h | cut -d ' ' -f 3) \
		-- $< $@~
	mv $@~ $@
