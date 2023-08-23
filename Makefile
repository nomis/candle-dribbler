.PHONY: all target config build clean flash erase-ota app-flash monitor cppcheck

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

cppcheck:
	cppcheck --enable=all --suppress=unusedFunction --suppress=useStlAlgorithm \
		--suppress=knownConditionTrueFalse --suppress=missingIncludeSystem \
		--suppress=internalAstError --inline-suppr -I src/ src/*.cpp
