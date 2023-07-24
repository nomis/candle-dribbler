FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.cpp)
idf_component_register(
    SRCS
		device.cpp
		light.cpp
		main.cpp
		zigbee.cpp

    REQUIRES
		driver
		nvs_flash
)
