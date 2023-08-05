idf_component_register(
    SRCS
		device.cpp
		light.cpp
		main.cpp
		zigbee.cpp

    REQUIRES
		app_update
		driver
		nvs_flash
		pthread
)

target_compile_options(
	${COMPONENT_LIB}
	PRIVATE
		-Wall
		-Wextra
		-Wshadow
		-Werror
)
