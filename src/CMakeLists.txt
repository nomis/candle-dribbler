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

target_compile_options(
	${COMPONENT_LIB}
	PRIVATE
		-Wall
		-Wextra
		-Wshadow
		-Werror
)
