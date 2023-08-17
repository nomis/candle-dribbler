idf_component_register(
    SRCS
		device.cpp
		light.cpp
		log.cpp
		main.cpp
		thread.cpp
		ui.cpp
		zigbee.cpp

    REQUIRES
		app_update
		driver
		esp_timer
		freertos
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
