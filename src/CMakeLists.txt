idf_component_register(
    SRCS
		debounce.cpp
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

execute_process(
	WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
	OUTPUT_VARIABLE git_commit_time
	COMMAND git log -n 1 --format=%ctUL
)

target_compile_options(
	${COMPONENT_LIB}
	PRIVATE
		-Wall
		-Wextra
		-Wshadow
		-Werror
		-DNUTT_GIT_COMMIT_TIME=${git_commit_time}+0UL
)
