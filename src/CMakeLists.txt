idf_component_register(
    SRCS
		base64.cpp
		debounce.cpp
		device.cpp
		light.cpp
		log.cpp
		main.cpp
		ota.cpp
		thread.cpp
		ui.cpp
		util.cpp
		zigbee.cpp

    REQUIRES
		app_update
		driver
		espcoredump
		esp_timer
		freertos
		ieee802154
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
		-DZLIB_CONST
		-DNUTT_GIT_COMMIT_TIME=${git_commit_time}+0UL
)

target_link_libraries(
	${COMPONENT_LIB}
	INTERFACE
	-Wl,--undefined=__wrap_zb_zcl_send_report_attr_command
	-Wl,--undefined=__wrap_zb_abort
	-Wl,--undefined=__wrap_zb_assert
	-Wl,--wrap=zb_abort
	-Wl,--wrap=zb_assert
	-Wl,--wrap=zb_zcl_send_report_attr_command
)
