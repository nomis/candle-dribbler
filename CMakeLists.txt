cmake_minimum_required(VERSION 3.16.0)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS src)
idf_build_set_property(DEPENDENCIES_LOCK dependencies.lock.${IDF_TARGET})
project(candle-dribbler)

idf_build_get_property(build_dir BUILD_DIR)
idf_build_get_property(elf_name EXECUTABLE_NAME GENERATOR_EXPRESSION)
set(app_bin "${build_dir}/${elf_name}.bin")
set(app_elf "${build_dir}/${elf_name}.elf")

execute_process(
	WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
	OUTPUT_STRIP_TRAILING_WHITESPACE
	ERROR_QUIET
	OUTPUT_VARIABLE git_version
	COMMAND git describe --dirty=+ --always
)

add_custom_command(
	TARGET app
	POST_BUILD
	COMMAND "${PROJECT_SOURCE_DIR}/bin/esp32-app-set-desc.py" -v "${git_version}" "${app_elf}" "${app_bin}"
)
