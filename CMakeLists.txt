cmake_minimum_required(VERSION 3.16.3)

project(navigation_stack)

set(CMAKE_MODULE_PATH ${navigation_stack_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

include(MacroOutOfSourceBuild)

macro_ensure_out_of_source_build("${PROJECT_NAME} requires an out of source build.")

add_subdirectory(src/common)
add_subdirectory(src/modules)
