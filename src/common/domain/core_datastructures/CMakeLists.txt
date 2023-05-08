cmake_minimum_required(VERSION 3.5)
project(lib_core_datastructures)
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")

file(GLOB core_datastructures_cpp_files "*.cpp")

add_library(${PROJECT_NAME} ${core_datastructures_cpp_files})

target_link_libraries(${PROJECT_NAME} common_domain)
# target_include_directories(lib_core_datastructures PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")