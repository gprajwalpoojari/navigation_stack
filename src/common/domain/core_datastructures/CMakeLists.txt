
file(GLOB cpp_files "*.cpp")

add_library(lib_core_datastructures ${cpp_files})

target_link_libraries(lib_core_datastructures ${PROJECT_NAME})
target_include_directories(lib_core_datastructures PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")