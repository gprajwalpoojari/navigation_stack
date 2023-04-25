
file(GLOB cpp_files "*.cpp")

add_library(core_datastructures ${cpp_files})


target_include_directories(core_datastructures PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")