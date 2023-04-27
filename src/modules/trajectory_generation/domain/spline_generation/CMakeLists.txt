file(GLOB spline_gen "*.cpp")

add_library(lib_spline_generation ${spline_gen})

target_link_libraries(lib_spline_generation lib_core_datastructures)

target_include_directories(lib_spline_generation PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../")
target_include_directories(lib_spline_generation PRIVATE "${CMAKE_SOURCE_DIR}/src/common")
