cmake_minimum_required(VERSION 3.5)
project(trajectory_generation_domain)

find_package(ament_cmake REQUIRED)
find_package(common_domain REQUIRED)

set(SPLINE_GENERATION_LIB spline_generation)

file(GLOB cpp_files "${SPLINE_GENERATION_LIB}/*.cpp")
file(GLOB hpp_files "${SPLINE_GENERATION_LIB}/*.hpp")


add_library(${SPLINE_GENERATION_LIB} ${cpp_files})

ament_target_dependencies(${SPLINE_GENERATION_LIB} common_domain)

ament_export_targets(${SPLINE_GENERATION_LIB}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain)

target_include_directories(${SPLINE_GENERATION_LIB} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SPLINE_GENERATION_LIB}>  
    $<INSTALL_INTERFACE:include/${SPLINE_GENERATION_LIB}>  # <prefix>/include/mylib
)



install(
        FILES ${hpp_files}
        DESTINATION include/${SPLINE_GENERATION_LIB}
)

install(TARGETS ${SPLINE_GENERATION_LIB}
        EXPORT ${SPLINE_GENERATION_LIB}Targets
        LIBRARY DESTINATION lib/${SPLINE_GENERATION_LIB}
        INCLUDES DESTINATION include/${SPLINE_GENERATION_LIB}
)

ament_package()