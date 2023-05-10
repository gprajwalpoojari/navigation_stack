cmake_minimum_required(VERSION 3.5)
project(common_domain)

find_package(ament_cmake REQUIRED)

set(LIB_DIR core_datastructures)

file(GLOB cpp_files "${LIB_DIR}/*.cpp")
file(GLOB hpp_files "${LIB_DIR}/*.hpp")


add_library(${LIB_DIR} ${cpp_files})

ament_export_targets(${LIB_DIR}Targets HAS_LIBRARY_TARGET)

target_include_directories(${LIB_DIR} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${LIB_DIR}>  
    $<INSTALL_INTERFACE:include/${LIB_DIR}>  # <prefix>/include/mylib
)



install(
        FILES ${hpp_files}
        DESTINATION include/${LIB_DIR}
)

install(TARGETS ${LIB_DIR}
        EXPORT ${LIB_DIR}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)


ament_package()

