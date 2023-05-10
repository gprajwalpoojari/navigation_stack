cmake_minimum_required(VERSION 3.5)
project(common_domain)

find_package(ament_cmake REQUIRED)

set(CORE_DATASTRUCTURES_LIB core_datastructures)

file(GLOB cpp_files "${CORE_DATASTRUCTURES_LIB}/*.cpp")
file(GLOB hpp_files "${CORE_DATASTRUCTURES_LIB}/*.hpp")


add_library(${CORE_DATASTRUCTURES_LIB} ${cpp_files})

ament_export_targets(${CORE_DATASTRUCTURES_LIB}Targets HAS_LIBRARY_TARGET)

target_include_directories(${CORE_DATASTRUCTURES_LIB} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CORE_DATASTRUCTURES_LIB}>  
    $<INSTALL_INTERFACE:include/${CORE_DATASTRUCTURES_LIB}>  # <prefix>/include/mylib
)



install(
        FILES ${hpp_files}
        DESTINATION include/${CORE_DATASTRUCTURES_LIB}
)

install(TARGETS ${CORE_DATASTRUCTURES_LIB}
        EXPORT ${CORE_DATASTRUCTURES_LIB}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)


ament_package()

