cmake_minimum_required(VERSION 3.5)
project(common_domain)

find_package(ament_cmake REQUIRED)

set(CORE_DATASTRUCTURES core_datastructures)

file(GLOB cpp_files "${CORE_DATASTRUCTURES}/*.cpp")
file(GLOB hpp_files "${CORE_DATASTRUCTURES}/*.hpp")


add_library(${CORE_DATASTRUCTURES} ${cpp_files})

ament_export_targets(${CORE_DATASTRUCTURES}Targets HAS_LIBRARY_TARGET)

target_include_directories(${CORE_DATASTRUCTURES} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CORE_DATASTRUCTURES}>  
    $<INSTALL_INTERFACE:include/${CORE_DATASTRUCTURES}>  # <prefix>/include/mylib
)



install(
        FILES ${hpp_files}
        DESTINATION include/${CORE_DATASTRUCTURES}
)

install(TARGETS ${CORE_DATASTRUCTURES}
        EXPORT ${CORE_DATASTRUCTURES}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)


set(MATH_UTILS math_utils)

file(GLOB cpp_files "${MATH_UTILS}/*.cpp")
file(GLOB hpp_files "${MATH_UTILS}/*.hpp")
add_library(${MATH_UTILS} ${cpp_files})
target_link_libraries(${MATH_UTILS} ${CORE_DATASTRUCTURES})

ament_export_targets(${MATH_UTILS}Targets HAS_LIBRARY_TARGET)

target_include_directories(${MATH_UTILS} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${MATH_UTILS}>  
    $<INSTALL_INTERFACE:include/${MATH_UTILS}>  # <prefix>/include/mylib
)



install(
        FILES ${hpp_files}
        DESTINATION include/${MATH_UTILS}
)

install(TARGETS ${MATH_UTILS}
        EXPORT ${MATH_UTILS}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)


ament_package()

