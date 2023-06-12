cmake_minimum_required(VERSION 3.5)
project(common_domain)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

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


set(SENSOR_DATASTRUCTURES sensor_datastructures)

file(GLOB cpp_files "${SENSOR_DATASTRUCTURES}/*.cpp")
file(GLOB hpp_files "${SENSOR_DATASTRUCTURES}/*.hpp")

add_library(${SENSOR_DATASTRUCTURES} ${cpp_files})

ament_export_targets(${SENSOR_DATASTRUCTURES}Targets HAS_LIBRARY_TARGET)

target_include_directories(${SENSOR_DATASTRUCTURES} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SENSOR_DATASTRUCTURES}>  
    $<INSTALL_INTERFACE:include/${SENSOR_DATASTRUCTURES}>  # <prefix>/include/mylib
)

install(
        FILES ${hpp_files}
        DESTINATION include/${SENSOR_DATASTRUCTURES}
)

install(TARGETS ${SENSOR_DATASTRUCTURES}
        EXPORT ${SENSOR_DATASTRUCTURES}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)



set(CONVERTERS converters)

file(GLOB cpp_files "${CONVERTERS}/*.cpp")
file(GLOB hpp_files "${CONVERTERS}/*.hpp")

add_library(${CONVERTERS} ${cpp_files})

ament_target_dependencies(${CONVERTERS} Eigen3)

target_link_libraries(${CONVERTERS} ${MATH_UTILS} ${CORE_DATASTRUCTURES})

ament_export_targets(${CONVERTERS}Targets HAS_LIBRARY_TARGET)

target_include_directories(${CONVERTERS} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${CONVERTERS}>  
    $<INSTALL_INTERFACE:include/${CONVERTERS}>  # <prefix>/include/mylib
)

install(
        FILES ${hpp_files}
        DESTINATION include/${CONVERTERS}
)

install(TARGETS ${CONVERTERS}
        EXPORT ${CONVERTERS}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)

ament_package()

