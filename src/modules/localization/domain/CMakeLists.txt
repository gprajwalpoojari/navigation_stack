cmake_minimum_required(VERSION 3.5)
project(localization_domain)

find_package(ament_cmake REQUIRED)
find_package(common_domain REQUIRED)
find_package(Eigen3 REQUIRED)

# set variable name (optional)
set(KALMAN_FILTER kalman_filter)
set(EXTENDED_KALMAN_FILTER extended_kalman_filter)

# # Store all .cpp and .hpp files in a variable
file(GLOB kalman_filter_cpp_files "${KALMAN_FILTER}/*.cpp")
file(GLOB kalman_filter_hpp_files "${KALMAN_FILTER}/*.hpp")

file(GLOB extended_kalman_filter_cpp_files "${EXTENDED_KALMAN_FILTER}/*.cpp")
file(GLOB extended_kalman_filter_hpp_files "${EXTENDED_KALMAN_FILTER}/*.hpp")

# # Add library
add_library(${KALMAN_FILTER} ${kalman_filter_cpp_files})
add_library(${EXTENDED_KALMAN_FILTER} ${extended_kalman_filter_cpp_files})

# # Add all dependencies of this library
ament_target_dependencies(${KALMAN_FILTER} Eigen3)
ament_target_dependencies(${EXTENDED_KALMAN_FILTER} Eigen3)

# # Export target and also its dependencies so that you don't have to call 
# # find_package() for each of the dependencies
ament_export_targets(${KALMAN_FILTER}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain Eigen3)

ament_export_targets(${EXTENDED_KALMAN_FILTER}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain Eigen3)

# # Specify the include directories to enable cmake to build and install
target_include_directories(${KALMAN_FILTER} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${KALMAN_FILTER}>  
    $<INSTALL_INTERFACE:include/${KALMAN_FILTER}>  # <prefix>/include/mylib
)

target_include_directories(${EXTENDED_KALMAN_FILTER} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${EXTENDED_KALMAN_FILTER}>  
    $<INSTALL_INTERFACE:include/${EXTENDED_KALMAN_FILTER}>  # <prefix>/include/mylib
)

# # Install all header files to the desired location
install(
        FILES ${kalman_filter_hpp_files}
        DESTINATION include/${KALMAN_FILTER}
)

install(
        FILES ${extended_kalman_filter_hpp_files}
        DESTINATION include/${EXTENDED_KALMAN_FILTER}
)

# # Install the library
install(TARGETS ${KALMAN_FILTER}
        EXPORT ${KALMAN_FILTER}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)

install(TARGETS ${EXTENDED_KALMAN_FILTER}
        EXPORT ${EXTENDED_KALMAN_FILTER}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)

ament_package()