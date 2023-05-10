cmake_minimum_required(VERSION 3.5)
project(trajectory_generation_domain)

find_package(ament_cmake REQUIRED)
find_package(common_domain REQUIRED)

# set variable name (optional)
set(SPLINE_GENERATION_LIB spline_generation)

# Store all .cpp and .hpp files in a variable
file(GLOB cpp_files "${SPLINE_GENERATION_LIB}/*.cpp")
file(GLOB hpp_files "${SPLINE_GENERATION_LIB}/*.hpp")


# Add library
add_library(${SPLINE_GENERATION_LIB} ${cpp_files})

# Add all dependencies of this library
ament_target_dependencies(${SPLINE_GENERATION_LIB} common_domain)

# Export target and also its dependencies so that you don't have to call 
# find_package() for each of the dependencies
ament_export_targets(${SPLINE_GENERATION_LIB}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain)

# Specify the include directories to enable cmake to build and install
target_include_directories(${SPLINE_GENERATION_LIB} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SPLINE_GENERATION_LIB}>  
    $<INSTALL_INTERFACE:include/${SPLINE_GENERATION_LIB}>  # <prefix>/include/mylib
)


# Install all header files to the desired location
install(
        FILES ${hpp_files}
        DESTINATION include/${SPLINE_GENERATION_LIB}
)

# Install the library
install(TARGETS ${SPLINE_GENERATION_LIB}
        EXPORT ${SPLINE_GENERATION_LIB}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)

ament_package()