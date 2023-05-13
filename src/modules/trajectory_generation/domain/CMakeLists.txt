cmake_minimum_required(VERSION 3.5)
project(trajectory_generation_domain)

find_package(ament_cmake REQUIRED)
find_package(common_domain REQUIRED)
find_package(Eigen3 REQUIRED)

# set variable name (optional)
set(LIB_DIR spline_generation)

# Store all .cpp and .hpp files in a variable
file(GLOB cpp_files "${LIB_DIR}/*.cpp")
file(GLOB hpp_files "${LIB_DIR}/*.hpp")


# Add library
add_library(${LIB_DIR} ${cpp_files})

# Add all dependencies of this library
ament_target_dependencies(${LIB_DIR} common_domain)

# Export target and also its dependencies so that you don't have to call 
# find_package() for each of the dependencies
ament_export_targets(${LIB_DIR}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain)

# Specify the include directories to enable cmake to build and install
target_include_directories(${LIB_DIR} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${LIB_DIR}>  
    $<INSTALL_INTERFACE:include/${LIB_DIR}>  # <prefix>/include/mylib
)


# Install all header files to the desired location
install(
        FILES ${hpp_files}
        DESTINATION include/${LIB_DIR}
)

# Install the library
install(TARGETS ${LIB_DIR}
        EXPORT ${LIB_DIR}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)

ament_package()