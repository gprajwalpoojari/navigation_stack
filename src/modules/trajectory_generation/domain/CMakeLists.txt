cmake_minimum_required(VERSION 3.5)
project(trajectory_generation_domain)

find_package(ament_cmake REQUIRED)
find_package(common_domain REQUIRED)
find_package(Eigen3 REQUIRED)

# set variable name (optional)
set(SPLINE_GENERATION spline_generation)
set(TRAJECTORY_GENERATION trajectory_generation)
set(GRAPH_GENERATION graph_generation)
set(COST_CALCULATION COST_CALCULATION)

# Store all .cpp and .hpp files in a variable
file(GLOB spline_cpp_files "${SPLINE_GENERATION}/*.cpp")
file(GLOB trajectory_cpp_files "${TRAJECTORY_GENERATION}/*.cpp")
file(GLOB graph_cpp_files "${GRAPH_GENERATION}/*.cpp")
file(GLOB cost_cpp_files "${COST_CALCULATION}/*.cpp")


file(GLOB spline_hpp_files "${SPLINE_GENERATION}/*.hpp")
file(GLOB trajectory_hpp_files "${TRAJECTORY_GENERATION}/*.hpp")
file(GLOB graph_hpp_files "${GRAPH_GENERATION}/*.hpp")
file(GLOB cost_hpp_files "${COST_CALCULATION}/*.hpp")

# Add library
add_library(${SPLINE_GENERATION} ${spline_cpp_files})
add_library(${TRAJECTORY_GENERATION} ${trajectory_cpp_files})
add_library(${GRAPH_GENERATION} ${graph_cpp_files})
add_library(${COST_CALCULATION} ${cost_cpp_files})

# Link Dependencies
target_link_libraries(${TRAJECTORY_GENERATION} ${SPLINE_GENERATION})
target_link_libraries(${GRAPH_GENERATION} ${TRAJECTORY_GENERATION})


# Add all dependencies of this library
ament_target_dependencies(${SPLINE_GENERATION} common_domain)
ament_target_dependencies(${TRAJECTORY_GENERATION} common_domain)
ament_target_dependencies(${GRAPH_GENERATION} common_domain)
ament_target_dependencies(${COST_CALCULATION} common_domain)

# Export target and also its dependencies so that you don't have to call 
# find_package() for each of the dependencies
ament_export_targets(${SPLINE_GENERATION}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain)
ament_export_targets(${TRAJECTORY_GENERATION}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain)
ament_export_targets(${GRAPH_GENERATION}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain)
ament_export_targets(${COST_CALCULATION}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(common_domain)

# Specify the include directories to enable cmake to build and install
target_include_directories(${SPLINE_GENERATION} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SPLINE_GENERATION}>  
    $<INSTALL_INTERFACE:include/${SPLINE_GENERATION}>  # <prefix>/include/mylib
)
target_include_directories(${TRAJECTORY_GENERATION} PUBLIC  
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${TRAJECTORY_GENERATION}>  
$<INSTALL_INTERFACE:include/${TRAJECTORY_GENERATION}>  # <prefix>/include/mylib
)
target_include_directories(${GRAPH_GENERATION} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${GRAPH_GENERATION}>  
    $<INSTALL_INTERFACE:include/${GRAPH_GENERATION}>  # <prefix>/include/mylib
)
target_include_directories(${COST_CALCULATION} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${COST_CALCULATION}>  
    $<INSTALL_INTERFACE:include/${COST_CALCULATION}>  # <prefix>/include/mylib
)

# Install all header files to the desired location
install(
        FILES ${spline_hpp_files}
        DESTINATION include/${SPLINE_GENERATION}
)
install(
        FILES ${trajectory_hpp_files}
        DESTINATION include/${TRAJECTORY_GENERATION}
)
install(
        FILES ${graph_hpp_files}
        DESTINATION include/${GRAPH_GENERATION}
)
install(
        FILES ${cost_hpp_files}
        DESTINATION include/${COST_CALCULATION}
)

# Install the library
install(TARGETS ${SPLINE_GENERATION}
        EXPORT ${SPLINE_GENERATION}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)
install(TARGETS ${TRAJECTORY_GENERATION}
        EXPORT ${TRAJECTORY_GENERATION}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)
install(TARGETS ${GRAPH_GENERATION}
        EXPORT ${GRAPH_GENERATION}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)
install(TARGETS ${COST_CALCULATION}
        EXPORT ${COST_CALCULATION}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)

ament_package()