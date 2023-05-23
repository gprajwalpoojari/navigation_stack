cmake_minimum_required(VERSION 3.5)
project(common_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(common_domain REQUIRED)

#Generate ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "interface_msgs/msg/Posture.msg"
  "interface_msgs/msg/Spline.msg"
)

# Export message so that other packages can find it
ament_export_dependencies(rosidl_default_runtime)

set(LIB_DIR converters)

file(GLOB cpp_files "${LIB_DIR}/*.cpp")
file(GLOB hpp_files "${LIB_DIR}/*.hpp")

add_library(${LIB_DIR} ${cpp_files})

# Add the generated messages as a dependency to this library
rosidl_target_interfaces(${LIB_DIR}
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(${LIB_DIR} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${LIB_DIR}>  
    $<INSTALL_INTERFACE:include/${LIB_DIR}>  # <prefix>/include/mylib
)

ament_export_targets(${LIB_DIR}Targets HAS_LIBRARY_TARGET)

ament_target_dependencies(${LIB_DIR} common_domain)


install(
        FILES ${hpp_files}
        DESTINATION include/${LIB_DIR}
)

install(TARGETS ${LIB_DIR}
        EXPORT ${LIB_DIR}Targets
        LIBRARY DESTINATION lib/
        INCLUDES DESTINATION include/
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
