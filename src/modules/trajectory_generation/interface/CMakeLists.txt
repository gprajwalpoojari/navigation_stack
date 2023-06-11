cmake_minimum_required(VERSION 3.5)
project(trajectory_generation_ros2)

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
find_package(rclcpp REQUIRED)
find_package(common_ros2 REQUIRED)
find_package(common_domain REQUIRED)
find_package(trajectory_generation_domain REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(main src/main.cpp src/trajectory_publisher.cpp src/rviz_visualization.cpp)

# Specify where the files to build are and where the include directory is
target_include_directories(main PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  
    $<INSTALL_INTERFACE:include/>  # <prefix>/include/mylib
)
ament_target_dependencies(main 
                          rclcpp 
                          common_ros2 
                          std_msgs 
                          trajectory_generation_domain 
                          visualization_msgs)


install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}
)
install(
    FILES src/trajectory_publisher.hpp
          src/rviz_visualization.hpp
    DESTINATION include/
)

install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
