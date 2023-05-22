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
find_path(SPLINE_GENERATION_INCLUDE_DIRS spline_generation)
find_path(CORE_DATASTRUCTURES_INCLUDE_DIRS core_datastructures)

add_executable(talker src/trajectory_publisher.cpp )
ament_target_dependencies(talker rclcpp common_ros2 std_msgs trajectory_generation_domain)


install(TARGETS 
  talker
  DESTINATION lib/${PROJECT_NAME}
)
target_include_directories(
    talker PUBLIC
    ${SPLINE_GENERATION_INCLUDE_DIRS}
    ${CORE_DATASTRUCTURES_INCLUDE_DIRS}
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
