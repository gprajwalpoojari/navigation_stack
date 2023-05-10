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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_path(SPLINE_GENERATION_INCLUDE_DIRS spline_generation)
find_path(CORE_DATASTRUCTURES_INCLUDE_DIRS core_datastructures)

add_executable(talker src/trajectory_publisher.cpp )
# target_link_libraries(talker core_datastructures spline_generation)
# target_link_libraries(${PROJECT_NAME} PUBLIC core_datastructures Eigen3::Eigen)
# target_link_directories(talker PRIVATE SPLINE_GENERATION_INCLUDE_DIRS CORE_DATASTRUCTURES_INCLUDE_DIRS)
# target_link_libraries(talker spline_generation core_datastructures)

ament_target_dependencies(talker rclcpp common_ros2 std_msgs trajectory_generation_domain)


install(TARGETS 
  talker
  DESTINATION lib/${PROJECT_NAME}
)
target_include_directories(
    # ${PROJECT_NAME} PUBLIC 
    talker PUBLIC

    # ${CMAKE_CURRENT_SOURCE_DIR}/../ 
    ${SPLINE_GENERATION_INCLUDE_DIRS}
    ${CORE_DATASTRUCTURES_INCLUDE_DIRS}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
