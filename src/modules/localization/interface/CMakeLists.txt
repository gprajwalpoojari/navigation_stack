cmake_minimum_required(VERSION 3.5)
project(localization_ros2)

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
find_package(localization_domain REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(main src/main.cpp src/localization_publisher.cpp)
add_executable(publisher src/publisher.cpp)


add_executable(check_kf dev_tools/check_kf.cpp)
add_executable(check_ekf dev_tools/check_ekf.cpp)
ament_target_dependencies(check_kf localization_domain)
ament_target_dependencies(check_ekf localization_domain)

target_include_directories(main PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>  
    $<INSTALL_INTERFACE:include/>  # <prefix>/include/mylib
)

ament_target_dependencies(main rclcpp std_msgs nav_msgs localization_domain)
ament_target_dependencies(publisher rclcpp std_msgs)


install(TARGETS 
check_kf
DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dev_tools
)

install(TARGETS 
check_ekf
DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/dev_tools
)

install(TARGETS 
  main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES src/localization_publisher.hpp
  DESTINATION include/
)


install(TARGETS 
publisher
DESTINATION lib/${PROJECT_NAME}
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
