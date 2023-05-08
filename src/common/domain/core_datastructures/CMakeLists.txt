cmake_minimum_required(VERSION 3.5)
project(core_datastructures)

file(GLOB core_datastructures_cpp_files "*.cpp")
file(GLOB core_datastructures_hpp_files "*.hpp")

add_library(${PROJECT_NAME} ${core_datastructures_cpp_files})

install(
  FILES ${core_datastructures_hpp_files} 
  DESTINATION include/${PROJECT_NAME}
  )

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )
    
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)