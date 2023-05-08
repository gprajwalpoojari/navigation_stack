cmake_minimum_required(VERSION 3.5)
project(core_datastructures)

file(GLOB cpp_files "*.cpp")
file(GLOB hpp_files "*.hpp")

add_library(${PROJECT_NAME} ${cpp_files})

install(
  FILES ${hpp_files} DESTINATION include/${PROJECT_NAME}
  )

install(
  TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME}
  )
    
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../)