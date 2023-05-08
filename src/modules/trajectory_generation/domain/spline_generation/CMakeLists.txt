cmake_minimum_required(VERSION 3.5)
project(lib_spline_generation)

file(GLOB spline_gen_cpp_files "*.cpp")
file(GLOB spline_gen_hpp_files "*.hpp")

add_library(${PROJECT_NAME} ${spline_gen_cpp_files})


find_library(core_datastructures REQUIRED)
find_path(core_datastructures_include_path lib_core_datastructures)
message("core_datastructures_include_path = ${core_datastructures_include_path}")

# target_link_libraries(${PROJECT_NAME} PUBLIC lib_core_datastructures)
install(
  FILES ${spline_gen_hpp_files} 
  DESTINATION include/${PROJECT_NAME}
  )

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

target_include_directories(
    ${PROJECT_NAME} PUBLIC
    
    ${CMAKE_CURRENT_SOURCE_DIR}/../
    ${core_datastructures_include_path}/
)
