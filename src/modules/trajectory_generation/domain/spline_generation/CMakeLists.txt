cmake_minimum_required(VERSION 3.5)
project(spline_generation)

file(GLOB cpp_files "*.cpp")
file(GLOB hpp_files "*.hpp")

add_library(${PROJECT_NAME} ${cpp_files})

target_link_libraries(${PROJECT_NAME} PUBLIC core_datastructures)

# Find the path to the include directories of the core_datastructures library
find_path(CORE_DATASTRUCTURES_INCLUDE_DIRS core_datastructures)

# Specify destination for all header files of this library
install(FILES ${hpp_files} DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})


target_include_directories(
    ${PROJECT_NAME} PUBLIC 

    ${CMAKE_CURRENT_SOURCE_DIR}/../ 
    ${CORE_DATASTRUCTURES_INCLUDE_DIRS}
)
