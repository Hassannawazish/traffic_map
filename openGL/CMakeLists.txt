cmake_minimum_required (VERSION 3.2)
project(graphic)

add_subdirectory(glfw)
add_subdirectory(glew)


add_executable(graphic main.cpp)

target_include_directories(graphic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/glfw/include)
target_include_directories(graphic PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/glew/include) 