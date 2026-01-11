cmake_minimum_required(VERSION 3.10)
project(dio)

add_executable(dio dio_pc.c)
target_link_libraries(dio ${CMAKE_SOURCE_DIR}/ecb_306.a)