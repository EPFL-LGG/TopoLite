cmake_minimum_required(VERSION 3.9)
project(quickhull)
set(CMAKE_CXX_STANDARD 11)

file(GLOB SOURCES *.cpp)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/Structs)