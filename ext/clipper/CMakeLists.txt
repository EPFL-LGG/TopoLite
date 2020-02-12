cmake_minimum_required(VERSION 3.9)
project(Clipper)
set(CMAKE_CXX_STANDARD 11)

file(GLOB SOURCES *.cpp)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})
