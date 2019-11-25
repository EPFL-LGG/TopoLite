add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/clipper)
set(Clipper_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/clipper)
set(Clipper_LIBRARY Clipper)
include_directories(${Clipper_INCLUDE_DIR})