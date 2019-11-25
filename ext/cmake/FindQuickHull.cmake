add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/quickhull)
set(QuickHull_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/quickhull)
set(QuickHull_LIBRARY QuickHull)
include_directories(${QuickHull_INCLUDE_DIR})