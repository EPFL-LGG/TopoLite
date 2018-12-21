cmake_minimum_required(VERSION 3.9)

#Add Common File
file(GLOB COMMON_SRC ../Common/PolyMesh/*.cpp
                     ../Common/*.cpp)
add_executable(gui viewer.cpp main.cpp ${COMMON_SRC})

#Add NanoGui
target_link_libraries(gui nanogui ${NANOGUI_EXTRA_LIBS})
target_link_libraries(gui ${OPENMESH_LIBRARIES})