cmake_minimum_required(VERSION 3.9)
project(TopoLite)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)


#########################################
#####                               #####
#####           Option              #####
#####                               #####
#########################################

option(USE_GUI "USE_GUI" OFF)
option(USE_PYTHON_PLUGIN "USE PYBIND11 PLUGIN" OFF)
option(USE_UNIT_TEST "USE CATCH2 TEST" OFF)
option(USE_C_EXTENSION "USE_C_EXTENSION" ON)

#########################################
#####                               #####
#####      External Library         #####
#####                               #####
#########################################


#3rd party library
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/ext/")

#########################################
#####                               #####
#####      Core Library             #####
#####                               #####
#########################################

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Release)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Release)

file(GLOB TpCoreFiles   ${CMAKE_CURRENT_SOURCE_DIR}/src/Utility/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/Utility/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/Mesh/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/Mesh/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/IO/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/CrossMesh/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/CrossMesh/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/Structure/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/Structure/*.h)

add_library(TpCorelib STATIC ${TpCoreFiles})

target_compile_definitions(TpCorelib PUBLIC ${ext_defs})

target_include_directories(TpCorelib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/)

target_link_libraries(TpCorelib PUBLIC ${ext_lib})

#########################################
#####                               #####
#####               GUI             #####
#####                               #####
#########################################

IF(USE_GUI)
    add_executable(TopoGUI
            src/GUI/main.cpp
            src/GUI/NanoViewer.cpp
            src/GUI/NanoViewer.h)
    target_link_libraries(TopoGUI PUBLIC TpCorelib)
endif()


#########################################
#####                               #####
#####        Pybind11  Plugin       #####
#####                               #####
#########################################

if(USE_PYTHON_PLUGIN)
    file(GLOB TpPythonFiles
            ${CMAKE_CURRENT_SOURCE_DIR}/src/PyInterface/*.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/PyInterface/*.h)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/pybind11)
    pybind11_add_module(pyTopo ${TpPythonFiles})
    target_link_libraries(pyTopo PUBLIC TpCorelib)
endif()


##########################################
######                               #####
######        Catch2    Test         #####
######                               #####
##########################################

if(USE_UNIT_TEST)
    set(CMAKE_MODULE_PATH   ${CMAKE_MODULE_PATH}
                            ${CMAKE_CURRENT_SOURCE_DIR}/ext/Catch2/contrib)
    add_subdirectory(ext/Catch2)
    include(CTest)
    include(ParseAndAddCatchTests)
    add_executable(testTopo
            src/Mesh/Polygon.cpp
            src/Mesh/PolyMesh.cpp
            src/Mesh/Cross.cpp
            src/IO/XMLIO.cpp)
    target_link_libraries(testTopo PUBLIC Catch2::Catch2)
    target_link_libraries(testTopo PUBLIC TpCorelib)
    target_compile_definitions(testTopo PUBLIC -DCATCH2_UNITTEST)
    ParseAndAddCatchTests(testTopo)
endif()


##########################################
######                               #####
######        Extern C Plugin        #####
######                               #####
##########################################
if(USE_C_EXTENSION)
    add_library(dllTopo SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ExternC/CSharpExtern.h
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ExternC/CSharpExtern.cpp)
    target_link_libraries(dllTopo PUBLIC TpCorelib)
    target_include_directories(dllTopo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/)
endif()
