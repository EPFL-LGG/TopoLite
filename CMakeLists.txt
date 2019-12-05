cmake_minimum_required(VERSION 3.9)
project(TopoLite)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 11)


#########################################
#####                               #####
#####           Option              #####
#####                               #####
#########################################

option(USE_PYTHON_PLUGIN "USE PYBIND11 PLUGIN" OFF)
option(USE_UNIT_TEST "USE CATCH2 TEST" ON)
option(USE_C_EXTENSION "USE_C_EXTENSION" ON)

#########################################
#####                               #####
#####      External Library         #####
#####                               #####
#########################################


#3rd party library
#set(TOPO_RUNTIME_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/Release)
set(TOPO_RUNTIME_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../TopoRhino/MacBin)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/ext/")

#########################################
#####                               #####
#####      Core Library             #####
#####                               #####
#########################################

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${TOPO_RUNTIME_OUTPUT})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TOPO_RUNTIME_OUTPUT})

file(GLOB TpCoreFiles   ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/Utility/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/Utility/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/Mesh/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/Mesh/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/IO/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/IO/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/CrossMesh/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/CrossMesh/*.h
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/Structure/*.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/Structure/*.h)

add_library(TpCorelib STATIC ${TpCoreFiles})

target_compile_definitions(TpCorelib PUBLIC ${ext_defs})

target_include_directories(TpCorelib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite)

target_link_libraries(TpCorelib PUBLIC ${ext_lib})

#########################################
#####                               #####
#####        Pybind11  Plugin       #####
#####                               #####
#########################################

if(USE_PYTHON_PLUGIN)
    file(GLOB TpPythonFiles
            ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/TopoLite/PyInterface/*.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/PyInterface/*.h)

    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/TopoLite/pybind11)
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
            src/TopoLite/Mesh/Polygon.cpp
            src/TopoLite/Mesh/PolyMesh.cpp
            src/TopoLite/Mesh/Cross.cpp
            src/TopoLite/IO/XMLIO.cpp)
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
            ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/ExternC/CSharpExtern.h
            ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/ExternC/CSharpExtern.cpp)
    target_link_libraries(dllTopo PUBLIC TpCorelib)
    target_include_directories(dllTopo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/)
endif()
