cmake_minimum_required(VERSION 3.9)
project(TopoLite)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)

#########################################
#####                               #####
#####           Option              #####
#####                               #####
#########################################

option(USE_OPENGL_DRAW "USE OPENGL DRAW" OFF)
option(USE_PYTHON_PLUGIN "USE PYBIND11 PLUGIN" OFF)
option(USE_UNIT_TEST "USE CATCH2 TEST" ON)
option(USE_C_EXTENSION "USE_C_EXTENSION" OFF)

if(APPLE)
    set(CMAKE_OSX_ARCHITECTURES "x86_64")
    if(CMAKE_GENERATOR STREQUAL Xcode)
        set(CMAKE_OSX_DEPLOYMENT_TARGET "10.8")
    endif()
endif()

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

target_compile_definitions(TpCorelib PUBLIC -DUSE_OPENGL_DRAW=0)
target_compile_options(TpCorelib PUBLIC "-Wno-deprecated-declarations" )

target_include_directories(TpCorelib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/)

target_link_libraries(TpCorelib PUBLIC
                                ${Boost_FILESYSTEM_LIBRARY}
                                ${Boost_SYSTEM_LIBRARY}
                                ${ext_lib})

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
    pybind11_add_module(pyTopoCreator ${TpPythonFiles})
    target_link_libraries(pyTopoCreator PUBLIC TpCorelib)
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
    add_executable(testTopoLockCreator
            src/Mesh/Polygon.cpp
            src/Mesh/PolyMesh.cpp
            src/Mesh/Cross.cpp)
    target_link_libraries(testTopoLockCreator PUBLIC Catch2::Catch2)
    target_link_libraries(testTopoLockCreator PUBLIC TpCorelib)
    target_compile_definitions(testTopoLockCreator PUBLIC -DCATCH2_UNITTEST)
    ParseAndAddCatchTests(testTopoLockCreator)
endif()


##########################################
######                               #####
######        Extern C Plugin        #####
######                               #####
##########################################
if(USE_C_EXTENSION)
    add_library(dllTopoLockCreator SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/src/PyInterface/PyInterface.h
            ${CMAKE_CURRENT_SOURCE_DIR}/src/PyInterface/PyInterface.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ExternC/CSharpExtern.h
            ${CMAKE_CURRENT_SOURCE_DIR}/src/ExternC/CSharpExtern.cpp)
    target_link_libraries(dllTopoLockCreator PUBLIC TpCorelib)
    target_include_directories(dllTopoLockCreator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/)
endif()
