cmake_minimum_required(VERSION 3.11)
project(TopoLite)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-DUNITTEST_DATAPATH="${CMAKE_CURRENT_SOURCE_DIR}/data")

#########################################
#####                               #####
#####      Compiler flags           #####
#####                               #####
#########################################

option(HAVE_NANOGUI "Compile the gui" ON)
option(HAVE_TEST "Compile the united test" ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")

# Handles GCC and CLANG - Note that GCC on macOS is actually CLANG

if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    message("Compiling ${CMAKE_CXX_COMPILER_ID}")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -stdlib=libc++")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -stdlib=libc++")
elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    message("Compiling with ${CMAKE_CXX_COMPILER_ID} GCC")
    set(CMAKE_CXX_FLAGS_DEBUG "-g")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
endif()


#3rd party library
#if(USE_UNIT_TEST OR USE_PYTHON_PLUGIN OR USE_C_EXTENSION)
#    set(TOPO_RUNTIME_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/Release)
#	if(MSVC)
#		set(TOPO_RUNTIME_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../TopoRhino/WinBin)
#	else()
#		set(TOPO_RUNTIME_OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/../TopoRhino/MacBin)
#	endif()
#endif()

#########################################
#####                               #####
#####      Dependencies             #####
#####                               #####
#########################################

# Download and define targets for third-party dependencies

# Catch2 and TBB are handled in the following CMAKE module

include(TopoliteDependencies)

# The rest of dep. are located here:

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/ext/")

#########################################
#####                               #####
#####      Core Library             #####
#####                               #####
#########################################

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${TOPO_RUNTIME_OUTPUT})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TOPO_RUNTIME_OUTPUT})

file(GLOB TpCoreFiles
        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/IO/XMLIO.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite/IO/InputVar.cpp)

add_library(TpCorelib STATIC ${TpCoreFiles})

target_compile_definitions(TpCorelib PUBLIC ${ext_defs})

target_include_directories(TpCorelib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite)

target_link_libraries(TpCorelib PUBLIC ${ext_lib})


if(HAVE_TEST)
    set(CMAKE_MODULE_PATH   ${CMAKE_MODULE_PATH}
            ${CMAKE_CURRENT_SOURCE_DIR}/ext/Catch2/contrib)

    include(CTest)
    include(ParseAndAddCatchTests)

    file(GLOB testTopoFiles test/Interlocking/Test_ContactGraph.cpp)
    #file(GLOB testTopoFiles test/CrossMesh/*.cpp
    #                        test/Mesh/*.cpp
    #                        test/IO/*.cpp
    #                        test/Utility/*.cpp)
    #file(GLOB testTopoFiles test/Mesh/*.cpp
    #                        test/Utility/*.cpp)

    set(testTopoFiles ${testTopoFiles} test/Test_Main.cpp)

    add_executable(testTopo ${testTopoFiles})

    target_link_libraries(testTopo PUBLIC Catch2::Catch2)
    target_link_libraries(testTopo PUBLIC TpCorelib)
    ParseAndAddCatchTests(testTopo)
endif()

if(HAVE_NANOGUI)
    add_executable(TopoGUI src/TopoLite/GUI/guiMain.cpp)
    target_link_libraries(TopoGUI nanogui)
    target_include_directories(TopoGUI PUBLIC SYSTEM ${NANOGUI_INCLUDE_DIRS})
endif()
