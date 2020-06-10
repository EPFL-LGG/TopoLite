#TopoLite Project

cmake_minimum_required(VERSION 3.11)
project(TopoLite)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_definitions(-DUNITTEST_DATAPATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
set(CMAKE_CXX_STANDARD 17)

#########################################
#####                               #####
#####      Compiler flags           #####
#####                               #####
#########################################

set(BUILD_TOPOGUI         ON CACHE STRING "Build gui" FORCE)
set(BUILD_TOPOTEST        ON CACHE STRING "Build test" FORCE)
#set(BUILD_TOPOPYBIND      ON CACHE STRING "Build pybind11" FORCE)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# set(CMAKE_CXX_FLAGS "-Wall -Wextra")

# Handles GCC and CLANG - Note that GCC on macOS is actually an alias to LLVM CLANG

if (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    message("Compiling ${CMAKE_CXX_COMPILER_ID}")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -stdlib=libc++")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -stdlib=libc++")
elseif(${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    message("Compiling with ${CMAKE_CXX_COMPILER_ID} GCC")
    set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++17")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++17")
endif()

if(MSVC)
    add_definitions(-D_HAS_STD_BYTE=0 -D_USE_MATH_DEFINES)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
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

include(TopoliteDependencies)

# Add deps directory

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/ext/")

#########################################
#####                               #####
#####      Core Library             #####
#####                               #####
#########################################

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${TOPO_RUNTIME_OUTPUT})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${TOPO_RUNTIME_OUTPUT})

add_subdirectory(src)
if(MSVC)
    add_library(TpCorelib SHARED ${TpCoreFiles})
else()
    add_library(TpCorelib STATIC ${TpCoreFiles})
endif()

get_property(ext_include GLOBAL PROPERTY ext_include_property)
get_property(ext_defs GLOBAL PROPERTY ext_defs_property)
get_property(ext_lib GLOBAL PROPERTY ext_lib_property)

target_compile_definitions(TpCorelib PUBLIC ${ext_defs})

target_include_directories(TpCorelib PUBLIC SYSTEM
        ${ext_include}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/TopoLite)

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8)
		message("GCC 7 does not have <filesystem>, using <experimental/filesytem> instead")
		target_compile_definitions(TpCorelib PUBLIC -DGCC_VERSION_LESS_8)
		target_link_libraries(TpCorelib PUBLIC stdc++fs)
	endif()
endif()

target_link_libraries(TpCorelib PUBLIC ${ext_lib})

if(NOT ${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME})
    set(TPCORELIB_EXTRA_INCLUDE ${ext_include} PARENT_SCOPE)
endif()

add_subdirectory(test)
add_subdirectory(gui)
#add_subdirectory(python)
