cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#EIGEN
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Eigen)

#set(EIGEN3_INCLUDE_DIR /usr/local/include/eigen3)

if (WIN32)
    set(BOOST_ROOT "$ENV{USERPROFILE}/anaconda3/envs/interlock/Library/include")
else ()
    set(Boost_USE_STATIC_LIBS ON)
    set(BOOST_ROOT "$ENV{HOME}/anaconda3/envs/interlock/include")
endif()

set(Boost_USE_MULTITHREADED ON)

find_package(Boost COMPONENTS system filesystem REQUIRED)
message(${Boost_LIBRARY_DIR_RELEASE})
#Cpp Subprocess
set(CPPSUBPROCESS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cpp-subprocess)

#PUGIXML
option(BUILD_SHARED_LIBS OFF)
find_package(PUGIXML REQUIRED)

#QuickHull
find_package(QuickHull)

#Clipper
find_package(Clipper)

#LIBIGL
set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libigl/include)
list(APPEND CMAKE_MODULE_PATH "${LIBIGL_INCLUDE_DIR}/../cmake")
include(libigl)

#ShapeOp
set(SHAPEOP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shapeop)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/shapeop)

if(USE_UNIT_TEST OR USE_PYTHON_PLUGIN OR USE_C_EXTENSION)
set_target_properties(ShapeOp  PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../Release"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../Release")
endif()

#TBB
set(TBB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tbb/include)
set(TBB_BUILD_SHARED          OFF  CACHE BOOL " " FORCE)
set(TBB_BUILD_STATIC          ON   CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC       OFF  CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC_PROXY OFF  CACHE BOOL " " FORCE)
set(TBB_BUILD_TESTS           OFF  CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tbb)
set_property(TARGET tbb_static tbb_def_files PROPERTY FOLDER "dependencies")

#Pybind11
if(USE_PYTHON_PLUGIN)
    #Pybind11
    set(PYBIND11_CPP_STANDARD -std=c++11)
    set(PYBIND11_PYTHON_VERSION 3.7)
    add_subdirectory(pybind11)
endif()

set(ext_lib_dir ${Boost_LIBRARY_DIR} PARENT_SCOPE)

set(ext_include ${CPPSUBPROCESS_INCLUDE_DIR}
                ${PUGIXML_INCLUDE_DIR}
                ${QuickHull_INCLUDE_DIR}
                ${Clipper_INCLUDE_DIR}
                ${LIBIGL_INCLUDE_DIR}
                ${SHAPEOP_INCLUDE_DIR}
                ${TBB_INCLUDE_DIR}
                ${Boost_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIR}
                PARENT_SCOPE)

set(ext_lib pugixml
            QuickHull
            Clipper
            igl::core
            ShapeOp
            tbb_static
            ${Boost_LIBRARIES}
            PARENT_SCOPE)