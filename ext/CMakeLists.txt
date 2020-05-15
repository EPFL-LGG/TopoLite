cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# eigen
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)

# filesystem
set(FILESYSTEM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/filesystem)

# pugixml
find_package(PUGIXML REQUIRED)

# quickhull
find_package(QuickHull)

# clipper
find_package(Clipper)

#LIBIGL
set(LIBIGL_WITH_TRIANGLE ON CACHE BOOL "Use Triangle" FORCE)
set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libigl/include)
list(APPEND CMAKE_MODULE_PATH "${LIBIGL_INCLUDE_DIR}/../cmake")
include(libigl)

##ShapeOp
#set(SHAPEOP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shapeop)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/shapeop)

#NANOGUI
if(HAVE_NANOGUI)
#    set(NANOGUI_BACKEND "OpenGL" CACHE STRING "Choose the backend used for rendering" FORCE)
    find_package(NanoGUI)
    set(NANOGUI_INCLUDE_DIRS ${NANOGUI_INCLUDE_DIRS} PARENT_SCOPE)
    set(NANOGUI_EXTRA_DEFS ${NANOGUI_EXTRA_DEFS} PARENT_SCOPE)
    set(NANOGUI_DIR ${NANOGUI_DIR} PARENT_SCOPE)
endif()

#if(USE_UNIT_TEST OR USE_PYTHON_PLUGIN OR USE_C_EXTENSION)
#set_target_properties(ShapeOp  PROPERTIES
#        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../Release"
#        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/../Release")
#endif()

# TBB
# Handled in cmake/TopoliteDependencies.cmake
set(TBB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tbb/include)

# CLP
set(CLP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/clp/Clp/src
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/Clp/src/OsiClp
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/ext/CoinUtils/src
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/ext/Osi/src/Osi)

## IPOPT
find_package(IPOPT REQUIRED)

## Pybind11

if(HAVE_PYBIND)
    #Pybind11
    if(WIN32)
        set(PYTHON_EXECUTABLE $ENV{USERPROFILE}/anaconda3/python.exe)
    endif()
    set(PYBIND11_CPP_STANDARD -std=c++11)
    set(PYBIND11_PYTHON_VERSION 3.7)
    add_subdirectory(pybind11)
endif()


set(ext_include ${CPPSUBPROCESS_INCLUDE_DIR}
                ${PUGIXML_INCLUDE_DIR}
                ${QuickHull_INCLUDE_DIR}
                ${Clipper_INCLUDE_DIR}
                ${LIBIGL_INCLUDE_DIR}
                ${SHAPEOP_INCLUDE_DIR}
                ${TBB_INCLUDE_DIR}
                ${Boost_INCLUDE_DIRS}
                ${EIGEN3_INCLUDE_DIR}
                ${CLP_INCLUDE_DIR}
                ${IPOPT_INCLUDE_DIRS}           # defined in findIPOPT.cmake
                 PARENT_SCOPE)

set(ext_lib pugixml
            QuickHull
            Clipper
            igl::core
            igl::triangle
#            ShapeOp
            ${TBB_IMPORTED_TARGETS}
            lib_Osi
            lib_Clp
            lib_CoinUtils
            ${IPOPT_LIBRARIES}                  # defined in findIPOPT.cmake
            ${Boost_LIBRARIES}
            PARENT_SCOPE)