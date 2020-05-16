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

#NANOGUI
if(BUILD_TOPOGUI)
#    set(NANOGUI_BACKEND "OpenGL" CACHE STRING "Choose the backend used for rendering" FORCE)
    find_package(NanoGUI)
    set(NANOGUI_INCLUDE_DIRS ${NANOGUI_INCLUDE_DIRS} ${NANOGUI_EXTRA_INCS} PARENT_SCOPE)
    set(NANOGUI_EXTRA_DEFS ${NANOGUI_EXTRA_DEFS} PARENT_SCOPE)
    set(NANOGUI_DIR ${NANOGUI_DIR} PARENT_SCOPE)
endif()

# TBB
# Handled in cmake/TopoliteDependencies.cmake
set(TBB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tbb/include)

# CLP
set(CLP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/clp/Clp/src
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/Clp/src/OsiClp
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/ext/CoinUtils/src
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/ext/Osi/src/Osi)

## IPOPT
find_package(IPOPT)

## Pybind11
if(BUILD_TOPOPYBIND)
    #Pybind11
    if(WIN32)
        set(PYTHON_EXECUTABLE $ENV{USERPROFILE}/anaconda3/python.exe)
    endif()
    set(PYBIND11_CPP_STANDARD -std=c++11)
    set(PYBIND11_PYTHON_VERSION 3.7)
    add_subdirectory(pybind11)
endif()

if(BUILD_TOPOTEST)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/catch2)
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/catch2/contrib)
endif()


set(ext_include ${CPPSUBPROCESS_INCLUDE_DIR}
                ${PUGIXML_INCLUDE_DIR}
                ${QUICKHULL_INCLUDE_DIR}
                ${CLIPPER_INCLUDE_DIR}
                ${LIBIGL_INCLUDE_DIR}
                ${TBB_INCLUDE_DIR}
                ${EIGEN3_INCLUDE_DIR}
                ${CLP_INCLUDE_DIR}
		        PARENT_SCOPE)

set(ext_lib ${PUGIXML_LIBRARIES}
            ${QUICKHULL_LIBRARIES}
            ${CLIPPER_LIBRARIES}
            ${TBB_IMPORTED_TARGETS}
            igl::core
            igl::triangle
            ${CLP_LIBRARIES}
            PARENT_SCOPE)

if(IPOPT_FOUND)
	list(APPEND ext_include ${IPOPT_INCLUDE_DIRS})
	list(APPEND ext_lib ${IPOPT_LIBRARIES})
endif()