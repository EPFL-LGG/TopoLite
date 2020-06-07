cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# eigen
set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen)

# filesystem
set(FILESYSTEM_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/filesystem)

# pugixml
find_package(PUGIXML REQUIRED)

# quickhull
find_package(QuickHull REQUIRED)

# clipper
find_package(Clipper REQUIRED)

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
if(WIN32)
    list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tbb/cmake)
    find_package(TBB REQUIRED tbb_preview)
else()
    set(TBB_BUILD_STATIC            ON CACHE BOOL " " FORCE)
    set(TBB_BUILD_SHARED            OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TBBMALLOC         ON  CACHE BOOL " " FORCE) # needed for CGAL's parallel mesher
    set(TBB_BUILD_TBBMALLOC_PROXY   OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TESTS             OFF CACHE BOOL " " FORCE)
    add_subdirectory(${TOPOLITE_EXTERNAL}/tbb)
    set(TBB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tbb/include)
    set(TBB_IMPORTED_TARGETS tbb_static tbbmalloc_static)
endif()


# CLP
set(BUILD_SHARED_LIBS OFF CACHE STRING "Build clp using static library" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/clp)
set(CLP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/clp/Clp/src
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/Clp/src/OsiClp
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/ext/CoinUtils/src
        ${CMAKE_CURRENT_SOURCE_DIR}/clp/ext/Osi/src/Osi)
set(CLP_LIBRARIES CoinUtils Osi Clp OsiClpSolver)

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


#JSON
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)
set(JSON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/json/include)
#Set external inlude path/library/definitions

set(ext_include ${CPPSUBPROCESS_INCLUDE_DIR}
                ${PUGIXML_INCLUDE_DIR}
                ${QUICKHULL_INCLUDE_DIR}
                ${CLIPPER_INCLUDE_DIR}
                ${LIBIGL_INCLUDE_DIR}
                ${TBB_INCLUDE_DIR}
                ${EIGEN3_INCLUDE_DIR}
                ${CLP_INCLUDE_DIR}
                ${JSON_INCLUDE_DIR})

set(ext_lib ${PUGIXML_LIBRARIES}
            ${QUICKHULL_LIBRARIES}
            ${CLIPPER_LIBRARIES}
            ${TBB_IMPORTED_TARGETS}
            igl::core
            igl::triangle
            ${CLP_LIBRARIES}
            nlohmann_json::nlohmann_json)

set(ext_defs ${NANOGUI_EXTRA_DEFS})

if(IPOPT_FOUND)
    set(ext_include ${ext_include} ${IPOPT_INCLUDE_DIRS})
    set(ext_lib ${ext_lib} ${IPOPT_LIBRARIES})
endif()

set_property(GLOBAL PROPERTY ext_include_property "${ext_include}")
set_property(GLOBAL PROPERTY ext_lib_property "${ext_lib}")
set_property(GLOBAL PROPERTY ext_defs_property "${ext_defs}")
set_property(GLOBAL PROPERTY IPOPT_FOUND ${IPOPT_FOUND})
