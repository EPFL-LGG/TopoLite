include(FetchContent)


### Configuration
set(TOPOLITE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(TOPOLITE_EXTERNAL "${TOPOLITE_ROOT}/ext")

# Download and update 3rdparty libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
list(REMOVE_DUPLICATES CMAKE_MODULE_PATH)
include(TopoliteDownloadExternal)

# Eigen
if(NOT TARGET Eigen3::Eigen)
    topolite_download_eigen()
endif()

# COIN-OR CLP
if(NOT TARGET lib_Clp)
    topolite_download_clp()
    add_subdirectory(${TOPOLITE_EXTERNAL}/clp)
endif()
#

# BUG: Cmakelist is not complete
## COIN-OR IPOPT
#if(NOT TARGET ipoptfort)
#    topolite_download_Ipopt()
#    add_subdirectory(${TOPOLITE_EXTERNAL}/ipopt)
#endif()

# BUG: does not work - clone recurse fails because ampl/mp is outdated and is sourcing ampl/gecode which does not exist anymore
## COIN-OR suite
#if(NOT TARGET coinor_lp)
#    topolite_download_coinor_lp()
#    add_subdirectory(${TOPOLITE_EXTERNAL}/coinor_lp)
#endif()


#FetchContent_Declare(
#        coinor
#        GIT_REPOSITORY https://github.com/robin-forks/COIN-OR_suite_cmake_compatible.git
#        GIT_TAG        7d69f4b
#)
#FetchContent_MakeAvailable(coinor)
#set(coinor_SOURCE_DIR ${coinor_SOURCE_DIR}/coinor)
#include_directories(${coinor_SOURCE_DIR} ${coinor_BINARY_DIR})
# ENDBUG

# Catch2
if(NOT TARGET catch2::catch2)
    topolite_download_catch()
    add_subdirectory(${TOPOLITE_EXTERNAL}/catch2)
    list(APPEND CMAKE_MODULE_PATH ${TOPOLITE_EXTERNAL}/catch2/contrib)
endif()

# libigl
if(NOT TARGET igl::core)
    topolite_download_libigl()
endif()

## shapeop
#if(NOT TARGET ShapeOp)
#    topolite_download_shapeop()
#endif()

# pugixml
if(NOT TARGET pugixml)
    topolite_download_pugixml()
endif()

# pybind11
if(NOT TARGET pybind11::pybind11 AND HAVE_PYBIND)
    topolite_download_pybind11()
endif()

# nanogui
if(NOT TARGET nanogui AND HAVE_NANOGUI)
    topolite_download_nanogui()
endif()

# TBB library
if(NOT TARGET tbb)
    set(TBB_BUILD_STATIC            ON  CACHE BOOL " " FORCE)
    set(TBB_BUILD_SHARED            OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TBBMALLOC         ON  CACHE BOOL " " FORCE) # needed for CGAL's parallel mesher
    set(TBB_BUILD_TBBMALLOC_PROXY   OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TESTS             OFF CACHE BOOL " " FORCE)


    if(WIN32)
         topolite_download_tbb_binary()
	 list(APPEND CMAKE_PREFIX_PATH ${TOPOLITE_EXTERNAL}/tbb/cmake)
    else()
    	topolite_download_tbb()
	include(${TOPOLITE_EXTERNAL}/tbb/cmake/TBBBuild.cmake)
    	tbb_build(TBB_ROOT ${TOPOLITE_EXTERNAL}/tbb CONFIG_DIR TBB_DIR MAKE_ARGS tbb_cpf=1)
    	list(APPEND CMAKE_MODULE_PATH ${TOPOLITE_EXTERNAL}/tbb/cmake)
    endif()
        find_package(TBB REQUIRED tbb_preview)
endif()

