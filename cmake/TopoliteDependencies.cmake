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
if(NOT TARGET catch2::catch2 AND BUILD_TOPOTEST)
    topolite_download_catch()
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
if(NOT TARGET pybind11::pybind11 AND BUILD_TOPOPYBIND)
    topolite_download_pybind11()
endif()

# nanogui
if(NOT TARGET nanogui AND BUILD_TOPOGUI)
    topolite_download_nanogui()
endif()

# TBB library
if(NOT TARGET tbb)
    if(WIN32)
        topolite_download_tbb()
    else()
        topolite_download_tbb()
    endif()

endif()

#json
if(NOT TARGET nlohmann_json::nlohmann_json)
    topolite_download_json()
endif()
