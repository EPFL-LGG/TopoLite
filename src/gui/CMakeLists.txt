cmake_minimum_required(VERSION 3.5)

project(TopoGui)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/shader/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Utility/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Widget/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Widget/Canvas/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Widget/Controls/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Widget/Dialog/*.*"
)

option(CXX "enable C++ compilation" ON)
find_package(TBB REQUIRED)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "Eigen3::Eigen"
        "nanogui-obj"
        "nanogui"
)

get_property(nanogui_include_dirs GLOBAL PROPERTY nanogui_include_dir)
# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
        PUBLIC "${nanogui_include_dirs}/include"

)

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "Eigen3::Eigen"
        PUBLIC
        "nanogui-obj"
        PUBLIC
        "nanogui"
)

# compile definitions
list(APPEND
        CRL_COMPILE_DEFINITIONS #
        ""
)

# create target
create_crl_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}"
)