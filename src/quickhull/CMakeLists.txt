cmake_minimum_required(VERSION 3.5)

project(quickhull)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/Structs/*.h"
)

option(CXX "enable C++ compilation" ON)
find_package(TBB REQUIRED)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "Eigen3::Eigen"
)

message("${CMAKE_CURRENT_SOURCE_DIR}")
# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "Eigen3::Eigen"
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