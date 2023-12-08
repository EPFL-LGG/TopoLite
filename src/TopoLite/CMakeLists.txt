cmake_minimum_required(VERSION 3.5)

project(TopoLiteLib)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/CrossMesh/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Structure/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/IO/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Mesh/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Utility/*.*"
)

option(CXX "enable C++ compilation" ON)
find_package(TBB REQUIRED)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "Eigen3::Eigen"
        "TBB::tbb"
        "nlohmann_json::nlohmann_json"
        "pugixml"
        "igl::core"
        "igl_restricted::triangle"
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
        PUBLIC
        "TBB::tbb"
        PUBLIC
        "nlohmann_json::nlohmann_json"
        PUBLIC
        "pugixml"
        PUBLIC
        "igl::core"
        PUBLIC
        "igl_restricted::triangle"
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