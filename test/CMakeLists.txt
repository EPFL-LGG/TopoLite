cmake_minimum_required(VERSION 3.5)

project(TopoLiteTest)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/CrossMesh/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Structure/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/IO/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Mesh/*.*"
        "${CMAKE_CURRENT_SOURCE_DIR}/Utility/*.*"
)

option(CXX "enable C++ compilation" ON)

# basic dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "TopoLiteLib"
        "Catch2::Catch2WithMain"
        "quickhull"
)

# target include dirs
list(APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC
        "TopoLiteLib"
        PUBLIC
        "Catch2::Catch2WithMain"
        PUBLIC
        "quickhull"
)

# compile definitions
list(APPEND
        CRL_COMPILE_DEFINITIONS #
        ""
)

# create target
create_crl_app(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}"
)