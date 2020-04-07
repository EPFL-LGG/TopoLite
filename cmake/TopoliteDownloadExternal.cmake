################################################################################
include(DownloadProject)

# With CMake 3.8 and above, we can hide warnings about git being in a
# detached head by passing an extra GIT_CONFIG option
if(NOT (${CMAKE_VERSION} VERSION_LESS "3.8.0"))
    set(TOPOLITE_EXTRA_OPTIONS "GIT_CONFIG advice.detachedHead=False")
else()
    set(TOPOLITE_EXTRA_OPTIONS "")
endif()

# Shortcut function
function(topolite_download_project name)
    download_project(
        PROJ         ${name}
        SOURCE_DIR   ${TOPOLITE_EXTERNAL}/${name}
        DOWNLOAD_DIR ${TOPOLITE_EXTERNAL}/.cache/${name}
        QUIET
        ${TOPOLITE_EXTRA_OPTIONS}
        ${ARGN}
    )
endfunction()

################################################################################
## Eigen
function(topolite_download_eigen)
    topolite_download_project(eigen
            GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
            GIT_TAG        3.3.7
            )
endfunction()

## Catch2
function(topolite_download_catch)
    topolite_download_project(catch2
            GIT_REPOSITORY https://github.com/catchorg/Catch2.git
            GIT_TAG        d63307279412de3870cf97cc6802bae8ab36089e # v 2.7.0
            )
endfunction()

## TBB
function(topolite_download_tbb)
    topolite_download_project(tbb
            GIT_REPOSITORY https://github.com/wjakob/tbb.git
            GIT_TAG        20357d83871e4cb93b2c724fe0c337cd999fd14f
            )
endfunction()

## CLP
function(topolite_download_clp)
    topolite_download_project(clp
            GIT_REPOSITORY https://github.com/robin-forks/COIN-OR-osi-clp.git
            GIT_TAG        cc27768b2ab4caec068ab0158881a76294c2a711
            )
endfunction()

## filesystem
function(topolite_download_filesystem)
    topolite_download_project(filesystem
            GIT_REPOSITORY     https://github.com/KIKI007/filesystem.git
            GIT_TAG            984cf9bd716c38c18c73525b7b7aa87ae45ecb77
            )
endfunction()

## libigl
function(topolite_download_libigl)
    topolite_download_project(libigl
            GIT_REPOSITORY     https://github.com/libigl/libigl.git
            GIT_TAG            87b54fc4dcc5aea182dd842a55ac75536fb69e1a
            )
endfunction()

## ShapeOp
function(topolite_download_shapeop)
    topolite_download_project(shapeop
            GIT_REPOSITORY     https://github.com/EPFL-LGG/ShapeOp.git
            GIT_TAG            6319182302de443f9d837dd3f9cd365f6a812907
            )
endfunction()

# pugixml
function(topolite_download_pugixml)
    topolite_download_project(pugixml
            GIT_REPOSITORY     https://github.com/KIKI007/pugixml.git
            GIT_TAG            5e64076af9691ce8100aedf7a61cb338f2596151
            )
endfunction()

# pybind11
function(topolite_download_pybind11)
    topolite_download_project(pybind11
            GIT_REPOSITORY     https://github.com/pybind/pybind11.git
            GIT_TAG            023487164915c5b62e16a9b4e0b37cb01cd5500a
            )
endfunction()