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

#function(topolite_download_tbb)
#    topolite_download_project(tbb
#            GIT_REPOSITORY https://github.com/oneapi-src/oneTBB.git
#            GIT_TAG        60b7d0a78f8910976678ba63a19fdaee22c0ef65)
#endfunction()


function(topolite_download_tbb_binary)
    topolite_download_project(tbb
            GIT_REPOSITORY https://github.com/KIKI007/tbb_binary.git
            GIT_TAG        8fb255d3d464b787e1629318c262d80befc87497)
endfunction()


## CLP
function(topolite_download_clp)
    topolite_download_project(clp
            GIT_REPOSITORY https://github.com/KIKI007/COIN-OR-osi-clp.git
            GIT_TAG        4c11a2ce842cd7e667c6f69cef35a0461404c0dc            # v1.0.0
            )
endfunction()


## IPOPT
function(topolite_download_ipopt)
    topolite_download_project(ipopt
            GIT_REPOSITORY https://github.com/robin-forks/COIN-OR_Ipopt
            GIT_TAG        v1.2.0
            )
endfunction()

## COIN-OR LP
function(topolite_download_coinor_lp)
    topolite_download_project(coinor_lp
            GIT_REPOSITORY https://github.com/robin-forks/COIN-OR_suite_cmake_compatible.git
            GIT_TAG        7d69f4b48869e3f30db11cf04e1b3bc49480fca8
            )
endfunction()

## filesystem
function(topolite_download_filesystem)
    topolite_download_project(filesystem
            GIT_REPOSITORY     https://github.com/KIKI007/filesystem.git
            GIT_TAG            90fb57f9698bce2961d3a23bc6c957abddaeff68
            )
endfunction()

## libigl
function(topolite_download_libigl)
    topolite_download_project(libigl
            GIT_REPOSITORY     https://github.com/libigl/libigl.git
            GIT_TAG            87b54fc4dcc5aea182dd842a55ac75536fb69e1a
            )
endfunction()

### ShapeOp
#function(topolite_download_shapeop)
#    topolite_download_project(shapeop
#            GIT_REPOSITORY     https://github.com/EPFL-LGG/ShapeOp.git
#            GIT_TAG            9a79b808c5e4144b8b6f0f429df7273f61e95547
#            )
#endfunction()

# pugixml
function(topolite_download_pugixml)
    topolite_download_project(pugixml
            GIT_REPOSITORY     https://github.com/robin-forks/pugixml.git
            GIT_TAG            bbcc25dc72ecb8b9d53bd2342d7b37e9e8f33264
            )
endfunction()

# pybind11
function(topolite_download_pybind11)
    topolite_download_project(pybind11
            GIT_REPOSITORY     https://github.com/pybind/pybind11.git
            GIT_TAG            023487164915c5b62e16a9b4e0b37cb01cd5500a
            )
endfunction()

# nanogui
function(topolite_download_nanogui)
    topolite_download_project(nanogui
            GIT_REPOSITORY     https://github.com/mitsuba-renderer/nanogui.git
            GIT_TAG            741f3323c73891f62c71427c1fd766b8a32035f5
            )
endfunction()

function(topolite_download_json)
    topolite_download_project(json
            GIT_REPOSITORY     https://github.com/nlohmann/json.git
            GIT_TAG            dd7e25927fe7a49c81d07943c32444f0a9011665
            )
endfunction()
