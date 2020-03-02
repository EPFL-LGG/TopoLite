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
## Catch2
function(topolite_download_catch)
    topolite_download_project(Catch2
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


