# inspired FROM https://github.com/libigl/libigl-examples/blob/master/cmake/FindNANOGUI.cmake
#
# Try to find NANOGUI library and include path.
# Once done this will define
#
# NANOGUI_FOUND
# NANOGUI_INCLUDE_DIR
# NANOGUI_LIBRARY
#

if(NOT NANOGUI_FOUND)

    set(NANOGUI_DIR "${PROJECT_SOURCE_DIR}/ext/nanogui")
    # Disable the examples and python stuff from nanogui
    set(NANOGUI_BUILD_PYTHON OFF CACHE BOOL " " FORCE)
    set(NANOGUI_BUILD_EXAMPLES OFF CACHE BOOL " " FORCE)
    set(NANOGUI_BUILD_SHARED_DEFAULT OFF CACHE BOOL " " FORCE)
    set(NANOGUI_BUILD_SHARED OFF CACHE BOOL " " FORCE)
    add_subdirectory("${NANOGUI_DIR}" "nanogui")

    # For reliability of parallel build, make the NanoGUI targets dependencies
    set_property(TARGET glfw glfw_objects nanogui PROPERTY FOLDER "dependencies")

    set(NANOGUI_INCLUDE_DIRS
        "${NANOGUI_DIR}/include"
        "${NANOGUI_DIR}/ext/nanovg/src"
        "${NANOGUI_DIR}/ext/glfw/include")
    include_directories(${NANOGUI_INCLUDE_DIRS})
    add_definitions(${NANOGUI_EXTRA_DEFS})

endif(NOT NANOGUI_FOUND)
