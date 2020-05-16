# Find the QUICKHULL XML parsing library.
#
# Sets the usual variables expected for find_package scripts:
#
# QUICKHULL_INCLUDE_DIR - header location
# QUICKHULL_LIBRARIES - library to link against
# QUICKHULL_FOUND - true if QUICKHULL was found.

if(QUICKHULL_FOUND)
    return()
endif()

find_path(QUICKHULL_INCLUDE_DIR QuickHull.hpp
        HINTS
        ENV QUICKHULL_ROOT
        PATHS
        ${CMAKE_SOURCE_DIR}/ext/quickhull
        )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(QUICKHULL
        "\nQuickhull not found"
        QUICKHULL_INCLUDE_DIR)
mark_as_advanced(QUICKHULL_INCLUDE_DIR)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/quickhull)
set(QUICKHULL_LIBRARIES quickhull)