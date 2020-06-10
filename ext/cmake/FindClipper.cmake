if(CLIPPER_FOUND)
    return()
endif()

find_path(CLIPPER_INCLUDE_DIR clipper.hpp
        HINTS
        ENV CLIPPER_ROOT
        PATHS
        ${CMAKE_SOURCE_DIR}/ext/clipper)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Clipper
        "\nClipper not found"
        CLIPPER_INCLUDE_DIR)
mark_as_advanced(CLIPPER_INCLUDE_DIR)

add_subdirectory(${CLIPPER_INCLUDE_DIR})
set(CLIPPER_LIBRARIES Clipper)