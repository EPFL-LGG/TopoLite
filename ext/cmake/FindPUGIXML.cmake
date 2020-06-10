# Find the pugixml XML parsing library.
#
# Sets the usual variables expected for find_package scripts:
#
# PUGIXML_INCLUDE_DIR - header location
# PUGIXML_LIBRARIES - library to link against
# PUGIXML_FOUND - true if pugixml was found.

if(PUGIXML_FOUND)
    return()
endif()

find_path(PUGIXML_INCLUDE_DIR pugixml.hpp
        HINTS
        ENV PUGIXML_ROOT
        PATHS
        ${CMAKE_SOURCE_DIR}/ext/pugixml
        PATH_SUFFIXES src
        )

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PUGIXML
        "\nPUGIXML not found"
        PUGIXML_INCLUDE_DIR)
mark_as_advanced(PUGIXML_INCLUDE_DIR)

if(MSVC)
    set(BUILD_SHARED_LIBS ON CACHE STRING "Build pugixml using static library" FORCE)
else()
    set(BUILD_SHARED_LIBS OFF CACHE STRING "Build pugixml using static library" FORCE)
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/pugixml)
set(PUGIXML_LIBRARIES pugixml)