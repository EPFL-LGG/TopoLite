# FetchContent command is available with cmake >= 3.11
include(FetchContent)

# -----------------------------------------------------------------------------
# helper funtion to download a 'fetch content declare'
# -----------------------------------------------------------------------------
macro(fetch what)
    FetchContent_GetProperties("${what}")
    if(NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
    endif()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()


#------------------------------------------------------------------------------
# json
FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG v3.11.3
)
fetch(json)
add_subdirectory(${json_SOURCE_DIR} json)

# pugixml
FetchContent_Declare(
        pugixml
        GIT_REPOSITORY https://github.com/robin-forks/pugixml.git
        GIT_TAG bbcc25dc72ecb8b9d53bd2342d7b37e9e8f33264
)
fetch(pugixml)
add_subdirectory(${pugixml_SOURCE_DIR} pugixml)

#------------------------------------------------------------------------------
# libigl
#option(LIBIGL_GLFW             "Build target igl::glfw"             ON)
#option(LIBIGL_IMGUI            "Build target igl::imgui"            ON)
option(LIBIGL_RESTRICTED_TRIANGLE "Build target igl_restricted::triangle"           ON)
option(LIBIGL_COPYLEFT_CORE    "Build target igl_copyleft::core"                    ON)
FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG        v2.4.0
)
fetch(libigl)
add_subdirectory(${libigl_SOURCE_DIR} libigl)

#----------
# catch2
FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.0.0-preview4
)
FetchContent_MakeAvailable(Catch2)

#---------
option (NANOGUI_BACKEND "backend" "Metal")
# nanogui
FetchContent_Declare(
        nanogui
        GIT_REPOSITORY     https://github.com/mitsuba-renderer/nanogui.git
        GIT_TAG            741f3323c73891f62c71427c1fd766b8a32035f5
)
set_property(GLOBAL PROPERTY nanogui_include_dir "${nanogui_SOURCE_DIR}")
FetchContent_MakeAvailable(nanogui)

##------------------------------------------------------------------------------
## pybind11
#FetchContent_Declare(
#        pybind11
#        GIT_REPOSITORY https://github.com/pybind/pybind11.git
#)
#fetch(pybind11)
#add_subdirectory(${pybind11_SOURCE_DIR} pybind11)

