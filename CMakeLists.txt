# we need cmake >= 3.11 to support fetch_content that downloads dependencies
cmake_minimum_required(VERSION 3.11)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# This is needed for the YuMi setup. If vcpkg is installed via the script from crl-yumi, the VCPKG_DIR environment variable is set
if(DEFINED ENV{VCPKG_DIR} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_DIR}/scripts/buildsystems/vcpkg.cmake"
      CACHE STRING "")
endif()

# set name of the project
project(TopoLite)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_FLAGS "-fPIC")

#SET (CMAKE_C_COMPILER "/usr/bin/clang" CACHE STRING "C compiler" FORCE)
#SET (CMAKE_CXX_COMPILER "/usr/bin/clang++" CACHE STRING "C++ compiler" FORCE)
add_definitions(-DUNITTEST_DATAPATH="${CMAKE_CURRENT_SOURCE_DIR}/data")
# data folder path
set(ROBOCRAFT_DATA_FOLDER
    "\"${CMAKE_CURRENT_LIST_DIR}/data\""
    CACHE STRING ""
)

# top level folder
set(ROBOCRAFT_TOPLEVEL_FOLDER "${CMAKE_CURRENT_LIST_DIR}"  CACHE STRING "")
set(GUI ON CACHE BOOL "")

# -----------------------------------------------------------------------------
# macro and functions
# -----------------------------------------------------------------------------
# create static library named LIB_NAME
function(
    create_crl_library #
    LIB_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
)
    add_library(${LIB_NAME} STATIC ${SOURCE})
    add_library(crl::${LIB_NAME} ALIAS ${LIB_NAME})
    add_dependencies(${LIB_NAME} ${DEPENDENCY})
    target_include_directories(${LIB_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        # scope is PUBLIC so these definitions are propagated
        target_compile_definitions(${LIB_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${LIB_NAME} PROPERTY FOLDER "Libs")
endfunction()

function(
    create_crl_shared_library #
    LIB_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
)
    add_library(${LIB_NAME} SHARED ${SOURCE})
    add_library(crl::${LIB_NAME} ALIAS ${LIB_NAME})
    add_dependencies(${LIB_NAME} ${DEPENDENCY})
    target_include_directories(${LIB_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${LIB_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        # scope is PUBLIC so these definitions are propagated
        target_compile_definitions(${LIB_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${LIB_NAME} PROPERTY FOLDER "Libs")
endfunction()

# create executable named APP_NAME
function(
    create_crl_app #
    APP_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
    )
    
    add_executable(${APP_NAME} ${SOURCE})
    add_dependencies(${APP_NAME} ${DEPENDENCY})
    target_include_directories(${APP_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${APP_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        target_compile_definitions(${APP_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${APP_NAME} PROPERTY FOLDER "Apps")
endfunction()

# create executable named DEMO_NAME
function(
    create_crl_demo #
    DEMO_NAME #
    SOURCE #
    DEPENDENCY #
    INCLUDE_DIRS #
    LINK_LIBS #
    COMPILE_DEFINITIONS #
    )
    
    add_executable(${DEMO_NAME} ${SOURCE})
    add_dependencies(${DEMO_NAME} ${DEPENDENCY})
    target_include_directories(${DEMO_NAME} ${INCLUDE_DIRS})
    target_link_libraries(${DEMO_NAME} ${LINK_LIBS})

    if(COMPILE_DEFINITIONS)
        target_compile_definitions(${DEMO_NAME} ${COMPILE_DEFINITIONS})
    endif()

    # For solution explorer in visual studios
    set_property(TARGET ${DEMO_NAME} PROPERTY FOLDER "Demos")
endfunction()

# demo options
option(ROBOCRAFT_DEMO_ALLEGROHAND "Build allegro hand demo" OFF)
option(ROBOCRAFT_DEMO_KINOVA "Build kinova gen3 robot demo" OFF)
option(ROBOCRAFT_DEMO_OPTITRACK "Build optitrack demo" OFF)
option(ROBOCRAFT_DEMO_SAKEGRIPPER "Build sake gripper demo" OFF)
option(ROBOCRAFT_DEMO_UR5 "Build ur5 robot demo" OFF)
option(ROBOCRAFT_DEMO_WIDOWX "Build widowx robot demo" OFF)
option(ROBOCRAFT_DEMO_YUMI "Build yumi robot demo" OFF)

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------
# thirdparty code
add_subdirectory(ext)

# our code
add_subdirectory(src)

# test
add_subdirectory(test)

#app
#add_subdirectory(app)


# -----------------------------------------------------------------------------
# clean up solutions view
# -----------------------------------------------------------------------------
if(WIN32)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
    set_target_properties(glfw PROPERTIES FOLDER Ext)
    set_target_properties(glad PROPERTIES FOLDER Ext)
    set_target_properties(imgui PROPERTIES FOLDER Ext)
    set_target_properties(tinyobjloader PROPERTIES FOLDER Ext)
    set_target_properties(DCA PROPERTIES FOLDER Ext)
    set_target_properties(utils PROPERTIES FOLDER Ext)
    set_target_properties(gui PROPERTIES FOLDER Ext)
    set_target_properties(tinyxml2 PROPERTIES FOLDER Ext)
endif(WIN32)
