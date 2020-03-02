### Configuration
set(TOPOLITE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(TOPOLITE_EXTERNAL "${TOPOLITE_ROOT}/ext")

# Download and update 3rdparty libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
list(REMOVE_DUPLICATES CMAKE_MODULE_PATH)
include(TopoliteDownloadExternal)

# Catch2
if(NOT TARGET Catch2::Catch2 AND (CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR))
    topolite_download_catch()
    add_subdirectory(${TOPOLITE_EXTERNAL}/Catch2)
    list(APPEND CMAKE_MODULE_PATH ${TOPOLITE_EXTERNAL}/Catch2/contrib)
endif()

# TBB library
if(NOT TARGET tbb::tbb)
    set(TBB_BUILD_STATIC            ON  CACHE BOOL " " FORCE)
    set(TBB_BUILD_SHARED            OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TBBMALLOC         ON  CACHE BOOL " " FORCE) # needed for CGAL's parallel mesher
    set(TBB_BUILD_TBBMALLOC_PROXY   OFF CACHE BOOL " " FORCE)
    set(TBB_BUILD_TESTS             OFF CACHE BOOL " " FORCE)

    topolite_download_tbb()
    add_subdirectory(${TOPOLITE_EXTERNAL}/tbb tbb)
    set_property(TARGET tbb_static tbb_def_files PROPERTY FOLDER "dependencies")
    #set_target_properties(tbb_static PROPERTIES COMPILE_FLAGS "-Wno-implicit-fallthrough -Wno-missing-field-initializers -Wno-unused-parameter -Wno-keyword-macro")

    add_library(tbb INTERFACE)
    target_include_directories(tbb SYSTEM INTERFACE ${TOPOLITE_EXTERNAL}/tbb/include)
    target_link_libraries(tbb INTERFACE tbb_static tbbmalloc_static)
    add_library(tbb::tbb ALIAS tbb)
endif()

