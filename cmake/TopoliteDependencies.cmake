### Configuration
set(TOPOLITE_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
set(TOPOLITE_EXTERNAL "${TOPOLITE_ROOT}/ext")

# Download and update 3rdparty libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
list(REMOVE_DUPLICATES CMAKE_MODULE_PATH)
include(TopoliteDownloadExternal)

# Eigen
if(NOT TARGET Eigen3::Eigen)
    topolite_download_eigen()
endif()

# COIN-OR CLP
if(NOT TARGET lib_Clp)
    topolite_download_clp()
    add_subdirectory(${TOPOLITE_EXTERNAL}/clp)
endif()

# COIN-OR IPOPT
if(NOT TARGET lib_Ipopt)
    topolite_download_Ipopt()
    add_subdirectory(${TOPOLITE_EXTERNAL}/ipopt)
endif()

# Catch2
if(NOT TARGET catch2::catch2)
    topolite_download_catch()
    add_subdirectory(${TOPOLITE_EXTERNAL}/catch2)
    list(APPEND CMAKE_MODULE_PATH ${TOPOLITE_EXTERNAL}/catch2/contrib)
endif()

# Filesystem
if(NOT TARGET filesystem::filesystem)
    topolite_download_filesystem()
    add_subdirectory(${TOPOLITE_EXTERNAL}/filesystem)
endif()

# libigl
if(NOT TARGET igl::core)
    topolite_download_libigl()
endif()

# shapeop
if(NOT TARGET ShapeOp)
    topolite_download_shapeop()
endif()

# pugixml
if(NOT TARGET pugixml)
    topolite_download_pugixml()
endif()

# pybind11
if(NOT TARGET pybind11::pybind11 AND HAVE_PYBIND)
    topolite_download_pybind11()
endif()

# nanogui
if(NOT TARGET nanogui AND HAVE_NANOGUI)
    topolite_download_nanogui()
endif()

# TBB library
if(NOT TARGET tbb)
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

