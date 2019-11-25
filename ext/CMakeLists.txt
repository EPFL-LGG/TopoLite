cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#EIGEN
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#Boost
find_package(Boost COMPONENTS system filesystem thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#Cpp Subprocess
set(CPPSUBPROCESS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cpp-subprocess)

#PUGIXML
find_package(PUGIXML REQUIRED)

#QuickHull
find_package(QuickHull)

#Clipper
find_package(Clipper)

#LIBIGL
set(IGL_STATIC_LIBRARY ON)
option(LIBIGL_WITH_TRIANGLE "Use Triangle" ON)
find_package(LIBIGL REQUIRED)
if(LIBIGL_FOUND)
    add_definitions(-DLIBIGL_FOUND=1)
endif()

#ShapeOp
set(SHAPEOP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/shapeop)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/shapeop)

#TBB
set(TBB_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tbb/include)
set(TBB_BUILD_SHARED          OFF  CACHE BOOL " " FORCE)
set(TBB_BUILD_STATIC          ON   CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC       OFF  CACHE BOOL " " FORCE)
set(TBB_BUILD_TBBMALLOC_PROXY OFF  CACHE BOOL " " FORCE)
set(TBB_BUILD_TESTS           OFF  CACHE BOOL " " FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tbb)
set_property(TARGET tbb_static tbb_def_files PROPERTY FOLDER "dependencies")

#fast-winding-number-soups
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/fast-winding-number-soups)


set(ext_include ${CPPSUBPROCESS_INCLUDE_DIR}
                ${PUGIXML_INCLUDE_DIR}
                ${QuickHull_INCLUDE_DIR}
                ${Clipper_INCLUDE_DIR}
                ${LIBIGL_INCLUDE_DIR}
                ${SHAPEOP_INCLUDE_DIR}
                ${TBB_INCLUDE_DIR}
                # ${FASTWIND_INCLUDE_DIR}
                PARENT_SCOPE)
set(ext_lib pugixml
            QuickHull
            Clipper
            igl::core
            igl::triangle
            ShapeOp
            tbb_static
            PARENT_SCOPE)
