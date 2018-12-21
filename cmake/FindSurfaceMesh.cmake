set(SURFACEMESH_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/external/surface_mesh")
include_directories(${SURFACEMESH_INCLUDE_DIR})
add_subdirectory(${SURFACEMESH_INCLUDE_DIR})
target_link_libraries(surface_mesh ${LIBRARIES})