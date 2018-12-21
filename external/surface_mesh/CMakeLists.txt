file(GLOB SOURCES ./surface_mesh/*.cpp)
file(GLOB HEADERS ./surface_mesh/*.h)

if(UNIX)
    add_library(surface_mesh SHARED ${SOURCES} ${HEADERS})
elseif(WIN32)
    add_library(surface_mesh STATIC ${SOURCES} ${HEADERS})
endif()

set_target_properties(surface_mesh PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(surface_mesh PROPERTIES VERSION 1.0)
