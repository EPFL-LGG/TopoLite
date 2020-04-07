if(HAVE_PYBIND)
    file(GLOB TpPythonFiles
            ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/*.h)

    pybind11_add_module(pyTopo MODULE ${TpPythonFiles})
    target_link_libraries(pyTopo PUBLIC TpCorelib)
endif()