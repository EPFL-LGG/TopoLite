if(BUILD_TOPOTEST)

    set(CMAKE_MODULE_PATH   ${CMAKE_MODULE_PATH}
            ${CMAKE_CURRENT_SOURCE_DIR}/../ext/catch2/contrib)

    include(CTest)
    include(ParseAndAddCatchTests)

    add_subdirectory(CrossMesh)
    add_subdirectory(Interlocking)
    add_subdirectory(IO)
    add_subdirectory(Mesh)
    add_subdirectory(Structure)
    add_subdirectory(Utility)

    set(testTopoFiles ${TestIO} ${TestStructure} ${TestUtility} ${TestMesh} ${TestInterlocking} ${TestCrossMesh})

    file(GLOB singleTestFile IO/Test_JsonReaderIO.cpp)

    add_executable(testTopo ${testTopoFiles} Test_Main.cpp)

    target_link_libraries(testTopo PUBLIC Catch2::Catch2)
    target_link_libraries(testTopo PUBLIC TpCorelib)
    ParseAndAddCatchTests(testTopo)

    file(COPY ../data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    if(CMAKE_GENERATOR STREQUAL Xcode)
        target_compile_definitions(testTopo PUBLIC -DTOPOCREATOR_DATA_PATH="../data/")
    else()
        target_compile_definitions(testTopo PUBLIC -DTOPOCREATOR_DATA_PATH="data/")
    endif()
endif()
