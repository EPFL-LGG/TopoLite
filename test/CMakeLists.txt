if(HAVE_TEST)

    set(CMAKE_MODULE_PATH   ${CMAKE_MODULE_PATH}
            ${CMAKE_CURRENT_SOURCE_DIR}/../ext/Catch2/contrib)

    include(CTest)
    include(ParseAndAddCatchTests)

    file(GLOB testTopoFiles Interlocking/Test_ipopt.cpp)

#    file(GLOB testTopoFiles
#            Interlocking/*.cpp
#            CrossMesh/*.cpp
#            Mesh/*.cpp
#            IO/*.cpp
#            Utility/*.cpp
#            )

    set(testTopoFiles ${testTopoFiles} Test_Main.cpp)

    add_executable(testTopo ${testTopoFiles})

    target_link_libraries(testTopo PUBLIC Catch2::Catch2)
    target_link_libraries(testTopo PUBLIC TpCorelib)
    ParseAndAddCatchTests(testTopo)

    file(COPY ../data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()