if(BUILD_TOPOTEST)

    set(CMAKE_MODULE_PATH   ${CMAKE_MODULE_PATH}
            ${CMAKE_CURRENT_SOURCE_DIR}/../ext/Catch2/contrib)

    include(CTest)
    include(ParseAndAddCatchTests)

    file(GLOB testTopoFiles
            Interlocking/*.*
            CrossMesh/*.*
            Mesh/*.*
            IO/*.*
            Utility/*.*
            )

    file(GLOB singleTestFile
           #Interlocking/Problem_LP_simple.cpp
           #Interlocking/Problem_LP_bigm.cpp
            # Interlocking/Test_toymodel_Ipopt.cpp
           #Interlocking/Test_interlockingSolver_Clp.cpp
#            Utility/Test_SparseOperations.cpp
            IO/Test_JsonIO.cpp
            )

    add_executable(testTopo ${singleTestFile} Test_Main.cpp)

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
