if(BUILD_TOPOGUI)
    #for testing interlocking

    add_subdirectory(RenderObjects)
    add_subdirectory(Widget)
    add_subdirectory(Utility)
    add_subdirectory(UI)

    ################################
    #       Structure Checker
    ################################
    add_executable(StructureChecker ${StructureCheckerFiles} ${WidgetFiles} ${RenderObjectsFiles} ${UtilityFiles})
    target_link_libraries(StructureChecker nanogui)
    target_link_libraries(StructureChecker TpCorelib)
    target_include_directories(StructureChecker PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Utility
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(StructureChecker PUBLIC ${NANOGUI_EXTRA_DEFS})


    ################################
    #       TopoCreator
    ################################
    add_executable(TopoCreator ${TopoCreatorFiles} ${WidgetFiles} ${RenderObjectsFiles} ${UtilityFiles})
    target_link_libraries(TopoCreator nanogui)
    target_link_libraries(TopoCreator TpCorelib)
    target_include_directories(TopoCreator PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Utility
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(TopoCreator PUBLIC ${NANOGUI_EXTRA_DEFS})

    ################################
    #           BezierTest
    ################################
    add_executable(BezierTest ${BezierTestFiles} ${FilesWidgets} ${FilesRenderObjects} ${FilesUtility})
    target_link_libraries(BezierTest nanogui)
    target_link_libraries(BezierTest TpCorelib)
    target_include_directories(BezierTest PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Utility
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(BezierTest PUBLIC ${NANOGUI_EXTRA_DEFS})


    ################################
    #           Copy files
    ################################
    file(COPY shader DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY ../data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    if(CMAKE_GENERATOR STREQUAL Xcode)
        target_compile_definitions(TopoCreator PUBLIC -DTOPOCREATOR_SHADER_PATH="../shader/")
        target_compile_definitions(StructureChecker PUBLIC -DTOPOCREATOR_SHADER_PATH="../shader/")
        target_compile_definitions(BezierTest PUBLIC -DTOPOCREATOR_SHADER_PATH="../shader/")
    else()
        target_compile_definitions(TopoCreator PUBLIC -DTOPOCREATOR_SHADER_PATH="shader/")
        target_compile_definitions(StructureChecker PUBLIC -DTOPOCREATOR_SHADER_PATH="shader/")
        target_compile_definitions(BezierTest PUBLIC -DTOPOCREATOR_SHADER_PATH="shader/")
    endif()

    get_property(IPOPT_FOUND GLOBAL PROPERTY IPOPT_FOUND)
    if(IPOPT_FOUND)
        target_compile_definitions(StructureChecker PUBLIC -DIPOPT_INSTALLED)
    ENDIF()
endif()
