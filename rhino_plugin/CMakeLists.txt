add_library(dllTopo )
target_link_libraries(StructureChecker nanogui)
target_link_libraries(StructureChecker TpCorelib)
    target_include_directories(StructureChecker PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}
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
            ${CMAKE_CURRENT_SOURCE_DIR}
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
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Utility
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(BezierTest PUBLIC ${NANOGUI_EXTRA_DEFS})


    ################################
    #        add defination
    ################################
    get_property(IPOPT_FOUND GLOBAL PROPERTY IPOPT_FOUND)
    if(IPOPT_FOUND)
        target_compile_definitions(StructureChecker PUBLIC -DIPOPT_INSTALLED)
    ENDIF()
endif()
