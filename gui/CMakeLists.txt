if(BUILD_TOPOGUI)
    #for testing interlocking

    add_subdirectory(RenderObjects)
    add_subdirectory(Widget)
    add_subdirectory(Utility)

    add_executable(InterlockingGUI ui/gui_InterlockingMain.cpp ${FilesWidgets} ${FilesRenderObjects} ${FilesUtility})
    target_link_libraries(InterlockingGUI nanogui)
    target_link_libraries(InterlockingGUI TpCorelib)
    target_include_directories(InterlockingGUI PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Utility
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(InterlockingGUI PUBLIC ${NANOGUI_EXTRA_DEFS})


    #for viewing topocreator
    add_executable(TopoCreator ui/gui_TopoCreatorMain.cpp ${FilesWidgets} ${FilesRenderObjects} ${FilesUtility})
    target_link_libraries(TopoCreator nanogui)
    target_link_libraries(TopoCreator TpCorelib)
    target_include_directories(TopoCreator PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Utility
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(TopoCreator PUBLIC ${NANOGUI_EXTRA_DEFS})


    file(COPY shader DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY ../data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    if(CMAKE_GENERATOR STREQUAL Xcode)
        target_compile_definitions(TopoCreator PUBLIC -DTOPOCREATOR_SHADER_PATH="../shader/")
        target_compile_definitions(InterlockingGUI PUBLIC -DTOPOCREATOR_SHADER_PATH="../shader/")
    else()
        target_compile_definitions(TopoCreator PUBLIC -DTOPOCREATOR_SHADER_PATH="shader/")
        target_compile_definitions(InterlockingGUI PUBLIC -DTOPOCREATOR_SHADER_PATH="shader/")
    endif()
endif()
