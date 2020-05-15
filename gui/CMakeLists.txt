if(HAVE_NANOGUI)
    #for testing interlocking

    add_subdirectory(RenderObjects)
    add_subdirectory(Widget)

    add_executable(InterlockingGUI ui/gui_InterlockingMain.cpp ${FilesWidgets} ${FilesRenderObjects})
    target_link_libraries(InterlockingGUI nanogui)
    target_link_libraries(InterlockingGUI TpCorelib)
    target_include_directories(InterlockingGUI PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(InterlockingGUI PUBLIC ${NANOGUI_EXTRA_DEFS})
    target_compile_options(InterlockingGUI PRIVATE "/D_HAS_STD_BYTE=0")


    #for viewing topocreator
    add_executable(TopoCreator ui/gui_TopoCreatorMain.cpp ${FilesWidgets} ${FilesRenderObjects})
    target_link_libraries(TopoCreator nanogui)
    target_link_libraries(TopoCreator TpCorelib)
    target_include_directories(TopoCreator PUBLIC SYSTEM
            ${NANOGUI_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/RenderObjects
            ${CMAKE_CURRENT_SOURCE_DIR}/Widget)
    target_compile_definitions(TopoCreator PUBLIC ${NANOGUI_EXTRA_DEFS})

    file(COPY shader DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY ../data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()