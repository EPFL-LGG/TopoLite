add_subdirectory(CrossMesh)
add_subdirectory(Interlocking)
add_subdirectory(IO)
add_subdirectory(Mesh)
add_subdirectory(Utility)

set(TpCoreFiles
    ${CrossMeshFiles}
    ${InterlockingFiles}
    ${IOFiles}
    ${MeshFiles}
    ${UtilityFiles}
    PARENT_SCOPE
    )