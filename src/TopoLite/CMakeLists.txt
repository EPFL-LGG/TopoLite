add_subdirectory(CrossMesh)
add_subdirectory(Interlocking)
add_subdirectory(IO)
add_subdirectory(Mesh)
add_subdirectory(Utility)
add_subdirectory(Structure)

set(TpCoreFiles
        ${CrossMeshFiles}
        ${InterlockingFiles}
        ${IOFiles}
        ${MeshFiles}
        ${UtilityFiles}
        ${StructureFiles}
        PARENT_SCOPE)