add_subdirectory(CrossMesh)
add_subdirectory(Interlocking)
add_subdirectory(IO)
add_subdirectory(Mesh)
add_subdirectory(Utility)
add_subdirectory(Utility_tests)

set(TpCoreFiles ${CrossMeshFiles} ${InterlockingFiles} ${IOFiles} ${MeshFiles} ${UtilityFiles} ${UtilityTestFiles} PARENT_SCOPE)