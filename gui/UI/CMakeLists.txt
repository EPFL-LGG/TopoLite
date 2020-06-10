add_subdirectory(StructureChecker)
add_subdirectory(Test)
add_subdirectory(TopoCreator)

set(StructureCheckerFiles ${StructureCheckerFiles} PARENT_SCOPE)
set(BezierTestFiles ${BezierTestFiles} PARENT_SCOPE)
set(TopoCreatorFiles ${TopoCreatorFiles} PARENT_SCOPE)