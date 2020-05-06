file(GLOB InterlockingFiles ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
set(InterlockingFiles ${InterlockingFiles} ${CMAKE_CURRENT_SOURCE_DIR}/Ipopt_Interlocking_Problem.cpp)
set(InterlockingFiles ${InterlockingFiles} PARENT_SCOPE)