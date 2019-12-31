
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was libigl-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

include(${CMAKE_CURRENT_LIST_DIR}/libigl-export.cmake)

if (TARGET igl::core)
  if (NOT TARGET Eigen3::Eigen)
    find_package(Eigen3 QUIET)
    if (NOT Eigen3_FOUND)
      # try with PkgCOnfig
      find_package(PkgConfig REQUIRED)
      pkg_check_modules(Eigen3 QUIET IMPORTED_TARGET eigen3)
    endif()

    if (NOT Eigen3_FOUND)
      message(FATAL_ERROR "Could not find required dependency Eigen3")
      set(libigl_core_FOUND FALSE)
    else()
      target_link_libraries(igl::core INTERFACE PkgConfig::Eigen3)
      set(libigl_core_FOUND TRUE)
    endif()
  else()
    target_link_libraries(igl::core INTERFACE Eigen3::Eigen)
    set(libigl_core_FOUND TRUE)
  endif()

endif()

check_required_components(libigl)

