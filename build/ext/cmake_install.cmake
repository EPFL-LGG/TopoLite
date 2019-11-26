# Install script for directory: /Users/ziqwang/Documents/GitHub/TopoLite/ext

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/libigl/cmake" TYPE FILE FILES "/Users/ziqwang/Documents/GitHub/TopoLite/build/libigl-config.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/libigl/cmake/libigl-export.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/libigl/cmake/libigl-export.cmake"
         "/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/CMakeFiles/Export/share/libigl/cmake/libigl-export.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/libigl/cmake/libigl-export-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/libigl/cmake/libigl-export.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/libigl/cmake" TYPE FILE FILES "/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/CMakeFiles/Export/share/libigl/cmake/libigl-export.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/pugixml/cmake_install.cmake")
  include("/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/quickhull/cmake_install.cmake")
  include("/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/clipper/cmake_install.cmake")
  include("/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/triangle/cmake_install.cmake")
  include("/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/shapeop/cmake_install.cmake")
  include("/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/tbb/cmake_install.cmake")
  include("/Users/ziqwang/Documents/GitHub/TopoLite/build/ext/nanogui/cmake_install.cmake")

endif()

