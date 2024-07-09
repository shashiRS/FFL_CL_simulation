# Install script for directory: D:/FFL_Closed_loop/mf_sil

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "D:/FFL_Closed_loop/mf_sil/conan_workarea/package.mf_sil.entry.6.0.0-fallback.vs2017_debug")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "PACKAGE_INFO" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/mf_sil/share/cmake/mf_sil" TYPE FILE FILES "D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/mf_sil-config.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "PACKAGE_INFO" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/mf_sil/share/cmake/mf_sil" TYPE FILE FILES "D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/mf_sil-config-version.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "PACKAGE_INFO" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/.bricks" TYPE FILE OPTIONAL FILES "D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/compile_commands.json")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "DOCUMENTATION" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/doc/mf_sil" TYPE DIRECTORY FILES "D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/html")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "__BRICKS_IMPLICIT_DEPENDENCIES" OR NOT CMAKE_INSTALL_COMPONENT)
  include("D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/.bricks/d/install_dependencies.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/tests/SIL/CarMaker/src/cmake_install.cmake")
  include("D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/conf/package/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "D:/FFL_Closed_loop/mf_sil/conan_workarea/h/be22c1aef4d90e3a0b6d3e47b4a92643/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
