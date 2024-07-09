# general
cmake_minimum_required(VERSION 3.5)
project(SEF_Powertrain)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

if ((UNIX) AND (CIP_BUILD_SYSTEM_VERSION))
	find_package(carmaker REQUIRED)
endif()

if(NOT DEFINED CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Debug)
ENDIF()

# CARMAKER DIRECTORIES
IF(WIN32)
	SET(CARMAKER_INSTALL_DIR $ENV{IPGHOME})
	SET(ARCH win64)

ELSEIF(WIN64)
if (NOT CIP_BUILD_SYSTEM_VERSION)
	SET(CARMAKER_INSTALL_DIR $ENV{IPGHOME})
	SET(ARCH win64)
ENDIF()

ELSEIF(UNIX)
	if (CIP_BUILD_SYSTEM_VERSION)
		SET(CARMAKER_INSTALL_DIR ${carmaker_PACKAGE_PATH}/ipg)
	else()
		SET(CARMAKER_INSTALL_DIR /opt/ipg)
	endif()
	SET(ARCH linux64)
	set(CMAKE_GENERATOR_PLATFORM linux64)

ENDIF()

MACRO(SUBDIRLIST result curdir)
  FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
  SET(dirlist "")
  FOREACH(child ${children})
    IF(IS_DIRECTORY ${curdir}/${child})
      LIST(APPEND dirlist ${child})
    ENDIF()
  ENDFOREACH()
  SET(${result} ${dirlist})
ENDMACRO()

SUBDIRLIST(SUBDIRS  C:/cip_tools/carmaker/IPG/carmaker/)

message("subdirs: ${SUBDIRS}")
message("System environment variable: $ENV{IPGHOME}")

string(REPLACE "\\" "/" CARMAKER_INSTALL_DIR "${CARMAKER_INSTALL_DIR}")

IF(NOT DEFINED CARMAKER_VER)
SET(CARMAKER_VER 10.2.1)
ENDIF()

# CARMAKER VERSION
message("CarMaker version: ${CARMAKER_VER}")

IF(NOT DEFINED CM_NUMVER)

IF(CARMAKER_VER STREQUAL "10.2.0")
SET(CM_NUMVER 100200)
ELSEIF (CARMAKER_VER STREQUAL "10.2.1")
SET(CM_NUMVER 100201)
ELSE()
SET(CM_NUMVER)
ENDIF()

ENDIF()

SET(CARMAKER_DIR ${CARMAKER_INSTALL_DIR}/carmaker/${ARCH}-${CARMAKER_VER})
set (dirlist "")
message("CarMaker directory: ${CARMAKER_DIR}, ${CARMAKER_INSTALL_DIR}/carmaker/${ARCH}-${CARMAKER_VER}")
LIST(APPEND dirlist ${CARMAKER_DIR})
# Check if the mentioned CarMaker version exists
IF( NOT (EXISTS ${CARMAKER_DIR}))
message (FATAL_ERROR " CarMaker Version ${CARMAKER_VER} not installed on this PC. Modify the CarMaker version number in the CMakeLists.txt file or install the correct CarMaker version")
ENDIF()

# adjacent configuration
add_subdirectory(src/package/OCTAGONEngine)

IF(CEM_PROJECT) #build in a VSP-ROS2-SIL project environment with own CarMaker project does not require building SEF* standalone project
ELSE()
	add_subdirectory(tests/carmaker)
ENDIF()

# set documentation target
find_package(Doxygen)
if(Doxygen_FOUND)
	message("Doxygen found")
	doxygen_add_docs(DOCUMENTATION
	doc)
else()
	message("Doxygen not found")
endif()

# install documentation target to destination directory
install(
	DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/
	COMPONENT DOCUMENTATION
	DESTINATION doc/${PROJECT_NAME}
)
