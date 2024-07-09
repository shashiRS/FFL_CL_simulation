# general
cmake_minimum_required(VERSION 3.5)
project(SEF_Brake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "" FORCE)

if ((UNIX) AND (CIP_BUILD_SYSTEM_VERSION))
	find_package(carmaker REQUIRED)
endif()

message("###########SEF Brake variant###########: ${SEF_BRAKE}")
message("###########CM binary build###########: ${CM_BUILD}")
message("###########Enabled MoCo_Brake###########: ${MoCo_Brake}")
message("###########Enabled CT Test###########: ${Enable_CT}")

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

string(REPLACE "\\" "/" CARMAKER_INSTALL_DIR "${CARMAKER_INSTALL_DIR}")

# CARMAKER VERSION
message("Carmaker Version ${CARMAKER_VER}")
IF(NOT DEFINED CARMAKER_VER)
SET(CARMAKER_VER 10.2.1)
ENDIF()

IF(NOT DEFINED CM_NUMVER)

IF(CARMAKER_VER STREQUAL "10.2.0")
SET(CM_NUMVER 100200)
ELSEIF (CARMAKER_VER STREQUAL "10.2.1")
SET(CM_NUMVER 100201)
ELSE()
SET(CM_NUMVER)
ENDIF()

ENDIF()

message("Ist IPGHOME env variable set: $ENV{IPGHOME}")
message("carmaker install directory: ${CARMAKER_INSTALL_DIR}")
#IF(NOT (EXISTS ${CARMAKER_INSTALL_DIR}))
#message("Overwrite carmaker installation directory")
#	SET(CARMAKER_INSTALL_DIR "C:/cip_tools/carmaker/IPG")
#ENDIF()


SET(CARMAKER_DIR ${CARMAKER_INSTALL_DIR}/carmaker/${ARCH}-${CARMAKER_VER})
message(-----------------------------${CARMAKER_DIR}--------------------)

# Check if the mentioned CarMaker version exists
IF( NOT (EXISTS ${CARMAKER_DIR}))
	message (FATAL_ERROR " CarMaker Version ${CARMAKER_VER} not installed on this PC. Modify the CarMaker version number in the CMakeLists.txt file or install the correct CarMaker version")
ENDIF()

# build for MoCo Brake
IF(MoCo_Brake)
	add_subdirectory(src/package/MoCo_Brake)
	add_subdirectory(src/package/MoCo_Brake/slprj/CarMaker)
ELSE()
ENDIF()

# build for OCTAGON Brake
add_subdirectory(src/package/OCTAGONBrake)

# build in a CLS-SIL project environment with own CarMaker project does not require building SEF* standalone project
if(CM_BUILD)
	add_subdirectory(tests/carmaker)
endif()

# "Postinstall" for running the Smoke Test.
# "add_subdirectory" is executed in order, therefore this one must be last directory added.
if (Enable_CT)
add_subdirectory(
    ${CMAKE_SOURCE_DIR}/tests/postinstall
	)
endif()

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
