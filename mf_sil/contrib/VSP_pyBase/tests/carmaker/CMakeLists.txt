cmake_minimum_required(VERSION 3.5)
project(carmaker)

if (UNIX)
	set(CMAKE_FIND_LIBRARY_SUFFIXES .so.4.4.4)
	find_library(LIBUSB REQUIRED
		NAMES
			usb-0.1
	)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
#MATLAB VERSION
SET(MATLAB_VER R2019b)

SET(CARMAKER_BIN_DIR ${CARMAKER_DIR}/bin)
SET(CARMAKER_LIB_DIR ${CARMAKER_DIR}/lib)
SET(CARMAKER_INC_DIR ${CARMAKER_DIR}/include)

IF(WIN32)
	ADD_COMPILE_DEFINITIONS(WIN32 WIN64 CM_CMAKE_AUTOCODE _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS CM_NUMVER=${CM_NUMVER})
	ADD_COMPILE_OPTIONS(/wd4251)
	ADD_LINK_OPTIONS(-SAFESEH:NO)
ELSEIF(UNIX)
	ADD_COMPILE_DEFINITIONS(LINUX LINUX64 CM_CMAKE_AUTOCODE _GNU_SOURCE _FILE_OFFSET_BITS=64 CM_NUMVER=${CM_NUMVER})
	ADD_COMPILE_OPTIONS(-O3 -m64 -fPIC -Wall -march=native)
	ADD_LINK_OPTIONS(-m64 -fPIC)
ENDIF()

SET(CARMAKER_LIB ${CARMAKER_LIB_DIR}/libcarmaker.a)
SET(CAR_LIB ${CARMAKER_LIB_DIR}/libcar.a)
SET(DRIVER_LIB ${CARMAKER_LIB_DIR}/libipgdriver.a)
SET(ROAD_LIB ${CARMAKER_LIB_DIR}/libipgroad.a)
SET(TIRE_LIB ${CARMAKER_LIB_DIR}/libipgtire.a)
SET(Z_LIB ${CARMAKER_LIB_DIR}/libz-${ARCH}.a)
SET(MATSUPP_LIB ${CARMAKER_DIR}/Matlab/${MATLAB_VER}/libmatsupp-${ARCH}.a)

IF(MoCo_Brake)
	add_definitions(-DMoCo_Brake_Enable)
	if(WIN32)
		SET(MoCo_Brake_LIB_1 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/package/MoCo_Brake/bin/Debug/MoCo_Brake.lib)
	elseif(UNIX)
		SET(MoCo_Brake_LIB_1 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/package/MoCo_Brake/bin/libMoCo_Brake.a)
	endif()

	if(WIN32)
		SET(MoCo_Brake_LIB_2 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/package/MoCo_Brake/slprj/CarMaker/bin/Debug/mdl_MoCo_Brake.lib)
	elseif(UNIX)
		SET(MoCo_Brake_LIB_2 ${CMAKE_CURRENT_SOURCE_DIR}/../../src/package/MoCo_Brake/slprj/CarMaker/bin/libmdl_MoCo_Brake.a)
	endif()
ENDIF(MoCo_Brake)

# Create executable from CarMaker sources
ADD_EXECUTABLE(${PROJECT_NAME}.${ARCH}
	src/CM_Main.c
	src/CM_Vehicle.c
	src/User.h
	src/User.c
	src/app_tmp.c

	${SIM_CONFIG_FILE}
	../../src/package/OCTAGONBrake/OCTAGONBrake.h
	../../src/package/OCTAGONBrake/OCTAGONBrake.cpp
)

ADD_CUSTOM_COMMAND(
	OUTPUT src/app_tmp.c
	COMMENT "Creating app_tmp.c"
	COMMAND ${CARMAKER_BIN_DIR}/CreateCarMakerAppInfo =av VSP_VirtualSimPlatform =sv ${CARMAKER_VER} =arch ${ARCH} =libs ${CARMAKER_LIB} ${CAR_LIB} ${DRIVER_LIB} ${ROAD_LIB} ${TIRE_LIB} ${MATSUPP_LIB} =o app_tmp.c
	COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/src/app_tmp.c ${CMAKE_CURRENT_BINARY_DIR}/src/app_tmp.c
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src)

ADD_CUSTOM_COMMAND(
	TARGET ${PROJECT_NAME}.${ARCH}
	POST_BUILD
	COMMENT "Deleting app_tmp.c"
	COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_CURRENT_SOURCE_DIR}/src/app_tmp.c)

# Specify path to internal CarMaker interfaces
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}.${ARCH}
	PUBLIC
		$<BUILD_INTERFACE:${CARMAKER_INC_DIR}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/>
)

# Link CarMaker libraries to executable
IF(MoCo_Brake)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}.${ARCH}
		${CAR_LIB}
		${MATSUPP_LIB}
		${CARMAKER_LIB}
		${DRIVER_LIB}
		${ROAD_LIB}
		${TIRE_LIB}
		${Z_LIB}
		${MoCo_Brake_LIB_1}
		${MoCo_Brake_LIB_2}
		${MATSUPP_LIB} #duplicate because of circular dependencies (will be analyzed later)
)

ELSE ()
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}.${ARCH} ${CAR_LIB} ${CARMAKER_LIB} ${DRIVER_LIB} ${ROAD_LIB} ${TIRE_LIB} ${MATSUPP_LIB} ${Z_LIB})
ENDIF()

IF(WIN32)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}.${ARCH} ${CARMAKER_LIB_DIR}/libcompat_${ARCH}.a user32.lib advapi32.lib ws2_32.lib legacy_stdio_definitions.lib winspool.lib ${CARMAKER_LIB_DIR}/liburiparser-${ARCH}.a)
ELSEIF(UNIX)
	TARGET_LINK_LIBRARIES(${PROJECT_NAME}.${ARCH}
		pthread
		dl
		m
		rt
		${LIBUSB}
		${CARMAKER_LIB_DIR}/liburiparser-${ARCH}.a
)
ENDIF()

install(TARGETS ${PROJECT_NAME}.${ARCH}
	COMPONENT TARGET_COMPONENT
	DESTINATION bin/${PROJECT_NAME}
)
