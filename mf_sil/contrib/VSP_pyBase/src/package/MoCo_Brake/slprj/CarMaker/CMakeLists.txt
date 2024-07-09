cmake_minimum_required(VERSION 3.5)
project(mdl_MoCo_Brake)


# CARMAKER & MATLAB VERSIONS
SET(MATLAB_VER R2019b)
SET(MATLAB_DIR ../../../VSP_Matlab_Dependencies/${MATLAB_VER})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

SET(CARMAKER_INC_DIR ${CARMAKER_DIR}/include)


IF(WIN32)
	ADD_COMPILE_DEFINITIONS(WIN32 WIN64 MATSUPP_MATVER=R2019b MATSUPP_NUMVER=90700 MODEL=mdl_MoCo_Brake NUMST=2 NCSTATES=7
		HAVESTDIO RT USE_RTMODEL ALLOCATIONFCN=0 TID01EQ=1 MAT_FILE=0 ONESTEPFCN=1 TERMFCN=1 MULTI_INSTANCE_CODE=0 INTEGER_CODE=0 MT=0
		_CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS CM_NUMVER=${CM_NUMVER})
	ADD_COMPILE_OPTIONS(/wd4251)
	ADD_LINK_OPTIONS(-SAFESEH:NO)
ELSEIF(UNIX)
	ADD_COMPILE_DEFINITIONS(LINUX LINUX64 _GNU_SOURCE _FILE_OFFSET_BITS=64 CM_NUMVER=${CM_NUMVER})
	ADD_COMPILE_OPTIONS(-O3 -m64 -fPIC -Wall -march=native)
	ADD_LINK_OPTIONS(-m64 -fPIC)
ENDIF()

# source files of the executable
add_library(${PROJECT_NAME} STATIC
	mdl_MoCo_Brake/mdl_MoCo_Brake.c
	mdl_MoCo_Brake/mdl_MoCo_Brake_capi.c
	_sharedutils/rtGetInf.c
	_sharedutils/rtGetNaN.c
	_sharedutils/rt_TDelayInterpolate.c
	_sharedutils/rt_nonfinite.c
	_sharedutils/rt_zcfcn.c

)

# Specify path to internal CarMaker interfaces
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}
	PUBLIC
		_sharedutils
		../../
		${MATLAB_DIR}/simulink/include
		${MATLAB_DIR}/extern/include
		${MATLAB_DIR}/rtw/c/src
		${MATLAB_DIR}/rtw/c/src/ext_mode/common
		${MATLAB_DIR}/rtw/c/libsrc
		${CARMAKER_DIR}/Matlab/${MATLAB_VER}
		${CARMAKER_DIR}/include/Car
		${CARMAKER_DIR}/include
		${CARMAKER_DIR}/Matlab/${MATLAB_VER}/include)
 
# group source files for IDE generation
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR})
 
# install target to destination directory
install(
    TARGETS ${PROJECT_NAME}
    COMPONENT TARGET_COMPONENT
    DESTINATION bin/${PROJECT_NAME}
)