cmake_minimum_required(VERSION 3.5)
project(rtwshared)

# MATLAB VERSION
SET(MATLAB_VER R2019b)
SET(MATLAB_DIR ../../../../VSP_Matlab_Dependencies/${MATLAB_VER})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

SET(CARMAKER_INC_DIR ${CARMAKER_DIR}/include)


IF(WIN32)
	ADD_COMPILE_DEFINITIONS(WIN32 WIN64 _CRT_SECURE_NO_WARNINGS _CRT_NONSTDC_NO_DEPRECATE _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
	ADD_COMPILE_OPTIONS(/wd4251)
	ADD_LINK_OPTIONS(-SAFESEH:NO)
ELSEIF(UNIX)
	ADD_COMPILE_DEFINITIONS(LINUX LINUX64 _GNU_SOURCE _FILE_OFFSET_BITS=64)
	ADD_COMPILE_OPTIONS(-O3 -m64 -fPIC -Wall -march=native)
	ADD_LINK_OPTIONS(-m64 -fPIC)
ENDIF()

# source files of the executable
add_library(${PROJECT_NAME} STATIC
	rtGetInf.c
	rtGetNaN.c
	rt_TDelayInterpolate.c
	rt_nonfinite.c
	rt_zcfcn.c

)

# Specify path to internal CarMaker interfaces
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME}
	PUBLIC
		${MATLAB_DIR}/simulink/include
		${MATLAB_DIR}/extern/include
		${MATLAB_DIR}/rtw/c/src
		${MATLAB_DIR}/rtw/c/src/ext_mode/common
		)
 
# group source files for IDE generation
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR})
 
# install target to destination directory
install(
    TARGETS ${PROJECT_NAME}
    COMPONENT TARGET_COMPONENT
    DESTINATION bin/${PROJECT_NAME}
)