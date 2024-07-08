cmake_minimum_required (VERSION 3.1)

project(ParkingVisuUserLib)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

FILE(GLOB_RECURSE  VISULIB_SOURCES *.cpp)
FILE(GLOB_RECURSE  VISULIB_HEADERS *.h)

QT5_WRAP_CPP(VISULIB_MOC_SRCS  ${VISULIB_HEADERS})

set(PARKING_CODE_SOURCES
    ../../../../../jsoncpp/src/lib_json/json_reader.cpp
    ../../../../../jsoncpp/src/lib_json/json_value.cpp
    ../../../../../jsoncpp/src/lib_json/json_writer.cpp
    ../../../../../mf_trjpla/src/component/PlanningObject.cpp
    ../../../../../geoml/src/component/Ellipse.cpp
    ../../../../../geoml/src/component/LSM_Math.cpp
)

set(VISULIB_INCLUDE_DIRS
        src
        "../../../../../geoml/interface/component"
        "../../../../../geoml/interface/cml"
        "../../../../../mf_trjctl/src/component/include"
        "../../../../../mf_trjctl/interface/component"
)

add_definitions(
-DBUILD_MTS
-DSIMULATION_CDL_ADDRESSES
-DCML_CMATH_USE_STD_LIB
-DPLP_LOG_ENABLED
)

# set additional cus_only preprocessor definitions
set(CUS_PREDEF
    ULTRASONIC_ONLY
)

set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Gui PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Widgets PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")

add_library(parking_visu_user_lib ${VISULIB_SOURCES} ${VISULIB_HEADERS} ${PARKING_CODE_SOURCES} ${VISULIB_MOC_SRCS})
target_link_libraries(parking_visu_user_lib parking_visu_core_lib Qt5::Core Qt5::Gui Qt5::Widgets)
target_include_directories(parking_visu_user_lib PUBLIC ${VISULIB_INCLUDE_DIRS})
target_compile_definitions(parking_visu_user_lib PUBLIC $<$<CONFIG:Debug_Cus>:${CUS_PREDEF}>)
