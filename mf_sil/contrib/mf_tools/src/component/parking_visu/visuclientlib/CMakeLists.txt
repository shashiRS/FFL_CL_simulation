cmake_minimum_required (VERSION 3.1)

project(ParkingVisuClientLib)

find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Network CONFIG REQUIRED)

FILE(GLOB_RECURSE  VISUCLIENTLIB_SOURCES *.cpp)
FILE(GLOB_RECURSE  VISUCLIENTLIB_HEADERS *.h)

QT5_WRAP_CPP(VISUCLIENTLIB_MOC_SRCS  ${VISUCLIENTLIB_HEADERS})

set(PARKING_CODE_SOURCES
    ../../../../../jsoncpp/src/lib_json/json_reader.cpp
    ../../../../../jsoncpp/src/lib_json/json_value.cpp
    ../../../../../jsoncpp/src/lib_json/json_writer.cpp
    ../../../../../mf_common/src/component/EgoVehicle.cpp
    ../../../../../mf_common/src/component/HelperFunctions.cpp
    ../../../../../geoml/src/component/Ellipse.cpp
    ../../../../../geoml/src/component/LSM_Math.cpp
    ../../../../../geoml/src/component/LSM_LookupTable.cpp
    ../../../../../mf_trjpla/src/component/PathSampler.cpp
    ../../../../../mf_trjpla/src/component/PlanningObject.cpp
    ../../../../../mf_trjpla/src/component/TrjPlaPlpLogAdapter.cpp
)

set(CORE_INCLUDE_DIRS
    src
    "../../../../contrib/cip/vs2017_debug/mf_common/mf_common/include"
    "../../../../../geoml/interface/component"
    "../../../../../geoml/interface/cml"
    "../../../../../mf_trjpla/interface/platform"
    "../../../../../mf_trjpla/tests/unit/generic/include"
    "../../../../../mf_taposd/src/component/include"
    "../../../../../mf_taposd/interface/component"
)

add_definitions(
-DBUILD_MTS
-DSIMULATION_CDL_ADDRESSES
-DCML_CMATH_USE_STD_LIB
-DAP_DEBUG_DRAW_ENABLE
-DPLP_LOG_ENABLED
-DMOCKABLE_FUNCTION=
)

set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Network PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")

add_library(parking_visu_client_lib ${VISUCLIENTLIB_SOURCES} ${VISUCLIENTLIB_HEADERS} ${PARKING_CODE_SOURCES} ${VISUCLIENTLIB_MOC_SRCS})
target_link_libraries(parking_visu_client_lib parking_visu_client_interface Qt5::Core Qt5::Network)
target_include_directories(parking_visu_client_lib PUBLIC ${CORE_INCLUDE_DIRS})
target_compile_definitions(parking_visu_client_lib PUBLIC $<$<CONFIG:Debug_Cus>:${CUS_PREDEF}>)
