cmake_minimum_required (VERSION 3.1)

project(ParkingVisuApp)

find_package(Qt5Charts REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5DataVisualization REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

FILE(GLOB_RECURSE  APP_SOURCES *.cpp)
FILE(GLOB_RECURSE  APP_HEADERS *.h)
FILE(GLOB_RECURSE  APP_FORMS *.ui)

SET(UI_RESOURCES
    resources.qrc
)

QT5_WRAP_CPP(APP_MOC_SRCS  ${APP_HEADERS})
QT5_WRAP_UI(APP_UI_HEADERS ${APP_FORMS})
QT5_ADD_RESOURCES(UI_RESOURCES_RCC ${UI_RESOURCES})

set(PARKING_CODE_SOURCES
    ../../../../../jsoncpp/src/lib_json/json_reader.cpp
    ../../../../../jsoncpp/src/lib_json/json_value.cpp
    ../../../../../jsoncpp/src/lib_json/json_writer.cpp
    ../../../../../geoml/src/component/AxisAlignedRectangular2D.cpp
    ../../../../../geoml/src/component/Ellipse.cpp
    ../../../../../geoml/src/component/LineSegment.cpp
    ../../../../../geoml/src/component/LSM_Math.cpp
    ../../../../../geoml/src/component/LSM_LookupTable.cpp
    ../../../../../mf_trjpla/src/component/CollisionDetectorBase.cpp
    ../../../../../mf_trjpla/src/component/DrivenPath.cpp
    ../../../../../mf_trjpla/src/component/GeometricPath.cpp
    ../../../../../mf_trjpla/src/component/GeometricPathTypes.cpp
    ../../../../../mf_trjpla/src/component/PlanningEgoVehicle.cpp
    ../../../../../mf_trjpla/src/component/PlanningObject.cpp
    ../../../../../mf_trjpla/src/component/PlanningTargetPose.cpp
    ../../../../../mf_trjpla/src/component/ParkingPath.cpp
    ../../../../../mf_trjpla/src/component/PathSampler.cpp
    ../../../../../mf_trjpla/src/component/TrjPlaMapEvaluatorBase.cpp
    ../../../../../mf_trjpla/src/component/TrjPlaMath.cpp
    ../../../../../mf_trjpla/src/component/TrjPlaPlpLogAdapter.cpp
    ../../../../../plp_log/src/component/log_config.cpp
    ../../../../../plp_log/src/component/log_exitadapter.cpp
    ../../../../../plp_log/src/component/log_filteredadapter.cpp
    ../../../../../plp_log/src/component/log_filteredconsoleadapter.cpp
    ../../../../../plp_log/src/component/log_logger.cpp
    ../../../../../plp_log/src/component/log_logif.cpp
)

add_definitions(
-DSIMULATION_CDL_ADDRESSES
-DBUILD_MTS
-DCML_CMATH_USE_STD_LIB
-DAP_DEBUG_DRAW_ENABLE
-DPLP_LOG_ENABLED
-DMOCKABLE_FUNCTION=
)

set_target_properties(Qt5::Charts PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::DataVisualization PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Gui PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Network PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Widgets PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")

add_executable(parking_visu_app ${APP_SOURCES} ${APP_HEADERS} ${APP_MOC_SRCS} ${APP_UI_HEADERS} ${PARKING_CODE_SOURCES} ${UI_RESOURCES_RCC} appicon.rc)
target_compile_definitions(parking_visu_app PUBLIC $<$<CONFIG:Debug_Cus>:${CUS_PREDEF}>)
target_link_libraries(parking_visu_app parking_visu_core_lib parking_visu_client_lib 
                      ../../../../contrib/cip/vs2017_debug/mf_common/mf_common/lib/consts 
                      Qt5::Charts Qt5::Core Qt5::DataVisualization Qt5::Gui Qt5::Network Qt5::Widgets)
