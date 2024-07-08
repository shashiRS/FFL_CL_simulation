cmake_minimum_required (VERSION 3.1)

find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

FILE(GLOB_RECURSE  CORE_SOURCES *.cpp)
FILE(GLOB_RECURSE  CORE_HEADERS *.h)
FILE(GLOB_RECURSE  SRC "../../../jsoncpp/src/lib_json/*.cpp")
FILE(GLOB_RECURSE  CORE_FORMS *.ui)

# defining the target enforces use of compile definitions
QT5_WRAP_CPP(CORE_MOC_SRCS ${CORE_HEADERS} TARGET parking_visu_core_lib)
QT5_WRAP_UI(CORE_UI_HEADERS ${CORE_FORMS})

set(CORE_INCLUDE_DIRS
    src
    "../../../../conan_workarea/build.mf_tools.$<IF:$<CONFIG:Debug_Cus>,cus_only,generic>.4.0.0-fallback.vs2017_debug/cip_build/src/types"
    "../../../../contrib/cip/vs2017_debug/appl_srv_layer_types/appl_srv_layer_types/include"
    "../../../../contrib/cip/vs2017_debug/eco/eco/include/auxiliary"
    "../../../../contrib/cip/vs2017_debug/mf_common/mf_common/include"
    "../../../../contrib/cip/vs2017_debug/mf_common_types/mf_common_types/include"
    "../../../../contrib/cip/vs2017_debug/si_types/si_types/include"
    "../../../../contrib/cip/vs2017_debug/mf_trjpla_types/mf_trjpla_types/include"
    "../../../../contrib/cip/vs2017_debug/mf_trjpla_types/mf_trjpla_types/include/mf_trjpla_types"
    "../../../../../CopiedIncludes"
    "../../../../../jsoncpp/include"
    "../../../../../geoml/src/component"
    "../../../../../mf_trjpla/interface/component"
    "../../../../../mf_trjpla/src/component/include"
    "../../../../../mf_trjpla/tests/unit/generic/include"
    "../../../../../mf_taposd/interface/component"
    "../../../../../mf_taposd/src/component/include"
    "../../../../../plp_log/src/component/include"
)

add_definitions(
-DBUILD_MTS
-DSIMULATION_CDL_ADDRESSES
-DCML_CMATH_USE_STD_LIB
-DPLP_LOG_ENABLED
-DMOCKABLE_FUNCTION=
)

set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Gui PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")
set_target_properties(Qt5::Widgets PROPERTIES MAP_IMPORTED_CONFIG_DEBUG_CUS "DEBUG")

add_library(parking_visu_core_lib ${CORE_HEADERS} ${CORE_SOURCES} ${CORE_MOC_SRCS} ${CORE_UI_HEADERS})
target_link_libraries(parking_visu_core_lib Qt5::Core Qt5::Gui Qt5::Widgets)
target_include_directories(parking_visu_core_lib PUBLIC ${CORE_INCLUDE_DIRS})
target_compile_definitions(parking_visu_core_lib PUBLIC $<$<CONFIG:Debug_Cus>:${CUS_PREDEF}>)
