cmake_minimum_required (VERSION 3.1)

project(ParkingVisuClientInterface)

FILE(GLOB_RECURSE  VISUCLIENTINTERFACE_SOURCES *.cpp)
FILE(GLOB_RECURSE  VISUCLIENTINTERFACE_HEADERS *.h)

set(VISUCLIENTINTERFACE_INCLUDE_DIRS
    src
    "../../../../conan_workarea/build.mf_tools.$<IF:$<CONFIG:Debug_Cus>,cus_only,generic>.4.0.0-fallback.vs2017_debug/cip_build/src/types"
    "../../../../contrib"
    "../../../../contrib/cip/vs2017_debug/appl_srv_layer_types/appl_srv_layer_types/include"
    "../../../../contrib/cip/vs2017_debug/eco/eco/include/auxiliary"
    "../../../../contrib/cip/vs2017_debug/mf_common_types/mf_common_types/include"
    "../../../../contrib/cip/vs2017_debug/mf_trjpla_types/mf_trjpla_types/include"
    "../../../../contrib/cip/vs2017_debug/mf_trjpla_types/mf_trjpla_types/include/mf_trjpla_types"
    "../../../../contrib/cip/vs2017_debug/si_types/si_types/include"
    "../../../../../CopiedIncludes"
    "../../../../../jsoncpp/include"
    "../../../../../mf_common/interface/component"
    "../../../../../mf_trjpla/interface/component"
    "../../../../../mf_trjpla/src/component/include"
    "../../../../../plp_log/src/component/include"
)

add_definitions(
-DBUILD_MTS
-DSIMULATION_CDL_ADDRESSES
-DCML_CMATH_USE_STD_LIB
-DPLP_LOG_ENABLED
)

add_library(parking_visu_client_interface ${VISUCLIENTINTERFACE_SOURCES} ${VISUCLIENTINTERFACE_HEADERS} ${PARKING_CODE_SOURCES})
add_custom_command(
    TARGET parking_visu_client_interface
    PRE_BUILD
    COMMAND "${PYTHON_EXECUTABLE}" "copyInterfaceHeaders.py" $<IF:$<CONFIG:Debug_Cus>,--cus_only,--generic>
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../../scripts
    COMMENT "+++++++++++  execute copyInterfaceHeaders.py +++++++++++"
    VERBATIM
)
target_include_directories(parking_visu_client_interface PUBLIC ${VISUCLIENTINTERFACE_INCLUDE_DIRS})
target_compile_definitions(parking_visu_client_interface PUBLIC $<$<CONFIG:Debug_Cus>:${CUS_PREDEF}>)
