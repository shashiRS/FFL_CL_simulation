find_package(eco REQUIRED)
find_package(mf_common REQUIRED)
find_package(mf_common_types REQUIRED)
find_package(mf_parksm_core_types REQUIRED)
find_package(mf_taposd REQUIRED)
find_package(mf_trjpla REQUIRED)
find_package(mf_trjpla_types REQUIRED)
find_package(mf_trjctl_types REQUIRED)
find_package(mf_vedodo_types REQUIRED)
find_package(mf_manager_types REQUIRED)
find_package(plp_log REQUIRED)
find_package(si_types REQUIRED)
find_package(geoml REQUIRED)
find_package(appl_srv_layer_types REQUIRED)
find_package(vehicle_types REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    codegen_types
    mf_common::mf_common
    mf_common_types::mf_common_types
    mf_parksm_core_types::mf_parksm_core_types
    mf_taposd::mf_taposd
    mf_trjpla::mf_trjpla
    mf_trjpla_types::mf_trjpla_types
    mf_trjctl_types::mf_trjctl_types
    mf_vedodo_types::mf_vedodo_types
    mf_manager_types::mf_manager_types
    plp_log::log
    plp_log::log_if
    si_types::si_types
    geoml::geoml
)

set(PLANTUML_FILES
    "${eco_PACKAGE_PATH}/tools2build/PlantUML/macros/framework_types.plantuml"                  # eco basis types
    "${mf_trjpla_types_PACKAGE_PATH}/plantuml/types/AP_TRJPLA_includes.plantuml"                # input port: AP_TP.TargetPosesPort
    "${mf_trjctl_types_PACKAGE_PATH}/plantuml/types/AP_TRJCTL_includes.plantuml"                # input port: AP_TRJCTL.MFControlStatusPort
    "${si_types_PACKAGE_PATH}/plantuml/types/SI_includes.plantuml"                              # input ports: SI.EgoMotionPort, SI.ApEnvModelPort
    "${vehicle_types_PACKAGE_PATH}/plantuml/types/AP_CommonVehSigProvider_includes.plantuml"    # input port: AP_CommonVehSigProvider.PARKSMCoreSampleTimePort
    "${mf_parksm_core_types_PACKAGE_PATH}/plantuml/types/AP_PARKSM_Core_includes.plantuml"      # all output ports
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types/com_includes.plantuml"                 # for component errors: com.ComResult
    "${mf_common_PACKAGE_PATH}/plantuml/types/MF_COMMON_CONSTANTS_includes.plantuml"
)

set(PLANTUML_INCLUDES
    # include path for data structure used for SET_COMPONENT_ERRORS
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types"
    "${mf_trjpla_types_PACKAGE_PATH}/plantuml/types"
    "${mf_trjctl_types_PACKAGE_PATH}/plantuml/types"
    "${si_types_PACKAGE_PATH}/plantuml/types"
    "${vehicle_types_PACKAGE_PATH}/plantuml/types"
    "${mf_parksm_core_types_PACKAGE_PATH}/plantuml/types"
    # include path for nested data structures
    "${mf_common_types_PACKAGE_PATH}/plantuml/types"
    "${mf_common_PACKAGE_PATH}/plantuml/types"
    # include path of own PlantUMLs
    "${PROJECT_SOURCE_DIR}/interface/plantuml"
)

set(COBOLT_GENERATOR_DIRECTORY ${CMAKE_BINARY_DIR})
set(ADDITIONAL_GENERATOR_PLATFORM) 

convertPlantUmlToTypes(
  INPUTPLANTUMLFILES ${PLANTUML_FILES}
  PLANTUMLINCLUDES ${PLANTUML_INCLUDES}
  CODEGENFOLDER ${COBOLT_GENERATOR_DIRECTORY}
  PLATFORM ${ADDITIONAL_GENERATOR_PLATFORM}
  LOGFILE ${COBOLT_GENERATOR_DIRECTORY}/log/convertPlantUmlToTypes.log
)

installTypes(
  LIBS types
  CODEGENFOLDER ${COBOLT_GENERATOR_DIRECTORY}
)

install(
  TARGETS ${PROJECT_NAME}
  COMPONENT TARGET_COMPONENT
  DESTINATION lib
  INCLUDES DESTINATION include
)


