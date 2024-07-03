if(${ULTRASONIC_ONLY})
    set(VARIANT "ultrasonic")
else()
    set(VARIANT "premium")
endif()

set(PLANTUML_FILES
##  Get the PlantUMLs for data structures used by the ports
    "${eco_PACKAGE_PATH}/tools2build/PlantUML/macros/framework_types.plantuml"              # eco basis types
## inputs
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types/com_includes.plantuml"
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types/cfg_mgr_includes.plantuml"
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types/cml_includes.plantuml"
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types/ecu_ctrl_includes.plantuml"
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types/lsm_geoml_includes.plantuml"
    "${mf_common_types_PACKAGE_PATH}/plantuml/types/MF_COMMON_includes.plantuml"
    "${vehicle_types_PACKAGE_PATH}/plantuml/types/AP_CommonVehSigProvider_includes.plantuml"
    "${parking_hmi_types_PACKAGE_PATH}/plantuml/types/AP_HMIHUD_includes.plantuml"
    "${parking_hmi_types_PACKAGE_PATH}/plantuml/types/AP_HMIREM_includes.plantuml"
    "${dmc_types_PACKAGE_PATH}/plantuml/types/AP_LADMC_includes.plantuml"
    "${dmc_types_PACKAGE_PATH}/plantuml/types/AP_LODMC_includes.plantuml"
    "${vehicle_types_PACKAGE_PATH}/plantuml/types/AP_VehStateSigProvider_includes.plantuml"
    "${mf_pdwarnproc_types_PACKAGE_PATH}/plantuml/types/PDCP_includes.plantuml"
    "${appdemo_drvwarnsm_types_PACKAGE_PATH}/plantuml/types/DrvWarnSM_includes.plantuml"
    "${appdemo_parksm_types_PACKAGE_PATH}/plantuml/types/AP_PARKSM_includes.plantuml"
    "${si_types_PACKAGE_PATH}/plantuml/types/SI_includes.plantuml"
    "${mf_parksm_core_types_PACKAGE_PATH}/plantuml/types/AP_PARKSM_Core_includes.plantuml"
    "${mf_trjpla_types_PACKAGE_PATH}/plantuml/types/AP_TRJPLA_includes.plantuml"
    "${mf_trjctl_types_PACKAGE_PATH}/plantuml/types/AP_TRJCTL_includes.plantuml"
    "${mf_vedodo_types_PACKAGE_PATH}/plantuml/types/LSM_VEDODO_includes.plantuml"
    "${tce_types_PACKAGE_PATH}/plantuml/types/tce_includes.plantuml"
    "${mf_manager_types_PACKAGE_PATH}/plantuml/types/MF_Manager_includes.plantuml"
    "${mf_lsca_types_PACKAGE_PATH}/plantuml/types/MF_LSCA_includes.plantuml"
    "${mf_lvmd_types_PACKAGE_PATH}/plantuml/types/MF_LVMD_includes.plantuml"
    "${appdemo_hmih_types_PACKAGE_PATH}/plantuml/types/MFHMIHandler_includes.plantuml"
    "${mf_drvwarnsm_core_types_PACKAGE_PATH}/plantuml/types/DrvWarnSMCore_includes.plantuml"
    "${appdemo_tonh_types_PACKAGE_PATH}/plantuml/types/MF_ToneHandler_includes.plantuml"
    "${mf_memory_parking_types_PACKAGE_PATH}/plantuml/types/MemPark_includes.plantuml"
    "${viewcontroller_types_PACKAGE_PATH}/plantuml/types/Viewcontroller_includes.plantuml"
    "${mf_whlprotectproc_types_PACKAGE_PATH}/plantuml/types/MF_WhlProtectProc_includes.plantuml"
    "${us_drv_types_PACKAGE_PATH}/plantuml/types/US_DRV_includes.plantuml"
    "${us_processing_types_PACKAGE_PATH}/plantuml/types/US_PROCESSING_includes.plantuml"
    "${us_em_types_PACKAGE_PATH}/plantuml/types/US_EM_includes.plantuml"
    "${plp_hpsd_types_PACKAGE_PATH}/plantuml/types/plp_hpsd.plantuml"
    "${avga_swc_types_PACKAGE_PATH}/plantuml/types/avga_swc_includes.plantuml"
)

if(NOT ${ULTRASONIC_ONLY})
    list(APPEND PLANTUML_FILES
        "${relocalization_types_PACKAGE_PATH}/plantuml/types/relocalization_include.plantuml"
        "${cem_lsm_types_PACKAGE_PATH}/plantuml/types/aupdf.plantuml"
        "${cv_common_types_PACKAGE_PATH}/plantuml/types/cv_common.plantuml"
        "${gdr_types_PACKAGE_PATH}/plantuml/types/gdr_output.plantuml"
        "${pmsd_types_PACKAGE_PATH}/plantuml/types/pmsd_output.plantuml"
        "${spp_types_PACKAGE_PATH}/plantuml/types/spp.plantuml"
        "${tpp_types_PACKAGE_PATH}/plantuml/types/tpp_output.plantuml"
        "${mecal_types_PACKAGE_PATH}/plantuml/types/MECAL_includes.plantuml"
    )
endif()

if(CIP_BUILD_SYSTEM_VERSION)
    set(PLANTUML_INTERFACES "")
else()
    set(PLANTUML_INTERFACES
        "${CMAKE_BINARY_DIR}/tce/interface/plantuml/tce_interface.plantuml"
        "${CMAKE_BINARY_DIR}/si_core/interface/plantuml/${VARIANT}_parking/si_${VARIANT}_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_whlprotectproc/interface/plantuml/mf_whlprotectproc_interface.plantuml"
        "${CMAKE_BINARY_DIR}/appdemo_drvwarnsm/interface/plantuml/DrvWarnSM_interface.plantuml"
        "${CMAKE_BINARY_DIR}/appdemo_hmih/interface/plantuml/MFHMIH_interface.plantuml"
        "${CMAKE_BINARY_DIR}/appdemo_parksm/interface/plantuml/parksm_interface.plantuml"
        "${CMAKE_BINARY_DIR}/appdemo_tonh/interface/plantuml/appdemo_tonh_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_drvwarnsm_core/interface/plantuml/DrvWarnSM_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_lsca/interface/plantuml/mf_lsca_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_manager/interface/plantuml/mf_manager_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_parksm_core/interface/component/plantuml/mf_parksm_core_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_pdwarnproc/interface/plantuml/pdcp_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_trjctl/interface/plantuml/ap_trjctl_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_trjpla/interface/plantuml/mf_trjpla_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_vedodo/interface/plantuml/vedodo_interface.plantuml"
        "${CMAKE_BINARY_DIR}/us_processing/interface/plantuml/us_processing_interface.plantuml"
        "${CMAKE_BINARY_DIR}/plp_parameter_handler/interface/plantuml/plp_parameter_handler_interface.plantuml"
        "${CMAKE_BINARY_DIR}/avga_swc/interface/plantuml/avga_swc_interface.plantuml"
    )

    if(NOT ${ULTRASONIC_ONLY})
        list(APPEND PLANTUML_INTERFACES
        "${CMAKE_BINARY_DIR}/mf_memory_parking/interface/plantuml/relocalization_module/relocalization_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_memory_parking/interface/plantuml/mf_memory_parking/mf_memory_parking_interface.plantuml"
        "${CMAKE_BINARY_DIR}/mf_lvmd/interface/plantuml/mf_lvmd_interface.plantuml"
        )
    endif()
endif()

set(PLANTUML_INCLUDES
## include paths for data structures uses for ports
    "${appl_srv_layer_types_PACKAGE_PATH}/plantuml/types"
    "${mf_common_types_PACKAGE_PATH}/plantuml/types"
    "${vehicle_types_PACKAGE_PATH}/plantuml/types"
    "${parking_hmi_types_PACKAGE_PATH}/plantuml/types"
    "${dmc_types_PACKAGE_PATH}/interface/types"
    "${mf_pdwarnproc_types_PACKAGE_PATH}/plantuml/types"
    "${appdemo_drvwarnsm_types_PACKAGE_PATH}/plantuml/types"
    "${appdemo_parksm_types_PACKAGE_PATH}/plantuml/types"
    "${si_types_PACKAGE_PATH}/plantuml/types"
    "${mf_parksm_core_types_PACKAGE_PATH}/plantuml/types"
    "${mf_trjpla_types_PACKAGE_PATH}/plantuml/types"
    "${mf_trjctl_types_PACKAGE_PATH}/plantuml/types"
    "${mf_vedodo_types_PACKAGE_PATH}/plantuml/types"
    "${tce_types_PACKAGE_PATH}/plantuml/types"
    "${mf_manager_types_PACKAGE_PATH}/plantuml/types"
    "${mf_lsca_types_PACKAGE_PATH}/plantuml/types"
    "${appdemo_hmih_types_PACKAGE_PATH}/plantuml/types"
    "${mf_lvmd_types_PACKAGE_PATH}/plantuml/types"
    "${mf_drvwarnsm_core_types_PACKAGE_PATH}/plantuml/types"
    "${appdemo_tonh_types_PACKAGE_PATH}/plantuml/types"
    "${mf_memory_parking_types_PACKAGE_PATH}/plantuml/types"
    "${viewcontroller_types_PACKAGE_PATH}/plantuml/types"
    "${mf_whlprotectproc_types_PACKAGE_PATH}/plantuml/types"
    "${mf_common_PACKAGE_PATH}/plantuml/consts"
    "${us_drv_types_PACKAGE_PATH}/plantuml/types"
    "${us_processing_types_PACKAGE_PATH}/plantuml/types"
    "${parking_hmi_types_PACKAGE_PATH}/plantuml/types"
    "${dmc_types_PACKAGE_PATH}/plantuml/types"
    "${us_em_types_PACKAGE_PATH}/plantuml/types"
    "${plp_hpsd_types_PACKAGE_PATH}/plantuml/types"
    "${avga_swc_types_PACKAGE_PATH}/plantuml/types"
)

if(NOT ${ULTRASONIC_ONLY})
    list(APPEND PLANTUML_INCLUDES
        "${relocalization_types_PACKAGE_PATH}/plantuml/types"
        "${cem_lsm_types_PACKAGE_PATH}/plantuml/types"
        "${cv_common_types_PACKAGE_PATH}/plantuml/types"
        "${gdr_types_PACKAGE_PATH}/plantuml/types"
        "${pmsd_types_PACKAGE_PATH}/plantuml/types"
        "${spp_types_PACKAGE_PATH}/plantuml/types"
        "${tpp_types_PACKAGE_PATH}/plantuml/types"
        "${mecal_types_PACKAGE_PATH}/plantuml/types"
    )
endif()

set(COBOLT_GENERATOR_DIRECTORY ${CMAKE_BINARY_DIR})

set(ADDITIONAL_GENERATOR_PLATFORM ecal)

# Set some variables with target link libraries:

# promote types repositories as INTERFACE dependency
set(SIL_DEPS_INTERFACE
    codegen_interface
    eco::component_interface
    mf_common_types::types
    si_types::types
    mf_parksm_core_types::types #???
    mf_trjpla_types::mf_trjpla_types
    mf_trjctl_types::types
    mf_vedodo_types::mf_vedodo_types
    tce_types::types
    mf_manager_types::types #???
    mf_lsca_types::mf_lsca_types
    mf_lvmd_types::types
    appdemo_hmih_types::types #???
    mf_drvwarnsm_core_types::types #???
    appdemo_tonh_types::types #???
    mf_memory_parking_types::types #???
    us_drv_types::types
    us_processing_types::types
    us_processing::us_processing # public since US_PROCESSING_Interface.h needs to be exposed
    mf_memory_parking_types::types
    parking_hmi_types::types
    dmc_types::types
    appl_srv_layer_types::types
    us_em_types::types
    avga_swc_types::types
)

if(NOT ${ULTRASONIC_ONLY})
    list(APPEND SIL_DEPS_INTERFACE
        cem_lsm_types::types
    )
endif()

# pull in all the other libs for compilation privately
set (SIL_DEPS_PRIVATE
    codegen_types                               # pull in the generated c++03 data types from our plantuml generation
    # plp_log::log
    # plp_log::log_if
    eco::auxiliary                              # Pull in auxiliary headers for eco basic types
    mf_common::mf_common
    mf_parksm_core::mf_parksm_core
    mf_taposd::mf_taposd
    mf_trjpla::mf_trjpla
    mf_trjctl::mf_trjctl
    mf_vedodo::mf_vedodo
    tce::tce
    mf_manager::mf_manager
    mf_lsca::mf_lsca
    appdemo_hmih::appdemo_hmih
    appdemo_parksm::appdemo_parksm
    appdemo_drvwarnsm::appdemo_drvwarnsm
    geoml::geoml
    mf_drvwarnsm_core::mf_drvwarnsm_core
    mf_pdwarnproc::mf_pdwarnproc
    appdemo_tonh::appdemo_tonh
    mf_whlprotectproc::mf_whlprotectproc
    plp_log::log
    adas_platforms::adas_platforms
	us_em::us_em
    # no explicit linking should be needed for these libraries if they were exported by other packages
    jsoncpp_static
    viewcontroller_types::types
    mf_whlprotectproc_types::types
    avga_swc::avga_swc
)

if(NOT ${ULTRASONIC_ONLY})
    list(APPEND SIL_DEPS_PRIVATE
       mf_memory_parking::mf_memory_parking
       plp_parameter_handler::plp_parameter_handler
       mf_lvmd::mf_lvmd
)
endif()
