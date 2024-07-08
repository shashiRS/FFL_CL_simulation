# copy configuration files from the used packages

# The mf_common and mf_trjpla configuration files are required by the regression test
# framework (AUP_Eval_STET.py, AUP_Eval_STET_KPIs_L1_L3.py) also in the cus_only variant.
set(PACKAGES
    mf_common
    mf_trjpla
)

# In the cus_only variant, the configuration files are not used
# and parameter initialization headers are provided instead by the packages!
if(NOT ${ULTRASONIC_ONLY})
  list(APPEND PACKAGES
    mf_vedodo
    tce
    appdemo_parksm
    mf_parksm_core
    mf_trjctl
    mf_lsca
    appdemo_hmih
    appdemo_drvwarnsm
    appdemo_tonh
    mf_whlprotectproc
    mf_pdwarnproc
    mf_drvwarnsm_core
    mf_manager
    si_core
    mf_memory_parking
    mf_lvmd
  )
endif()

foreach(PACKAGE_NAME IN LISTS PACKAGES)
    find_package(${PACKAGE_NAME} REQUIRED)

    # find path to extern package
    find_path(PACKAGE_PATH ${PACKAGE_NAME})

    if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND "$ENV{BRICKS_BUILD_PLATFORM}" STREQUAL "")
        set(PACKAGE_PATH ${${PACKAGE_NAME}_PACKAGE_PATH}/data) # workaround needed for generating debugging workspace (https://github-am.geo.conti.de/ADAS/plp_debugging_workspace)
    endif()

    # remove configuration files copied to mf_sil
    file(REMOVE_RECURSE ${CMAKE_CURRENT_SOURCE_DIR}/data/${PACKAGE_NAME})

    # set the configuration files of the package
    set(CONFIG_FILES unset)
    if("${PACKAGE_NAME}" STREQUAL "mf_vedodo")
        set(CONFIG_FILES ../data/mf_vedodo/AP_Sim/FC_VEDODO_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "tce")
        set(CONFIG_FILES ../data/tce/FC_TCE_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "appdemo_parksm")
        set(CONFIG_FILES ../data/appdemo_parksm/FC_PARKSM_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_parksm_core")
        set(CONFIG_FILES ../data/mf_parksm_core/FC_PARKSM_Core_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_trjpla")
        set(CONFIG_FILES ../data/mf_trjpla/AP_Sim/FC_TRJPLA_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_trjctl")
        set(CONFIG_FILES ../data/mf_trjctl/FC_TRJCTL_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_common")
        set(CONFIG_FILES
            ../data/mf_common/Sys_Func_Params_config.json
            ../data/mf_common/AP_Sim/Vehicle_Params_config.json
        )
    elseif("${PACKAGE_NAME}" STREQUAL "mf_lsca")
        set(CONFIG_FILES ../data/mf_lsca/AP_Sim/LSCA_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "appdemo_hmih")
        set(CONFIG_FILES ../data/appdemo_hmih/FC_MFHMIH_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "appdemo_drvwarnsm")
        set(CONFIG_FILES ../data/appdemo_drvwarnsm/FC_DrvWarnSM_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_pdwarnproc")
        set(CONFIG_FILES ../data/mf_pdwarnproc/FC_PDCP_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_drvwarnsm_core")
        set(CONFIG_FILES ../data/mf_drvwarnsm_core/FC_DrvWarnSMCore_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_manager")
        set(CONFIG_FILES ../data/mf_manager/FC_MF_MANAGER_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "si_core")
        set(CONFIG_FILES ../data/si_core/SI_Config.json)
        list(APPEND CONFIG_FILES ../data/si_core/AP_Sim/SI_Config_Diff.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_memory_parking")
        set(CONFIG_FILES ../data/mf_memory_parking/AP_Sim/MemPark_Config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "appdemo_tonh")
        set(CONFIG_FILES ../data/appdemo_tonh/FC_MF_ToneHandler_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_whlprotectproc")
        set(CONFIG_FILES ../data/mf_whlprotectproc/FC_MF_WhlProtectProc_Params_config.json)
    elseif("${PACKAGE_NAME}" STREQUAL "mf_lvmd")
        set(CONFIG_FILES ../data/mf_lvmd/LVMD_Params_config.json)
    endif()

    # copy configuration files to mf_sil
    if(NOT "${CONFIG_FILES}" STREQUAL "unset")
        foreach(CONFIG_FILE_PATH IN LISTS CONFIG_FILES)
            file(COPY ${PACKAGE_PATH}/${CONFIG_FILE_PATH}
                 DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/data/${PACKAGE_NAME})
        endforeach()
    endif()

    unset(PACKAGE_PATH CACHE)
endforeach()
