

# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(adas_platforms QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(adas_platforms COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (adas_platforms_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package adas_platforms failed: ${adas_platforms_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(appdemo_drvwarnsm QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(appdemo_drvwarnsm COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (appdemo_drvwarnsm_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package appdemo_drvwarnsm failed: ${appdemo_drvwarnsm_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(appdemo_hmih QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(appdemo_hmih COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (appdemo_hmih_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package appdemo_hmih failed: ${appdemo_hmih_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(appdemo_hmih_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(appdemo_hmih_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (appdemo_hmih_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package appdemo_hmih_types failed: ${appdemo_hmih_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(appdemo_parksm QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(appdemo_parksm COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (appdemo_parksm_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package appdemo_parksm failed: ${appdemo_parksm_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(appdemo_tonh QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(appdemo_tonh COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (appdemo_tonh_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package appdemo_tonh failed: ${appdemo_tonh_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(appdemo_tonh_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(appdemo_tonh_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (appdemo_tonh_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package appdemo_tonh_types failed: ${appdemo_tonh_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(appl_srv_layer_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(appl_srv_layer_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (appl_srv_layer_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package appl_srv_layer_types failed: ${appl_srv_layer_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(avga_swc QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(avga_swc COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (avga_swc_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package avga_swc failed: ${avga_swc_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(avga_swc_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(avga_swc_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (avga_swc_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package avga_swc_types failed: ${avga_swc_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(dmc_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(dmc_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (dmc_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package dmc_types failed: ${dmc_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(eco QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(eco COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (eco_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package eco failed: ${eco_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(geoml QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(geoml COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (geoml_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package geoml failed: ${geoml_NOT_FOUND_MESSAGE}")
endif()



# Code generated by <bricks_cmake_groups>/cmake/post/implicit_dependencies/generate.py; DO NOT EDIT!

# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(jsoncpp QUIET)
else()
    find_package(jsoncpp)
endif()

if (NOT (jsoncpp_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(
        mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE
        "Importing required package jsoncpp failed: ${jsoncpp_NOT_FOUND_MESSAGE}"
    )
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_common QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_common COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_common_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_common failed: ${mf_common_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_common_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_common_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_common_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_common_types failed: ${mf_common_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_drvwarnsm_core QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_drvwarnsm_core COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_drvwarnsm_core_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_drvwarnsm_core failed: ${mf_drvwarnsm_core_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_drvwarnsm_core_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_drvwarnsm_core_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_drvwarnsm_core_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_drvwarnsm_core_types failed: ${mf_drvwarnsm_core_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_lsca QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_lsca COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_lsca_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_lsca failed: ${mf_lsca_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_lsca_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_lsca_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_lsca_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_lsca_types failed: ${mf_lsca_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_lvmd_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_lvmd_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_lvmd_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_lvmd_types failed: ${mf_lvmd_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_manager QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_manager COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_manager_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_manager failed: ${mf_manager_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_manager_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_manager_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_manager_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_manager_types failed: ${mf_manager_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_memory_parking_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_memory_parking_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_memory_parking_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_memory_parking_types failed: ${mf_memory_parking_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_parksm_core QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_parksm_core COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_parksm_core_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_parksm_core failed: ${mf_parksm_core_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_parksm_core_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_parksm_core_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_parksm_core_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_parksm_core_types failed: ${mf_parksm_core_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_pdwarnproc QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_pdwarnproc COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_pdwarnproc_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_pdwarnproc failed: ${mf_pdwarnproc_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_plot QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_plot COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_plot_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_plot failed: ${mf_plot_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_taposd QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_taposd COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_taposd_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_taposd failed: ${mf_taposd_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_trjctl QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_trjctl COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_trjctl_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_trjctl failed: ${mf_trjctl_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_trjctl_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_trjctl_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_trjctl_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_trjctl_types failed: ${mf_trjctl_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_trjpla QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_trjpla COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_trjpla_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_trjpla failed: ${mf_trjpla_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_trjpla_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_trjpla_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_trjpla_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_trjpla_types failed: ${mf_trjpla_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_vedodo QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_vedodo COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_vedodo_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_vedodo failed: ${mf_vedodo_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_vedodo_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_vedodo_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_vedodo_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_vedodo_types failed: ${mf_vedodo_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_whlprotectproc QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(mf_whlprotectproc COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (mf_whlprotectproc_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_whlprotectproc failed: ${mf_whlprotectproc_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(mf_whlprotectproc_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(mf_whlprotectproc_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (mf_whlprotectproc_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package mf_whlprotectproc_types failed: ${mf_whlprotectproc_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(parking_hmi_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(parking_hmi_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (parking_hmi_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package parking_hmi_types failed: ${parking_hmi_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(plp_log QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(plp_log COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (plp_log_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package plp_log failed: ${plp_log_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(si_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(si_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (si_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package si_types failed: ${si_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(tce QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(tce COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (tce_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package tce failed: ${tce_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(tce_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(tce_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (tce_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package tce_types failed: ${tce_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(us_drv_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(us_drv_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (us_drv_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package us_drv_types failed: ${us_drv_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(us_em QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(us_em COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (us_em_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package us_em failed: ${us_em_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(us_em_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(us_em_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (us_em_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package us_em_types failed: ${us_em_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(us_processing QUIET COMPONENTS "TARGET_COMPONENT")
else()
    find_package(us_processing COMPONENTS "TARGET_COMPONENT")
endif()

if (NOT (us_processing_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package us_processing failed: ${us_processing_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(us_processing_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(us_processing_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (us_processing_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package us_processing_types failed: ${us_processing_types_NOT_FOUND_MESSAGE}")
endif()




# try importing package
if (mf_sil_FIND_QUIETLY)
    find_package(viewcontroller_types QUIET COMPONENTS "TYPES_COMPONENT")
else()
    find_package(viewcontroller_types COMPONENTS "TYPES_COMPONENT")
endif()

if (NOT (viewcontroller_types_FOUND))

    # depending package cannot be imported
    set(mf_sil_TARGET_COMPONENT_FOUND 0)
    set(mf_sil_TARGET_COMPONENT_NOT_FOUND_MESSAGE "Importing required package viewcontroller_types failed: ${viewcontroller_types_NOT_FOUND_MESSAGE}")
endif()

