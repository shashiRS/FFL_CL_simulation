set(Protobuf_USE_STATIC_LIBS ON)
# group source files for IDE generation
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR})

# --- for ap_wrapper library ---
find_package(appl_srv_layer_types REQUIRED)
find_package(mf_common REQUIRED)
find_package(mf_parksm_core REQUIRED)
find_package(mf_taposd REQUIRED)
find_package(mf_trjpla REQUIRED)
find_package(mf_trjctl REQUIRED)
find_package(mf_vedodo REQUIRED)
find_package(tce REQUIRED)
find_package(mf_manager REQUIRED)
find_package(mf_lsca REQUIRED)
find_package(appdemo_hmih REQUIRED)
find_package(appdemo_parksm REQUIRED)
find_package(appdemo_drvwarnsm REQUIRED)
find_package(geoml REQUIRED)
find_package(mf_drvwarnsm_core REQUIRED)
find_package(mf_pdwarnproc REQUIRED)
find_package(appdemo_tonh REQUIRED)
find_package(mf_whlprotectproc REQUIRED)
find_package(mf_memory_parking_types REQUIRED)
find_package(us_processing REQUIRED)
find_package(plp_log REQUIRED)
find_package(mf_common_types REQUIRED)
find_package(us_em REQUIRED)

# --- for CarMaker executable ---
find_package(carmaker_build_dep) # not REQUIRED: only for vs2017 & vs2017_debug
find_package(si_core REQUIRED)
find_package(plp_hpsd_types REQUIRED)
if(NOT ${ULTRASONIC_ONLY})
    find_package(plp_parameter_handler REQUIRED)
    find_package(mf_memory_parking REQUIRED)
    find_package(cv_common_types REQUIRED)
    find_package(gdr_types REQUIRED)
    find_package(tpp_types REQUIRED)
    find_package(pmsd_types REQUIRED)
    find_package(spp_types REQUIRED)
    find_package(mecal_types REQUIRED)
    find_package(eCAL REQUIRED)
    find_package(svc_model_processing REQUIRED)
	find_package(cem_lsm_types REQUIRED)
    find_package(cem200) # not REQUIRED: otherwise missing cml package is causing an error
    find_package(mf_lvmd REQUIRED)
endif()
find_package(mf_vedodo_types REQUIRED) # not publicly linked by mf_vedodo
find_package(tce_types REQUIRED)
find_package(jsoncpp) # not REQUIRED: only for vs2017 & vs2017_debug
find_package(viewcontroller_types REQUIRED) # not publicly linked by appdemo_hmih
find_package(mf_whlprotectproc_types REQUIRED) # not publicly linked by mf_drvwarnsm_core
find_package(mf_common_types REQUIRED)
find_package(avga_swc_types REQUIRED)
find_package(avga_swc REQUIRED)
if(${MOCO})
    find_package(TRATCO REQUIRED)
    find_package(VECONA REQUIRED)
endif()

#############################################################
######### generate data types from plantuml
#############################################################
find_package(Next QUIET)
if(Next_FOUND)
  set(ADDITIONAL_GENERATOR_PLATFORM "ecal") # add ecal/protobuf which we need later for the next node
else()
  set(ADDITIONAL_GENERATOR_PLATFORM)        # leave platform empty to only generate c, c++02 and c++14 types
endif()
# get the list of needed plantuml and dependencies
include("${PROJECT_SOURCE_DIR}/tests/SIL/CarMaker/interface/plantuml/set_plantuml_files_and_includes.cmake")
# issue a call to convertPlantUmlToTypes to generate eco framework types
convertPlantUmlToTypes(
  INPUTPLANTUMLFILES ${PLANTUML_FILES}
  PLANTUMLINCLUDES ${PLANTUML_INCLUDES}
  CODEGENFOLDER ${COBOLT_GENERATOR_DIRECTORY}
  PLATFORM ${ADDITIONAL_GENERATOR_PLATFORM}
  LOGFILE ${COBOLT_GENERATOR_DIRECTORY}/log/convertPlantUmlToTypes.log
)
installTypes(
  CODEGENFOLDER ${CMAKE_CURRENT_BINARY_DIR}
  LIBS
    types
)
######## end of plantuml data type generation

convertPlantUmlToComponentInterface(
    INPUTPLANTUMLFILES
        ${PLANTUML_FILES}
        ${PLANTUML_INTERFACES}
    PLANTUMLINCLUDES ${PLANTUML_INCLUDES}
    CODEGENFOLDER ${COBOLT_GENERATOR_DIRECTORY}
    LOGFILE ${COBOLT_GENERATOR_DIRECTORY}/log/cobolt_interface_generation.log
)
installComponentInterface(
    CODEGENFOLDER ${CMAKE_CURRENT_BINARY_DIR}
    LIBS interface
)

#----------------ap_wrapper---------------------
set(AP_WRAPPER_NAME ap_wrapper)

set(AP_WRAPPER_SRC
    carMakerAPWrapper.cpp
    uspWrapper.cpp
	us_em_wrapper.cpp
)

add_library(${AP_WRAPPER_NAME} STATIC
    ${AP_WRAPPER_SRC}
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND PLP_LOCAL_BUILD)
    # workarounds needed for generating debugging workspace (https://github-am.geo.conti.de/ADAS/plp_debugging_workspace)
    if(${ULTRASONIC_ONLY})
        target_compile_definitions(${AP_WRAPPER_NAME} PUBLIC -DULTRASONIC_ONLY)
    endif()
    target_compile_definitions(${AP_WRAPPER_NAME} PUBLIC -DBUILD_MTS LSCA_BUILD_FOR_CARMAKER USE_ENV_PLOTTER)
endif()

target_link_libraries(${AP_WRAPPER_NAME}
  # type libraries made public for users of this AP-wrapper library
  PUBLIC
    ${SIL_DEPS_INTERFACE}
  PRIVATE
    ${SIL_DEPS_PRIVATE}
)

target_compile_definitions(${AP_WRAPPER_NAME}
  PUBLIC
    CIP_BRICKS_BUILD
    CAR_MAKER_AP_DLL_EXPORT
    _CRT_SECURE_NO_WARNINGS
    $<$<CONFIG:Debug>:_DEBUG> # only for debug build
  PRIVATE
    GLUT_DISABLE_ATEXIT_HACK
    $<$<STREQUAL:"${CMAKE_CXX_COMPILER_ID}","MSVC">:WIN32_LEAN_AND_MEAN> # only for VS build
    BUILD_FOR_CARMAKER
    PLP_LOG_ENABLED # Logger
    NOGDI #Avoid defines from windows.h that clash with enum names from types. e.g. OPAQUE
)

if(${ULTRASONIC_ONLY})
  target_compile_definitions(${AP_WRAPPER_NAME}
    PRIVATE
      VARIANT_CUS_ONLY
  )
endif()

if(${PERFORMANCE})
  target_compile_definitions(${AP_WRAPPER_NAME}
    PRIVATE
      VARIANT_PERFORMANCE
  )
endif()

if(${MOCO})
  target_compile_definitions(${AP_WRAPPER_NAME}
    PUBLIC
      MOCO_REPLACES_LODMC
  )
endif()

# compiler optimizations for VisualStudio release build
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # deactivate runtime error checks which prevent compiler optimizations (/O options)
  # STRING (REGEX REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  target_compile_options(${AP_WRAPPER_NAME}
    PRIVATE
      $<$<CONFIG:Release>:/GL> # whole-program optimization
  )
endif()

# use MF_Plotter only in debug build (to disable for regression test execution in Jenkins)
if(${CMAKE_INSTALL_PREFIX} MATCHES "_debug$")
  find_package(mf_plot REQUIRED)

  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit architecture
    set(FREEGLUT_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/x64)
  else()
    set(FREEGLUT_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../lib/x86)
  endif()

  target_link_libraries(${AP_WRAPPER_NAME} 
    PUBLIC 
      mf_plot::mf_plot
    PRIVATE
      # freeglut is not exported/propagated by the mf_plot component
      ${FREEGLUT_LIB_DIR}/freeglut.lib
  )

  install(
    FILES ${FREEGLUT_LIB_DIR}/freeglut.dll
    COMPONENT TARGET_COMPONENT
    DESTINATION bin
  )
  else()
    # In release build (vs2017), copy Python 3.9 for regression test evaluation with TSF.
    set(Python3_FIND_VIRTUALENV STANDARD) # do not search for Python3 interpreter in virtual environment
    find_package(Python3 REQUIRED Interpreter)
    get_filename_component(Python3_BASE_DIR ${Python3_EXECUTABLE} DIRECTORY)
    file(COPY ${Python3_BASE_DIR}/ DESTINATION ${PROJECT_SOURCE_DIR}/contrib/cip/python)
    message(STATUS "Copy Python from ${Python3_BASE_DIR} to ${PROJECT_SOURCE_DIR}/contrib/cip/python")
endif()

install(
  TARGETS ${AP_WRAPPER_NAME}
  COMPONENT TARGET_COMPONENT
  DESTINATION lib
  INCLUDES DESTINATION include
)

#----------------CarMaker---------------------
# set OS platform architecture
if(CMAKE_HOST_WIN32)
    set(ARCH win64)
elseif(UNIX)
    set(ARCH linux64)
endif()

set(TARGET_NAME CarMaker.${ARCH})

# related version of CarMaker application
set(CARMAKER_VER 8.1.1)
set(CARMAKER_NUMVER 80101)
# directories from CarMaker build dependency package
set(CARMAKER_BIN_DIR ${carmaker_build_dep_PACKAGE_PATH}/bin)
set(CARMAKER_LIB_DIR ${carmaker_build_dep_PACKAGE_PATH}/lib)
set(CARMAKER_INC_DIR ${carmaker_build_dep_PACKAGE_PATH}/include)

set(VCUS_MODEL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../contrib/vCUS-Phen-Mod/vCUS_Phen_CM/src_PhenDevel/USS_PhenoModel)

set(THIS_SRC
  # CarMaker
    CM_Main.c
    CM_Vehicle.c
    User.cpp
    app_tmp.c
  # CarMaker models
    mdl_APCtrl.cpp
    VehTurnCntr.c
    mdl_RearSteering.c
  # surrogate models
    CemSurrogate.cpp
    PmdSurrogate.cpp
    PmdConfigReader.cpp
    SiUtility.cpp
    SICoordinateTransformer.cpp
    lowSpeedLateralDMC.cpp
    lowSpeedLongDMC.cpp
    wheelSpeedSensorSim.cpp
  #vCUS phenomenological model
    ${VCUS_MODEL_DIR}/Init_Clean.cpp
    ${VCUS_MODEL_DIR}/Reflection_Generator.cpp
    ${VCUS_MODEL_DIR}/ReflectionFunc.cpp
    ${VCUS_MODEL_DIR}/Sensor_Mapping.cpp
    ${VCUS_MODEL_DIR}/vCUS_Main.cpp
    ${VCUS_MODEL_DIR}/vCUS_Utils.cpp
  # misc
    TestEvaluation.cpp
    TestRunWrapper.cpp
    OdometryDataManager.cpp
    CarMakerLogger.cpp
    CarMakerSystemServices.cpp
)

if(NOT ${ULTRASONIC_ONLY})
  list(APPEND THIS_SRC
    # SI surrogate model
    EnvironmentPerception.cpp
    EpDelimiterManager.cpp
    GroundTruthEnvironment.cpp
    # software component wrappers
    SiHighWrapper.cpp
    #svc model
    SvcModelUtils.cpp
    SvcModelWrapper.cpp
    # CarMaker eCal node for CEM_LSM integration
    ecal_node/ecal_calls/CarmakerNode_Control.cpp
    ecal_node/ecal_calls/CEM_Call.cpp
    ecal_node/ecal_calls/cem_carmaker_user.cpp
    ecal_node/ecal_calls/MF_publishers.cpp
    ecal_node/cem_mapping/LSMO_mapping.cpp
    ecal_node/cem_mapping/SVC_mapping.cpp
    ecal_node/cem_mapping/USS_mapping.cpp
    ecal_node/simulation_log/sim_logger.cpp
    ecal_node/subscriptions/AUP/CEM_subs.cpp
    ecal_node/subscriptions/AUP/Subscribe_Share.cpp
    ecal_node/subscriptions/visualization/CEM_disp.cpp
  )
else()
  list(APPEND THIS_SRC
    SiLowWrapper.cpp
  )
endif()

# CAN communication to HMI
if(${USE_HMI})
  list(APPEND THIS_SRC
    VectorCMInterface.c
    PRIVATE_CAN.c
    PRIVATE_CAN_User.c
  )
endif()

if(${MOCO})
  set(OCTAGON_BRAKE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../contrib/VSP_SEF_Brake/src/package/OCTAGONBrake)
  set(OCTAGON_ENGINE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../contrib/VSP_SEF_Powertrain/src/package/OCTAGONEngine)
  list(APPEND THIS_SRC
    MocoWrapper.cpp
    ${OCTAGON_BRAKE_DIR}/OCTAGONBrake.cpp
    ${OCTAGON_ENGINE_DIR}/OCTAGONEngine.cpp
  )
endif()

add_executable(${TARGET_NAME}
  ${THIS_SRC}
)

target_include_directories(${TARGET_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/pmd>
    $<BUILD_INTERFACE:${CARMAKER_INC_DIR}>
    $<BUILD_INTERFACE:${VCUS_MODEL_DIR}>
)

target_link_libraries(${TARGET_NAME}
    ${CARMAKER_LIB_DIR}/libcarmaker.a
    ${CARMAKER_LIB_DIR}/libcar.a
    ${CARMAKER_LIB_DIR}/libipgdriver.a
    ${CARMAKER_LIB_DIR}/libipgroad.a
    ${CARMAKER_LIB_DIR}/libipgtire.a
    ${CARMAKER_LIB_DIR}/libz-${ARCH}.a
    ${AP_WRAPPER_NAME}
    mf_common::mf_common # for InflatedStaticObject, HelperFunctions
    jsoncpp_static # needed for PmdConfigReader and SiParamReader
    geoml::geoml # needed for cml::Vec2Df and CoordinateTransformer2D etc.
)

target_link_libraries(${TARGET_NAME}
        codegen_types
        si_core::si_core
        plp_log::log
        plp_log::log_if
        ${SIL_DEPS_PRIVATE}
)
if(NOT ${ULTRASONIC_ONLY})
  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ecal_node/ecal_calls>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ecal_node/cem_mapping>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ecal_node/simulation_log>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ecal_node/subscriptions/AUP>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ecal_node/subscriptions/visualization>
  )

    target_link_libraries(${TARGET_NAME}
        mf_memory_parking::mf_memory_parking
        eCAL::core
        svc_model_processing::svc_model_processing
    )
endif()

if(${MOCO})
  target_include_directories(${TARGET_NAME}
    PUBLIC
      $<BUILD_INTERFACE:${OCTAGON_BRAKE_DIR}>
      $<BUILD_INTERFACE:${OCTAGON_ENGINE_DIR}>
  )

    target_link_libraries(${TARGET_NAME}
        VECONA::VECONA_vsp
        VECONA::VECONA
        TRATCO::TRATCO_vsp
        TRATCO::TRATCO
    )
    
    set(VECONA_LIB_DIR ${VECONA_PACKAGE_PATH}/lib)
    set(TRATCO_LIB_DIR ${TRATCO_PACKAGE_PATH}/lib)
    
    install(
    FILES ${VECONA_LIB_DIR}/VECONA_vsp.dll
    COMPONENT TARGET_COMPONENT
    DESTINATION bin
    )
    install(
    FILES ${TRATCO_LIB_DIR}/TRATCO_vsp.dll
    COMPONENT TARGET_COMPONENT
    DESTINATION bin
    )
endif()

if(WIN32)
    target_link_libraries(${TARGET_NAME}
        ${CARMAKER_LIB_DIR}/libcompat_${ARCH}.a
        ${CARMAKER_LIB_DIR}/libcompat_msvs2015.a
        user32.lib
        advapi32.lib
        ws2_32.lib
        legacy_stdio_definitions.lib
        winspool.lib
    )
endif()

target_compile_definitions(${TARGET_NAME}
    PRIVATE
      PLP_LOG_ENABLED # Logger
      PLP_LOG_ERRORS_TO_WARNINGS # convert errors to warnings and allow for 1000 warnings per dt=5 ms
)

if(${ULTRASONIC_ONLY})
  target_compile_definitions(${TARGET_NAME}
    PRIVATE
      ULTRASONIC_ONLY
      NO_SCENE_INTERPRETATION
      VARIANT_CUS_ONLY
  )
endif()

if(${PERFORMANCE})
  target_compile_definitions(${TARGET_NAME}
    PRIVATE
      VARIANT_PERFORMANCE
  )
endif()

if(WIN32)
  target_compile_definitions(${TARGET_NAME}
    PRIVATE
      #WIN32 # automatically defined
      WIN64
      NOGDI #Avoid defines from windows.h that clash with enum names from types. e.g. OPAQUE
      _CRT_NONSTDC_NO_DEPRECATE
      _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
      CM_NUMVER=${CARMAKER_NUMVER}
      UNICODE # for support of wchar_t in windows headers
      LSCA_BUILD_FOR_CARMAKER # should be publicly defined by mf_lsca_types instead of CARMAKER_BUILD?
  )
endif()

# compiler optimizations for VisualStudio release build
if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # deactivate runtime error checks which prevent compiler optimizations (/O options)
  # STRING (REGEX REPLACE "/RTC1" "" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
  target_compile_options(${TARGET_NAME} 
    PRIVATE
      $<$<CONFIG:Release>:/GL> # whole-program optimization
  )
  target_link_options(${TARGET_NAME}
    PRIVATE
      $<$<CONFIG:Release>:/LTCG> # link-time code generation to perform whole-program optimization
  )
endif()

if(${USE_HMI})
  target_include_directories(${TARGET_NAME}
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/VectorXLDriver>
  )

  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit architecture
    set(VECTOR_DRIVER_LIB_DIR $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../lib/x64>)
  else()
    set(VECTOR_DRIVER_LIB_DIR $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../lib/x86>)
  endif()

  target_link_libraries(${TARGET_NAME}
    ${VECTOR_DRIVER_LIB_DIR}/vxlapi.lib
  )

  target_compile_definitions(${TARGET_NAME}
    PRIVATE
      USE_HMI
      DYNAMIC_XLDRIVER_DLL
      DO_NOT_DEFINE_EXTERN_DECLARATION
  )

  install(
    FILES ${VECTOR_DRIVER_LIB_DIR}/vxlapi.dll
    COMPONENT TARGET_COMPONENT
    DESTINATION bin
  )
endif()

install(
    TARGETS ${TARGET_NAME}
    COMPONENT TARGET_COMPONENT
    DESTINATION bin
)

# CarMaker project config files (build.yml added as meta-data containing versions)
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../../conf/package
  COMPONENT TARGET_COMPONENT
  DESTINATION conf
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../../conf/build.yml
  COMPONENT TARGET_COMPONENT
  DESTINATION conf
)

include(${CMAKE_CURRENT_SOURCE_DIR}/ContainerCMakeList.txt)

# custom command for generation and removal of mandatory file app_tmp.c
add_custom_command(
    OUTPUT app_tmp.c
    COMMENT "Creating app_tmp.c"
    COMMAND ${CARMAKER_BIN_DIR}/CreateCarMakerAppInfo
        =av MF_SimPlatform
        =sv ${CARMAKER_VER}
        =arch ${ARCH}
        =libs 
            ${CARMAKER_LIB_DIR}/libcarmaker.a
            ${CARMAKER_LIB_DIR}/libcar.a
            ${CARMAKER_LIB_DIR}/libipgdriver.a
            ${CARMAKER_LIB_DIR}/libipgroad.a
            ${CARMAKER_LIB_DIR}/libipgtire.a
        =o app_tmp.c
)

if(PLP_LOCAL_BUILD) # debugging workspace build
  set(CARMAKER_EXECUTABLE ${CMAKE_SOURCE_DIR}/bin/Debug/${TARGET_NAME}${CMAKE_EXECUTABLE_SUFFIX})
  set(CMAKE_INSTALL_BINDIR ${CMAKE_SOURCE_DIR}/bin/Debug)
else()
  set(CARMAKER_EXECUTABLE ${CMAKE_INSTALL_PREFIX}/bin/${TARGET_NAME}${CMAKE_EXECUTABLE_SUFFIX})
  set(CMAKE_INSTALL_BINDIR ${CMAKE_INSTALL_PREFIX}/bin)
endif()

if(${ULTRASONIC_ONLY}) # determine variant-specific vCUS model parameterfile
  set(VCUS_MODEL_PARAMETERS ${CMAKE_CURRENT_SOURCE_DIR}/../Data/Sensor/entry/vCUS_Parameterset)
else()
  set(VCUS_MODEL_PARAMETERS ${CMAKE_CURRENT_SOURCE_DIR}/../Data/Sensor/fusion/vCUS_Parameterset)
endif()

add_custom_command(
   TARGET ${TARGET_NAME}
   POST_BUILD
   COMMENT "Deleting app_tmp.c"
   COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_CURRENT_BINARY_DIR}/app_tmp.c
   COMMENT "Setting Carmaker executable path"
   COMMAND python ${CMAKE_CURRENT_SOURCE_DIR}/../../Report_Generator/Bricks_CI/adapt_exe_path.py --exe ${CARMAKER_EXECUTABLE} --project ${CMAKE_CURRENT_SOURCE_DIR}/../
   COMMENT "Copy variant-specific vCUS model parameterfile"
   COMMAND ${CMAKE_COMMAND} -E copy ${VCUS_MODEL_PARAMETERS} ${CMAKE_CURRENT_SOURCE_DIR}/../Data/Sensor/vCUS_Parameterset
)

if(NOT ${ULTRASONIC_ONLY})
  # install(
    # DIRECTORY ${cem200_PACKAGE_PATH}/bin
    # COMPONENT TARGET_COMPONENT
    # DESTINATION .
  # )

  # set ecal_core and ecaltime-localtime dll directory
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    set(ecal_core_dll ${eCAL_DIR}/../../../bin/ecal_corereldbg.dll)
    set(ecaltime_dll ${eCAL_DIR}/../../../bin/ecaltime_plugins/ecaltime-localtimereldbg.dll)
    # rename ecaltime-localtimereldbg.dll because of eCal bug: https://github.com/eclipse-ecal/ecal/issues/1251
    set(ecaltime_dll_rename ecaltime-localtime.dll)
  elseif(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(ecal_core_dll ${eCAL_DIR}/../../../bin/ecal_cored.dll)
    set(ecaltime_dll ${eCAL_DIR}/../../../bin/ecaltime_plugins/ecaltime-localtimed.dll)
    # no need to rename ecaltime-localtimed.dll
    set(ecaltime_dll_rename ecaltime-localtimed.dll)
  else()
    message(FATAL_ERROR Unsupported cmake build type
                        selected:${CMAKE_BUILD_TYPE})
  endif()

  install(
    FILES ${ecal_core_dll}
    COMPONENT TARGET_COMPONENT
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  install(
    FILES ${ecaltime_dll} RENAME ${ecaltime_dll_rename}
    COMPONENT TARGET_COMPONENT
    DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

message("#################START############################")
message("########eCAL: ${eCAL_DIR}")
message("########CarMakerEXE: ${CMAKE_INSTALL_BINDIR}")
message("#################END############################")