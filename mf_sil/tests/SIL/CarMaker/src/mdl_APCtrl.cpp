/*
******************************************************************************
**  CarMaker - Version 5.1.2
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    http://www.ipg.de
******************************************************************************
**
** Simple VehicleControl Model to demonstrate the manipulation of Driver Gas
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    VehicleControl_Register_mdl_APCtrl ()
**
******************************************************************************
*/

// needed to prevent that the Win10 header rpcndr.h defines the type boolean, which is also defined in geoml/cml_type.h
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include<time.h>
#include <queue>
#include <list>
#include<xstring>
#include <cmath>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include "CarMaker.h"
#pragma warning( pop )
#include "Car/Vehicle_Car.h"
#include "Car/Car_Steer_Pfeffer.h"
#include "Vehicle/Sensor_GNav.h"
#include "Vehicle/Sensor_Inertial.h"
//#include "Car/DASensor.h"
#include "DrivMan.h"
#include "mdl_APCtrl.h"
#include <stdint.h>
#include "Platform_Types.h"
#include "MfSilTypes.h"

#ifndef VARIANT_CUS_ONLY
#include "LSMO_mapping.h"
#include "USS_mapping.h"
#include "SVC_mapping.h"
#include "CEM_disp.hpp"
#include "MF_Publishers.h"
#include "CEM_Call.h"
#endif

//#define AP_DO_NOT_USE_ENUM_CLASS /*To make it compile with old vs c++ compiler... not sure if this is going to work though TODO*/
/*AP INCLUDES*/
#include "mf_mempark/memory_parking_status_port.h"
#include "mf_mempark/memory_parking_status_port_interface_version.h"
#include "si/si_generated_types.h"
// begin vehicle_types headers
#include "ap_vehstatesigprovider/accinformation_port.h"
#include "ap_vehstatesigprovider/additional_bcmstatus_port.h"
#include "ap_vehstatesigprovider/apctrl_status_port.h"
#include "ap_vehstatesigprovider/authentication_status_port.h"
#include "ap_vehstatesigprovider/convertible_top_status_port.h"
#include "ap_vehstatesigprovider/door_status_port.h"
#include "ap_vehstatesigprovider/escinformation_port.h"
#include "ap_vehstatesigprovider/external_function_status_port.h"
#include "ap_vehstatesigprovider/keyless_status_port.h"
#include "ap_vehstatesigprovider/starter_status_port.h"
#include "ap_vehstatesigprovider/steering_col_switches_status_port.h"
#include "ap_vehstatesigprovider/trailer_status_port.h"
#include "ap_vehstatesigprovider/trunk_lid_status_port.h"
#include "ap_vehstatesigprovider/vehicle_occupancy_status_port.h"
#include "ap_hmitoap/ap_hmitoap_generated_types.h"
#include "ap_hmitoap/remote_hmioutput_port_interface_version.h" // workaround: should be included in ap_hmitoap/ap_hmitoap_generated_types.h
#include "ap_commonvehsigprovider/ambient_data_port.h"
#include "ap_commonvehsigprovider/brake_ctrl_status_port.h"
#include "ap_commonvehsigprovider/dds_port.h"
#include "ap_commonvehsigprovider/engine_ctrl_status_port.h"
#include "ap_commonvehsigprovider/gearbox_ctrl_status_port.h"
#include "ap_commonvehsigprovider/motion_state_port.h"
#include "ap_commonvehsigprovider/odo_ext_ctrl_port.h"
#include "ap_commonvehsigprovider/odo_gps_port.h"
#include "ap_commonvehsigprovider/steer_ctrl_status_port.h"
#include "ap_commonvehsigprovider/suspension_port.h"
#include "ap_commonvehsigprovider/system_time_port.h"
#include "ap_commonvehsigprovider/veh_dynamics_port.h"
#include "ap_commonvehsigprovider/wheel_driving_directions_port.h"
#include "ap_commonvehsigprovider/wheel_pulse_port.h"
#include "ap_commonvehsigprovider/wheel_speed_port.h"
#include "ap_lodmc/ap_lodmc_generated_types.h"
#include "ap_ladmc/ap_ladmc_generated_types.h"
// end vehicle_types headers
#include "lowSpeedLongDMC.h"
#include "lowSpeedLateralDMC.h"
#include "WheelSpeedSensorSim.h"
#include "carMakerAPWrapper.h"
#include "ap_psm_app/ap_psm_app_generated_types.h"
#include "ap_psm/ap_psm_generated_types.h"
#include "ap_trjctl/ap_trjctl_generated_types.h"
#include "ap_tp/ap_tp_generated_types.h"
#include "mf_trjpla_types/FC_TRJPLA_Params.h" // for constant ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS
#include "tce/tce_generated_types.h"
//#include "US_ALGO_Interface.h"
#include "mf_lsca_types/LSCA360_ExternalTypes.h"//For openGL visualization data struct
#include "geoml/LSM_Math.h"
#include "geoml/LSM_LookupTable.h"
#ifndef VARIANT_CUS_ONLY
#include "EnvironmentPerception.h"
#include "GroundTruthEnvironment.h"
#else
#include "Vehicle/Sensor_Object.h" // for ObjectSensorCount, ObjectSensor_Disable variable // this header is included in "EnvironmentPerception.h"
#endif
#include "SiUtility.h"
#include "mf_hmih/mf_hmih_generated_types.h"
#include "mf_tonh/mf_tonh_generated_types.h"
#include "mf_whlprotectproc/mf_whlprotectproc_generated_types.h"
#include "ap_common/ap_common_generated_types.h"
#include "mf_drvwarnsm_core/drv_warn_core_status_port_interface_version.h"
#include "mf_lvmd/lvmd_status_port_interface_version.h"
#include <string>
#include <random>
#include "TestEvaluation.h"
#include "DataDictionaryDefinition.h"
#include "pod_class_converter.h"

#include "PRIVATE_CAN.h"
#include "PRIVATE_CAN_User.h"
#define PRIVATE_CAN_Slot_Rx 0
#define PRIVATE_CAN_Slot_Tx 0
#define PRIVATE_CAN_Channel_Rx 0
#define PRIVATE_CAN_Channel_Tx 0

#include "PhenoModel.h"
#ifdef USE_ENV_PLOTTER
#include "vCUS_mfPlot.h"
#endif
#include "uspWrapper.h"
#include "us_em_wrapper.h"
#include "us_em/us_em_consts.h"
#include "us_em/us_em_generated_types.h"

#ifndef VARIANT_CUS_ONLY
#include "SvcModelUtils.h"
#include "SvcModelWrapper.h"
#endif

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif
#include "CemSurrogate.h"
#ifndef NO_SCENE_INTERPRETATION
#undef rad2deg // resolve naming collisions between utl.h (Conti) and MathUtils.h (CarMaker) before including SiHighWrapper.h
#undef deg2rad
#include "SiHighWrapper.h"
#endif
#ifdef VARIANT_CUS_ONLY
#include "SiLowWrapper.h"
#endif
#include "mf_common/HelperFunctions.h"
#include "OdometryDataManager.h"
#define MIN_OBJ_HEIGHT_M 0.04f

#include <mf_mempark/mf_mempark_generated_types.h>


class SuspensionEstimation
{
public:
    enum { FL_WHL_IDX = 0, FR_WHL_IDX, RL_WHL_IDX, RR_WHL_IDX, WHEELS_NUM };

    struct Vec3D  // 3D vector
    {
        float32_t x{ 0.0F };
        float32_t y{ 0.0F };
        float32_t z{ 0.0F };
    };

    struct Vec2D  // 2D vector
    {
        float32_t x{ 0.0F };
        float32_t y{ 0.0F };
    };

    // input: suspension sensor raw data
    // output: linearized sensor data, offset zero when unloaded
    static std::array<float32_t, WHEELS_NUM> sensorModel(std::array<float32_t, WHEELS_NUM> rawSens_m);

    // input: calibrated sensor data, coordinates of requested height
    // output: suspension pitch/roll and height
    static std::tuple<float32_t, float32_t, float32_t> suspEstimation(std::array<float32_t, WHEELS_NUM> calibSens_m, Vec2D const reqPointOnPlane);

private:
    // vehicle parameters
    static constexpr float32_t wheelBase_m{ 2.78F };
    static constexpr float32_t trackFront_m{ 1.58F };
    static constexpr float32_t trackRear_m{ 1.57F };
    // defines the position of the data returned by the sensor model

    // sensor calibration
    static constexpr float32_t gradSuspFront_mpm{ 1.1853F };  // gradient to adjust fix point on car
    static constexpr float32_t offSuspFront_m{ 0.3061F };  // offset to adjust fix point on car
    static constexpr float32_t offZeroLoadFront_m{ -0.689F };  // adjust level to be zero when unloaded

    static constexpr float32_t gradSuspRear_mpm{ 1.05334962160862F };  // gradient to adjust fix point on car
    static constexpr float32_t offSuspRear_m{ 0.362F };  // offset to adjust fix point on car
    static constexpr float32_t offZeroLoadRear_m{ -0.689F };  // adjust level to be zero when unloaded
};


/* signals for driver concept in SemiAP mode */
unsigned char driverReqToBrakeFlag_nu;
unsigned char driverReqToAccelFlag_nu;
unsigned char undershoot_delay_cntr = 0;
bool carStopped_nu = true;
bool gearChangeRequested_nu = false;
unsigned char currStroke_nu = 0;
int currGearReq_nu;
float distToStopReq_m = 0.0f;
const bool doUndershoot_nu[10] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //undershoot = stop car earlier + change gear
const float undershootDist_m[10] = { 0.0f, 0.25f, 0.15f, 0.15f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }; //distance before end of stroke where to stop in case undershoot is desired
const float semiAPvelocityLimitReq_mps = 0.83f;
const float semiAPdistToStopAtAccel_m = 4.0f;

/*STATIC Variables&Consts*/
const float AP_DATA_PROC_CYCLE_TIME_S = 0.001f;
static const char ThisModelClass[] = "VehicleControl";
static const char ThisModelKind[] = "mdl_APCtrl";
static const int  ThisVersionId = 1;
static ap_psm::ResetOriginRequestPort prevResetOriginRequestPort;

enum class JsonExportType {
    NO_EXPORT, EXPORT_WHEN_FINISHED, EXPORT_WHEN_TARGET_POSE_SELECTED, EXPORT_WHEN_REPLAN_TRIGGERED
};
static JsonExportType jsonExportType_nu = JsonExportType::NO_EXPORT;
static std::string jsonFileGenerationRoot_nu;
static LSM_GEOML::Pose poseBeforePlanning;

const uint64_t REMOTE_HMI_CYCLE_TIME_MS = 100;
const uint8_t REMOTE_HMI_ALIVE_COUNTER_MAX = 15;
const uint64_t TICEST_SAMPLE_TIME_MS = 10;
const uint64_t CEM_LSM_CYCLE_TIME{ 33U };


static bool   isFirstCycle, isFirstGearReq;
static unsigned int pathBeforeActIndex;
static float steerAngReqIn, steerAngReqOut;
static float32_t longDistToTarget_m = 0.0f;
static LSM_GEOML::Pose intialPose;
LSM_GEOML::Pose lastTransformation{};
static LSM_GEOML::Pose targetPoseKPICheck{};
static bool targetPoseReachedStatusSet{ false };

// define variables for DVA definition
static const unsigned maxNumStaticStructureForDVA{ std::min(8U, static_cast<unsigned int>(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU)) };
static uint8_t numValidPointsStaticStructure[maxNumStaticStructureForDVA] = { 0 };
static float32_t dummyStaticObjectShapePoint{ 0.0f };
static float32_t dummyVirtualLinePoint{ 0.0f };
#ifdef VARIANT_CUS_ONLY
static uint8_t variant_identifier_nu{ 1 };
static uint8_t variant_refObjClass_nu{ 5U };
#else
static uint8_t variant_identifier_nu{ 0 };
#endif
// data for plotting replanning information
static LSM_GEOML::Pose replanPositionCMOrigin[256U];     //256 based on data type uint8_t of trjplaDebugPort.mNumOfReplanCalls
static uint8_t replanCounter_nu{ 0U };
static ap_tp::TrajPlanVisuPort previousTrajectoryPortCMOrigin;
static ap_tp::TargetPose previousTargetPose;
static float globalSCurveLength_m{ 0.0f };

/*input data*/
static ap_vehstatesigprovider::StarterStatusPort starterStatusPort;
static ap_vehstatesigprovider::TrunkLidStatusPort trunkLidStatusPort;
static ap_vehstatesigprovider::ConvertibleTopStatusPort convertibleTopStatusPort;
static ap_vehstatesigprovider::SteeringColSwitchesStatusPort steeringColSwitchesStatusPort;
static ap_vehstatesigprovider::TrailerStatusPort trailerStatusPort;
static ap_vehstatesigprovider::DoorStatusPort doorStatusPort;
static ap_vehstatesigprovider::VehicleOccupancyStatusPort vehicleOccupancyStatusPort;
static ap_vehstatesigprovider::AdditionalBCMStatusPort additionalBCMStatusPort;
static ap_vehstatesigprovider::ACCInformationPort accInformationPort;
static ap_vehstatesigprovider::KeylessStatusPort keylessStatusPort;
static ap_vehstatesigprovider::AuthenticationStatusPort authenticationStatusPort;
static ap_vehstatesigprovider::APCtrlStatusPort apCtrlStatusPort;
static ap_vehstatesigprovider::ESCInformationPort escInformationPort;
static ap_vehstatesigprovider::ExternalFunctionStatusPort externalFunctionStatusPort;
static ap_hmitoap::HMIOutputPort gHMIOutputPort;
static ap_hmitoap::RemoteHMIOutputPort remoteHMIOutputPort;
static ap_lodmc::LoDMCStatusPort loDMCStatusPort;

/* suspension */
tDDictEntry *pSuspFL, *pSuspFR, *pSuspRL, *pSuspRR;  // handle to Carmaker quantity
float32_t groundHeightCalcFL, groundHeightCalcFR, groundHeightCalcRL, groundHeightCalcRR;
float32_t cmSuspPitch_rad, cmSuspRoll_rad, cmSuspHeight_m;




#ifdef MOCO_REPLACES_LODMC
#include "mf_trjctl_types/MoCo_Outputs.h"
#include "MocoWrapper.h"
#include "OCTAGONBrake.h"

// HACK: Enable simultaneous inclusion of mf_vedodo_types/LSM_VEDODO_Outputs.h and mf_vedodo_types/LSM_VEDODO_Outputs_C.h
#undef LSM_VEDODO_OUTPUTS_HEADER_
// HACK: Enable simultaneous inclusion of mf_trjctl_types/AP_TRJCTL_Outputs.h and mf_trjctl_types/AP_TRJCTL_Outputs_C.h
#undef AP_TRJCTL_OUTPUTS_HEADER_
#undef AP_COMMONVEHSIGPROVIDER_HEADER_
// HACK: define missing C-type variant of com::ComSignalState_t
typedef enum {
    COMSIGSTATE_INIT = 0U,  //!< COMSIGSTATE_INIT   data or struct is not valid
    COMSIGSTATE_VALID = 1U,  //!< COMSIGSTATE_VALID  data is valid for other usage
    COMSIGSTATE_INVALID = 2U   //!< COMSIGSTATE_INVALID  data is invalid or out of range, usage with care
  // COMSIGSTATE_ERROR, COMSIGSTATE_TIMEOUT: incompatible to ADAS AlgoSignalState_t, removed. use COMSIGSTATE_INVALID instead
} ComSignalState_t;
#include "VECONA/vecona_vsp_ext.h"
#include "TRATCO/tratco_vsp_ext.h"

//MoCo Inputs
static DF_t_ControlData controlData_TRATCO{};
static DF_t_ControlData controlData_VECONA{};
static FCU_t_VehParam vehParam{};
static FCU_t_BrakeFb brakeFb{};
static FCU_t_PowertrainFb powertrainFb{};
static FCU_t_SteeringFrontFb steeringFrontFb{};
static FCU_t_SteeringRearFb steeringRearFb{};
static VECONA_t_VehDyn vehDynFcu{};
//TRATCO outputs
static TRATCO_t_CpldTratcoStatus tratcoStatusPort{};
static TRATCO_t_LongAccelReq longAccelReq{};
//VECONA outputs
static VECONA_t_LongVeconaStatus longVeconaStatusPort{};
static VECONA_t_BrakeReq veconaBrakeRequest{};
static VECONA_t_PowertrainReq veconaPowertrainRequest{};
static FCU_t_LongDriverFb longDriverFeedback{};
static VECONA_t_VehDyn vehDynVecona{};
//MocoCarmakerInput
static MocoSimulatorInput mocoCarmakerInput{};
static tDDictEntry* OCTAGONEngine_MaxTrq_DDictEntry{ nullptr };
static tDDictEntry* OCTAGONEngine_MinTrq_DDictEntry{ nullptr };
static tDDictEntry* gasInterpretedTrq_DDictEntry{ nullptr };
//TRATCO debug
static TRATCO_VSPProcMem tratcoProcessMemory{};
static TRATCO_Inter_Mem_Data tratcoInterMemData{ {0U} };
static TRATCO_InterMeas_Mem_Data tratcoInterMeasMemData{ {0U} };
static TRATCO_IntraMeas_Mem_Data tratcoIntraMeasMemData{ {0U} };
static TRATCO_ProcMemInternal tratcoProcMem{};
//VECONA debug
static VECONA_VSPProcMem veconaProcessMemory{};
static VECONA_Inter_Mem_Data veconaInterMemData{ {0U} };
static VECONA_InterMeas_Mem_Data veconaInterMeasMemData{ {0U} };
static VECONA_IntraMeas_Mem_Data veconaIntraMeasMemData{ {0U} };
static VECONA_ProcMemInternal veconaProcMem{};
#endif

static ap_ladmc::LaDMCStatusPort laDMCStatusPort;
static ap_commonvehsigprovider::AmbientDataPort ambientDataPort;
static ap_commonvehsigprovider::OdoGpsPort odoGpsPort;
static ap_commonvehsigprovider::OdoExtCtrlPort odoExtCtrlPort;
static ap_commonvehsigprovider::SystemTimePort systemTimePort;
static ap_commonvehsigprovider::EngineCtrlStatusPort engineCtrlStatusPort;
static ap_commonvehsigprovider::GearboxCtrlStatusPort gearboxCtrlStatusPort;
static ap_commonvehsigprovider::SuspensionPort suspensionPort;
static ap_commonvehsigprovider::WheelPulsePort wheelPulsePort;
static ap_commonvehsigprovider::WheelDrivingDirectionsPort wheelDrivingDirectionsPort;
static ap_commonvehsigprovider::WheelSpeedPort wheelSpeedPort;
static ap_commonvehsigprovider::MotionStatePort motionStatePort;
static ap_commonvehsigprovider::VehDynamicsPort vehDynamicsPort;
static ap_commonvehsigprovider::SteerCtrlStatusPort steerCtrlStatusPort;
static ap_commonvehsigprovider::TRJCTLGeneralInputPort trjctlGeneralInputPort;
static ap_commonvehsigprovider::DdsPort ddsPort;
static ap_commonvehsigprovider::BrakeCtrlStatusPort brakeCtrlStatusPort;
static si::ApEnvModelPort envModelPort;
static si::CollEnvModelPort collEnvModelPort;
static si::ApEnvModelPort envModelPortCMOrigin;
static si::ApParkingBoxPort gParkingBoxPort;
static si::ApParkingBoxPort parkingBoxPortCMOrigin;
static si::EgoMotionPort egoMotionPort;
static si::EgoMotionPort egoMotionPortCM;
static si::PerceptionAvailabilityPort perceptionAvailabilityPort;
static mf_mempark::MemoryParkingStatusPort memoryParkingStatusPort{};
static lsm_vedodo::OdoEstimationOutputPort odoEstimationOutputPortCM{};
static lsm_vedodo::OdoEstimation& odoEstimationPortCM{ odoEstimationOutputPortCM.odoEstimation };

// ultrasonic algorithm (us_processing + us_em)
static us_drv::UsDrvDetectionList usDrvDetectionListCopy{};
static us_drv::UsDrvRuntimeConfiguration usDrvRntConfigRequest{};
static us_processing::UsProcessingPointList uspPointListOutput{};
static us_processing::UsProcessingFilteredEchoList usFilteredEchoOutput{};
static us_processing::UsProcessingDistanceList uspDistListOutput{};
static us_processing::UsProcessingDiagOutput uspDiagOutput{};
static us_processing::UsProcessingDataIntegrity uspIntegrityOutput{};
static us_em::UsEmDebugOutputPort usEmDebugPort{};
static us_em::UsEnvModelPort usEnvModelPort{};
static us_em::PerceptionAvailabilityPort usPercAvailPort{};

#ifndef VARIANT_CUS_ONLY
static svc_model_processing::SvcModelProcessingOutput svcModelProcessingOutput;
static svc_model_processing::SvcModelProcessingInput svcModelProcessingInput;
static svc_model_processing::SvcModelProcessingSettings svcModelProcessingSettings;
static bool isSvcModelProcessingDataAllocated = false;
#endif

// latest wheel pulse
float latestWheelSpeed_radps[4] = { 0.0F,0.0F,0.0F,0.0F };
float realTimeStampLatestTicks_s[4] = { 0.0F,0.0F,0.0F,0.0F };
float TimeStampDelta_s[4] = { 0.0F,0.0F,0.0F,0.0F };
float DeltaTimeStampLatestTicks_nu[4] = { 0.0F,0.0F,0.0F,0.0F };
float TimeStampLatestTicks_nu[4] = { static_cast<float32_t>(SimCore.Time), static_cast<float32_t>(SimCore.Time), static_cast<float32_t>(SimCore.Time), static_cast<float32_t>(SimCore.Time) };
float32_t latestWheelSpeedFiltred_radps[4] = { 0, 0 , 0 , 0 };
uint16_t CountIterationsToZero_nu[4] = { 0, 0, 0, 0 }, diffWheelpulses_nu[4] = { 0,0,0,0 }, wheelPulsesLast_nu[4] = { 0, 0, 0, 0 }, wheelPulsesFL_nu = 0, wheelPulsesRL_nu = 0, wheelPulsesFR_nu = 0, wheelPulsesRR_nu = 0;
float TimeAfterJitter_s[4] = { 0,0,0,0 }, LatestWheelSpeedFilteredJt_radps[4] = { 0,0,0,0 }, WheelPulsesJt_nu[4] = { 0,0,0,0 };
double JitterTime_s[4] = { 0,0,0,0 }, JitterTimeLast_s[4] = { 0,0,0,0 }, TimeLast20ms_s[4] = { SimCore.Time,SimCore.Time,SimCore.Time,SimCore.Time }, Timedifference20ms_s[4] = { 0,0,0,0 };
static ap_common::DiscreteFilterPT1 PT1Filter[4];
uint16_t DesicionWhlMdlExt = 0;

/*output data*/
static ap_trjctl::LaDMCCtrlRequestPort laDMCCtrlRequestPort;
static ap_trjctl::LoDMCCtrlRequestPort loDMCCtrlRequestPort;
#ifdef MOCO_REPLACES_LODMC
static AP_TRJCTL::MF_CONTROL_t_LongManeuverRequestPort longManeuverRequestPort;
#endif
static ap_trjctl::GearboxCtrlRequestPort gearBoxCtrlRequestPort;
static mf_lsca::LscaStatusPort lscaStatusPort;
static ap_trjctl::MFControlStatusPort mfControlStatusPort;
static ap_psm::TrajCtrlRequestPort trajCtrlRequestPort;
static ap_psm_app::PSMToSSPPort psmToSSPPort;
static ap_psm::SlotCtrlPort slotCtrlPort;
static mf_hmih::HMIGeneralInputPort gHmiInputPort;
static mf_hmih::HeadUnitVisualizationPort headUnitVisualizationPort;
static mf_hmih::RemoteVisualizationPort remoteVisualizationPort;
static ap_hmitoap::VisuInputData visuInputData;
static mf_hmih::UserDefinedSlotPort userDefinedSlotPort;
static mf_hmih::APUserInteractionPort apUserInteractionPort;
static mf_hmih::LVMDUserInteractionPort lvmdUserInteractionPort;
static lsm_vedodo::OdoEstimationOutputPort odoEstimationOutputPort;
static lsm_vedodo::OdoNVMData odoPersistentDataPort;
static tce::TceEstimationPort tceEstimationPort;
static tce::TcePersDataPort tcePersDataPort;
static ap_trjctl::DrivingResistancePort drivingResistancePort;
static mf_manager::LaCtrlRequestPort laCtrlRequestPort;
static mf_manager::LoCtrlRequestPort loCtrlRequestPort;
static mf_manager::TrajRequestPort trajRequestPort;
static mf_drvwarnsm::AppToCoreSMPort appToCoreSMPort;
static mf_drvwarnsm::DrvWarnStatusPort drvWarnStatusPort;
static mf_drvwarnsm_core::DrvWarnCoreStatusPort drvWarnCoreStatusPort;
static mf_drvwarnsm::LogicToProcPort logicToProcPort;
static pdcp::PDCPSectorsPort pdcpSectorsPort;
static pdcp::PDCPDrivingTubePort pdcpDrivingTubePort;
static pdcp::ProcToLogicPort pdwProcToLogicPort;
static mf_tonh::ToneOutputPort toneOutputPort;
static mf_whlprotectproc::WHPProcOutputPort whpOutputPort;
static mf_lvmd::LvmdStatusPort lvmdStatusPort;
static avga_swc::AVGA_SupervisionRequest avgaSupervisionRequestLSCA;
static avga_swc::AVGA_SupervisionRequest avgaSupervisionRequestPDW;
static avga_swc::AVGA_AutomatedVehicleGuidanceState automatedVehicleGuidanceStateAUP;
static avga_swc::AvgaStopRequestPort avgaStopRequestMCRA;

/*debug data */
static ap_psm_app::CtrlCommandPort gCtrlCommandPort;
static ap_psm::PARKSMCoreStatusPort gPARKSMCoreStatusPort;
static ap_psm::PARKSMCoreDebugPort gPARKSMCoreDebugPort;
static ap_tp::PlannedTrajPort plannedTrajPort;
static ap_tp::TargetPosesPort targetPosesPort;
static ap_tp::TargetPosesPort targetPosesPortCMOrigin;
static ap_common::CarMakerInterface carMakerInterface;
static lsm_vedodo::OdoDebugPort odoDebugPort;
static tce::TceDebugPort tceDebugPort;
static ap_psm_app::PSMDebugPort psmDebugPort;
static mf_hmih::MFHmiHDebugPort gMFHmiHDebugPort;
static ap_trjctl::TrajCtrlDebugPort trajCtrlDebugPort;
static ap_tp::TAPOSDDebugPort taposdDebugPort;
static ap_tp::TAPOSDDebugPort taposdDebugPortCMOrigin;
static ap_tp::TrajPlanDebugPort trjplaDebugPort;
static ap_tp::TrajPlanVisuPort trjplaVisuPort;
static ap_tp::TrajPlanVisuPort trjplaVisuPortCMOrigin;
static ap_tp::ReverseAssistAvailabilityPort reverseAssistAvailabilityPort;
static mf_lsca::structs::plot_t lscaPlotDataPort;
static mf_drvwarnsm::DrvWarnDebugPort drvWarnDebugPort;
static pdcp::PDCPDebugPort pdcpDebugPort;

/*Hold data*/
static bool tapOnStartParkingConfirmed_nu;
static double lastAng_rad;
static bool hadSteerCtrlRequest_nu;
static int lastGearReq_nu;
static bool hadGearReq_nu;
static bool hadSteerReq_nu;
static bool parkingOnLeftSide_nu[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU] = { false };
static bool vehicleWasSecured_nu;
static bool vehicleWasSecuredMoCo_nu;
static uint8_t userActionHeadUnitCMQuant_nu;
static uint8_t userActionRemoteDeviceCMQuant_nu;
static bool useDeadManSwitchRemoteApp = 0;
static bool fakeKeyFonAliveCounterFlag = 0;
static unsigned long fingerPositionXCMQuant_px;
static unsigned long fingerPositionYCMQuant_px;
static bool remDevPaired_nu = 0;
static bool remDevConnected_nu = 0;
static uint8_t overwriteVehVelocityCur_nu;
static float vehVelocityCurCMQuant_mps;
static uint8_t overwritePathBeforeFuncActivation_nu;
static LSM_GEOML::Pose pathBeforeFuncActivation0Pose;
static double lastCallTime_us = 0.0;
static int segment_selector; //1-12
static float velocityRequestIntern_mps = 0.0f;
static LSM_GEOML::Pose inverseTransformation;
//static uint8_t leverPosFromCarMakerVariable = 0;
static double lastTimeVedodoReset_s{ -1.0 };
static LSM_GEOML::CoordinateTransformer2D vedodoOffsetAtComponentReset{};
/**/

// Test Evaluation
static TestEvaluation testEvaluation;
static OptimalTargetPose optimalTargetPoseSIOrigin;

//AlBu
TrafficContour2D trafficContour2D_t[MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM];

bool enableLimitFieldOfView_nu = false;
bool enableLatencyEffect_nu = false;
static float latencyEffectTime_s = 0.1f;
static float inflationLength_m = 0.0f;//amoraru
static float maxCycleTimeOfAUPStep_ms;

int detectedPB[2][12];
static float steeringWheelAngleAcceleration = 0.0f;
static float steeringAngleAcceleration = 0.0f;
static int TCEActive_nu = 0;

#ifndef VARIANT_CUS_ONLY
EnvironmentPerception environmentPerception;
GroundTruthEnvironment groundTruthEnvironment;
#endif

EvaluationPort gEvaluationPort;

// sRoad traffic object positions required for SI SlotDetectionTime and SlotPersistence evaluation.
static float32_t sRoadT01_m{ .0f };
static float32_t sRoadLim01_m{ .0f };
static float32_t sRoadFurthestObject_m{ .0f };
static float32_t yawAngleFurthestObject_rad{ .0f };

/*ThGo Start*/
static double c_i, c_p, c_d, c;
static double gain_p_f, gain_i_f, gain_p_r, gain_i_r, gain_slope;
static double cmFiltCoeff_ay;
static double curAccel_mps2;
float Acc_y_filt = 0;
float accelReq_mps2;
double delta_ax;
double *GB_Ratios_front;
double *GB_Ratios_rear;
static float accOld_mps;  // low pass filtering of longitudinal acceleration
bool ApCtrl_boolBrkAcv;
bool ApCtrl_boolHldBrk = false;
double ApCtrl_stBrkLstVal;
bool ApCtrl_boolNoRvsHld = false;
double curacc;
#define PI 3.14159265359f;
float32_t ApCtrl_stGrdtRoad_C;
float ApCtrl_stMinBrkValAntiExc_C;
float32_t ApCtrl_facForBrkCor_C;
float ApCtrl_dstLstToStop;
float32_t ApCtrl_aLimForCtrlFromSlope_C;
float32_t ApCtrl_stBrkLimFromSlope_C;
double slope_rad;
bool test_bool = false;
// Yaw rate sensor
bool extension_wanted = false;
float32_t yawRateEgoCurMod_rad;
/*ThGo End*/
static int sideslip_simulation_active{ 0 };
//distance by which the turn point is shifted
static double sideslip_lh_m{ 0.0 };

static std::mt19937 gRandomEngine{}; // explicitly use mt19937 instead of default_random_engine, since the latter is implementation specific

static VCEM::CemSurrogate cemSurrogate{};
static VCEM::CemSurrogateConfig cemSurrogateConfig{};

#ifndef NO_SCENE_INTERPRETATION
static SiHighWrapper siHighWrapper{};
#endif
static OdometryDataManager odometryDataManger{};

static eco::CarMakerSystemServices carMakerSystemServices{};

bool tap_on_start_selection{ false };
bool tap_on_start_parking{ false };
bool user_input{ false };

/******************************************************************************/
static void performValidDeadManSwitchRemoteApp() {
    static uint8_t counter = 0;
    if (remoteHMIOutputPort.fingerPositionX_px >= 1199 || remoteHMIOutputPort.fingerPositionX_px <= 801) {
        remoteHMIOutputPort.fingerPositionX_px = 1000;
    }
    if (remoteHMIOutputPort.fingerPositionY_px >= 2339 || remoteHMIOutputPort.fingerPositionY_px <= 1701) {
        remoteHMIOutputPort.fingerPositionY_px = 2000;
    }
    if (counter < 100) {
        remoteHMIOutputPort.fingerPositionX_px += 1;
        remoteHMIOutputPort.fingerPositionY_px += 1;
    }
    else if (counter >= 100 && counter < 200) {
        remoteHMIOutputPort.fingerPositionX_px -= 1;
        remoteHMIOutputPort.fingerPositionY_px -= 1;
    }
    if (counter >= 200) {
        counter = 0;
    }
    counter++;
}

static void fakeKeyFobAliveCounter() {
    static uint8_t aliveCounter = 0;
    if (15 == aliveCounter) {
        aliveCounter = 0;
    }
    else {
        aliveCounter++;
    }
    keylessStatusPort.keylessStatusPortCANAlive_nu = aliveCounter;
}

static bool startParkingPoss() {
    bool possible{ false };
    for (int i = 0; i < mf_hmih::MF_HMIH_Consts::AP_H_MAX_NUM_SLOTS_SIDE_NU; i++) {
        possible |= static_cast<bool>(gHmiInputPort.parkingSpaces.right.selected_nu[i]);
        possible |= static_cast<bool>(gHmiInputPort.parkingSpaces.left.selected_nu[i]);
        possible |= static_cast<bool>(gHmiInputPort.parkingSpaces.front.selected_nu[i]);
        possible |= static_cast<bool>(gHmiInputPort.parkingSpaces.rear.selected_nu[i]);
        if (possible) {
            break;
        }
    }
    return possible;
}

static bool directionSwitchPoss() {
    bool possible{ false };
    for (int i = 0; i < mf_hmih::MF_HMIH_Consts::AP_H_MAX_NUM_SLOTS_SIDE_NU; i++) {
        if (gHmiInputPort.parkingSpaces.right.selected_nu[i]) {
            possible = (mf_hmih::PossibleDirection::POSS_DIR_BOTH_DIRECTIONS == gHmiInputPort.parkingSpaces.right.possDirection_nu[i]);
            break;
        }
        if (gHmiInputPort.parkingSpaces.left.selected_nu[i]) {
            possible = (mf_hmih::PossibleDirection::POSS_DIR_BOTH_DIRECTIONS == gHmiInputPort.parkingSpaces.left.possDirection_nu[i]);
            break;
        }
        if (gHmiInputPort.parkingSpaces.front.selected_nu[i]) {
            possible = (mf_hmih::PossibleDirection::POSS_DIR_BOTH_DIRECTIONS == gHmiInputPort.parkingSpaces.front.possDirection_nu[i]);
            break;
        }
        if (gHmiInputPort.parkingSpaces.rear.selected_nu[i]) {
            possible = (mf_hmih::PossibleDirection::POSS_DIR_BOTH_DIRECTIONS == gHmiInputPort.parkingSpaces.rear.possDirection_nu[i]);
            break;
        }
    }
    return possible;
}

static bool requestedDirectionPoss(Parking_Maneuver requestedParkingManeuver_nu) {
    bool possible{ false };
    //For undefined requested direction don't check the direction
    if (Parking_Maneuver::NOT_SPECIFIED == requestedParkingManeuver_nu) {
        possible = true;
    }
    //Otherwise check for the direction
    else {
        for (int i = 0; i < mf_hmih::MF_HMIH_Consts::AP_H_MAX_NUM_SLOTS_SIDE_NU; i++) {
            if (gHmiInputPort.parkingSpaces.right.selected_nu[i]) {
                if (Parking_Maneuver::BACKWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_BACKWARDS == gHmiInputPort.parkingSpaces.right.selectedDirection_nu[i]);
                else if (Parking_Maneuver::FORWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_FORWARDS == gHmiInputPort.parkingSpaces.right.selectedDirection_nu[i]);
                else {}
                break;
            }
            if (gHmiInputPort.parkingSpaces.left.selected_nu[i]) {
                if (Parking_Maneuver::BACKWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_BACKWARDS == gHmiInputPort.parkingSpaces.left.selectedDirection_nu[i]);
                else if (Parking_Maneuver::FORWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_FORWARDS == gHmiInputPort.parkingSpaces.left.selectedDirection_nu[i]);
                else {}
                break;
            }
            if (gHmiInputPort.parkingSpaces.front.selected_nu[i]) {
                if (Parking_Maneuver::BACKWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_BACKWARDS == gHmiInputPort.parkingSpaces.front.selectedDirection_nu[i]);
                else if (Parking_Maneuver::FORWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_FORWARDS == gHmiInputPort.parkingSpaces.front.selectedDirection_nu[i]);
                else {}
                break;
            }
            if (gHmiInputPort.parkingSpaces.rear.selected_nu[i]) {
                if (Parking_Maneuver::BACKWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_BACKWARDS == gHmiInputPort.parkingSpaces.rear.selectedDirection_nu[i]);
                else if (Parking_Maneuver::FORWARD == requestedParkingManeuver_nu) possible = (mf_hmih::SelectedDirection::SEL_DIR_FORWARDS == gHmiInputPort.parkingSpaces.rear.selectedDirection_nu[i]);
                else {}
                break;
            }
        }
    }
    return possible;
}

static bool selectedSlotAvailable(const ap_hmitoap::HMIOutputPort& hmiOutputPort) {
    bool possible{ true };
    switch (hmiOutputPort.userActionHeadUnit_nu) {
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_1:
        if (!gHmiInputPort.parkingSpaces.right.free_nu[0]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_2:;
        if (!gHmiInputPort.parkingSpaces.right.free_nu[1]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_3:;
        if (!gHmiInputPort.parkingSpaces.right.free_nu[2]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_4:;
        if (!gHmiInputPort.parkingSpaces.right.free_nu[3]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_1:;
        if (!gHmiInputPort.parkingSpaces.left.free_nu[0]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_2:;
        if (!gHmiInputPort.parkingSpaces.left.free_nu[1]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_3:;
        if (!gHmiInputPort.parkingSpaces.left.free_nu[2]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_4:;
        if (!gHmiInputPort.parkingSpaces.left.free_nu[3]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_1:;
        if (!gHmiInputPort.parkingSpaces.front.free_nu[0]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_2:;
        if (!gHmiInputPort.parkingSpaces.front.free_nu[1]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_3:;
        if (!gHmiInputPort.parkingSpaces.front.free_nu[2]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_4:;
        if (!gHmiInputPort.parkingSpaces.front.free_nu[3]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_1:;
        if (!gHmiInputPort.parkingSpaces.rear.free_nu[0]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_2:;
        if (!gHmiInputPort.parkingSpaces.rear.free_nu[1]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_3:;
        if (!gHmiInputPort.parkingSpaces.rear.free_nu[2]) {
            possible = false;
        }
        break;
    case ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_4:;
        if (!gHmiInputPort.parkingSpaces.rear.free_nu[3]) {
            possible = false;
        }
        break;
    }
    return possible;
}

//iterates trough detected parking boxes, checks which parking box overlaps the most with the parking box markings from the CM testrun
//and uses gHMIOutputPort to select the coresponding parking box
static void chooseCorrectParkingSlot(const ap_hmitoap::HMIOutputPort& hmiOutputPort) {
    static bool automated_selection_warning{ false }; // Whether a warning has already been issued, since something went wrong during automated selection
    static double automated_selection_time_s{ -1.0 };  // timestamp when the automated slot selection has been made for the first time
    if (isFirstCycle) {
        automated_selection_warning = false;
        automated_selection_time_s = -1.0;
    }
    //the automated parking slot selection needs to happend after TAP_ON_START_SELECTION (tap_on_start_selection = true),
    //before TAP_ON_START_PARKING (tap_on_start_parking = false)
    //and when no user input of TAP_ON_PARKING_SPACE_ type was given in the minimanuvers (user_input = false)
    //this way automatic parking slot selection gives priority to the minimaneuver user inputs
    //Furthermore, the slot selection is only set for 100 ms (in the real vehicle, HMI user signals are only sent for 100 ms)
    if (!user_input && tap_on_start_selection && !tap_on_start_parking &&
        ap_hmitoap::UserActionHeadUnit::NO_USER_ACTION == hmiOutputPort.userActionHeadUnit_nu &&
        (automated_selection_time_s < 0.0 || (SimCore.Time - automated_selection_time_s) < 0.1) &&
        !automated_selection_warning) {
        //Get testrun name from SimCore.TestRun.Name and split by "/" until only the tesrun name remains
        std::string TestRunName = SimCore.TestRun.Name;

        while (TestRunName.find("/") != std::string::npos) {
            TestRunName = TestRunName.substr(TestRunName.find("/") + 1, TestRunName.length() - TestRunName.find("/"));
        }

        //Based on the test runs name determine parking type
        si::ParkingScenarioTypes parking_type{ si::ParkingScenarioTypes::MAX_NUM_PARKING_SCENARIO_TYPES };
        if (TestRunName.find("_Par") != std::string::npos || TestRunName.find("Parallel") != std::string::npos) {
            parking_type = si::ParkingScenarioTypes::PARALLEL_PARKING;
        }
        else if (TestRunName.find("_Perp") != std::string::npos || TestRunName.find("Perpendicular") != std::string::npos) {
            parking_type = si::ParkingScenarioTypes::PERPENDICULAR_PARKING;
        }
        else if (((TestRunName.find("_Ang") != std::string::npos) && (TestRunName.find("_B") != std::string::npos)) ||
            ((TestRunName.find("Angled_") != std::string::npos) && (TestRunName.find("_Backward") != std::string::npos))) {
            parking_type = si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_FRONT;
        }
        else if (((TestRunName.find("_Ang") != std::string::npos) && (TestRunName.find("_F") != std::string::npos)) ||
            (TestRunName.find("Angled_") != std::string::npos)) {
            parking_type = si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_BACK;
        }
        else {
            LogWarnStr(EC_General, "Automatic selection of parking box not possible: parking scenario type not encoded in TestRun name");
            automated_selection_warning = true;
            return;
        }

        // Pick the first traffic object that has "PB" in the name
        // use trafic object and a Pose object to compute the global coordinates of the parking box corners
        // (this way rotation is also taken into consideration)
        // create a Polygon2D object for the parking box using transformed points
        LSM_GEOML::Polygon2D<4> parkingBoxPoly;

        for (int i = 0; i < std::min(Traffic.nObjs, static_cast<int>(MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM)); i++) {
            if (strstr(trafficContour2D_t[i].name, "PB")) {
                const tTrafficObj& parkingBoxTrafficObj = *Traffic_GetByTrfId(trafficContour2D_t[i].trafficId);

                cml::Vec2Df trafficObjPosition{ static_cast<float32_t>(parkingBoxTrafficObj.t_0[0]), static_cast<float32_t>(parkingBoxTrafficObj.t_0[1]) };
                LSM_GEOML::Pose parkingBoxPose{ trafficObjPosition, static_cast<float32_t>(parkingBoxTrafficObj.r_zyx[2]) };

                const LSM_GEOML::CoordinateTransformer2D transformToFrTraffic{ parkingBoxPose };
                cml::Vec2Df p1{ trafficContour2D_t[i].points[0][0], trafficContour2D_t[i].points[1][0] };
                cml::Vec2Df p2{ trafficContour2D_t[i].points[0][1], trafficContour2D_t[i].points[1][1] };
                cml::Vec2Df p3{ trafficContour2D_t[i].points[0][2], trafficContour2D_t[i].points[1][2] };
                cml::Vec2Df p4{ trafficContour2D_t[i].points[0][3], trafficContour2D_t[i].points[1][3] };

                p1 = transformToFrTraffic.transform(p1);
                p2 = transformToFrTraffic.transform(p2);
                p3 = transformToFrTraffic.transform(p3);
                p4 = transformToFrTraffic.transform(p4);

                parkingBoxPoly = LSM_GEOML::Polygon2D<4>{ p1,p2,p3,p4 };

                break;
            }
        }

        // Warning: no parking box traffic object was found
        if (parkingBoxPoly.empty()) {
            LogWarnStr(EC_General, "Automatic selection of parking box not possible: no parking box traffic object found");
            automated_selection_warning = true;
            return;
        }

        uint16_t matching_slot_id{ std::numeric_limits<uint16_t>::max() };
        float max_overlap{ std::numeric_limits<float>::lowest() };

        //Go trough all the detected parking slots, pick the parking slot with correct parking scenario type
        //and the largest overlap with the parkingBoxPoly defined by the CarMaker traffic object.
        //The overlap of the parking box polygons is calculated via calcMinTranslationVectorOfPolyOverlap.
        for (int i{ 0 }; i < gParkingBoxPort.numValidParkingBoxes_nu; i++) {
            if (parking_type == gParkingBoxPort.parkingBoxes[i].parkingScenario_nu)
            {
                const LSM_GEOML::Polygon2D<4U>& parkingSlot = convert(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m);
                const float32_t parkingSlotOverlap = parkingBoxPoly.calcMinTranslationVectorOfPolyOverlap(parkingSlot).norm1();

                if (max_overlap < parkingSlotOverlap) {
                    max_overlap = parkingSlotOverlap;
                    matching_slot_id = gParkingBoxPort.parkingBoxes[i].parkingBoxID_nu;
                }
            }
        }
        // Warning: no matching parking box found
        if (std::numeric_limits<uint16_t>::max() == matching_slot_id) {
            LogErrStr(EC_General, "Automatic selection of parking box not possible: no matching parking box found");
            automated_selection_warning = true;
            return;
        }

        //use matching_slot_id to get the target pose ID because gHmiInputPort uses target pose IDs for parking slots
        uint8_t matching_target_pose_id{ std::numeric_limits<uint8_t>::max() };
        for (int j = 0; j < targetPosesPortCMOrigin.numValidPoses; j++) {
            //if perpendicular parking, check for maneuver direction
            if (parking_type == si::ParkingScenarioTypes::PERPENDICULAR_PARKING) {
                //if forward parking pick pose that has forward parking type
                if (testEvaluation.evaluationPort.parkingManeuver_nu == Parking_Maneuver::FORWARD) {
                    if (targetPosesPortCMOrigin.targetPoses[j].relatedParkingBoxID == matching_slot_id && targetPosesPortCMOrigin.targetPoses[j].type == ap_tp::PoseType::T_PERP_PARKING_FWD) {
                        matching_target_pose_id = targetPosesPortCMOrigin.targetPoses[j].pose_ID;
                        break;
                    }
                }//if maneuver is not forward it is either backwards or not specified, in either case use backwards parking pose
                else {
                    if (targetPosesPortCMOrigin.targetPoses[j].relatedParkingBoxID == matching_slot_id && targetPosesPortCMOrigin.targetPoses[j].type == ap_tp::PoseType::T_PERP_PARKING_BWD) {
                        matching_target_pose_id = targetPosesPortCMOrigin.targetPoses[j].pose_ID;
                        break;
                    }
                }
            }
            //if parking isn't perpendicular then treat it as parallel, where direcion does not matter
            else if (targetPosesPortCMOrigin.targetPoses[j].relatedParkingBoxID == matching_slot_id) {
                matching_target_pose_id = targetPosesPortCMOrigin.targetPoses[j].pose_ID;
                break;
            }
        }
        // Warning: no matching target pose was found
        if (std::numeric_limits<uint8_t>::max() == matching_target_pose_id) {
            LogErrStr(EC_General, "Automatic selection of parking box not possible: no matching target pose found");
            automated_selection_warning = true;
            return;
        }

        bool parking_space_selected{ false };
        auto selectParkingSpace = [&parking_space_selected](double& selection_time_s, const ap_hmitoap::UserActionHeadUnit parking_space_selection) {
            gHMIOutputPort.userActionHeadUnit_nu = parking_space_selection;
            // set first selection time
            if (selection_time_s < 0.0) {
                selection_time_s = SimCore.Time;
            }
            parking_space_selected = true;
        };
        //iterate trough gHmiInputPort.parkingSpaces to find the poseID_nu corresponding to matching_target_pose_id
        //and "tap" on the corresponding parking space
        for (int k = 0; k < mf_hmih::MF_HMIH_Consts::AP_H_MAX_NUM_SLOTS_SIDE_NU; k++) {
            if (gHmiInputPort.parkingSpaces.right.poseID_nu[k] == matching_target_pose_id) {
                switch (k)
                {
                case 0:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_1);
                    break;
                case 1:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_2);
                    break;
                case 2:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_3);
                    break;
                case 3:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_RIGHT_4);
                    break;
                }
            }
            else if (gHmiInputPort.parkingSpaces.left.poseID_nu[k] == matching_target_pose_id) {
                switch (k)
                {
                case 0:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_1);
                    break;
                case 1:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_2);
                    break;
                case 2:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_3);
                    break;
                case 3:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_LEFT_4);
                    break;
                }
            }
            else if (gHmiInputPort.parkingSpaces.front.poseID_nu[k] == matching_target_pose_id) {
                switch (k)
                {
                case 0:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_1);
                    break;
                case 1:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_2);
                    break;
                case 2:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_3);
                    break;
                case 3:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_FRONT_4);
                    break;
                }
            }
            else if (gHmiInputPort.parkingSpaces.rear.poseID_nu[k] == matching_target_pose_id) {
                switch (k)
                {
                case 0:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_1);
                    break;
                case 1:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_2);
                    break;
                case 2:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_3);
                    break;
                case 3:
                    selectParkingSpace(automated_selection_time_s, ap_hmitoap::UserActionHeadUnit::TAP_ON_PARKING_SPACE_REAR_4);
                    break;
                }
            }
            if (parking_space_selected) {
                break;
            }
        }

    }
}


static void checkUserInputPossible(const ap_hmitoap::HMIOutputPort& hmiOutputPort) {

    if (!selectedSlotAvailable(hmiOutputPort)) {
        LogErrStr(EC_General, "Invalid user action in MiniManeuver: Selected slot not available");
    }

    if ((ap_hmitoap::UserActionHeadUnit::TAP_ON_SWITCH_DIRECTION == hmiOutputPort.userActionHeadUnit_nu)
        && !directionSwitchPoss()) {
        LogErrStr(EC_General, "Invalid user action in MiniManeuver: Direction switch not possible; only current direction available");
    }

    //Reset tapOnStartParkingConfirmed_nu for new parking maneuver (e.g. parking out after parking in)
    if (ap_hmitoap::UserActionHeadUnit::TAP_ON_START_SELECTION == hmiOutputPort.userActionHeadUnit_nu) {
        //[automated parking slot selection] set tap_on_start_selection to true when TAP_ON_START_SELECTION input is given
        tap_on_start_selection = true;
        tapOnStartParkingConfirmed_nu = false;
    }

    if ((ap_hmitoap::UserActionHeadUnit::TAP_ON_START_PARKING == hmiOutputPort.userActionHeadUnit_nu) && (!tapOnStartParkingConfirmed_nu))
    {
        //[automated parking slot selection] set tap_on_start_parking to true when TAP_ON_START_PARKING input is given
        tap_on_start_parking = true;
        if ((!startParkingPoss()) ||
            ((static_cast<mf_hmih::ScreenHeadUnit>(carMakerInterface.headUnitScreen_nu) != mf_hmih::ScreenHeadUnit::PARKING_SPACE_SELECTION) &&
            (static_cast<mf_hmih::ScreenHeadUnit>(carMakerInterface.headUnitScreen_nu) != mf_hmih::ScreenHeadUnit::PARK_OUT_SIDE) &&
                (static_cast<mf_hmih::ScreenHeadUnit>(carMakerInterface.headUnitScreen_nu) != mf_hmih::ScreenHeadUnit::GP_START) &&
                (static_cast<mf_hmih::ScreenHeadUnit>(carMakerInterface.headUnitScreen_nu) != mf_hmih::ScreenHeadUnit::PDC_ACTIVE) &&
                (static_cast<mf_hmih::ScreenHeadUnit>(carMakerInterface.headUnitScreen_nu) != mf_hmih::ScreenHeadUnit::MP_USER_ADJUSMENTS)))
        {
            LogErrStr(EC_General, "Invalid user action in MiniManeuver: Start not possible. No parking slot selected (no free parking slot?!)");
        }
        else if (!requestedDirectionPoss(testEvaluation.evaluationPort.parkingManeuver_nu))
        {
            LogErrStr(EC_General, "Requested parking maneuver direction not possible.");
        }
        else {
            tapOnStartParkingConfirmed_nu = true;
        }
    }
    //[automated parking slot selection] set user_input to true when an input of type TAP_ON_PARKING_SPACE_ (has values between 1 and 16) is given
    if (userActionHeadUnitCMQuant_nu >= 1 && userActionHeadUnitCMQuant_nu <= 16) {
        user_input = true;
    }
}

float32_t returnEvaluationInfoFromTestrun(std::string testDescription_s)
{
    uint8_t ii = 0;
    // The input string has to start with a digit, so lets search for the first digit
    for (ii = 0; ii < testDescription_s.length(); ii++) { if (isdigit(testDescription_s[ii])) break; }
    //check if the number is negative/positive
    float32_t isNegPosValue = (testDescription_s[ii - 1] == '-') ? -1.0f : 1.0f;
    // remove the first chars, which aren't digits
    std::string tempString = testDescription_s.substr(ii, testDescription_s.length() - ii);
    // convert the remaining text to an float and return it
    //Parses the C string str, interpreting its content as a floating point number and returns its value as a double
    return static_cast<float32_t>(atof(tempString.c_str())) * isNegPosValue;
}

Parking_Maneuver returnManeuverInfoFromTestrun(std::string testDescription_s)
{
    // search for = and remove the first chars including = to extract the maneuver (e.g. forward, backward)
    std::string tempString{ testDescription_s };
    const size_t pos = testDescription_s.find('=');
    if (pos != std::string::npos) {
        tempString = tempString.substr(pos + 2);
    }
    //Check for the maneuvers (backward, forward) otherwise set NOT_SPECIFIED
    Parking_Maneuver maneuverFromTestrun{ Parking_Maneuver::NOT_SPECIFIED };
    if (tempString == "backward") maneuverFromTestrun = Parking_Maneuver::BACKWARD;
    else if (tempString == "forward") maneuverFromTestrun = Parking_Maneuver::FORWARD;
    else maneuverFromTestrun = Parking_Maneuver::NOT_SPECIFIED;
    return maneuverFromTestrun;
}

static void resetGlobalsTo0() {
    c_i = 0.0;
    gain_p_f = 0.0;
    gain_i_f = 0.0;
    gain_p_r = 0.0;
    gain_i_r = 0.0;
    accOld_mps = 0;
    curAccel_mps2 = 0;
    isFirstCycle = false;
    isFirstGearReq = false;
    poseBeforePlanning = LSM_GEOML::Pose{};
    driverReqToAccelFlag_nu = 0;
    driverReqToBrakeFlag_nu = 0;
    pathBeforeActIndex = 0;
    starterStatusPort = eco::create_default<ap_vehstatesigprovider::StarterStatusPort>();
    starterStatusPort.uiVersionNumber = ap_vehstatesigprovider::createStarterStatusPort_InterfaceVersion().StarterStatusPort_VERSION;
    trunkLidStatusPort = eco::create_default<ap_vehstatesigprovider::TrunkLidStatusPort>();
    trunkLidStatusPort.uiVersionNumber = ap_vehstatesigprovider::createTrunkLidStatusPort_InterfaceVersion().TrunkLidStatusPort_VERSION;
    convertibleTopStatusPort = eco::create_default<ap_vehstatesigprovider::ConvertibleTopStatusPort>();
    convertibleTopStatusPort.uiVersionNumber = ap_vehstatesigprovider::createConvertibleTopStatusPort_InterfaceVersion().ConvertibleTopStatusPort_VERSION;
    steeringColSwitchesStatusPort = eco::create_default<ap_vehstatesigprovider::SteeringColSwitchesStatusPort>();
    steeringColSwitchesStatusPort.uiVersionNumber = ap_vehstatesigprovider::createSteeringColSwitchesStatusPort_InterfaceVersion().SteeringColSwitchesStatusPort_VERSION;
    trailerStatusPort = eco::create_default<ap_vehstatesigprovider::TrailerStatusPort>();
    trailerStatusPort.uiVersionNumber = ap_vehstatesigprovider::createTrailerStatusPort_InterfaceVersion().TrailerStatusPort_VERSION;
    doorStatusPort = eco::create_default<ap_vehstatesigprovider::DoorStatusPort>();
    doorStatusPort.uiVersionNumber = ap_vehstatesigprovider::createDoorStatusPort_InterfaceVersion().DoorStatusPort_VERSION;
    vehicleOccupancyStatusPort = eco::create_default<ap_vehstatesigprovider::VehicleOccupancyStatusPort>();
    vehicleOccupancyStatusPort.uiVersionNumber = ap_vehstatesigprovider::createVehicleOccupancyStatusPort_InterfaceVersion().VehicleOccupancyStatusPort_VERSION;
    additionalBCMStatusPort = eco::create_default<ap_vehstatesigprovider::AdditionalBCMStatusPort>();
    additionalBCMStatusPort.uiVersionNumber = ap_vehstatesigprovider::createAdditionalBCMStatusPort_InterfaceVersion().AdditionalBCMStatusPort_VERSION;
    accInformationPort = eco::create_default<ap_vehstatesigprovider::ACCInformationPort>();
    accInformationPort.uiVersionNumber = ap_vehstatesigprovider::createACCInformationPort_InterfaceVersion().ACCInformationPort_VERSION;
    keylessStatusPort = eco::create_default<ap_vehstatesigprovider::KeylessStatusPort>();
    keylessStatusPort.uiVersionNumber = ap_vehstatesigprovider::createKeylessStatusPort_InterfaceVersion().KeylessStatusPort_VERSION;
    authenticationStatusPort = eco::create_default<ap_vehstatesigprovider::AuthenticationStatusPort>();
    authenticationStatusPort.uiVersionNumber = ap_vehstatesigprovider::createAuthenticationStatusPort_InterfaceVersion().AuthenticationStatusPort_VERSION;
    apCtrlStatusPort = eco::create_default<ap_vehstatesigprovider::APCtrlStatusPort>();
    apCtrlStatusPort.uiVersionNumber = ap_vehstatesigprovider::createAPCtrlStatusPort_InterfaceVersion().APCtrlStatusPort_VERSION;
    escInformationPort = eco::create_default<ap_vehstatesigprovider::ESCInformationPort>();
    escInformationPort.uiVersionNumber = ap_vehstatesigprovider::createESCInformationPort_InterfaceVersion().ESCInformationPort_VERSION;
    externalFunctionStatusPort = eco::create_default<ap_vehstatesigprovider::ExternalFunctionStatusPort>();
    externalFunctionStatusPort.uiVersionNumber = ap_vehstatesigprovider::createExternalFunctionStatusPort_InterfaceVersion().ExternalFunctionStatusPort_VERSION;
    gHMIOutputPort = eco::create_default<ap_hmitoap::HMIOutputPort>();
    gHMIOutputPort.uiVersionNumber = ap_hmitoap::createHMIOutputPort_InterfaceVersion().HMIOutputPort_VERSION;
    remoteHMIOutputPort = eco::create_default<ap_hmitoap::RemoteHMIOutputPort>();
    remoteHMIOutputPort.uiVersionNumber = ap_hmitoap::createRemoteHMIOutputPort_InterfaceVersion().RemoteHMIOutputPort_VERSION;
    loDMCStatusPort = eco::create_default<ap_lodmc::LoDMCStatusPort>();
    loDMCStatusPort.uiVersionNumber = ap_lodmc::createLoDMCStatusPort_InterfaceVersion().LoDMCStatusPort_VERSION;
#ifdef MOCO_REPLACES_LODMC
    controlData_TRATCO = {};
    controlData_VECONA = {};
    vehParam = {};
    brakeFb = {};
    powertrainFb = {};
    steeringFrontFb = {};
    steeringRearFb = {};
    vehDynFcu = {};
    memset(&tratcoStatusPort, 0, sizeof(tratcoStatusPort));
    memset(&longAccelReq, 0, sizeof(longAccelReq));
    memset(&longVeconaStatusPort, 0, sizeof(longVeconaStatusPort));
    veconaBrakeRequest = {};
    veconaPowertrainRequest = {};
    longDriverFeedback = {};
    vehDynVecona = {};
    mocoCarmakerInput = {};

    //tratcoProcessMemory
    std::fill(std::begin(tratcoInterMemData), std::end(tratcoInterMemData), 0U);
    std::fill(std::begin(tratcoInterMeasMemData), std::end(tratcoInterMeasMemData), 0U);
    std::fill(std::begin(tratcoIntraMeasMemData), std::end(tratcoIntraMeasMemData), 0U);
    tratcoProcessMemory.p_inter = &tratcoInterMemData;
    tratcoProcessMemory.p_interMeas = &tratcoInterMeasMemData;
    tratcoProcessMemory.p_intraMeas = &tratcoIntraMeasMemData;

    //veconaProcessMemory
    std::fill(std::begin(veconaInterMemData), std::end(veconaInterMemData), 0U);
    std::fill(std::begin(veconaInterMeasMemData), std::end(veconaInterMeasMemData), 0U);
    std::fill(std::begin(veconaIntraMeasMemData), std::end(veconaIntraMeasMemData), 0U);
    veconaProcessMemory.p_inter = &veconaInterMemData;
    veconaProcessMemory.p_interMeas = &veconaInterMeasMemData;
    veconaProcessMemory.p_intraMeas = &veconaIntraMeasMemData;

    //tratcoProcMem
    tratcoProcMem = {};
    tratcoProcMem.p_inter = (TRATCO_Inter*)tratcoProcessMemory.p_inter;
    tratcoProcMem.p_interMeas = (TRATCO_InterMeas*)tratcoProcessMemory.p_interMeas;
    tratcoProcMem.p_intraMeas = (TRATCO_IntraMeas*)tratcoProcessMemory.p_intraMeas;

    //veconaProcMem
    veconaProcMem = {};
    veconaProcMem.p_inter = (VECONA_Inter*)veconaProcessMemory.p_inter;
    veconaProcMem.p_interMeas = (VECONA_InterMeas*)veconaProcessMemory.p_interMeas;
    veconaProcMem.p_intraMeas = (VECONA_IntraMeas*)veconaProcessMemory.p_intraMeas;
#endif
    laDMCStatusPort = eco::create_default<ap_ladmc::LaDMCStatusPort>();
    laDMCStatusPort.uiVersionNumber = ap_ladmc::createLaDMCStatusPort_InterfaceVersion().LaDMCStatusPort_VERSION;
    odoExtCtrlPort = eco::create_default<ap_commonvehsigprovider::OdoExtCtrlPort>();
    odoExtCtrlPort.uiVersionNumber = ap_commonvehsigprovider::createOdoExtCtrlPort_InterfaceVersion().OdoExtCtrlPort_VERSION;
    gearboxCtrlStatusPort = eco::create_default<ap_commonvehsigprovider::GearboxCtrlStatusPort>();
    gearboxCtrlStatusPort.uiVersionNumber = ap_commonvehsigprovider::createGearboxCtrlStatusPort_InterfaceVersion().GearboxCtrlStatusPort_VERSION;
    suspensionPort = eco::create_default<ap_commonvehsigprovider::SuspensionPort>();
    suspensionPort.uiVersionNumber = ap_commonvehsigprovider::createSuspensionPort_InterfaceVersion().SuspensionPort_VERSION;
    engineCtrlStatusPort = eco::create_default<ap_commonvehsigprovider::EngineCtrlStatusPort>();
    engineCtrlStatusPort.uiVersionNumber = ap_commonvehsigprovider::createEngineCtrlStatusPort_InterfaceVersion().EngineCtrlStatusPort_VERSION;
    wheelPulsePort = eco::create_default<ap_commonvehsigprovider::WheelPulsePort>();
    wheelPulsePort.uiVersionNumber = ap_commonvehsigprovider::createWheelPulsePort_InterfaceVersion().WheelPulsePort_VERSION;
    wheelDrivingDirectionsPort = eco::create_default<ap_commonvehsigprovider::WheelDrivingDirectionsPort>();
    wheelDrivingDirectionsPort.uiVersionNumber = ap_commonvehsigprovider::createWheelDrivingDirectionsPort_InterfaceVersion().WheelDrivingDirectionsPort_VERSION;
    wheelSpeedPort = eco::create_default<ap_commonvehsigprovider::WheelSpeedPort>();
    wheelSpeedPort.uiVersionNumber = ap_commonvehsigprovider::createWheelSpeedPort_InterfaceVersion().WheelSpeedPort_VERSION;
    motionStatePort = eco::create_default<ap_commonvehsigprovider::MotionStatePort>();
    motionStatePort.uiVersionNumber = ap_commonvehsigprovider::createMotionStatePort_InterfaceVersion().MotionStatePort_VERSION;
    vehDynamicsPort = eco::create_default<ap_commonvehsigprovider::VehDynamicsPort>();
    vehDynamicsPort.uiVersionNumber = ap_commonvehsigprovider::createVehDynamicsPort_InterfaceVersion().VehDynamicsPort_VERSION;
    steerCtrlStatusPort = eco::create_default<ap_commonvehsigprovider::SteerCtrlStatusPort>();
    steerCtrlStatusPort.uiVersionNumber = ap_commonvehsigprovider::createSteerCtrlStatusPort_InterfaceVersion().SteerCtrlStatusPort_VERSION;
    systemTimePort = eco::create_default<ap_commonvehsigprovider::SystemTimePort>();
    systemTimePort.uiVersionNumber = ap_commonvehsigprovider::createSystemTimePort_InterfaceVersion().SystemTimePort_VERSION;
    envModelPort = eco::create_default<si::ApEnvModelPort>();
    envModelPort.uiVersionNumber = si::createApEnvModelPort_InterfaceVersion().ApEnvModelPort_VERSION;
    envModelPortCMOrigin = eco::create_default<si::ApEnvModelPort>();
    envModelPortCMOrigin.uiVersionNumber = si::createApEnvModelPort_InterfaceVersion().ApEnvModelPort_VERSION;
    collEnvModelPort = eco::create_default<si::CollEnvModelPort>();
    collEnvModelPort.uiVersionNumber = si::createCollEnvModelPort_InterfaceVersion().CollEnvModelPort_VERSION;
    gParkingBoxPort = eco::create_default<si::ApParkingBoxPort>();
    gParkingBoxPort.uiVersionNumber = si::createApParkingBoxPort_InterfaceVersion().ApParkingBoxPort_VERSION;
    parkingBoxPortCMOrigin = eco::create_default<si::ApParkingBoxPort>();
    parkingBoxPortCMOrigin.uiVersionNumber = si::createApParkingBoxPort_InterfaceVersion().ApParkingBoxPort_VERSION;
    egoMotionPort = eco::create_default<si::EgoMotionPort>();
    egoMotionPort.uiVersionNumber = si::createEgoMotionPort_InterfaceVersion().EgoMotionPort_VERSION;
    egoMotionPortCM = eco::create_default<si::EgoMotionPort>();
    egoMotionPortCM.uiVersionNumber = si::createEgoMotionPort_InterfaceVersion().EgoMotionPort_VERSION;
    perceptionAvailabilityPort = eco::create_default<si::PerceptionAvailabilityPort>();
    perceptionAvailabilityPort.uiVersionNumber = si::createPerceptionAvailabilityPort_InterfaceVersion().PerceptionAvailabilityPort_VERSION;
    memoryParkingStatusPort = eco::create_default<mf_mempark::MemoryParkingStatusPort>();
    memoryParkingStatusPort.uiVersionNumber = mf_mempark::createMemoryParkingStatusPort_InterfaceVersion().MemoryParkingStatusPort_VERSION;
    odoEstimationOutputPort = eco::create_default<lsm_vedodo::OdoEstimationOutputPort>();
    odoEstimationOutputPort.uiVersionNumber = lsm_vedodo::createOdoEstimationOutputPort_InterfaceVersion().OdoEstimationOutputPort_VERSION;
    odoPersistentDataPort = eco::create_default <lsm_vedodo::OdoNVMData>();
    odoPersistentDataPort.uiVersionNumber = lsm_vedodo::createOdoPersDataPort_InterfaceVersion().OdoPersDataPort_VERSION;
    odoEstimationOutputPortCM = eco::create_default<lsm_vedodo::OdoEstimationOutputPort>();
    odoEstimationOutputPortCM.uiVersionNumber = lsm_vedodo::createOdoEstimationOutputPort_InterfaceVersion().OdoEstimationOutputPort_VERSION;
    odoEstimationPortCM = eco::create_default<lsm_vedodo::OdoEstimation>();
    odoEstimationPortCM.uiVersionNumber = lsm_vedodo::createOdoEstimation_InterfaceVersion().OdoEstimation_VERSION;
    usDrvDetectionListCopy = eco::create_default<us_drv::UsDrvDetectionList>();
    usDrvDetectionListCopy.uiVersionNumber = us_drv::createUsDrvDetectionList_InterfaceVersion().UsDrvDetectionList_VERSION;
    usDrvRntConfigRequest = eco::create_default<us_drv::UsDrvRuntimeConfiguration>();
    usDrvRntConfigRequest.uiVersionNumber = us_drv::createUsDrvRuntimeConfiguration_InterfaceVersion().UsDrvRuntimeConfiguration_VERSION;
    uspPointListOutput = eco::create_default<us_processing::UsProcessingPointList>();
    uspPointListOutput.uiVersionNumber = us_processing::createUsProcessingPointList_InterfaceVersion().UsProcessingPointList_VERSION;
    usFilteredEchoOutput = eco::create_default<us_processing::UsProcessingFilteredEchoList>();
    usFilteredEchoOutput.uiVersionNumber = us_processing::createUsProcessingFilteredEchoList_InterfaceVersion().UsProcessingFilteredEchoList_VERSION;
    uspDistListOutput = eco::create_default<us_processing::UsProcessingDistanceList>();
    uspDistListOutput.uiVersionNumber = us_processing::createUsProcessingDistanceList_InterfaceVersion().UsProcessingDistanceList_VERSION;
    uspDiagOutput = eco::create_default<us_processing::UsProcessingDiagOutput>();
    uspDiagOutput.uiVersionNumber = us_processing::createUsProcessingDiagOutput_InterfaceVersion().UsProcessingDiagOutput_VERSION;
    uspIntegrityOutput = eco::create_default<us_processing::UsProcessingDataIntegrity>();
    uspIntegrityOutput.uiVersionNumber = us_processing::createUsProcessingDataIntegrity_InterfaceVersion().UsProcessingDataIntegrity_VERSION;
    usEmDebugPort = eco::create_default<us_em::UsEmDebugOutputPort>();
    usEmDebugPort.uiVersionNumber = us_em::createUsEmDebugOutputPort_InterfaceVersion().UsEmDebugOutputPort_VERSION;
    usEnvModelPort = eco::create_default<us_em::UsEnvModelPort>();
    usEnvModelPort.uiVersionNumber = us_em::createUsEnvModelPort_InterfaceVersion().UsEnvModelPort_VERSION;
    usPercAvailPort = eco::create_default<us_em::PerceptionAvailabilityPort>();
    usPercAvailPort.uiVersionNumber = us_em::createPerceptionAvailabilityPort_InterfaceVersion().PerceptionAvailabilityPort_VERSION;
    tceEstimationPort = eco::create_default<tce::TceEstimationPort>();
    tceEstimationPort.uiVersionNumber = tce::createTceEstimationPort_InterfaceVersion().TceEstimationPort_VERSION;
    tcePersDataPort = eco::create_default<tce::TcePersDataPort>();
    tcePersDataPort.uiVersionNumber = tce::createTcePersDataPort_InterfaceVersion().TcePersDataPort_VERSION;
    tceDebugPort = eco::create_default<tce::TceDebugPort>();
    tceDebugPort.uiVersionNumber = tce::createTceDebugPort_InterfaceVersion().TceDebugPort_VERSION;
    laDMCCtrlRequestPort = eco::create_default<ap_trjctl::LaDMCCtrlRequestPort>();
    laDMCCtrlRequestPort.uiVersionNumber = ap_trjctl::createLaDMCCtrlRequestPort_InterfaceVersion().LaDMCCtrlRequestPort_VERSION;
    loDMCCtrlRequestPort = eco::create_default<ap_trjctl::LoDMCCtrlRequestPort>();
    loDMCCtrlRequestPort.uiVersionNumber = ap_trjctl::createLoDMCCtrlRequestPort_InterfaceVersion().LoDMCCtrlRequestPort_VERSION;
#ifdef MOCO_REPLACES_LODMC
    memset(&longManeuverRequestPort, 0, sizeof(longManeuverRequestPort));
#endif
    gearBoxCtrlRequestPort = eco::create_default<ap_trjctl::GearboxCtrlRequestPort>();
    gearBoxCtrlRequestPort.uiVersionNumber = ap_trjctl::createGearboxCtrlRequestPort_InterfaceVersion().GearboxCtrlRequestPort_VERSION;
    mfControlStatusPort = eco::create_default<ap_trjctl::MFControlStatusPort>();
    mfControlStatusPort.uiVersionNumber = ap_trjctl::createMFControlStatusPort_InterfaceVersion().MFControlStatusPort_VERSION;
    drivingResistancePort = eco::create_default<ap_trjctl::DrivingResistancePort>();
    drivingResistancePort.uiVersionNumber = ap_trjctl::createDrivingResistancePort_InterfaceVersion().DrivingResistancePort_VERSION;
    laCtrlRequestPort = eco::create_default<mf_manager::LaCtrlRequestPort>();
    laCtrlRequestPort.uiVersionNumber = mf_manager::createLaCtrlRequestPort_InterfaceVersion().LaCtrlRequestPort_VERSION;
    loCtrlRequestPort = eco::create_default<mf_manager::LoCtrlRequestPort>();
    loCtrlRequestPort.uiVersionNumber = mf_manager::createLoCtrlRequestPort_InterfaceVersion().LoCtrlRequestPort_VERSION;
    trajRequestPort = eco::create_default<mf_manager::TrajRequestPort>();
    trajRequestPort.uiVersionNumber = mf_manager::createTrajRequestPort_InterfaceVersion().TrajRequestPort_VERSION;
    ddsPort = eco::create_default<ap_commonvehsigprovider::DdsPort>();
    ddsPort.uiVersionNumber = ap_commonvehsigprovider::createDdsPort_InterfaceVersion().DdsPort_VERSION;
    brakeCtrlStatusPort = eco::create_default<ap_commonvehsigprovider::BrakeCtrlStatusPort>();
    brakeCtrlStatusPort.uiVersionNumber = ap_commonvehsigprovider::createBrakeCtrlStatusPort_InterfaceVersion().BrakeCtrlStatusPort_VERSION;
    lscaStatusPort = eco::create_default<mf_lsca::LscaStatusPort>();
    lscaStatusPort.uiVersionNumber = mf_lsca::createLscaStatusPort_InterfaceVersion().LscaStatusPort_VERSION;
    lscaPlotDataPort = {};
    carMakerInterface = {};
    odoDebugPort = eco::create_default<lsm_vedodo::OdoDebugPort>();
    odoDebugPort.uiVersionNumber = lsm_vedodo::createOdoDebugPort_InterfaceVersion().OdoDebugPort_VERSION;
    psmDebugPort = eco::create_default<ap_psm_app::PSMDebugPort>();
    psmDebugPort.uiVersionNumber = ap_psm_app::createPSMDebugPort_InterfaceVersion().PSMDebugPort_VERSION;
    trajCtrlDebugPort = eco::create_default<ap_trjctl::TrajCtrlDebugPort>();
    trajCtrlDebugPort.uiVersionNumber = ap_trjctl::createTrajCtrlDebugPort_InterfaceVersion().TrajCtrlDebugPort_VERSION;
    trajCtrlRequestPort = eco::create_default<ap_psm::TrajCtrlRequestPort>();
    trajCtrlRequestPort.uiVersionNumber = ap_psm::createTrajCtrlRequestPort_InterfaceVersion().TrajCtrlRequestPort_VERSION;
    psmToSSPPort = eco::create_default<ap_psm_app::PSMToSSPPort>();
    psmToSSPPort.uiVersionNumber = ap_psm_app::createPSMToSSPPort_InterfaceVersion().PSMToSSPPort_VERSION;
    slotCtrlPort = eco::create_default<ap_psm::SlotCtrlPort>();
    slotCtrlPort.uiVersionNumber = ap_psm::createSlotCtrlPort_InterfaceVersion().SlotCtrlPort_VERSION;
    targetPosesPort = eco::create_default<ap_tp::TargetPosesPort>();
    targetPosesPort.uiVersionNumber = ap_tp::createTargetPosesPort_InterfaceVersion().TargetPosesPort_VERSION;
    targetPosesPortCMOrigin = eco::create_default<ap_tp::TargetPosesPort>();
    targetPosesPortCMOrigin.uiVersionNumber = ap_tp::createTargetPosesPort_InterfaceVersion().TargetPosesPort_VERSION;
    plannedTrajPort = eco::create_default<ap_tp::PlannedTrajPort>();
    plannedTrajPort.uiVersionNumber = ap_tp::createPlannedTrajPort_InterfaceVersion().PlannedTrajPort_VERSION;
    trjplaDebugPort = eco::create_default<ap_tp::TrajPlanDebugPort>();
    trjplaDebugPort.uiVersionNumber = ap_tp::createTrajPlanDebugPort_InterfaceVersion().TrajPlanDebugPort_VERSION;
    trjplaVisuPort = eco::create_default<ap_tp::TrajPlanVisuPort>();
    trjplaVisuPort.uiVersionNumber = ap_tp::createTrajPlanVisuPort_InterfaceVersion().TrajPlanVisuPort_VERSION;
    trjplaVisuPortCMOrigin = eco::create_default<ap_tp::TrajPlanVisuPort>();
    trjplaVisuPortCMOrigin.uiVersionNumber = ap_tp::createTrajPlanVisuPort_InterfaceVersion().TrajPlanVisuPort_VERSION;
    reverseAssistAvailabilityPort = eco::create_default<ap_tp::ReverseAssistAvailabilityPort>();
    reverseAssistAvailabilityPort.uiVersionNumber = ap_tp::createReverseAssistAvailabilityPort_InterfaceVersion().ReverseAssistAvailabilityPort_VERSION;
    taposdDebugPort = eco::create_default<ap_tp::TAPOSDDebugPort>();
    taposdDebugPort.uiVersionNumber = ap_tp::createTAPOSDDebugPort_InterfaceVersion().TAPOSDDebugPort_VERSION;
    taposdDebugPortCMOrigin = eco::create_default<ap_tp::TAPOSDDebugPort>();
    taposdDebugPortCMOrigin.uiVersionNumber = ap_tp::createTAPOSDDebugPort_InterfaceVersion().TAPOSDDebugPort_VERSION;
    gMFHmiHDebugPort = eco::create_default<mf_hmih::MFHmiHDebugPort>();
    gMFHmiHDebugPort.uiVersionNumber = mf_hmih::createMFHmiHDebugPort_InterfaceVersion().MFHmiHDebugPort_VERSION;
    odoGpsPort = eco::create_default<ap_commonvehsigprovider::OdoGpsPort>();
    odoGpsPort.uiVersionNumber = ap_commonvehsigprovider::createOdoGpsPort_InterfaceVersion().OdoGpsPort_VERSION;
    ambientDataPort = eco::create_default<ap_commonvehsigprovider::AmbientDataPort>();
    ambientDataPort.uiVersionNumber = ap_commonvehsigprovider::createAmbientDataPort_InterfaceVersion().AmbientDataPort_VERSION;
    gHmiInputPort = eco::create_default<mf_hmih::HMIGeneralInputPort>();
    gHmiInputPort.uiVersionNumber = mf_hmih::createHMIGeneralInputPort_InterfaceVersion().HMIGeneralInputPort_VERSION;
    headUnitVisualizationPort = eco::create_default<mf_hmih::HeadUnitVisualizationPort>();
    headUnitVisualizationPort.uiVersionNumber = mf_hmih::createHeadUnitVisualizationPort_InterfaceVersion().HeadUnitVisualizationPort_VERSION;
    remoteVisualizationPort = eco::create_default<mf_hmih::RemoteVisualizationPort>();
    remoteVisualizationPort.uiVersionNumber = mf_hmih::createRemoteVisualizationPort_InterfaceVersion().RemoteVisualizationPort_VERSION;
    userDefinedSlotPort = eco::create_default<mf_hmih::UserDefinedSlotPort>();
    userDefinedSlotPort.uiVersionNumber = mf_hmih::createUserDefinedSlotPort_InterfaceVersion().UserDefinedSlotPort_VERSION;
    apUserInteractionPort = eco::create_default<mf_hmih::APUserInteractionPort>();
    apUserInteractionPort.uiVersionNumber = mf_hmih::createAPUserInteractionPort_InterfaceVersion().APUserInteractionPort_VERSION;
    lvmdUserInteractionPort = eco::create_default<mf_hmih::LVMDUserInteractionPort>();
    lvmdUserInteractionPort.uiVersionNumber = mf_hmih::createLVMDUserInteractionPort_InterfaceVersion().LVMDUserInteractionPort_VERSION;
    replanCounter_nu = 0U;
    globalSCurveLength_m = 0.0f;
    std::fill(std::begin(replanPositionCMOrigin), std::end(replanPositionCMOrigin), LSM_GEOML::Pose());
    previousTrajectoryPortCMOrigin = eco::create_default<ap_tp::TrajPlanVisuPort>();
    previousTrajectoryPortCMOrigin.uiVersionNumber = ap_tp::createTrajPlanVisuPort_InterfaceVersion().TrajPlanVisuPort_VERSION;
    previousTargetPose = eco::create_default<ap_tp::TargetPose>();
#ifndef VARIANT_CUS_ONLY
    environmentPerception.resetVariables();
    groundTruthEnvironment.reset();
#endif
    memset(&trafficContour2D_t, 0, MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM * sizeof(trafficContour2D_t[0]));
    gCtrlCommandPort = eco::create_default<ap_psm_app::CtrlCommandPort>();
    gCtrlCommandPort.uiVersionNumber = ap_psm_app::createCtrlCommandPort_InterfaceVersion().CtrlCommandPort_VERSION;
    gPARKSMCoreStatusPort = eco::create_default<ap_psm::PARKSMCoreStatusPort>();
    gPARKSMCoreStatusPort.uiVersionNumber = ap_psm::createPARKSMCoreStatusPort_InterfaceVersion().PARKSMCoreStatusPort_VERSION;
    gPARKSMCoreDebugPort = eco::create_default<ap_psm::PARKSMCoreDebugPort>();
    gPARKSMCoreDebugPort.uiVersionNumber = ap_psm::createPARKSMCoreDebugPort_InterfaceVersion().PARKSMCoreDebugPort_VERSION;
    drvWarnStatusPort = eco::create_default<mf_drvwarnsm::DrvWarnStatusPort>();
    drvWarnStatusPort.uiVersionNumber = mf_drvwarnsm::createDrvWarnStatusPort_InterfaceVersion().DrvWarnStatusPort_VERSION;
    drvWarnDebugPort = eco::create_default<mf_drvwarnsm::DrvWarnDebugPort>();
    drvWarnDebugPort.uiVersionNumber = mf_drvwarnsm::createDrvWarnDebugPort_InterfaceVersion().DrvWarnDebugPort_VERSION;
    logicToProcPort = eco::create_default<mf_drvwarnsm::LogicToProcPort>();
    logicToProcPort.uiVersionNumber = mf_drvwarnsm::createLogicToProcPort_InterfaceVersion().LogicToProcPort_VERSION;
    appToCoreSMPort = eco::create_default<mf_drvwarnsm::AppToCoreSMPort>();
    appToCoreSMPort.uiVersionNumber = mf_drvwarnsm::createAppToCoreSMPort_InterfaceVersion().AppToCoreSMPort_VERSION;
    drvWarnCoreStatusPort = eco::create_default<mf_drvwarnsm_core::DrvWarnCoreStatusPort>();
    drvWarnCoreStatusPort.uiVersionNumber = mf_drvwarnsm_core::createDrvWarnCoreStatusPort_InterfaceVersion().DrvWarnCoreStatusPort_VERSION;
    pdcpSectorsPort = eco::create_default<pdcp::PDCPSectorsPort>();
    pdcpSectorsPort.uiVersionNumber = pdcp::createPDCPSectorsPort_InterfaceVersion().PDCPSectorsPort_VERSION;
    pdcpDrivingTubePort = eco::create_default<pdcp::PDCPDrivingTubePort>();
    pdcpDrivingTubePort.uiVersionNumber = pdcp::createPDCPDrivingTubePort_InterfaceVersion().PDCPDrivingTubePort_VERSION;
    pdwProcToLogicPort = eco::create_default<pdcp::ProcToLogicPort>();
    pdwProcToLogicPort.uiVersionNumber = pdcp::createProcToLogicPort_InterfaceVersion().ProcToLogicPort_VERSION;
    pdcpDebugPort = eco::create_default<pdcp::PDCPDebugPort>();
    pdcpDebugPort.uiVersionNumber = pdcp::createPDCPDebugPort_InterfaceVersion().PDCPDebugPort_VERSION;
    toneOutputPort = eco::create_default<mf_tonh::ToneOutputPort>();
    toneOutputPort.uiVersionNumber = mf_tonh::createToneOutputPort_InterfaceVersion().ToneOutputPort_VERSION;
    whpOutputPort = eco::create_default<mf_whlprotectproc::WHPProcOutputPort>();
    whpOutputPort.uiVersionNumber = mf_whlprotectproc::createWHPProcOutputPort_InterfaceVersion().WHPProcOutputPort_VERSION;
    lvmdStatusPort = eco::create_default<mf_lvmd::LvmdStatusPort>();
    lvmdStatusPort.uiVersionNumber = mf_lvmd::createLvmdStatusPort_InterfaceVersion().LvmdStatusPort_VERSION;
    avgaSupervisionRequestLSCA = eco::create_default<avga_swc::AVGA_SupervisionRequest>();
    avgaSupervisionRequestPDW = eco::create_default<avga_swc::AVGA_SupervisionRequest>();
    automatedVehicleGuidanceStateAUP = eco::create_default<avga_swc::AVGA_AutomatedVehicleGuidanceState>();
    avgaStopRequestMCRA = eco::create_default<avga_swc::AvgaStopRequestPort>();
    cemSurrogateConfig = VCEM::CemSurrogateConfig();
    enableLimitFieldOfView_nu = false;
    enableLatencyEffect_nu = false;
    latencyEffectTime_s = 0.1f;
    inflationLength_m = 0.0f;
    steeringWheelAngleAcceleration = 0.0f;
    steeringAngleAcceleration = 0.0f;
    lastAng_rad = 0.0;
    vehicleWasSecured_nu = false;
    vehicleWasSecuredMoCo_nu = false;
    hadSteerCtrlRequest_nu = false;
    lastGearReq_nu = 0;
    hadGearReq_nu = false;
    hadSteerReq_nu = false;
    std::fill(std::begin(parkingOnLeftSide_nu), std::end(parkingOnLeftSide_nu), false);
    userActionHeadUnitCMQuant_nu = 0;
    userActionRemoteDeviceCMQuant_nu = 0;
    fingerPositionXCMQuant_px = 0;
    fingerPositionYCMQuant_px = 0;
    overwriteVehVelocityCur_nu = 0;
    vehVelocityCurCMQuant_mps = 0.0f;
    overwritePathBeforeFuncActivation_nu = 0;
    pathBeforeFuncActivation0Pose = LSM_GEOML::Pose{};
    lastCallTime_us = 0.0;
    segment_selector = 0; //1-12
    useDeadManSwitchRemoteApp = 0;
    fakeKeyFonAliveCounterFlag = 0;
    maxCycleTimeOfAUPStep_ms = 0.0f;
    gearChangeRequested_nu = false;
    undershoot_delay_cntr = 0;
    carStopped_nu = true;
    currStroke_nu = 0;
    currGearReq_nu = 0;
    distToStopReq_m = 0.0;
    TCEActive_nu = 0;
    optimalTargetPoseSIOrigin = OptimalTargetPose{};
    testEvaluation.resetVariablesTo0();
    odometryDataManger.reset();
    SiUtility::getInstance().reset();
    lastTimeVedodoReset_s = -1.0;
    vedodoOffsetAtComponentReset.setRefPose(LSM_GEOML::Pose{});

#ifndef VARIANT_CUS_ONLY
    if (isSvcModelProcessingDataAllocated) {
        SvcModelWrapper::Instance().ResetInput(svcModelProcessingInput);
        SvcModelWrapper::Instance().ResetOutput(svcModelProcessingOutput);
        SvcModelWrapper::Instance().ResetSettings(svcModelProcessingSettings);
    }
#endif

    sRoadT01_m = .0f;
    sRoadLim01_m = .0f;
    sRoadFurthestObject_m = .0f;
    yawAngleFurthestObject_rad = .0f;

    tap_on_start_selection = false;
    tap_on_start_parking = false;
    user_input = false;
}

static void updateSignalHeader(const eco::AlgoDataTimeStamp& timestamp_us, eco::SignalHeader& sigHeader) {
    sigHeader.uiTimeStamp = timestamp_us;
    if (isFirstCycle) {
        sigHeader.uiCycleCounter = 1U;
        sigHeader.uiMeasurementCounter = 1U;
        sigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    }
    else {
        sigHeader.uiCycleCounter++;
        sigHeader.uiMeasurementCounter++;
    }
}

static void registerSignalHeaderToDVA(const std::string& prefix_text, eco::SignalHeader& sigHeader) {
    DDefULLong(NULL, (prefix_text + ".uiTimeStamp").c_str(), "us", &sigHeader.uiTimeStamp, DVA_None);

    DDefUShort(NULL, (prefix_text + ".uiMeasurementCounter").c_str(), "", &sigHeader.uiMeasurementCounter, DVA_None);

    DDefUShort(NULL, (prefix_text + ".uiCycleCounter").c_str(), "", &sigHeader.uiCycleCounter, DVA_None);

    DDefUChar(NULL, (prefix_text + ".eSigStatus").c_str(), "", (uint8*)&sigHeader.eSigStatus, DVA_None);
}

static void updateCANRxData() {
    const uint64_t timestamp_us = static_cast<uint64_t>(std::round(1e6 * SimCore.Time));
    /*SSPKcanPort*/
    updateSignalHeader(timestamp_us, authenticationStatusPort.sSigHeader);
    authenticationStatusPort.authKeyDetected_nu = true;

    updateSignalHeader(timestamp_us, additionalBCMStatusPort.sSigHeader);
    additionalBCMStatusPort.frontLidOpen_nu = false;
    additionalBCMStatusPort.ignition.ignitionOn_nu = Vehicle.Ignition > 0;
    additionalBCMStatusPort.light.lowBeamOn_nu = false;
    additionalBCMStatusPort.tankCapOpen_nu = false;

    updateSignalHeader(timestamp_us, convertibleTopStatusPort.sSigHeader);
    convertibleTopStatusPort.cTopState_nu = ap_vehstatesigprovider::ConvertibleTopState::LOCKED_CLOSED;

    updateSignalHeader(timestamp_us, doorStatusPort.sSigHeader);
    if (ap_vehstatesigprovider::DoorStatus::DOOR_STATUS_OPEN != doorStatusPort.status.driver_nu) { // avoid overwrite the door open signal
        doorStatusPort.status.driver_nu = ap_vehstatesigprovider::DoorStatus::DOOR_STATUS_CLOSED;
    }
    doorStatusPort.status.frontPsgr_nu = ap_vehstatesigprovider::DoorStatus::DOOR_STATUS_CLOSED;
    doorStatusPort.status.rearLeft_nu = ap_vehstatesigprovider::DoorStatus::DOOR_STATUS_CLOSED;
    doorStatusPort.status.rearRight_nu = ap_vehstatesigprovider::DoorStatus::DOOR_STATUS_CLOSED;

    updateSignalHeader(timestamp_us, steeringColSwitchesStatusPort.sSigHeader);
    steeringColSwitchesStatusPort.ctrlLeft_nu = true;
    steeringColSwitchesStatusPort.ctrlRight_nu = false;

    updateSignalHeader(timestamp_us, trailerStatusPort.sSigHeader);
    trailerStatusPort.trailerAttached_nu = false;
    trailerStatusPort.trailerHitchStatus_nu = ap_vehstatesigprovider::TrailerHitchStatus::NOT_INSTALLED;

    updateSignalHeader(timestamp_us, trunkLidStatusPort.sSigHeader);
    trunkLidStatusPort.open_nu = false;

    /*SSPAcanPort*/
    updateSignalHeader(timestamp_us, accInformationPort.sSigHeader);
    accInformationPort.accStatus_nu = ap_vehstatesigprovider::ACCStatus::ACC_TURN_OFF;
    updateSignalHeader(timestamp_us, starterStatusPort.sSigHeader);
    starterStatusPort.starterLocked_nu = false;

    /*SSPAcanPort*/
    updateSignalHeader(timestamp_us, apCtrlStatusPort.sSigHeader);
    apCtrlStatusPort.apActivationGranted_nu = true;
    apCtrlStatusPort.scanningActivationReq_nu = true;

    updateSignalHeader(timestamp_us, vehicleOccupancyStatusPort.sSigHeader);
    if (ap_vehstatesigprovider::BeltBuckle::BELT_STATUS_OPEN != vehicleOccupancyStatusPort.beltBuckleStatus.driver_nu) { // avoid overwrite the door open signal
        vehicleOccupancyStatusPort.beltBuckleStatus.driver_nu = ap_vehstatesigprovider::BeltBuckle::BELT_STATUS_LOCKED;
    }
    vehicleOccupancyStatusPort.beltBuckleStatus.frontPsgr_nu = ap_vehstatesigprovider::BeltBuckle::BELT_STATUS_LOCKED;
    vehicleOccupancyStatusPort.beltBuckleStatus.backrowLeft_nu = ap_vehstatesigprovider::BeltBuckle::BELT_STATUS_OPEN;
    vehicleOccupancyStatusPort.beltBuckleStatus.backrowRight_nu = ap_vehstatesigprovider::BeltBuckle::BELT_STATUS_OPEN;

    vehicleOccupancyStatusPort.seatOccupancyStatus.driver_nu = ap_vehstatesigprovider::SeatOccupancy::OCC_STATUS_OCCUPIED;
    vehicleOccupancyStatusPort.seatOccupancyStatus.frontPsgr_nu = ap_vehstatesigprovider::SeatOccupancy::OCC_STATUS_OCCUPIED;
    vehicleOccupancyStatusPort.seatOccupancyStatus.backrowLeft_nu = ap_vehstatesigprovider::SeatOccupancy::OCC_STATUS_FREE;
    vehicleOccupancyStatusPort.seatOccupancyStatus.backrowRight_nu = ap_vehstatesigprovider::SeatOccupancy::OCC_STATUS_FREE;

    updateSignalHeader(timestamp_us, engineCtrlStatusPort.sSigHeader);
    engineCtrlStatusPort.engineOn_nu = true;
    engineCtrlStatusPort.remoteStartPossible_nu = true;
    engineCtrlStatusPort.startStopStatus_nu = ap_commonvehsigprovider::StartStopStatus::STOP_PROHIBITED;
    if (!isFirstCycle) {
        engineCtrlStatusPort.throttleGradient_percps = (float)((100.0 * DrivMan.Gas) - engineCtrlStatusPort.throttlePos_perc) / AP_DATA_PROC_CYCLE_TIME_S;
    }
    else {
        engineCtrlStatusPort.throttleGradient_percps = 0;
    }
    engineCtrlStatusPort.throttleGradient_QF_nu = true;
    engineCtrlStatusPort.throttlePos_perc = 100.0F * (float)DrivMan.Gas;
    engineCtrlStatusPort.throttlePos_QF_nu = true;
    engineCtrlStatusPort.axleTorque_nm = static_cast<float32_t>(PowerTrain.GearBoxIF.Trq_out);

    updateSignalHeader(timestamp_us, brakeCtrlStatusPort.sSigHeader);
    // consider as braked when sum of all brake torque exceeds 50 Nm
    brakeCtrlStatusPort.vehicleBraked_nu = (fabs(Brake.Trq_tot[0]
        + Brake.Trq_tot[1]
        + Brake.Trq_tot[2]
        + Brake.Trq_tot[3]) > 50.0);

    updateSignalHeader(timestamp_us, escInformationPort.sSigHeader);
    escInformationPort.brakePressureDriver_bar = 0.0;
    escInformationPort.brakePressureGradient_barps = 0.0;
    escInformationPort.escState_nu = ap_vehstatesigprovider::ESCState::ESC_INACTIVE;
    escInformationPort.absState_nu = ap_vehstatesigprovider::ABSState::ABS_INACTIVE;
    escInformationPort.ebdState_nu = ap_vehstatesigprovider::EBDState::EBD_INACTIVE;
    escInformationPort.tcsState_nu = ap_vehstatesigprovider::TCSState::TCS_INACTIVE;

    updateSignalHeader(timestamp_us, externalFunctionStatusPort.sSigHeader);
    externalFunctionStatusPort.ebaStatus_nu = ap_vehstatesigprovider::EBAStatus::EBA_PASSIVE;
    externalFunctionStatusPort.rctaStatus_nu = ap_vehstatesigprovider::RCTAStatus::RCTA_PASSIVE;

    //sspFcanPort.gearboxLever.gearLeverPositionCur_nu        = static_cast<AP_CommonVehSigProvider::Gear>(leverPosFromCarMakerVariable); //sspAcanPort.gearBoxCurrent.gearCur_nu;

    updateSignalHeader(timestamp_us, odoEstimationOutputPortCM.sSigHeader);

    /*odoEstimationPortCM*/
    const float32_t lastCycleTime_s = static_cast<float32_t>(SimCore.Time - odoEstimationPortCM.sSigHeader.uiTimeStamp * 1e-6);
    updateSignalHeader(timestamp_us, odoEstimationPortCM.sSigHeader);
    odoEstimationPortCM.xPosition_m = (float)(InertialSensor[1].Pos_0[0]); /* SA1 Sensor */
    odoEstimationPortCM.yPosition_m = (float)(InertialSensor[1].Pos_0[1]);
    odoEstimationPortCM.yawAngle_rad = (float)Vehicle.Yaw;
    odoEstimationPortCM.longiVelocity_mps = (float)Car.ConBdy1.v_1[0];
    odoEstimationPortCM.longiAcceleration_mps2 = static_cast<float32_t>(Car.ConBdy1.a_1[0]);
    odoEstimationPortCM.drivenDistance_m += lastCycleTime_s * fabs(odoEstimationPortCM.longiVelocity_mps); /*TODO if not sufficiently precise, use multi step method like Adams-Bashforth instead */

    (fabsf((float)Car.ConBdy1.v_1[0]) < 0.1f) ? odoEstimationPortCM.motionStatus_nu = lsm_vedodo::MotionState::ODO_STANDSTILL : odoEstimationPortCM.motionStatus_nu = lsm_vedodo::MotionState::ODO_NO_STANDSTILL;

    odoEstimationPortCM.steerAngFrontAxle_rad = (float)(LSM_GEOML::lookupTable(fabsf((float)(Steering.IF.Ang)),
        carMakerInterface.vehicle_Params.AP_V_STEER_LOOKUP_ST_WHL_RAD,
        carMakerInterface.vehicle_Params.AP_V_STEER_LOOKUP_CTR_WHL_RAD, 10));
    if (Steering.IF.Ang < 0) odoEstimationPortCM.steerAngFrontAxle_rad *= -1.0f;
    odoEstimationPortCM.yawRate_radps = (float)Car.YawRate;
    odoEstimationPortCM.pitchAngle_rad = (float)Vehicle.Pitch;/*TODO ensure that this is the pitch angle and not Car.Fr1.r_zyx[1]*/
    odoEstimationPortCM.rollAngle_rad = (float)Vehicle.Roll;/*TODO ensure that this is the roll angle and not Car.Fr1.r_zyx[0]*/
    // Yaw rate model extension
    if (extension_wanted)
    {
        // Linearity Error

        yawRateEgoCurMod_rad = 1.02F *odoEstimationPortCM.yawRate_radps;
        // Offset
        constexpr float YawRateOffset_rad = -0.005f;
        yawRateEgoCurMod_rad = yawRateEgoCurMod_rad + YawRateOffset_rad;
        //Noise: The noise in raw sensor measurement are coupled with gaussian distributed white noise
        static std::normal_distribution<double> yawRateNoiseDistribution{ 7.72971E-10, 0.000949421 };
        const double YawRateNoise_rad = yawRateNoiseDistribution(gRandomEngine);

        yawRateEgoCurMod_rad = static_cast<float32_t>(yawRateEgoCurMod_rad + YawRateNoise_rad);
        odoEstimationOutputPort.odoEstimation.yawRate_radps = yawRateEgoCurMod_rad;
    }

    /*GearboxCtrlStatusPort*/
    updateSignalHeader(timestamp_us, gearboxCtrlStatusPort.sSigHeader);
    gearboxCtrlStatusPort.gearInformation.gearboxCtrlSystemState_nu = ap_commonvehsigprovider::GearBoxCtrlSystemState::GCTRL_AVAILABLE;
    if (isFirstCycle) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_NOT_DEFINED;
    }
    else if ((gearBoxCtrlRequestPort.gearSwitchRequest_nu == b_TRUE) && ((gearBoxCtrlRequestPort.gearReq_nu == ap_commonvehsigprovider::Gear::GEAR_N) || (gearBoxCtrlRequestPort.gearReq_nu == ap_commonvehsigprovider::Gear::GEAR_P))) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = gearBoxCtrlRequestPort.gearReq_nu;
    }
    else if (VehicleControl.SelectorCtrl == -9) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_P;
    }
#ifdef MOCO_REPLACES_LODMC
    else if (PowerTrain.GearBoxIF.GearNo > 0) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_D;
    }
    else if (PowerTrain.GearBoxIF.GearNo < 0) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_R;
    }
    else {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_N;
    }

#else //!MOCO_REPLACES_LODMC
    else if (VehicleControl.SelectorCtrl < 0) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_R;
    }
    else if (VehicleControl.SelectorCtrl == 0) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_N;
    }
    else if (PowerTrain.GearBoxIF.CfgIF->GBKind == GBKind_Manual) {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = static_cast<ap_commonvehsigprovider::Gear>(VehicleControl.SelectorCtrl);
    }
    else {
        gearboxCtrlStatusPort.gearInformation.gearCur_nu = ap_commonvehsigprovider::Gear::GEAR_D;
    }
    //gearboxCtrlStatusPort.gearLeverPositionCur_nu           = AP_CommonVehSigProvider::Gear::GEAR_N; //gearboxCtrlStatusPort.gearCur_nu; //TODO Fix
#endif //!MOCO_REPLACES_LODMC
    updateSignalHeader(timestamp_us, gHMIOutputPort.sSigHeader);
#ifdef USE_HMI
    /*HMIOutputPort*/
    gHMIOutputPort.userActionHeadUnit_nu = static_cast<ap_hmitoap::UserActionHeadUnit>(PRIVATE_CAN_GetUserActionHU());
    gHMIOutputPort.userActionHUCounter_nu = PRIVATE_CAN_GetUserActionHUCounter();
    userActionHeadUnitCMQuant_nu = static_cast<uint8_t>(gHMIOutputPort.userActionHeadUnit_nu);

    /*RemoteHMIOutputPort */
    remoteHMIOutputPort.userActionRemoteDevice_nu = static_cast<ap_hmitoap::UserActionRemoteDevice>(PRIVATE_CAN_GetUserActionRem());
    remoteHMIOutputPort.aliveCounter_nu = PRIVATE_CAN_GetAliveCounterRem();
    remoteHMIOutputPort.paired_nu = static_cast<bool>(PRIVATE_CAN_GetRemDevicePaired());
    remoteHMIOutputPort.connected_nu = static_cast<bool>(PRIVATE_CAN_GetRemDeviceConnected());
    remoteHMIOutputPort.deadMansSwitchBtn_nu = static_cast<bool>(PRIVATE_CAN_GetDeadManSwitchBtn());
    remoteHMIOutputPort.fingerPositionX_px = PRIVATE_CAN_GetFingerPositionX();
    remoteHMIOutputPort.fingerPositionY_px = PRIVATE_CAN_GetFingerPositionY();
    remoteHMIOutputPort.screenResolutionX_px = PRIVATE_CAN_GetScreenResolutionX();
    remoteHMIOutputPort.screenResolutionY_px = PRIVATE_CAN_GetScreenResolutionY();
    remoteHMIOutputPort.batteryLevel_perc = PRIVATE_CAN_GetBatteryLevelRem();
    remoteHMIOutputPort.userActionRemCounter_nu = PRIVATE_CAN_GetUserActionRemCounter();
    if ((static_cast<uint64_t>(std::round(SimCore.Time * 1000)) % REMOTE_HMI_CYCLE_TIME_MS) == 0) {
        updateSignalHeader(timestamp_us, remoteHMIOutputPort.sSigHeader);
    }
#else
    /*HMIOutputPort*/
    if (gHMIOutputPort.userActionHeadUnit_nu != static_cast<ap_hmitoap::UserActionHeadUnit>(userActionHeadUnitCMQuant_nu)) {
        gHMIOutputPort.userActionHUCounter_nu++;
        gHMIOutputPort.userActionHeadUnit_nu = static_cast<ap_hmitoap::UserActionHeadUnit>(userActionHeadUnitCMQuant_nu);/*TODO*//*read from CM quantity*/
    }
    /*RemoteHMIOutputPort */
    if (remoteHMIOutputPort.userActionRemoteDevice_nu != static_cast<ap_hmitoap::UserActionRemoteDevice>(userActionRemoteDeviceCMQuant_nu)) {
        remoteHMIOutputPort.userActionRemCounter_nu++;
        remoteHMIOutputPort.userActionRemoteDevice_nu = static_cast<ap_hmitoap::UserActionRemoteDevice>(userActionRemoteDeviceCMQuant_nu);
    }
    remoteHMIOutputPort.deadMansSwitchBtn_nu = false;
    if (useDeadManSwitchRemoteApp) {
        performValidDeadManSwitchRemoteApp();
    }
    else {
        remoteHMIOutputPort.fingerPositionX_px = static_cast<uint16_t>(fingerPositionXCMQuant_px);
        remoteHMIOutputPort.fingerPositionY_px = static_cast<uint16_t>(fingerPositionYCMQuant_px);
    }
    remoteHMIOutputPort.batteryLevel_perc = 100u;
    remoteHMIOutputPort.paired_nu = remDevPaired_nu;
    remoteHMIOutputPort.connected_nu = remDevConnected_nu;

    if ((static_cast<uint64_t>(std::round(SimCore.Time * 1000)) % REMOTE_HMI_CYCLE_TIME_MS) == 0) {
        remoteHMIOutputPort.aliveCounter_nu = (remoteHMIOutputPort.aliveCounter_nu + 1U) % (REMOTE_HMI_ALIVE_COUNTER_MAX + 1U);
        updateSignalHeader(timestamp_us, remoteHMIOutputPort.sSigHeader);
    }
#endif

    /*KeylessStatusPort*/
    updateSignalHeader(timestamp_us, keylessStatusPort.sSigHeader);
    if (fakeKeyFonAliveCounterFlag) {
        fakeKeyFobAliveCounter();
    }
    else {
        keylessStatusPort.keylessStatusPortCANAlive_nu = 0;
    }
    keylessStatusPort.keyFobButtonAliveCounter = 0;

    /*LoDMCStatusPort*/
    updateSignalHeader(timestamp_us, loDMCStatusPort.sSigHeader);
    //Set vehicleWasSecured_nu false in case of control request activation (e.g. new stroke) OR gas pedal was pressed
    if ((loDMCCtrlRequestPort.distanceToStopReq_m > 0.15F) || (VehicleControl.Gas > 0.01F)) vehicleWasSecured_nu = false;

    loDMCStatusPort.loDMCSystemState_nu = ap_lodmc::LoDMCSystemState::LSCA_MSP_APA_REMOTE_SUPPORT_AVAILABLE_REMOTE_READY;
    loDMCStatusPort.standstillHoldCur_nu = (abs(Car.ConBdy1.v_1[0]) < 0.02 && VehicleControl.Gas < 0.001);
    const bool maneuveringFinished_nu{ loDMCStatusPort.standstillHoldCur_nu && ((ap_trjctl::LoDMCCtrlRequestType::LODMC_NORMAL_REQUEST == loDMCCtrlRequestPort.loDMCCtrlRequest_nu) && loDMCCtrlRequestPort.distanceToStopReq_m < 0.05F) };
    if (maneuveringFinished_nu) {
        loDMCStatusPort.maneuveringFinished_nu = ap_lodmc::maneuveringFinished::MANEUVERING_FINISHED;
    }
    else {
        loDMCStatusPort.maneuveringFinished_nu = ap_lodmc::maneuveringFinished::MANEUVERING_NOT_STARTED;
        //TODO TATA Application Team (not blocking): Check missing enum types MANEUVERING_IN_PROGRESS, MANEUVERING_SATURATED, MANEUVERING_FAULT
    }
    vehicleWasSecured_nu = ((loDMCStatusPort.standstillHoldCur_nu && loDMCCtrlRequestPort.secureReq_nu) || vehicleWasSecured_nu);
    loDMCStatusPort.standstillSecureCur_nu = vehicleWasSecured_nu || (VehicleControl.BrakePark > 0.001);
    //Return active status equal to requested source always:
    if (ap_trjctl::LoDMCCtrlRequestSourceType::LODMC_REQ_SRC_AUP == loDMCCtrlRequestPort.loDMCCtrlRequestSource_nu) {
        loDMCStatusPort.longitudinalControlActiveStatus_nu = ap_lodmc::longitudinalControlActiveStatus::LODMC_AUP_HANDSHAKE_ACTIVE;
    }
    else if (ap_trjctl::LoDMCCtrlRequestSourceType::LODMC_REQ_SRC_LSCA == loDMCCtrlRequestPort.loDMCCtrlRequestSource_nu) {
        loDMCStatusPort.longitudinalControlActiveStatus_nu = ap_lodmc::longitudinalControlActiveStatus::LODMC_LSCA_HANDSHAKE_ACTIVE;
    }
    else if (ap_trjctl::LoDMCCtrlRequestSourceType::LODMC_REQ_SRC_MSP == loDMCCtrlRequestPort.loDMCCtrlRequestSource_nu) {
        loDMCStatusPort.longitudinalControlActiveStatus_nu = ap_lodmc::longitudinalControlActiveStatus::LODMC_MSP_HANDSHAKE_ACTIVE;
    }
    else if (ap_trjctl::LoDMCCtrlRequestSourceType::LODMC_REQ_SRC_REMOTE == loDMCCtrlRequestPort.loDMCCtrlRequestSource_nu) {
        loDMCStatusPort.longitudinalControlActiveStatus_nu = ap_lodmc::longitudinalControlActiveStatus::LODMC_REMOTE_PARKING_HANDSHAKE_ACTIVE;
    }
    else if (ap_trjctl::LoDMCCtrlRequestSourceType::LODMC_REQ_SRC_NO_REQEUESTER == loDMCCtrlRequestPort.loDMCCtrlRequestSource_nu) {
        loDMCStatusPort.longitudinalControlActiveStatus_nu = ap_lodmc::longitudinalControlActiveStatus::LODMC_NO_ACTIVE_HANDSHAKE;
    }
    else {
        loDMCStatusPort.longitudinalControlActiveStatus_nu = ap_lodmc::longitudinalControlActiveStatus::LODMC_NO_ACTIVE_HANDSHAKE;
    }

    /*LaDMCStatusPort*/
    updateSignalHeader(timestamp_us, laDMCStatusPort.sSigHeader);
    laDMCStatusPort.handSteeringTorque_Nm = 0.0f;
    laDMCStatusPort.handSteeringTorque_QF_nu = true;
    if (laDMCCtrlRequestPort.laDMCCtrlRequest_nu &&
        ap_trjctl::LaDMCCtrlRequestSourceType::LADMC_REQ_SRC_AUP == laDMCCtrlRequestPort.laDMCCtrlRequestSource_nu) {
        laDMCStatusPort.laDMCSystemState_nu = ap_ladmc::LaDMCSystemState::LADMC_CTRL_ACTIVE;
    }
    else {
        laDMCStatusPort.laDMCSystemState_nu = ap_ladmc::LaDMCSystemState::LADMC_AVAILABLE;
    }

    /*OdoGpsPort*/

    // extract position (in spherical coordinates) from LLH coordinate system
    const auto theta = GNavSensor.Receiver.UserPosLlhTsa[0]; // gpsLatitude_rad
    const auto phi = GNavSensor.Receiver.UserPosLlhTsa[1]; // gpsLongitude_rad

    // extract velocity values from ECEF coordinate system
    const auto v_x = GNavSensor.Receiver.UserVelEcefTsa[0];
    const auto v_y = GNavSensor.Receiver.UserVelEcefTsa[1];
    const auto v_z = GNavSensor.Receiver.UserVelEcefTsa[2];

    // calculate base vectors of plane on earth surface, phi representing the x-axis and theta the y-axis
    const auto etheta_x = cos(theta) * cos(phi);
    const auto etheta_y = cos(theta) * sin(phi);
    const auto etheta_z = -sin(theta);

    const auto ephi_x = -sin(phi);
    const auto ephi_y = cos(phi);
    const auto ephi_z = 0.0;

    // calculate the velocity components in relation to plane
    const auto v_phi = v_x * ephi_x + v_y * ephi_y + v_z * ephi_z;
    const auto v_theta = -(v_x * etheta_x + v_y * etheta_y + v_z * etheta_z); // minus for true north (mirroring y-value on x-axis)
    const auto absolute_v = sqrt(pow(v_phi, 2) + pow(v_theta, 2));

    if (absolute_v > LSM_GEOML::MIN_FLT_DIVISOR) {
        // calculate heading angle
        auto courseOverGround_rad = std::atan2(v_phi, v_theta);
        // conversion to [0, 2pi] range
        if (courseOverGround_rad < 0.0) {
            courseOverGround_rad += 2 * M_PI;
        }
        odoGpsPort.gpsData.gpsCourseOverGround = static_cast<float32_t>(courseOverGround_rad);
    }

    updateSignalHeader(timestamp_us, odoGpsPort.sSigHeader);
    const float32_t gpsLatitude_deg = static_cast<float32_t>(GNavSensor.Receiver.UserPosLlhTsa[0]) * 180.0f / (float)M_PI; //transform rad to deg
    const float32_t gpsLongitude_deg = static_cast<float32_t>(GNavSensor.Receiver.UserPosLlhTsa[1]) * 180.0f / (float)M_PI; //transform rad to deg

    odoGpsPort.gpsData.gpsAntennaHeight_m = 0.0f;
    odoGpsPort.gpsData.gpsLatitude_dd = static_cast<int32_t>(gpsLatitude_deg);
    odoGpsPort.gpsData.gpsLatitude_mm = (static_cast<float32_t>(gpsLatitude_deg) - static_cast<int16_t>(gpsLatitude_deg)) * 60;
    odoGpsPort.gpsData.gpsLongitude_dd = static_cast<int32_t>(gpsLongitude_deg);
    odoGpsPort.gpsData.gpsLongitude_mm = (static_cast<float32_t>(gpsLongitude_deg) - static_cast<int16_t>(gpsLongitude_deg)) * 60;
    odoGpsPort.gpsData.gpsSpeed_mps = static_cast<float32_t>(sqrt(GNavSensor.Receiver.UserVelEcefTsa[0] * GNavSensor.Receiver.UserVelEcefTsa[0] + GNavSensor.Receiver.UserVelEcefTsa[1] * GNavSensor.Receiver.UserVelEcefTsa[1]));
    odoGpsPort.gpsData.gpsR32SpeedOverGround_mps = static_cast<float32_t>(sqrt(GNavSensor.Receiver.UserVelEcefTsa[0] * GNavSensor.Receiver.UserVelEcefTsa[0] + GNavSensor.Receiver.UserVelEcefTsa[1] * GNavSensor.Receiver.UserVelEcefTsa[1]));
    odoGpsPort.gpsData.gpsUtcTime_hh = 0;
    odoGpsPort.gpsData.gpsUtcTime_mm = 0;
    odoGpsPort.gpsData.gpsUtcTime_ss = 0;
    odoGpsPort.gpsData.gpsLatitudeHemisphere_nu = ap_commonvehsigprovider::Hemisphere::ODO_GPS_NORTH;
    odoGpsPort.gpsData.gpsLongitudeHemisphere_nu = ap_commonvehsigprovider::Hemisphere::ODO_GPS_EAST;
    odoGpsPort.gpsData.gpsDateDay_dd = 0;
    odoGpsPort.gpsData.gpsDateMonth_mm = 0;
    odoGpsPort.gpsData.gpsDateYear_yy = 0;
    odoGpsPort.gpsData.gpsNoOfSatellites = static_cast<uint8_t>(GNavSensor.Receiver.NoVisibleSat);
    odoGpsPort.gpsData.ReceiverStatus_nu = ap_commonvehsigprovider::GpsReceiverStatus::ODO_GPS_VALID;
    odoGpsPort.gpsData.verticalDOP = static_cast<float32_t>(GNavSensor.Receiver.VDOP);
    odoGpsPort.gpsData.horizontalDOP = static_cast<float32_t>(GNavSensor.Receiver.HDOP);
    odoGpsPort.gpsData.timeDOP = static_cast<float32_t>(GNavSensor.Receiver.TDOP);
    odoGpsPort.gpsData.geometricDOP = static_cast<float32_t>(GNavSensor.Receiver.GDOP);
    odoGpsPort.gpsData.positionDOP = static_cast<float32_t>(GNavSensor.Receiver.PDOP);

    /*AmbientDataPort*/
    updateSignalHeader(timestamp_us, ambientDataPort.sSigHeader);
    ambientDataPort.sSigHeader.uiTimeStamp = timestamp_us;
    ambientDataPort.Ambient_pressure = 1;
    ambientDataPort.Ambient_temperature = 20;

    updateSignalHeader(timestamp_us, ddsPort.sSigHeader);
    ddsPort.Dds_warning_active = false;
    ddsPort.Tire_change_reset_button = false;
}

#ifdef USE_HMI
static void updateCANTxData() {

    PRIVATE_CAN_SetBtnFullyAutomParkPoss(static_cast<unsigned char>(gHmiInputPort.general.btnFullyAutomParkingPoss_nu));
    PRIVATE_CAN_SetBtnSemiAutomParkPoss(static_cast<unsigned char>(gHmiInputPort.general.btnSemiAutomParkingPoss_nu));
    PRIVATE_CAN_SetRemoteModeActive(static_cast<unsigned char>(gHmiInputPort.general.remoteModeActive_nu));
    PRIVATE_CAN_SetRemoteAppActive(static_cast<unsigned char>(gHmiInputPort.general.remoteAppActive_nu));
    PRIVATE_CAN_SetRemoteAppAuthorized(static_cast<unsigned char>(gHmiInputPort.general.remoteAppAuthorized_nu));
    PRIVATE_CAN_SetRemoteAppCoded(static_cast<unsigned char>(gHmiInputPort.general.remoteAppCoded_nu));
    PRIVATE_CAN_SetContinuePoss(static_cast<unsigned char>(gHmiInputPort.general.continuePoss_nu));
    PRIVATE_CAN_SetParkInPoss(static_cast<unsigned char>(gHmiInputPort.general.parkInPoss_nu));
    PRIVATE_CAN_SetParkOutPoss(static_cast<unsigned char>(gHmiInputPort.general.parkOutPoss_nu));
    PRIVATE_CAN_SetRemManPoss(static_cast<unsigned char>(gHmiInputPort.general.remManPoss_nu));
    PRIVATE_CAN_SetUndoPoss(static_cast<unsigned char>(gHmiInputPort.general.undoPoss_nu));
    PRIVATE_CAN_SetSvPoss(static_cast<unsigned char>(gHmiInputPort.general.svPoss_nu));
    PRIVATE_CAN_SetBtnForwardPoss(static_cast<unsigned char>(gHmiInputPort.general.btnForwardPoss_nu));
    PRIVATE_CAN_SetBtnBackwardPoss(static_cast<unsigned char>(gHmiInputPort.general.btnBackwardPoss_nu));
    PRIVATE_CAN_SetDistanceToStop(static_cast<float>(gHmiInputPort.general.distanceToStop_perc) * 0.4f); //parameter expected in physical value, but 0.4 factor already applied in HMIH
    PRIVATE_CAN_SetDrivingDirection(static_cast<unsigned char>(gHmiInputPort.general.drivingDirection_nu));
    PRIVATE_CAN_SetAVGType(static_cast<unsigned char>(gHmiInputPort.general.avgType_nu));
    PRIVATE_CAN_SetCurrentGear(static_cast<unsigned char>(gHmiInputPort.general.currentGear_nu));
    PRIVATE_CAN_SetParkInOutFinished(static_cast<unsigned char>(gHmiInputPort.general.finishType_nu != ap_psm_app::APFinishType::AP_NOT_FINISHED));
    PRIVATE_CAN_SetRemoteKeyPoss(static_cast<unsigned char>(gHmiInputPort.general.remoteKeyPoss_nu));
    PRIVATE_CAN_SetStateVarPPC(static_cast<unsigned char>(psmDebugPort.stateVarPPC_nu));
    PRIVATE_CAN_SetHeadUnitScreen(static_cast<unsigned char>(headUnitVisualizationPort.screen_nu));
    PRIVATE_CAN_SetHeadUnitMessage(static_cast<unsigned char>(headUnitVisualizationPort.message_nu));
    PRIVATE_CAN_SetRemoteScreen(static_cast<unsigned char>(remoteVisualizationPort.screen_nu));
    PRIVATE_CAN_SetRemoteMessage(static_cast<unsigned char>(remoteVisualizationPort.message_nu));
    PRIVATE_CAN_SetGarageParking(static_cast<unsigned char>(gHmiInputPort.general.garageParking_nu));
    PRIVATE_CAN_SetReverseAssistPoss(static_cast<unsigned char>(gHmiInputPort.reverseAssistAvailabilityPort_nu.pathAvailable));
    PRIVATE_CAN_SetMemParkingPoss(static_cast<unsigned char>(gHmiInputPort.general.memoryParkingPoss_nu));
    PRIVATE_CAN_SetSlotUnreachReason(static_cast<unsigned char>(gHmiInputPort.general.slotUnreachReason_nu));
    PRIVATE_CAN_SetMemorySlotsStatus(static_cast<unsigned char>(gHmiInputPort.general.memorySlotsStatus_nu));
    PRIVATE_CAN_SetMemorizedPoseSlotId(static_cast<unsigned char>(gHmiInputPort.general.redetectedPoseMemSlotId_nu));
    PRIVATE_CAN_SetDisplayBackButton(static_cast<unsigned char>(gHmiInputPort.general.displayBackButton_nu));
    PRIVATE_CAN_SetAdjustmentButtons(static_cast<unsigned char>(gHmiInputPort.general.adjustmentButtons_nu));

    //following 2 signals are filled with default values just to allow testing in SIL part of video stream functionality in HMI
    //they are normally filled with ViewController output, but ViewController is not integrated in SIL
    //PRIVATE_CAN_SetStreamAvailable(1U);
    //PRIVATE_CAN_SetSVScreenResponse(static_cast<unsigned char>((AP_HMIToAP::ScreenTypes::TOP_VIEW)));

    PRIVATE_CAN_SetParkingSpacesScanned((unsigned char*)gHmiInputPort.parkingSpaces.front.scanned_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.right.scanned_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.rear.scanned_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.left.scanned_nu);
    PRIVATE_CAN_SetParkingSpacesFree((unsigned char*)gHmiInputPort.parkingSpaces.front.free_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.right.free_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.rear.free_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.left.free_nu);
    PRIVATE_CAN_SetParkingSpacesSelected((unsigned char*)gHmiInputPort.parkingSpaces.front.selected_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.right.selected_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.rear.selected_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.left.selected_nu);
    PRIVATE_CAN_SetParkingSpacesPossOrientation((unsigned char*)gHmiInputPort.parkingSpaces.front.possOrientation_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.right.possOrientation_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.rear.possOrientation_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.left.possOrientation_nu);
    PRIVATE_CAN_SetParkingSpacesSelOrientation((unsigned char*)gHmiInputPort.parkingSpaces.front.selectedOrientation_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.right.selectedOrientation_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.rear.selectedOrientation_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.left.selectedOrientation_nu);
    PRIVATE_CAN_SetParkingSpacesPossDirection((unsigned char*)gHmiInputPort.parkingSpaces.front.possDirection_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.right.possDirection_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.rear.possDirection_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.left.possDirection_nu);
    PRIVATE_CAN_SetParkingSpacesSelDirection((unsigned char*)gHmiInputPort.parkingSpaces.front.selectedDirection_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.right.selectedDirection_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.rear.selectedDirection_nu,
        (unsigned char*)gHmiInputPort.parkingSpaces.left.selectedDirection_nu);
    if (gHmiInputPort.parkingSpaces.right.memorizedPoseYaw_rad) {
        PRIVATE_CAN_SetParkingSpaceMemorizedPoseYaw(static_cast<float>(gHmiInputPort.parkingSpaces.right.memorizedPoseYaw_rad));
    }
    else if (gHmiInputPort.parkingSpaces.left.memorizedPoseYaw_rad) {
        PRIVATE_CAN_SetParkingSpaceMemorizedPoseYaw(static_cast<float>(gHmiInputPort.parkingSpaces.left.memorizedPoseYaw_rad));
    }
    else if (gHmiInputPort.parkingSpaces.front.memorizedPoseYaw_rad) {
        PRIVATE_CAN_SetParkingSpaceMemorizedPoseYaw(static_cast<float>(gHmiInputPort.parkingSpaces.front.memorizedPoseYaw_rad));
    }
    else {
        PRIVATE_CAN_SetParkingSpaceMemorizedPoseYaw(static_cast<float>(gHmiInputPort.parkingSpaces.rear.memorizedPoseYaw_rad));
    }
    PRIVATE_CAN_SetParkingSituationFront(gHmiInputPort.parkingSituation.front.notAvailable_nu, gHmiInputPort.parkingSituation.front.perpendicularParkingOut_nu);
    PRIVATE_CAN_SetParkingSituationRear(gHmiInputPort.parkingSituation.rear.notAvailable_nu, gHmiInputPort.parkingSituation.rear.perpendicularParkingOut_nu);
    PRIVATE_CAN_SetParkingSituationLeft(gHmiInputPort.parkingSituation.left.notAvailable_nu,
        gHmiInputPort.parkingSituation.left.parallelParkingOut_nu,
        gHmiInputPort.parkingSituation.left.uncertainSituation_nu,
        gHmiInputPort.parkingSituation.left.street_nu,
        gHmiInputPort.parkingSituation.left.angledStandardSpaces_nu,
        gHmiInputPort.parkingSituation.left.angledReverseSpaces_nu,
        gHmiInputPort.parkingSituation.left.parallelParkingSpaces_nu,
        gHmiInputPort.parkingSituation.left.perpendicularParkingSpaces_nu);
    PRIVATE_CAN_SetParkingSituationRight(gHmiInputPort.parkingSituation.right.notAvailable_nu,
        gHmiInputPort.parkingSituation.right.parallelParkingOut_nu,
        gHmiInputPort.parkingSituation.right.uncertainSituation_nu,
        gHmiInputPort.parkingSituation.right.street_nu,
        gHmiInputPort.parkingSituation.right.angledStandardSpaces_nu,
        gHmiInputPort.parkingSituation.right.angledReverseSpaces_nu,
        gHmiInputPort.parkingSituation.right.parallelParkingSpaces_nu,
        gHmiInputPort.parkingSituation.right.perpendicularParkingSpaces_nu);
    PRIVATE_CAN_SetParkEgoRelativePosLeft(gHmiInputPort.parkingSituation.left.egoRelativePos_nu);
    PRIVATE_CAN_SetParkEgoRelativePosRight(gHmiInputPort.parkingSituation.right.egoRelativePos_nu);

    PRIVATE_CAN_SetPdcStatus(static_cast<unsigned char>(gHmiInputPort.general.pdcSystemState_nu),
        static_cast<unsigned char>(gHmiInputPort.general.pdcShutdownCause_nu));
    PRIVATE_CAN_SetPdcDrvTube(static_cast<unsigned char>(gHmiInputPort.drivingTube.drvTubeDirection_nu),
        static_cast<unsigned char>(gHmiInputPort.drivingTube.drvTubeDisplay_nu),
        gHmiInputPort.drivingTube.frontRadius_cm,
        gHmiInputPort.drivingTube.rearRadius_cm);
    for (int i = 0; i < mf_hmih::MF_HMIH_Consts::MAX_NUM_SECTORS_PER_SIDE_HMIH; i++) {
        PRIVATE_CAN_SetPdcFrontSector(i, static_cast<unsigned char>(gHmiInputPort.pdcSectors.front[i].criticalityLevel_nu),
            gHmiInputPort.pdcSectors.front[i].intersectsDrvTube_nu, gHmiInputPort.pdcSectors.front[i].slice_nu);
        PRIVATE_CAN_SetPdcLeftSector(i, static_cast<unsigned char>(gHmiInputPort.pdcSectors.left[i].criticalityLevel_nu),
            gHmiInputPort.pdcSectors.left[i].intersectsDrvTube_nu, gHmiInputPort.pdcSectors.left[i].slice_nu);
        PRIVATE_CAN_SetPdcRearSector(i, static_cast<unsigned char>(gHmiInputPort.pdcSectors.rear[i].criticalityLevel_nu),
            gHmiInputPort.pdcSectors.rear[i].intersectsDrvTube_nu, gHmiInputPort.pdcSectors.rear[i].slice_nu);
        PRIVATE_CAN_SetPdcRightSector(i, static_cast<unsigned char>(gHmiInputPort.pdcSectors.right[i].criticalityLevel_nu),
            gHmiInputPort.pdcSectors.right[i].intersectsDrvTube_nu, gHmiInputPort.pdcSectors.right[i].slice_nu);
    }

    PRIVATE_CAN_SetFrontSpeaker(toneOutputPort.speakerOutput[0].pitch_nu, toneOutputPort.speakerOutput[0].volume_nu, toneOutputPort.speakerOutput[0].soundOn_nu);
    PRIVATE_CAN_SetRearSpeaker(toneOutputPort.speakerOutput[1].pitch_nu, toneOutputPort.speakerOutput[1].volume_nu, toneOutputPort.speakerOutput[1].soundOn_nu);

    PRIVATE_CAN_SetWhlWarnFL(static_cast<unsigned char>(gHmiInputPort.wheelWarnings.warningLevel_nu[0]), gHmiInputPort.wheelWarnings.whlAngleAbs_deg[0], static_cast<unsigned char>(gHmiInputPort.wheelWarnings.whlAngDirection_nu[0]));
    PRIVATE_CAN_SetWhlWarnFR(static_cast<unsigned char>(gHmiInputPort.wheelWarnings.warningLevel_nu[1]), gHmiInputPort.wheelWarnings.whlAngleAbs_deg[1], static_cast<unsigned char>(gHmiInputPort.wheelWarnings.whlAngDirection_nu[1]));
    PRIVATE_CAN_SetWhlWarnRL(static_cast<unsigned char>(gHmiInputPort.wheelWarnings.warningLevel_nu[2]), gHmiInputPort.wheelWarnings.whlAngleAbs_deg[2], static_cast<unsigned char>(gHmiInputPort.wheelWarnings.whlAngDirection_nu[2]));
    PRIVATE_CAN_SetWhlWarnRR(static_cast<unsigned char>(gHmiInputPort.wheelWarnings.warningLevel_nu[3]), gHmiInputPort.wheelWarnings.whlAngleAbs_deg[3], static_cast<unsigned char>(gHmiInputPort.wheelWarnings.whlAngDirection_nu[3]));
    PRIVATE_CAN_SetWhpStatus(static_cast<unsigned char>(gHmiInputPort.general.whpState_nu), static_cast<unsigned char>(gHmiInputPort.general.whpDisplayReq_nu));
}
#endif

static void updateEgoMotionPort(const lsm_vedodo::OdoEstimation& odoEstimation, si::EgoMotionPort& portToUpdate) {
    portToUpdate.uiVersionNumber = si::createEgoMotionPort_InterfaceVersion().EgoMotionPort_VERSION;
    updateSignalHeader(odoEstimation.sSigHeader.uiTimeStamp, portToUpdate.sSigHeader);
    portToUpdate.motionState_nu = (lsm_vedodo::MotionState::ODO_STANDSTILL == odoEstimation.motionStatus_nu) ?
        si::SIMotionState::SI_ODO_STANDSTILL : si::SIMotionState::SI_ODO_NO_STANDSTILL;
    portToUpdate.pitch_rad = odoEstimation.pitchAngle_rad;
    portToUpdate.roll_rad = odoEstimation.rollAngle_rad;
    portToUpdate.vel_mps = odoEstimation.longiVelocity_mps;
    portToUpdate.yawRate_radps = odoEstimation.yawRate_radps;
    portToUpdate.accel_mps2 = odoEstimation.longiAcceleration_mps2;
    portToUpdate.drivenDistance_m = odoEstimation.drivenDistance_m;
    portToUpdate.frontWheelAngle_rad = odoEstimation.steerAngFrontAxle_rad;
    portToUpdate.rearWheelAngle_rad = odoEstimation.steerAngRearAxle_rad;
}

static void updateEgoMotionData() {
    updateEgoMotionPort(odoEstimationPortCM, egoMotionPortCM);
    if (carMakerInterface.vedodoActive_nu) {
        updateEgoMotionPort(odoEstimationOutputPort.odoEstimation, egoMotionPort);
    }
    else {
        egoMotionPort = egoMotionPortCM;
    }
}

//The given function is local and not referenced in the body of the module; therefore, the function is dead code.
//uncomment this function if is need it.

//static void updateNumberOfDelimiters() {
//    for (int ii = 0U; ((ii < gParkingBoxPort.numValidParkingBoxes_nu) && (ii < AP_Common::AP_G_MAX_NUM_PARKING_BOXES_NU)); ii++) {
//        int numValidDelimiters_nu = 0;
//        for (auto delimiters : gParkingBoxPort.parkingBoxes[ii].delimiters) {
//            if (delimiters.delimitingSide_nu != SI::RelativeLocationToParkingBox::UNDEFINED_EDGE) numValidDelimiters_nu++;
//        }
//        gParkingBoxPort.parkingBoxes[ii].numValidDelimiters_nu = numValidDelimiters_nu;
//    }
//}

static LSM_GEOML::Pose getPose(const lsm_vedodo::OdoEstimation &odoEstimation) {
    return { odoEstimation.xPosition_m, odoEstimation.yPosition_m, odoEstimation.yawAngle_rad };
}

static void updateOdometryData() {
    static float steeringWheelAngle_lastloop_rad{ 0.0f };
    static float steeringWheelAngleVelocity_lastloop_rad{ 0.0f };
    static unsigned short count{ 9U };
    static std::array<bool, 4> b_velInc_nu = { true, true, true, true };   // used by velocity hysteresis

    // initialize static local variables to have reproducible simulation runs
    if (isFirstCycle) {
        steeringWheelAngle_lastloop_rad = 0.0f;
        steeringWheelAngleVelocity_lastloop_rad = 0.0f;
        count = 9U;
        b_velInc_nu = { true, true, true, true };
    }
    const uint64_t timestamp_us = static_cast<uint64_t>(std::round(1e6 * SimCore.Time));

    /* OdoExtCtrlPort */
    updateSignalHeader(timestamp_us, odoExtCtrlPort.sSigHeader);
    if (SimCore.Time > 0.015 && SimCore.Time < 0.017) {
        odoExtCtrlPort.resetPoseEstimation_nu = b_TRUE;
        lastTimeVedodoReset_s = SimCore.Time;
    }
    // keep the resetPoseEstimate request active for one mf_vedodo cycle
    if (SimCore.Time > (lastTimeVedodoReset_s + (SHORT_SAMPLE_TIME_MS * 0.001))) {
        odoExtCtrlPort.resetPoseEstimation_nu = b_FALSE;
    }

    /* OdoSigFcanPort */
    updateSignalHeader(timestamp_us, motionStatePort.sSigHeader);
    motionStatePort.vehicleDrivingDirection_nu = ap_commonvehsigprovider::VehicleDrivingDirection::VEHICLE_DIRECTION_REVERSE;
    motionStatePort.vehicleStandstill_nu = false;
    motionStatePort.vRefESC_mps = fabsf((float)Car.ConBdy1.v_1[0]);
    motionStatePort.vRefESC_QF_nu = true;

    updateSignalHeader(timestamp_us, steerCtrlStatusPort.sSigHeader);
    steerCtrlStatusPort.steeringWheelAngle_rad = (float)Steering.IF.Ang;
    steerCtrlStatusPort.steeringWheelAngle_QF_nu = true;
    steerCtrlStatusPort.steeringWheelAngleOffset_rad = 0.0f;

    if (++count >= 10)
    {
        steerCtrlStatusPort.steeringWheelAngleVelocity_radps = (float)(Steering.IF.Ang - steeringWheelAngle_lastloop_rad) / 0.01f * 1.f;
        steeringWheelAngle_lastloop_rad = (float)Steering.IF.Ang;   // Steering.IF.AngVel remains zero by Carmaker.
        //odoSigFcanPort.steerCtrlStatus.steeringWheelAngleVelocity_radps     = (float)Steering.IF.AngVel * 1.018599348148148f; // see steeringWheelAngle_rad

        //AlBu
        if (SimCore.Time > 0.1) steeringWheelAngleAcceleration = (float)(steerCtrlStatusPort.steeringWheelAngleVelocity_radps - steeringWheelAngleVelocity_lastloop_rad) / 0.01f;
        steeringWheelAngleVelocity_lastloop_rad = (float)steerCtrlStatusPort.steeringWheelAngleVelocity_radps;
        steeringAngleAcceleration = (steeringWheelAngleAcceleration / carMakerInterface.vehicle_Params.AP_V_STEER_RATIO_NU);
        count = 0;
    }

    steerCtrlStatusPort.steeringWheelAngleVelocity_QF_nu = true;
    steerCtrlStatusPort.epsAppliedTieRodForce_Nm = 0.0f;

    updateSignalHeader(timestamp_us, vehDynamicsPort.sSigHeader);
    vehDynamicsPort.lateralAccelerationOffset_mps2 = 0.0f;
    vehDynamicsPort.lateralAcceleration_mps2 = static_cast<float>(InertialSensor[5].Acc_B[1]);   // acc y
    vehDynamicsPort.lateralAcceleration_QF_nu = true;

    vehDynamicsPort.longitudinalAccelerationOffset_mps2 = 0.0f;
    Acc_y_filt = (float)(InertialSensor[5].Acc_B[0]);   // acc x
    FILTER_LP1(Acc_y_filt, accOld_mps, (float)cmFiltCoeff_ay);
    vehDynamicsPort.longitudinalAcceleration_mps2 = Acc_y_filt;
    accOld_mps = Acc_y_filt;
    Acc_y_filt = Acc_y_filt * PowerTrain.GearBoxIF.GearNo;
    vehDynamicsPort.longitudinalAcceleration_QF_nu = true;

    vehDynamicsPort.yawRateOffset_radps = 0.0f;
    vehDynamicsPort.yawRate_QF_nu = true;
    vehDynamicsPort.yawRate_radps = static_cast<float> (InertialSensor[5].Omega_B[2]);
    //vehDynamicsPort.yawRate_radps = (float)Vehicle.YawRate;

    #ifndef VARIANT_CUS_ONLY
        vehDynamicsPort.verticalAcceleration_mps2 = static_cast<float>(InertialSensor[5].Acc_B[2]); // need to check acc z assuming it as vertical body acceleration
        vehDynamicsPort.verticalAccelerationOffset_mps2 = 0;
        vehDynamicsPort.verticalAcceleration_QF_nu = true;

        vehDynamicsPort.pitchRate_radps = static_cast<float> (InertialSensor[5].Omega_B[1]);
        //vehDynamicsPort.pitchRate_radps = (float)Vehicle.PitchVel; // need to check
        vehDynamicsPort.pitchRateOffset_radps = 0.0f;
        vehDynamicsPort.pitchRate_QF_nu = true;            //need to check

        vehDynamicsPort.rollRate_radps = static_cast<float> (InertialSensor[5].Omega_B[0]);
        //vehDynamicsPort.rollRate_radps = (float)Vehicle.RollVel;   //need to check
        vehDynamicsPort.rollRateOffset_radps = 0.0f;
        vehDynamicsPort.rollRate_QF_nu = true;
    #else
        vehDynamicsPort.verticalAcceleration_mps2 = 0.0f;
        vehDynamicsPort.verticalAccelerationOffset_mps2 = 0.0f;
        vehDynamicsPort.verticalAcceleration_QF_nu = false;

        vehDynamicsPort.pitchRate_radps = 0.0f; // need to check
        vehDynamicsPort.pitchRateOffset_radps = 0.0f;
        vehDynamicsPort.pitchRate_QF_nu = false;            //need to check

        vehDynamicsPort.rollRate_radps = 0.0f;   //need to check
        vehDynamicsPort.rollRateOffset_radps = 0.0f;
        vehDynamicsPort.rollRate_QF_nu = false;
    #endif

    updateSignalHeader(timestamp_us, wheelPulsePort.sSigHeader);
    wheelPulsePort.wheelPulsesFL_QF_nu = true;
    wheelPulsePort.wheelPulsesFR_QF_nu = true;
    wheelPulsePort.wheelPulsesRL_QF_nu = true;
    wheelPulsePort.wheelPulsesRR_QF_nu = true;
    // Wheel pulse of last iterations
    wheelPulsesLast_nu[0] = wheelPulsesFL_nu;
    wheelPulsesLast_nu[1] = wheelPulsesFR_nu;
    wheelPulsesLast_nu[2] = wheelPulsesRL_nu;
    wheelPulsesLast_nu[3] = wheelPulsesRR_nu;
    // Wheel pulses' simulation
    wheelSpeedSensorSimulation(
        wheelPulsesFL_nu,
        wheelPulsesFR_nu,
        wheelPulsesRL_nu,
        wheelPulsesRR_nu,
        wheelDrivingDirectionsPort.wheelDrivingDirection_FL_nu,
        wheelDrivingDirectionsPort.wheelDrivingDirection_FR_nu,
        wheelDrivingDirectionsPort.wheelDrivingDirection_RL_nu,
        wheelDrivingDirectionsPort.wheelDrivingDirection_RR_nu,
        (float)Vehicle.FL.rot,
        (float)Vehicle.FR.rot,
        (float)Vehicle.RL.rot,
        (float)Vehicle.RR.rot,
        latestWheelSpeed_radps[0],
        latestWheelSpeed_radps[1],
        latestWheelSpeed_radps[2],
        latestWheelSpeed_radps[3],
        TimeStampDelta_s[0],
        TimeStampDelta_s[1],
        TimeStampDelta_s[2],
        TimeStampDelta_s[3],
        DesicionWhlMdlExt
    );
    updateSignalHeader(timestamp_us, wheelDrivingDirectionsPort.sSigHeader);
    updateSignalHeader(timestamp_us, wheelSpeedPort.sSigHeader);

    if (DesicionWhlMdlExt == 0) {
        std::array<float, 4> f_WhlSpd_radps;

        for (int idx = 0; idx < 4; idx++)
        {
            // set velocity threshold depending on hysteresis
            const float f_velThreas_radps = (true == b_velInc_nu.at(idx) ? 0.2577f : 0.1052f);

            // convert wheel velocity from radps to rpm
            const float f_cmWhlSpd_radps = abs((float)Car.Tire[idx].WheelSpd);

            if (f_cmWhlSpd_radps > f_velThreas_radps)
            {
                f_WhlSpd_radps.at(idx) = f_cmWhlSpd_radps;
                b_velInc_nu.at(idx) = false;
            }
            else
            {
                f_WhlSpd_radps.at(idx) = 0.0F;
                b_velInc_nu.at(idx) = true;
            }
        }

        // calibration:  circumference dependent on the tire model (default: 2.059 m)
        wheelSpeedPort.wheelRotSpeedFL_radps = f_WhlSpd_radps.at(0);
        wheelSpeedPort.wheelRotSpeedFR_radps = f_WhlSpd_radps.at(1);
        wheelSpeedPort.wheelRotSpeedRL_radps = f_WhlSpd_radps.at(2);
        wheelSpeedPort.wheelRotSpeedRR_radps = f_WhlSpd_radps.at(3);
    }

    // calculation of the wheel speed based on the wheel pulses simulated
    diffWheelpulses_nu[0] = wheelPulsesFL_nu - wheelPulsesLast_nu[0];
    diffWheelpulses_nu[1] = wheelPulsesFR_nu - wheelPulsesLast_nu[1];
    diffWheelpulses_nu[2] = wheelPulsesRL_nu - wheelPulsesLast_nu[2];
    diffWheelpulses_nu[3] = wheelPulsesRR_nu - wheelPulsesLast_nu[3];
    for (unsigned int i = 0; i < 4u; i++)
    {
        if (diffWheelpulses_nu[i] > 0)
        {
            CountIterationsToZero_nu[i] = 0;
            DeltaTimeStampLatestTicks_nu[i] = static_cast<float32_t>(SimCore.Time - TimeStampLatestTicks_nu[i]);
            realTimeStampLatestTicks_s[i] = DeltaTimeStampLatestTicks_nu[i] - abs(TimeStampDelta_s[i]);
            latestWheelSpeed_radps[i] = (float)diffWheelpulses_nu[i] * (float)M_PI / 43.0f / realTimeStampLatestTicks_s[i];
            TimeStampLatestTicks_nu[i] = static_cast<float32_t>(SimCore.Time);
        }
        Timedifference20ms_s[i] = SimCore.Time - TimeLast20ms_s[i];
        if (Timedifference20ms_s[i] > 0.019 && Timedifference20ms_s[i] < 0.021) {
            if (diffWheelpulses_nu[i] == 0 && wheelPulsesLast_nu[i] > 0)
            {
                CountIterationsToZero_nu[i] += 1;
                if (CountIterationsToZero_nu[i] > 15) //0.3s delay
                {
                    latestWheelSpeed_radps[i] = 0;
                }
            }
            TimeLast20ms_s[i] = SimCore.Time;
        }
        if (CountIterationsToZero_nu[i] <= 15)
        {
            PT1Filter[i].step(latestWheelSpeed_radps[i], realTimeStampLatestTicks_s[i], 1.0f);
            latestWheelSpeedFiltred_radps[i] = PT1Filter[i].getOutput();

        }
        else
        {
            PT1Filter[i].step(latestWheelSpeed_radps[i], 0.02F, 0.0f);
            latestWheelSpeedFiltred_radps[i] = PT1Filter[i].getOutput();
        }
        // Jitter Effect for the wheel speed sensor
        //Jitter effect calculated from a gaussian curve +/- 5ms to the rate of 20ms for the ticks and the speed, which means the output value can be scheduled from -15ms to 25 ms
        if (SimCore.Time - TimeAfterJitter_s[i] > 0.014 && SimCore.Time - TimeAfterJitter_s[i] < 0.016)
        {
            static std::normal_distribution<double> temporalJitterDistribution{ 0.0, 0.005 / 4 };
            JitterTime_s[i] = temporalJitterDistribution(gRandomEngine);
        }
        if (JitterTimeLast_s[i] != JitterTime_s[i])
        {
            if (SimCore.Time - TimeAfterJitter_s[i] > 0.014 + JitterTime_s[i] && SimCore.Time - TimeAfterJitter_s[i] < 0.016 + JitterTime_s[i])
            {
                LatestWheelSpeedFilteredJt_radps[i] = latestWheelSpeedFiltred_radps[i];
                switch (i)
                {
                case 0: WheelPulsesJt_nu[i] = wheelPulsesFL_nu;
                    break;
                case 1: WheelPulsesJt_nu[i] = wheelPulsesFR_nu;
                    break;
                case 2: WheelPulsesJt_nu[i] = wheelPulsesRL_nu;
                    break;
                case 3: WheelPulsesJt_nu[i] = wheelPulsesRR_nu;
                    break;
                }
            }
            JitterTimeLast_s[i] = JitterTime_s[i];
            TimeAfterJitter_s[i] = static_cast<float32_t>(SimCore.Time);
        }
    }

    // Decision regarding using ideal or medium fidelity wheel speed sensor model: Decision includes also if the jitter is to be included or not
    // wheelPulsePort.DesicionWhlMdlExt has to be added to the vehicle additional parameters in Carmaker
    wheelPulsePort.wheelPulsesFL_nu = wheelPulsesFL_nu;
    wheelPulsePort.wheelPulsesFR_nu = wheelPulsesFR_nu;
    wheelPulsePort.wheelPulsesRL_nu = wheelPulsesRL_nu;
    wheelPulsePort.wheelPulsesRR_nu = wheelPulsesRR_nu;
    switch (DesicionWhlMdlExt)
    {
        /*case 0:
                    // Carmaker speed is used /calibration:  circumference dependent on the tire model (default: 2.059 m)
                    wheelSpeedPort.wheelSpeedFL_mps = abs((float)Car.Tire[0].WheelSpd)*(2.059f / 2 / 3.14159265359f);
                    wheelSpeedPort.wheelSpeedFR_mps = abs((float)Car.Tire[1].WheelSpd)*(2.059f / 2 / 3.14159265359f);
                    wheelSpeedPort.wheelSpeedRL_mps = abs((float)Car.Tire[2].WheelSpd)*(2.059f / 2 / 3.14159265359f);
                    wheelSpeedPort.wheelSpeedRR_mps = abs((float)Car.Tire[3].WheelSpd)*(2.059f / 2 / 3.14159265359f);
                    break;
                    */
    case 1:
        // Wheels' speeds are calculated from wheel ticks but no jitter is included and no variable delays in the implementation are included
        wheelSpeedPort.wheelRotSpeedFL_radps = latestWheelSpeedFiltred_radps[0];
        wheelSpeedPort.wheelRotSpeedFR_radps = latestWheelSpeedFiltred_radps[1];
        wheelSpeedPort.wheelRotSpeedRL_radps = latestWheelSpeedFiltred_radps[2];
        wheelSpeedPort.wheelRotSpeedRR_radps = latestWheelSpeedFiltred_radps[3];
        break;
    case 2:
        // Wheels' speeds are calculated from wheel ticks but no jitter is included, only variable delays in the implementation are included
        // Delays inclusion is made inside the wheelSpeedSensorSim.cpp
        wheelSpeedPort.wheelRotSpeedFL_radps = latestWheelSpeedFiltred_radps[0];
        wheelSpeedPort.wheelRotSpeedFR_radps = latestWheelSpeedFiltred_radps[1];
        wheelSpeedPort.wheelRotSpeedRL_radps = latestWheelSpeedFiltred_radps[2];
        wheelSpeedPort.wheelRotSpeedRR_radps = latestWheelSpeedFiltred_radps[3];
        break;
    case 3: // Delay and jitter effects are included
        wheelSpeedPort.wheelRotSpeedFL_radps = LatestWheelSpeedFilteredJt_radps[0];
        wheelSpeedPort.wheelRotSpeedFR_radps = LatestWheelSpeedFilteredJt_radps[1];
        wheelSpeedPort.wheelRotSpeedRL_radps = LatestWheelSpeedFilteredJt_radps[2];
        wheelSpeedPort.wheelRotSpeedRR_radps = LatestWheelSpeedFilteredJt_radps[3];
        wheelPulsePort.wheelPulsesFL_nu = static_cast<uint16_t>(WheelPulsesJt_nu[0]);
        wheelPulsePort.wheelPulsesFR_nu = static_cast<uint16_t>(WheelPulsesJt_nu[1]);
        wheelPulsePort.wheelPulsesRL_nu = static_cast<uint16_t>(WheelPulsesJt_nu[2]);
        wheelPulsePort.wheelPulsesRR_nu = static_cast<uint16_t>(WheelPulsesJt_nu[3]);
        break;
    }

    // suspension //
    updateSignalHeader(timestamp_us, suspensionPort.sSigHeader);
    suspensionPort.suspensionTravelFL_m = static_cast<float32>(pSuspFL->GetFunc(pSuspFL->Var));
    suspensionPort.suspensionTravelFR_m = static_cast<float32>(pSuspFR->GetFunc(pSuspFR->Var));
    suspensionPort.suspensionTravelRL_m = static_cast<float32>(pSuspRL->GetFunc(pSuspRL->Var));
    suspensionPort.suspensionTravelRR_m = static_cast<float32>(pSuspRR->GetFunc(pSuspRR->Var));

    /* OdoSampleTimePort */
    updateSignalHeader(timestamp_us, systemTimePort.sSigHeader);
}

static void updateSuspensionData()
{
    double suspFL, suspFR, suspRL, suspRR;  // suspension travel from Carmaker

    // read suspension travel from Carmaker
    suspFL = pSuspFL->GetFunc(pSuspFL->Var);
    suspFR = pSuspFR->GetFunc(pSuspFR->Var);
    suspRL = pSuspRL->GetFunc(pSuspRL->Var);
    suspRR = pSuspRR->GetFunc(pSuspRR->Var);

    SuspensionEstimation suspEstim;  // instantiate suspension estimation

    // call suspension sensor calibration
    std::array<float32_t, SuspensionEstimation::WHEELS_NUM> calibSens_m{
                                    suspEstim.sensorModel({ static_cast<float32_t>(suspFL),
                                                            static_cast<float32_t>(suspFR),
                                                            static_cast<float32_t>(suspRL),
                                                            static_cast<float32_t>(suspRR) }) };

    groundHeightCalcFL = calibSens_m.at(SuspensionEstimation::FL_WHL_IDX);
    groundHeightCalcFR = calibSens_m.at(SuspensionEstimation::FR_WHL_IDX);
    groundHeightCalcRL = calibSens_m.at(SuspensionEstimation::RL_WHL_IDX);
    groundHeightCalcRR = calibSens_m.at(SuspensionEstimation::RR_WHL_IDX);

    // call suspension estimation
    SuspensionEstimation::Vec2D const reqPointOnPlane{ -1.096F, 0.0F };  // point on plane where height is requested
    std::tie(cmSuspPitch_rad, cmSuspRoll_rad, cmSuspHeight_m) = suspEstim.suspEstimation(calibSens_m, reqPointOnPlane);
}


std::array<float32_t, SuspensionEstimation::WHEELS_NUM> SuspensionEstimation::sensorModel(std::array<float32_t, WHEELS_NUM> const rawSens_m)
{
    std::array<float32_t, SuspensionEstimation::WHEELS_NUM> const groundHeightCalc{
        gradSuspFront_mpm * rawSens_m.at(FL_WHL_IDX) + offSuspFront_m + offZeroLoadFront_m,
        gradSuspFront_mpm * rawSens_m.at(FR_WHL_IDX) + offSuspFront_m + offZeroLoadFront_m,
        gradSuspRear_mpm * rawSens_m.at(RL_WHL_IDX) + offSuspRear_m + offZeroLoadRear_m,
        gradSuspRear_mpm * rawSens_m.at(RR_WHL_IDX) + offSuspRear_m + offZeroLoadRear_m
    };

    return groundHeightCalc;
}

std::tuple<float32_t, float32_t, float32_t> SuspensionEstimation::suspEstimation(
    std::array<float32_t, WHEELS_NUM> const calibSens_m,
    Vec2D const reqPointOnPlane)
{
    // compute normal vector based on plane equations
    Vec3D normVec{
        ((trackFront_m / 2.0F) - (trackRear_m / 2.0F)) * (calibSens_m.at(FL_WHL_IDX) - calibSens_m.at(FR_WHL_IDX)) - (trackFront_m * (calibSens_m.at(FL_WHL_IDX) - calibSens_m.at(RL_WHL_IDX))),
        -wheelBase_m * (calibSens_m.at(FL_WHL_IDX) - calibSens_m.at(FR_WHL_IDX)),
        trackFront_m * wheelBase_m
    };

    // length of vector for normalization
    float32_t const vecLength{ std::sqrt(std::pow(normVec.x, 2.0F) +
                                         std::pow(normVec.y, 2.0F) +
                                         std::pow(normVec.z, 2.0F)) };
    // normalize
    normVec.x /= vecLength;
    normVec.y /= vecLength;
    normVec.z /= vecLength;

    // compute angles
    float32_t const suspPitch_rad = std::asin(normVec.x);
    float32_t const suspRoll_rad = std::asin(-normVec.y);

    float32_t const suspHeight_m{
        ((static_cast<float32_t>(calibSens_m.at(FL_WHL_IDX)) * normVec.z) + (trackFront_m * normVec.y / 2.0F) + (wheelBase_m * normVec.x) - (reqPointOnPlane.x * normVec.x) - (reqPointOnPlane.y * normVec.y)) / normVec.z
    };

    return std::make_tuple(suspPitch_rad, suspRoll_rad, suspHeight_m);
}





static void initializeActuators() {
    initializeLowSpeedLateralDMC();
#ifdef MOCO_REPLACES_LODMC
    MocoWrapper::getInstance().init(carMakerInterface.vehicle_Params);
#endif
}

static void ctrlActuators() {
    /*SteerCtrlRequestPort*/
    if (ap_trjctl::LaDMCCtrlRequestInterfaceType::LADMC_REQ_TYPE_TORQUE_FRONT != laDMCCtrlRequestPort.laDMCCtrlRequestInterface_nu) {
        lowSpeedLateralDMC(&steerAngReqIn, &steerAngReqOut, laDMCCtrlRequestPort.laDMCCtrlRequest_nu, laDMCCtrlRequestPort.steerWheelAngReq_rad, (float)(SimCore.Time - lastCallTime_us), (float)Steering.IF.Ang);
        Steering.IF.AssistTrqCol_Ext = 0.0;
        if (laDMCCtrlRequestPort.laDMCCtrlRequest_nu) {
            VehicleControl.Steering.Ang = steerAngReqOut;
            VehicleControl.Steering.AngVel = (steerAngReqOut - lastAng_rad) / 0.001;
            hadSteerReq_nu = true;
        }
        //After a steering mf steering -> smoothly return to CM steering
        else if (hadSteerReq_nu) {
            const float32_t wheelSpeedHandOverMax_radps = 10.0F;

            const float32_t deltaMaxSteer_rad = wheelSpeedHandOverMax_radps * static_cast<float32_t>(SimCore.DeltaT);// =0.01F;
            if ((VehicleControl.Steering.Ang - lastAng_rad) > LSM_GEOML::MIN_FLT_DIVISOR)
            {
                VehicleControl.Steering.Ang = lastAng_rad + std::min(static_cast<float32_t>(VehicleControl.Steering.Ang - lastAng_rad), deltaMaxSteer_rad);
            }
            else if ((VehicleControl.Steering.Ang - lastAng_rad) < LSM_GEOML::MIN_FLT_DIVISOR)
            {
                VehicleControl.Steering.Ang = lastAng_rad + std::max(static_cast<float32_t>(VehicleControl.Steering.Ang - lastAng_rad), -deltaMaxSteer_rad);
            }
            else
            {
                hadSteerReq_nu = false; // entirely give control back to CM
            }
        }
    }

    if (laDMCCtrlRequestPort.laDMCCtrlRequest_nu &&
        ap_trjctl::LaDMCCtrlRequestInterfaceType::LADMC_REQ_TYPE_TORQUE_FRONT == laDMCCtrlRequestPort.laDMCCtrlRequestInterface_nu) {
        //forward the torque from lsca
        Steering.IF.AssistTrqCol_Ext = laDMCCtrlRequestPort.frontSteerTorqueReq_Nm;
    }

    lastAng_rad = VehicleControl.Steering.Ang;

#ifdef MOCO_REPLACES_LODMC
        mocoCarmakerInput.timestamp_ms = static_cast<uint64>(std::round(1e3 * SimCore.Time));
        mocoCarmakerInput.overrideAccel = (DrivMan.Gas > 0.01); // If gas pedal pressed
        mocoCarmakerInput.overrideDecel = (DrivMan.Brake > 0.01); // If brake pedal pressed
        mocoCarmakerInput.engineMaxTrq_Nm = OCTAGONEngine_MaxTrq_DDictEntry ? OCTAGONEngine_MaxTrq_DDictEntry->GetFunc(OCTAGONEngine_MaxTrq_DDictEntry->Var) : 0.0;
        mocoCarmakerInput.engineMinTrq_Nm = OCTAGONEngine_MinTrq_DDictEntry ? OCTAGONEngine_MinTrq_DDictEntry->GetFunc(OCTAGONEngine_MinTrq_DDictEntry->Var) : 0.0;
        mocoCarmakerInput.engineTrq_Nm = PowerTrain.EngineIF.Trq;
        mocoCarmakerInput.engineTrqDriverReq_Nm = gasInterpretedTrq_DDictEntry ? gasInterpretedTrq_DDictEntry->GetFunc(gasInterpretedTrq_DDictEntry->Var) : 0.0;
        mocoCarmakerInput.steerAngleFront_rad = odoEstimationPortCM.steerAngFrontAxle_rad;
        mocoCarmakerInput.vehicleWasSecuredMoCo_nu = vehicleWasSecuredMoCo_nu;
        const auto &odoEstimationPortToUse = carMakerInterface.vedodoActive_nu ? odoEstimationOutputPort.odoEstimation : odoEstimationPortCM;

    MocoWrapper::getInstance().run(mocoCarmakerInput,
        odoEstimationPortToUse,
        gearboxCtrlStatusPort,
        longManeuverRequestPort,
        controlData_TRATCO,
        controlData_VECONA,
        longDriverFeedback,
        vehParam,
        vehDynFcu,
        brakeFb,
        powertrainFb,
        steeringFrontFb,
        steeringRearFb,
        tratcoStatusPort,
        longAccelReq,
        tratcoProcessMemory,
        vehDynVecona,
        longVeconaStatusPort,
        veconaBrakeRequest,
        veconaPowertrainRequest,
        veconaProcessMemory);

    //Set vehicleWasSecuredMoCo_nu false in case of control request activation (e.g. new stroke) OR gas pedal was pressed OR VECONA requests to open parking brake
    if ((longManeuverRequestPort.ctrlReq.distanceToStopReq_m > 0.15F) || (VehicleControl.Gas > 0.01F) || (veconaBrakeRequest.ssmReq == VECONA_SSM_REQ_GO)) vehicleWasSecuredMoCo_nu = false;
    vehicleWasSecuredMoCo_nu = (veconaBrakeRequest.secureStandstill || (VehicleControl.BrakePark > 0.001) || vehicleWasSecuredMoCo_nu);

    if (longManeuverRequestPort.activateCtrl)
    {
        // OCTAGON Brake torque request
        setBrakeRequest(veconaBrakeRequest.axleTrqSumReq);
        // OCTAGON Engine torque request
        const double factorGearbox2Axle{ PowerTrain.DriveLineIF.CfgIF->iDiff_mean };
        const double engineTorqueReq{ veconaPowertrainRequest.axleTrqSumReq / (std::abs(PowerTrain.GearBoxIF.i) * factorGearbox2Axle) };
        if (engineTorqueReq == 0) {
            DVA_WriteRequest("PT.Control.Engine.Load", OWMode_Abs, MocoWrapper::VECONA_CYCLE_TIME_MS, 0.0, 0.0, 0.0, "");
            // Workaround (VDE-5182): Issue solved by setting the engine load to zero if MoCo is giving no request
            // The problem is not fully understood, but the selector control only changes gear if the engine rotv is below ControlUinit/TCU/ManualShiftingLimits/Min.
            // For whatever reason if a target torque of zero is requested, the current engine rotation speed is maintained but not controlled to idle speed, so no gear change is performed.
            // Setting the engine load to zero leads to rotv around the idle speed which is below this threshold. I believe it has to do with the PTControl model from CarMaker which is controlling the engine speed.
        }
        else if (veconaPowertrainRequest.activateCtrl) {
            DVA_WriteRequest("PT.Control.Engine.Trq_trg", OWMode_Abs, MocoWrapper::VECONA_CYCLE_TIME_MS, 0.0, 0.0, engineTorqueReq, "");
        }
        else {
        }

        //#define MOCO_DEACTIVATE_VECONA
#ifdef MOCO_DEACTIVATE_VECONA
        //HACK to override VECONA gear request based on MF_Control outputs.
        if (gearBoxCtrlRequestPort.gearboxCtrlRequest_nu) {
            if (gearBoxCtrlRequestPort.gearReq_nu == AP_CommonVehSigProvider::Gear::GEAR_D) veconaPowertrainRequest.gearReq = VECONA_GEAR_REQ_FORWARD;
            else if (gearBoxCtrlRequestPort.gearReq_nu == AP_CommonVehSigProvider::Gear::GEAR_R) veconaPowertrainRequest.gearReq = VECONA_GEAR_REQ_REVERSE;
            else veconaPowertrainRequest.gearReq = VECONA_GEAR_REQ_NEUTRAL;
        }
#endif //MOCO_DEACTIVATE_VECONA
        if (VECONA_GEAR_REQ_NO_REQ != veconaPowertrainRequest.gearReq)
        {
            switch (veconaPowertrainRequest.gearReq)
            {
            case VECONA_GEAR_REQ_REVERSE:
                DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, -1, NULL);
                lastGearReq_nu = -1;
                break;
            case VECONA_GEAR_REQ_NEUTRAL:
                DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, 0, NULL);
                lastGearReq_nu = 0;
                break;
            case VECONA_GEAR_REQ_FORWARD:
                DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, 1, NULL);
                lastGearReq_nu = 1;
                break;
            default:
                DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, 1, NULL);
                lastGearReq_nu = 1;
            }
            if (!hadGearReq_nu)
            {
                testEvaluation.setNumberOfGearStrokes(1U);  // count first stroke
            }
            hadGearReq_nu = true;
        }
        else if (hadGearReq_nu) {
            DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, 0, NULL);
        }
    }
    else
    {
        setBrakeRequest(0.0);
    }

#ifdef MOCO_DEACTIVATE_VECONA

    static AP_TRJCTL::LoDMCCtrlRequestPort loDMCCtrlRequestPort_longAccelReq{};
    loDMCCtrlRequestPort.loDMCCtrlRequest_nu = longAccelReq.activateCtrl;
    loDMCCtrlRequestPort_longAccelReq.emergencyHoldReq_nu = longAccelReq.fullBrakingReq;
    loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu = (longAccelReq.drivingDirReq == 2U) ? false : true;
    loDMCCtrlRequestPort_longAccelReq.distanceToStopReq_m = longManeuverRequestPort.ctrlReq.distanceToStopReq_m;
    accelReq_mps2 = longAccelReq.accelReq;


    /*longAccelReq.activateCtrl*/
    if (longAccelReq.activateCtrl)
    {
        curAccel_mps2 = Vehicle.PoI_Acc_1[0];

        if (loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu == false) {/*change sign of current accel. if driving backwards is desired*/
            curAccel_mps2 *= -1.0;
        }

        slope_rad = gain_slope * Car.FARoadSensor.Route.LongSlope;

        /*begin Code taken from AccelCtrl CM Example*/
        /* Controller for converting desired ax to gas or brake */
        if (loDMCCtrlRequestPort_longAccelReq.holdReq_nu) {
            VehicleControl.Gas = 0;
            // VehicleControl.Brake = 0.1 @ 3kph results in 3 m/ss deceleration
            //changed for driving on slopes until 16%
            if (abs(slope_rad) * 180 / (float)M_PI > 3)
            {
                VehicleControl.Brake = fmaxf(ApCtrl_stMinBrkValAntiExc_C, 0.3f); //max( 0.3, ApCtrl_stMinBrkValAntiExc_C);
            }
            else
            {
                VehicleControl.Brake = fmaxf(ApCtrl_stMinBrkValAntiExc_C, 0.1f);//max(0.10, ApCtrl_stMinBrkValAntiExc_C);
            }
            //Detect falling edge for slope drive control
            if (VehicleControl.Brake == 0 && ApCtrl_boolBrkAcv == true)
            {
                ApCtrl_boolHldBrk = true;
                ApCtrl_boolBrkAcv = false;
            }
            c_i = 0; c = 0; delta_ax = 0;

        }
        else if (loDMCCtrlRequestPort_longAccelReq.emergencyHoldReq_nu) {
            VehicleControl.Gas = 0;
            if (abs(slope_rad) * 180 / (float)M_PI > 3)
            {
                VehicleControl.Brake = 0.95;

            }
            else
            {
                // VehicleControl.Brake = 0.3 @ 3kph results in 6 m/ss deceleration
                VehicleControl.Brake = 0.86;
            }
            //Detect falling edge for slope drive control
            if (VehicleControl.Brake == 0 && ApCtrl_boolBrkAcv == true)
            {
                ApCtrl_boolHldBrk = true;
                ApCtrl_boolBrkAcv = false;
            }
            c_i = 0; c = 0; delta_ax = 0;
        }
        else {
            delta_ax = accelReq_mps2 - curAccel_mps2;
            if (accelReq_mps2 > 0.0f)
            {
                c_p = gain_p_f * delta_ax;
                c_i += gain_i_f * (0.001 / 2)* delta_ax;
            }
            else
            {
                // (accelReq_mps2 < 0.0f)
                c_p = gain_p_r * delta_ax;
                c_i += gain_i_r * (0.001 / 2)*delta_ax;
            }

            c = c_p + c_i;    /* PI-Controller */
            if (c > 1)
            { /* Limitation */
                c = 1;
            }
            else if (c < -1)
            {
                c = -1;
            }
            c_i = c - c_p;

            float ratio = 1.0f; // Convert from front to backwards gear ratio
            if (PowerTrain.ControlIF.GearBoxOut.GearNoTrg < 0) {
                ratio = static_cast<float32_t>(abs(GB_Ratios_front[0] / GB_Ratios_rear[0]));
            }
            /* Gas or Brake */
            if (c >= 0) {

                VehicleControl.Brake = 0;
                VehicleControl.Gas = c * ratio;
            }
            else {
                VehicleControl.Gas = 0;
                VehicleControl.Brake = -1.0f*c;
            }
            //Detect falling edge for slope drive control

            if (VehicleControl.Brake == 0 && ApCtrl_boolBrkAcv == true)
            {
                ApCtrl_boolHldBrk = true;
                ApCtrl_boolBrkAcv = false;
            }

            const float32_t ApCtrl_degGrdtSlopForALim_T[] = { 0.043040F, 0.203895F };
            const float32_t ApCtrl_aLimForCtrl_T[] = { -0.001F,-0.02F };
            // empirical Limit for controller output by ramped-down brake release calculated from slope scope (Data analytically defined)
            ApCtrl_aLimForCtrlFromSlope_C = LSM_GEOML::lookupTable(fabsf(static_cast<float32_t>(slope_rad)), ApCtrl_degGrdtSlopForALim_T, ApCtrl_aLimForCtrl_T, 2);

            const float32_t ApCtrl_degGrdtSlopForBrkDet_T[] = { 0.043040F, 0.085935F,0.203895F };
            const float32_t ApCtrl_stBrkLimForCtrl_T[] = { 0.1F,0.15F,0.35F };
            // brake value for standstill position until enough torque available for ascension (Calculation based on analytical data)
            ApCtrl_stBrkLimFromSlope_C = LSM_GEOML::lookupTable(static_cast<float32_t>(abs(slope_rad)), ApCtrl_degGrdtSlopForBrkDet_T, ApCtrl_stBrkLimForCtrl_T, 3);
            // Brake loosening depending on slope lookup table
            const float32_t ApCtrl_degGrdtBrkLoosen_T[] = { 0.05F,0.09F,0.15F,0.2F,0.3F };
            const float32_t ApCtrl_stBrkLoosen_T[] = { 42.0F,90.0F,100.0F,300.0F,400.0F };
            float32_t ApCtrl_facBrkLoosen_C = LSM_GEOML::lookupTable(static_cast<float32_t>(abs(slope_rad)), ApCtrl_degGrdtBrkLoosen_T, ApCtrl_stBrkLoosen_T, 5);

            // All special fixes for slopes are clammed and available just in case the slope is higher enough (the limit is empirically fixed)
            if (abs(slope_rad) * 180 / (float)M_PI > 3)
            {
                // If the vehicle has to run in direction of the slope, no drive torque is actuated. All the drive force is generated from car weight
                if (((c > ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad > 0) && (loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu == false)) || ((c >= ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad < 0) && (loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu == true))) {
                    VehicleControl.Gas = 0;
                    if (ApCtrl_stBrkLstVal - (ApCtrl_stBrkLstVal / (ApCtrl_facBrkLoosen_C)) > 0)
                    {
                        VehicleControl.Brake = ApCtrl_stBrkLstVal - ApCtrl_stBrkLstVal / (ApCtrl_facBrkLoosen_C);
                    }
                    else
                    {
                        VehicleControl.Brake = 0;
                    }
                    // If the current acceleration exceeds request, brake is pushed back to the latest known point and PI-controller takes over with changed integrated error
                    /*if (delta_ax < -0.01 && c<0 && ApCtrl_stBrkLstVal < -1.0f*c) {
                        VehicleControl.Brake = -1.0f*c;//ApCtrl_stBrkLstVal+ abs(delta_ax) *0.01;// Add dependency on the slope as a factor
                        if (accelReq_mps2 > 0)
                        {
                            c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_f;
                        }
                        else
                        {
                            c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_r;
                        }
                    }*/
                }
                if (((c < ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad > 0) && (loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu == false)) || ((c >= ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad < 0) && (loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu == true)))
                {
                    VehicleControl.Brake = -1.0f*c;//ApCtrl_stBrkLstVal+ abs(delta_ax) *0.01;// Add dependency on the slope as a factor
                    if (accelReq_mps2 > 0)
                    {
                        c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_f;
                    }
                    else
                    {
                        c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_r;
                    }

                }
                // If the vehicle is to be driven against the slope, brakes are held active until "enough" torque is generated through gas pedal actuation
                if (((slope_rad > 0) && (loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu == true) && (ApCtrl_boolHldBrk == true)) || ((slope_rad < 0) && (loDMCCtrlRequestPort_longAccelReq.drivingForwardReq_nu == false) && (ApCtrl_boolHldBrk == true)))
                {
                    // A factor of 1.5 is added to the formula after simulation observation
                    if (abs(Vehicle.FL.Trq_Drive) + abs(Vehicle.FR.Trq_Drive) >= (Brake.Trq_tot[0] + Brake.Trq_tot[1] + Brake.Trq_tot[2] + Brake.Trq_tot[3]) / (26.33*0.66))//*1.5)
                    {
                        ApCtrl_boolHldBrk = false;
                        VehicleControl.Brake = 0;
                        ApCtrl_boolNoRvsHld = false;
                        // Accumulated error's rectification
                        if (accelReq_mps2 > 0) {
                            c_i = (VehicleControl.Gas / ratio) - gain_p_f * delta_ax;
                        }
                        else
                        {
                            c_i = (VehicleControl.Gas / ratio) - gain_p_r * delta_ax;
                        }
                    }
                    else
                    {

                        VehicleControl.Brake = ApCtrl_stBrkLimFromSlope_C;
                        VehicleControl.Gas += 0.03;
                        ApCtrl_boolNoRvsHld = true;
                    }

                }

            }
            const float32_t ApCtrl_degGrdtLimBrkForDistStopCtrl_T[] = { 0.0F,0.167F,0.19F };
            const float32_t ApCtrl_stBrkForDistStopCtrl_T[] = { 0.15F,0.3F,0.4F };
            const float32_t ApCtrl_vCurForBrkCor_T[] = { 0.0F,0.4F,0.5F,1.0F };
            const float32_t ApCtrl_facForBrkCor_T[] = { 1.0F,1.7F,2.1F,2.5F };
            //// If target point near "enough (0.03)" brake is actuated to prevent overshooting
            //if (ApCtrl_dstLstToStop > 0.035 && loDMCCtrlRequestPort_longAccelReq.distanceToStopReq_m <= 0.035)
            //{
            //    // Minimal brake value for preventing overshoot will be empirically defined with lookup-tables
            //    ApCtrl_stMinBrkValAntiExc_C = LSM_GEOML::lookupTable(static_cast<float32_t>(abs(slope_rad)), ApCtrl_degGrdtLimBrkForDistStopCtrl_T, ApCtrl_stBrkForDistStopCtrl_T, 2);
            //    ApCtrl_facForBrkCor_C = LSM_GEOML::lookupTable(abs((float)Car.ConBdy1.v_1[0]), ApCtrl_vCurForBrkCor_T, ApCtrl_facForBrkCor_T, 4);
            //    ApCtrl_stMinBrkValAntiExc_C = ApCtrl_stMinBrkValAntiExc_C * ApCtrl_facForBrkCor_C / 2;
            //    if (VehicleControl.Brake > ApCtrl_stMinBrkValAntiExc_C)
            //    {
            //        ApCtrl_stMinBrkValAntiExc_C = static_cast<float32_t>(VehicleControl.Brake);
            //    }
            //}
            /*if (loDMCCtrlRequestPort_longAccelReq.distanceToStopReq_m <= 0.03)
            {
                VehicleControl.Gas = 0;
                VehicleControl.Brake = ApCtrl_stMinBrkValAntiExc_C;
                if (accelReq_mps2 > 0)
                {
                    c_i = -VehicleControl.Brake - delta_ax * gain_p_f;
                }
                else
                {
                    c_i = -VehicleControl.Brake - delta_ax * gain_p_r;
                }
            }*/
            //ApCtrl_dstLstToStop = loDMCCtrlRequestPort_longAccelReq.distanceToStopReq_m;

            // Control end
        }

        // Brake values are registered for internal falling edge detection

        if (VehicleControl.Brake != 0)
        {
            ApCtrl_stBrkLstVal = VehicleControl.Brake;
        }

        // positive brake values
        if (VehicleControl.Brake > 0 && ApCtrl_boolNoRvsHld == false)
        {
            ApCtrl_boolBrkAcv = true;
        }

    }
    else {
        resetLowSpeedLongDMC();
        test_bool = true;
    }
#endif //MOCO_DEACTIVATE_VECONA

#else //!MOCO_REPLACES_LODMC

    /*GearboxCtrlRequestPort*/
    //only perform gear control if ap is active or (lsca) loDMC request is ongoing. Otherwise, let CM do what it wants
    const bool apActive = slotCtrlPort.planningCtrlCommands.apState == ap_psm::APState::AP_AVG_ACTIVE_IN ||
        slotCtrlPort.planningCtrlCommands.apState == ap_psm::APState::AP_AVG_ACTIVE_OUT ||
        slotCtrlPort.planningCtrlCommands.apState == ap_psm::APState::AP_ACTIVE_HANDOVER_AVAILABLE;
    const bool gpActive = slotCtrlPort.planningCtrlCommands.gpState == ap_psm::GPState::GP_AVG_ACTIVE_IN ||
        slotCtrlPort.planningCtrlCommands.gpState == ap_psm::GPState::GP_AVG_ACTIVE_OUT;
    const bool rmActive = slotCtrlPort.planningCtrlCommands.rmState == ap_psm::RMState::RM_AVG_ACTIVE;

    if (//AP active ?
        (apActive || gpActive || rmActive) ||
        (ap_trjctl::LoDMCCtrlRequestType::LODMC_NORMAL_REQUEST == loDMCCtrlRequestPort.loDMCCtrlRequest_nu)) {
        if (carMakerInterface.variantSemiAPActive_nu == 0) {
            if ((gearBoxCtrlRequestPort.gearboxCtrlRequest_nu) && (gearBoxCtrlRequestPort.gearSwitchRequest_nu)) {
                switch (gearBoxCtrlRequestPort.gearReq_nu) {
                case ap_commonvehsigprovider::Gear::GEAR_R:
                    DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, -1, NULL);
                    lastGearReq_nu = -1;
                    break;
                case ap_commonvehsigprovider::Gear::GEAR_N:
                    DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, 0, NULL);
                    lastGearReq_nu = 0;
                default:
                    DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, 1, NULL);
                    lastGearReq_nu = 1;
                }
                if (!hadGearReq_nu) testEvaluation.setNumberOfGearStrokes(1U);  // count first stroke
                hadGearReq_nu = true;
            }
            else if (hadGearReq_nu) {
                DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, 0, NULL);
            }
        }
        else
        {
            if (!gearChangeRequested_nu) {

                /* Fill current gear request depending on the HMI message */
                switch (static_cast<ap_psm_app::HMIMessage>(carMakerInterface.headUnitMessage_nu)) {
                case ap_psm_app::HMIMessage::SHIFT_TO_R:
                    currGearReq_nu = -1;
                    break;
                case ap_psm_app::HMIMessage::SHIFT_TO_D:
                case ap_psm_app::HMIMessage::SHIFT_TO_1:
                    currGearReq_nu = 1;
                    break;
                case ap_psm_app::HMIMessage::STOP:
                    currGearReq_nu = lastGearReq_nu;
                    /* Request simulated driver to brake */
                    driverReqToAccelFlag_nu = 0;
                    driverReqToBrakeFlag_nu = 1;
                    break;
                default:
                    currGearReq_nu = lastGearReq_nu;
                    break;
                }

                /* Change gear in case an undershoot is desired in this stroke */
                if (doUndershoot_nu[currStroke_nu] && carStopped_nu) {
                    currGearReq_nu = (-1) * currGearReq_nu;
                    undershoot_delay_cntr = 0;
                }
            }

            /* Request gear */
            DVA_WriteRequest("DM.SelectorCtrl", OWMode_Abs, 10, 0, 0, currGearReq_nu, NULL);

            /* Check if a new gear was requested */
            if ((currGearReq_nu != lastGearReq_nu) || isFirstGearReq) {
                gearChangeRequested_nu = true;
                /* Check if the gear was really changed (there is some delay) */
                if (VehicleControl.SelectorCtrl != currGearReq_nu) {
                    /* Do nothing while gear is not yet changed as requested */
                    driverReqToAccelFlag_nu = 0;
                    driverReqToBrakeFlag_nu = 0;
                    //gearChangeDone = false;
                }
                else {
                    /* Gear changed, request simulated driver to accelerate */
                    driverReqToAccelFlag_nu = 1;
                    driverReqToBrakeFlag_nu = 0;
                    lastGearReq_nu = currGearReq_nu;
                    gearChangeRequested_nu = false;
                    /* New stroke */
                    currStroke_nu++;
                    carStopped_nu = false;
                    isFirstGearReq = false;
                }
            }
            else {
                gearChangeRequested_nu = false;
            }
        }
    }

    /*LoDMCCtrlRequestPort*/
    if ((ap_trjctl::LoDMCCtrlRequestType::LODMC_NORMAL_REQUEST == loDMCCtrlRequestPort.loDMCCtrlRequest_nu) ||
        driverReqToAccelFlag_nu ||
        driverReqToBrakeFlag_nu)
    {
        float brakeReq_mps2;
        curAccel_mps2 = Vehicle.PoI_Acc_1[0];
        bool AccelReqEnabled, BrakeReqEnabled, standStillHold, standStillSecure, maneuveringFinished;
        if (ap_trjctl::LoDMCCtrlRequestType::LODMC_NORMAL_REQUEST == loDMCCtrlRequestPort.loDMCCtrlRequest_nu) {
            lowSpeedLongDMC(&brakeReq_mps2,
                &BrakeReqEnabled,
                &accelReq_mps2,
                &AccelReqEnabled,
                &standStillHold,
                &standStillSecure,
                &maneuveringFinished,
                &velocityRequestIntern_mps,
                (ap_trjctl::LoDMCCtrlRequestType::LODMC_NORMAL_REQUEST ==  loDMCCtrlRequestPort.loDMCCtrlRequest_nu),
                loDMCCtrlRequestPort.velocityLimitReq_mps,
                loDMCCtrlRequestPort.distanceToStopReq_m,
                (ap_trjctl::LoDMCHoldRequestType::LODMC_HOLD_REQ_ON == loDMCCtrlRequestPort.holdReq_nu),
                loDMCCtrlRequestPort.emergencyHoldReq_nu,
                loDMCCtrlRequestPort.secureReq_nu,
                (float)Car.ConBdy1.v_1[0],
                (float)(SimCore.Time - lastCallTime_us));
        }
        else {
            if (driverReqToAccelFlag_nu) {
                lowSpeedLongDMC(&brakeReq_mps2,
                    &BrakeReqEnabled,
                    &accelReq_mps2,
                    &AccelReqEnabled,
                    &standStillHold,
                    &standStillSecure,
                    &maneuveringFinished,
                    &velocityRequestIntern_mps,
                    1,    //loDMCCtrlRequestPort.loDMCCtrlRequest_nu
                    semiAPvelocityLimitReq_mps, //loDMCCtrlRequestPort.velocityLimitReq_mps
                    semiAPdistToStopAtAccel_m,  //loDMCCtrlRequestPort.distanceToStopReq_m
                    0,    //(ap_trjctl::LoDMCHoldRequestType::LODMC_HOLD_REQ_ON == loDMCCtrlRequestPort.holdReq_nu)
                    0,    //loDMCCtrlRequestPort.emergencyHoldReq_nu
                    0,    //loDMCCtrlRequestPort.secureReq_nu
                    (float)Car.ConBdy1.v_1[0],
                    (float)(SimCore.Time - lastCallTime_us));
            }
            else if (driverReqToBrakeFlag_nu)
            {
                if (doUndershoot_nu[currStroke_nu]) {
                    distToStopReq_m = loDMCCtrlRequestPort.distanceToStopReq_m - undershootDist_m[currStroke_nu];
                    if (distToStopReq_m < 0) {
                        distToStopReq_m = 0;
                    }
                }
                else {
                    distToStopReq_m = loDMCCtrlRequestPort.distanceToStopReq_m;
                }
                if (!carStopped_nu) {
                    lowSpeedLongDMCSemiAPBrake(&brakeReq_mps2,
                        &BrakeReqEnabled,
                        &accelReq_mps2,
                        &AccelReqEnabled,
                        &standStillHold,
                        &standStillSecure,
                        &maneuveringFinished,
                        &velocityRequestIntern_mps,
                        1,    //loDMCCtrlRequestPort.loDMCCtrlRequest_nu
                        semiAPvelocityLimitReq_mps, //loDMCCtrlRequestPort.velocityLimitReq_mps
                        distToStopReq_m,
                        0,    //(ap_trjctl::LoDMCHoldRequestType::LODMC_HOLD_REQ_ON == loDMCCtrlRequestPort.holdReq_nu)
                        0,    //loDMCCtrlRequestPort.emergencyHoldReq_nu
                        0,    //loDMCCtrlRequestPort.secureReq_nu
                        (float)Car.ConBdy1.v_1[0]);
                }
                if (fabsf((float)Car.ConBdy1.v_1[0]) < 0.001)
                {
                    undershoot_delay_cntr++;
                    if (undershoot_delay_cntr > 100) {
                        undershoot_delay_cntr = 100;
                        carStopped_nu = true;
                    }
                }
            }
        }

        if (loDMCCtrlRequestPort.drivingForwardReq_nu == false) {/*change sign of current accel. if driving backwards is desired*/
            curAccel_mps2 *= -1.0;
        }

        slope_rad = gain_slope * Car.FARoadSensor.Route.LongSlope;

        /*begin Code taken from AccelCtrl CM Example*/
        /* Controller for converting desired ax to gas or brake */
        if (ap_trjctl::LoDMCHoldRequestType::LODMC_HOLD_REQ_ON == loDMCCtrlRequestPort.holdReq_nu) {
            VehicleControl.Gas = 0;
            // VehicleControl.Brake = 0.1 @ 3kph results in 3 m/ss deceleration
            //changed for driving on slopes until 16%
            if (abs(slope_rad) * 180 / (float)M_PI > 3)
            {
                VehicleControl.Brake = fmaxf(ApCtrl_stMinBrkValAntiExc_C, 0.3f); //max( 0.3, ApCtrl_stMinBrkValAntiExc_C);
            }
            else
            {
                VehicleControl.Brake = fmaxf(ApCtrl_stMinBrkValAntiExc_C, 0.1f);//max(0.10, ApCtrl_stMinBrkValAntiExc_C);
            }
            //Detect falling edge for slope drive control
            if (VehicleControl.Brake == 0 && ApCtrl_boolBrkAcv == true)
            {
                ApCtrl_boolHldBrk = true;
                ApCtrl_boolBrkAcv = false;
            }
            c_i = 0; c = 0; delta_ax = 0;

        }
        else if (loDMCCtrlRequestPort.emergencyHoldReq_nu || (carMakerInterface.variantSemiAPActive_nu && carStopped_nu)) {
            VehicleControl.Gas = 0;
            if (abs(slope_rad) * 180 / (float)M_PI > 3)
            {
                VehicleControl.Brake = 0.95;

            }
            else
            {
                // VehicleControl.Brake = 0.3 @ 3kph results in 6 m/ss deceleration
                VehicleControl.Brake = 0.86;
            }
            //Detect falling edge for slope drive control
            if (VehicleControl.Brake == 0 && ApCtrl_boolBrkAcv == true)
            {
                ApCtrl_boolHldBrk = true;
                ApCtrl_boolBrkAcv = false;
            }
            c_i = 0; c = 0; delta_ax = 0;
        }
        else {
            delta_ax = accelReq_mps2 - curAccel_mps2;
            if (accelReq_mps2 > 0.0f)
            {
                c_p = gain_p_f * delta_ax;
                c_i += gain_i_f * (0.001 / 2)* delta_ax;
            }
            else
            {
                // (accelReq_mps2 < 0.0f)
                c_p = gain_p_r * delta_ax;
                c_i += gain_i_r * (0.001 / 2)*delta_ax;
            }

            c = c_p + c_i;    /* PI-Controller */
            if (c > 1)
            {   /* Limitation */
                c = 1;
            }
            else if (c < -1)
            {
                c = -1;
            }
            c_i = c - c_p;

            float ratio = 1.0f; // Convert from front to backwards gear ratio
            if (PowerTrain.ControlIF.GearBoxOut.GearNoTrg < 0) {
                ratio = static_cast<float32_t>(abs(GB_Ratios_front[0] / GB_Ratios_rear[0]));
            }
            /* Gas or Brake */
            if (c >= 0) {

                VehicleControl.Brake = 0;
                VehicleControl.Gas = c * ratio;
            }
            else {
                VehicleControl.Gas = 0;
                VehicleControl.Brake = -1.0f*c;
            }
            //Detect falling edge for slope drive control

            if (VehicleControl.Brake == 0 && ApCtrl_boolBrkAcv == true)
            {
                ApCtrl_boolHldBrk = true;
                ApCtrl_boolBrkAcv = false;
            }

            const float32_t ApCtrl_degGrdtSlopForALim_T[] = { 0.043040F, 0.203895F };
            const float32_t ApCtrl_aLimForCtrl_T[] = { -0.001F ,-0.02F };
            // empirical Limit for controller output by ramped-down brake release calculated from slope scope (Data analytically defined)
            ApCtrl_aLimForCtrlFromSlope_C = LSM_GEOML::lookupTable(fabsf(static_cast<float32_t>(slope_rad)), ApCtrl_degGrdtSlopForALim_T, ApCtrl_aLimForCtrl_T, 2);

            const float32_t ApCtrl_degGrdtSlopForBrkDet_T[] = { 0.043040F, 0.085935F,0.203895F };
            const float32_t ApCtrl_stBrkLimForCtrl_T[] = { 0.1F ,0.15F,0.35F };
            // brake value for standstill position until enough torque available for ascension (Calculation based on analytical data)
            ApCtrl_stBrkLimFromSlope_C = LSM_GEOML::lookupTable(static_cast<float32_t>(abs(slope_rad)), ApCtrl_degGrdtSlopForBrkDet_T, ApCtrl_stBrkLimForCtrl_T, 3);
            // Brake loosening depending on slope lookup table
            const float32_t ApCtrl_degGrdtBrkLoosen_T[] = { 0.05F,0.09F,0.15F,0.2F,0.3F };
            const float32_t ApCtrl_stBrkLoosen_T[] = { 42.0F,90.0F,100.0F,300.0F,400.0F };
            float32_t ApCtrl_facBrkLoosen_C = LSM_GEOML::lookupTable(static_cast<float32_t>(abs(slope_rad)), ApCtrl_degGrdtBrkLoosen_T, ApCtrl_stBrkLoosen_T, 5);

            // All special fixes for slopes are clammed and available just in case the slope is higher enough (the limit is empirically fixed)
            if (abs(slope_rad) * 180 / (float)M_PI > 3)
            {
                // If the vehicle has to run in direction of the slope, no drive torque is actuated. All the drive force is generated from car weight
                if (((c > ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad > 0) && (loDMCCtrlRequestPort.drivingForwardReq_nu == b_FALSE)) || ((c >= ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad < 0) && (loDMCCtrlRequestPort.drivingForwardReq_nu == b_TRUE))) {
                    VehicleControl.Gas = 0;
                    if (ApCtrl_stBrkLstVal - (ApCtrl_stBrkLstVal / (ApCtrl_facBrkLoosen_C)) > 0)
                    {
                        VehicleControl.Brake = ApCtrl_stBrkLstVal - ApCtrl_stBrkLstVal / (ApCtrl_facBrkLoosen_C);
                    }
                    else
                    {
                        VehicleControl.Brake = 0;
                    }
                    // If the current acceleration exceeds request, brake is pushed back to the latest known point and PI-controller takes over with changed integrated error
                    /*if (delta_ax < -0.01 && c<0 && ApCtrl_stBrkLstVal < -1.0f*c) {
                        VehicleControl.Brake = -1.0f*c;//ApCtrl_stBrkLstVal+ abs(delta_ax) *0.01;// Add dependency on the slope as a factor
                        if (accelReq_mps2 > 0)
                        {
                            c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_f;
                        }
                        else
                        {
                            c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_r;
                        }
                    }*/
                }
                if (((c < ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad > 0) && (loDMCCtrlRequestPort.drivingForwardReq_nu == b_FALSE)) || ((c >= ApCtrl_aLimForCtrlFromSlope_C) && (slope_rad < 0) && (loDMCCtrlRequestPort.drivingForwardReq_nu == b_TRUE)))
                {
                    VehicleControl.Brake = -1.0f*c;//ApCtrl_stBrkLstVal+ abs(delta_ax) *0.01;// Add dependency on the slope as a factor
                    if (accelReq_mps2 > 0)
                    {
                        c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_f;
                    }
                    else
                    {
                        c_i = -ApCtrl_stBrkLstVal - delta_ax * gain_p_r;
                    }

                }
                // If the vehicle is to be driven against the slope, brakes are held active until "enough" torque is generated through gas pedal actuation
                if (((slope_rad > 0) && (loDMCCtrlRequestPort.drivingForwardReq_nu == b_TRUE) && (ApCtrl_boolHldBrk == true)) || ((slope_rad < 0) && (loDMCCtrlRequestPort.drivingForwardReq_nu == b_FALSE) && (ApCtrl_boolHldBrk == true)))
                {
                    // A factor of 1.5 is added to the formula after simulation observation
                    if (abs(Vehicle.FL.Trq_Drive) + abs(Vehicle.FR.Trq_Drive) >= (Brake.Trq_tot[0] + Brake.Trq_tot[1] + Brake.Trq_tot[2] + Brake.Trq_tot[3]) / (26.33*0.66))//*1.5)
                    {
                        ApCtrl_boolHldBrk = false;
                        VehicleControl.Brake = 0;
                        ApCtrl_boolNoRvsHld = false;
                        // Accumulated error's rectification
                        if (accelReq_mps2 > 0) {
                            c_i = (VehicleControl.Gas / ratio) - gain_p_f * delta_ax;
                        }
                        else
                        {
                            c_i = (VehicleControl.Gas / ratio) - gain_p_r * delta_ax;
                        }
                    }
                    else
                    {

                        VehicleControl.Brake = ApCtrl_stBrkLimFromSlope_C;
                        VehicleControl.Gas += 0.03;
                        ApCtrl_boolNoRvsHld = true;
                    }

                }

            }
            const float32_t ApCtrl_degGrdtLimBrkForDistStopCtrl_T[] = { 0.0F,0.167F ,0.19F };
            const float32_t ApCtrl_stBrkForDistStopCtrl_T[] = { 0.15F,0.3F,0.4F };
            const float32_t ApCtrl_vCurForBrkCor_T[] = { 0.0F,0.4F,0.5F,1.0F };
            const float32_t ApCtrl_facForBrkCor_T[] = { 1.0F,1.7F,2.1F,2.5F };
            // If target point near "enough (0.03)" brake is actuated to prevent overshooting
            if (ApCtrl_dstLstToStop > 0.035 && loDMCCtrlRequestPort.distanceToStopReq_m <= 0.035)
            {
                // Minimal brake value for preventing overshoot will be empirically defined with lookup-tables
                ApCtrl_stMinBrkValAntiExc_C = LSM_GEOML::lookupTable(static_cast<float32_t>(abs(slope_rad)), ApCtrl_degGrdtLimBrkForDistStopCtrl_T, ApCtrl_stBrkForDistStopCtrl_T, 2);
                ApCtrl_facForBrkCor_C = LSM_GEOML::lookupTable(abs((float)Car.ConBdy1.v_1[0]), ApCtrl_vCurForBrkCor_T, ApCtrl_facForBrkCor_T, 4);
                ApCtrl_stMinBrkValAntiExc_C = ApCtrl_stMinBrkValAntiExc_C * ApCtrl_facForBrkCor_C / 2;
                if (VehicleControl.Brake > ApCtrl_stMinBrkValAntiExc_C)
                {
                    ApCtrl_stMinBrkValAntiExc_C = static_cast<float32_t>(VehicleControl.Brake);
                }
            }
            if (loDMCCtrlRequestPort.distanceToStopReq_m <= 0.03)
            {
                VehicleControl.Gas = 0;
                VehicleControl.Brake = ApCtrl_stMinBrkValAntiExc_C;

                if (accelReq_mps2 > 0)
                {
                    c_i = -VehicleControl.Brake - delta_ax * gain_p_f;
                }
                else
                {

                    c_i = -VehicleControl.Brake - delta_ax * gain_p_r;
                }
            }
            ApCtrl_dstLstToStop = loDMCCtrlRequestPort.distanceToStopReq_m;

            // Control end
        }

        // Brake values are registered for internal falling edge detection

        if (VehicleControl.Brake != 0)
        {
            ApCtrl_stBrkLstVal = VehicleControl.Brake;
        }

        // positive brake values
        if (VehicleControl.Brake > 0 && ApCtrl_boolNoRvsHld == false)
        {
            ApCtrl_boolBrkAcv = true;
        }

    }
    else {
        resetLowSpeedLongDMC();
        test_bool = true;
    }
#endif //!MOCO_REPLACES_LODMC

    lastCallTime_us = SimCore.Time;

    /*PSMToSSPPort*/
    if (psmToSSPPort.bcmReq.dirIndLeftReq_nu) {
        DrivMan.Lights.Indicator = 1;
    }
    else if (psmToSSPPort.bcmReq.dirIndRightReq_nu) {
        DrivMan.Lights.Indicator = -1;
    }
    else {
        DrivMan.Lights.Indicator = 0;
    }
}

#ifndef VARIANT_CUS_ONLY
static void mdl_APCtrl_DeclQuants_CollEnvModelPort(void *MP)
{
    //collEnvModelPort
    registerSignalHeaderToDVA("AP.collEnvModelPort.sSigHeader", collEnvModelPort.sSigHeader);
    tDDefault *collEnvPrefix = DDefaultCreate("AP.collEnvModelPort.");
    DDefUChar(collEnvPrefix, "numberOfStaticObjects_u8", "", &collEnvModelPort.numberOfStaticObjects_u8, DVA_IO_In);
    DDefUChar(collEnvPrefix, "numberOfDynamicObjects_u8", "", &collEnvModelPort.numberOfDynamicObjects_u8, DVA_IO_In);
    DDefUChar(collEnvPrefix, "firstStatObjOutDetZoneIdx_u8", "", &collEnvModelPort.firstStatObjOutDetZoneIdx_u8, DVA_IO_In);
    DDefUChar(collEnvPrefix, "firstDynObjOutDetZoneIdx_u8", "", &collEnvModelPort.firstDynObjOutDetZoneIdx_u8, DVA_IO_In);
    //collEnvModelPort.dynamicObjects
    for (uint8_t i{ 0U }; i < ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_DYN_OBJECTS_NU; ++i) {
        DDefPrefix(collEnvPrefix, "AP.collEnvModelPort.dynamicObjects_%d.", i);
        DDefUChar(collEnvPrefix, "existenceProb_perc", "", &collEnvModelPort.dynamicObjects[i].existenceProb_perc, DVA_None);
        DDefFloat(collEnvPrefix, "vel_mps_0", "m/s", &collEnvModelPort.dynamicObjects[i].vel_mps.x_dir, DVA_None);
        DDefFloat(collEnvPrefix, "vel_mps_1", "m/s", &collEnvModelPort.dynamicObjects[i].vel_mps.y_dir, DVA_None);
        DDefFloat(collEnvPrefix, "accel_mps2_0", "m/s^2", &collEnvModelPort.dynamicObjects[i].accel_mps2.x_dir, DVA_None);
        DDefFloat(collEnvPrefix, "accel_mps2_1", "m/s^2", &collEnvModelPort.dynamicObjects[i].accel_mps2.y_dir, DVA_None);
        DDefFloat(collEnvPrefix, "headingAngle_rad", "rad", &collEnvModelPort.dynamicObjects[i].headingAngle_rad, DVA_None);
        DDefUChar(collEnvPrefix, "measurementState_nu", "", (uint8_t*)&collEnvModelPort.dynamicObjects[i].measurementState_nu, DVA_None);
        DDefUInt(collEnvPrefix, "refObjID_nu", "", &collEnvModelPort.dynamicObjects[i].refObjID_nu, DVA_None);
        for (unsigned int j{ 0U }; j < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU; ++j) {
            DDefPrefix(collEnvPrefix, "AP.collEnvModelPort.dynamicObjects_%d.objShape_m_%d.", i, j);
            DDefFloat(collEnvPrefix, "x", "m", &collEnvModelPort.dynamicObjects[i].objShape_m.array[j].x_dir, DVA_None);
            DDefFloat(collEnvPrefix, "y", "m", &collEnvModelPort.dynamicObjects[i].objShape_m.array[j].y_dir, DVA_None);
        }
    }
    //collEnvModelPort.staticObjects
    for (uint8_t i{ 0U }; i < ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_STATIC_OBJ_NU; i++) {
        DDefPrefix(collEnvPrefix, "AP.collEnvModelPort.staticObjects_%d.", i);
        DDefUInt(collEnvPrefix, "refObjID_nu", "", &collEnvModelPort.staticObjects[i].refObjID_nu, DVA_None);
        DDefUChar(collEnvPrefix, "refObjClass_nu", "", (uint8_t*)&collEnvModelPort.staticObjects[i].refObjClass_nu, DVA_None);
        DDefUChar(collEnvPrefix, "existenceProb_perc", "", &collEnvModelPort.staticObjects[i].existenceProb_perc, DVA_None);
        DDefUShort(collEnvPrefix, "objAgeInCycles_nu", "", &collEnvModelPort.staticObjects[i].objAgeInCycles_nu, DVA_None);
        DDefUShort(collEnvPrefix, "objMeasLastUpdateInCycles_nu", "", &collEnvModelPort.staticObjects[i].objMeasLastUpdateInCycles_nu, DVA_None);
        DDefUShort(collEnvPrefix, "objTrendLastUpdateInCycles_nu", "", &collEnvModelPort.staticObjects[i].objTrendLastUpdateInCycles_nu, DVA_None);
        DDefUChar(collEnvPrefix, "objTrend_nu", "", (uint8_t*)&collEnvModelPort.staticObjects[i].objTrend_nu, DVA_None);
        DDefUChar(collEnvPrefix, "readFromNVRAM_nu", "", (uint8_t*)&collEnvModelPort.staticObjects[i].readFromNVRAM_nu, DVA_None);
        DDefUChar(collEnvPrefix, "objHeightClass_nu", "", (uint8_t*)&collEnvModelPort.staticObjects[i].objHeightClass_nu, DVA_None);
        DDefUChar(collEnvPrefix, "objHeightClassConfidence_perc", "", &collEnvModelPort.staticObjects[i].objHeightClassConfidence_perc, DVA_None);
        for (unsigned int j{ 0U }; j < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU; ++j) {
            DDefPrefix(collEnvPrefix, "AP.collEnvModelPort.staticObjects_%d.objShape_m_%d.", i, j);
            DDefFloat(collEnvPrefix, "x", "m", &collEnvModelPort.staticObjects[i].objShape_m.array[j].x_dir, DVA_None);
            DDefFloat(collEnvPrefix, "y", "m", &collEnvModelPort.staticObjects[i].objShape_m.array[j].y_dir, DVA_None);
        }
    }
    DDefaultDelete(collEnvPrefix);
}
#endif

static void
mdl_APCtrl_DeclQuants(void *MP)
{
    DDefUChar(NULL, "AP.variant_identifier_nu", "", &variant_identifier_nu, DVA_None);
    DDefFloat(NULL, "AP.Steer.steeringWheelAngle_rad", "rad", &steerCtrlStatusPort.steeringWheelAngle_rad, DVA_None);
    DDefFloat(NULL, "AP.Steer.steeringWheelAngleVelocity_radps", "rad/s", &steerCtrlStatusPort.steeringWheelAngleVelocity_radps, DVA_None);
    DDefUChar(NULL, "AP.hmiOutputPort.userActionHeadUnit_nu", "", &userActionHeadUnitCMQuant_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.hmiOutputPort.userActionHeadUnit_nu_global", "", (uint8_t*)&(gHMIOutputPort.userActionHeadUnit_nu), DVA_IO_In);
    DDefUChar(NULL, "AP.hmiOutputPort.userActionHUCounter_nu", "", &gHMIOutputPort.userActionHUCounter_nu, DVA_IO_In);
    registerSignalHeaderToDVA("AP.hmiOutputPort.sSigHeader", gHMIOutputPort.sSigHeader);
    DDefUChar(NULL, "AP.useDeadManSwitchRemoteApp", "", (uint8_t*)&useDeadManSwitchRemoteApp, DVA_IO_In);
    DDefUChar(NULL, "AP.fakeKeyFonAliveCounterFlag", "", (uint8_t*)&fakeKeyFonAliveCounterFlag, DVA_IO_In);
    DDefUChar(NULL, "AP.remoteHMIOutputPort.paired_nu", "", (uint8_t*)&remDevPaired_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.remoteHMIOutputPort.connected_nu", "", (uint8_t*)&remDevConnected_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.keylessStatusPort.keyFobInRange", "", (uint8_t*)&keylessStatusPort.keyFobInRange, DVA_IO_In);
    DDefUChar(NULL, "AP.keylessStatusPort.keyFobUserAction", "", (uint8_t*)&keylessStatusPort.keyFobUserAction, DVA_IO_In);
    registerSignalHeaderToDVA("AP.keylessStatusPort.sSigHeader", keylessStatusPort.sSigHeader);
    DDefUChar(NULL, "AP.gearboxCtrlStatusPort.gearLeverInformation.gearLeverPositionCur_nu", "", (uint8_t*)&gearboxCtrlStatusPort.gearLeverInformation.gearLeverPositionCur_nu, DVA_IO_In);
    registerSignalHeaderToDVA("AP.gearboxCtrlStatusPort.sSigHeader", gearboxCtrlStatusPort.sSigHeader);
    DDefUChar(NULL, "AP.pdwDisabled_nu", "", &carMakerInterface.pdwDisabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.lvmdDisabled_nu", "", &carMakerInterface.lvmdDisabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.lvmdWarningInput_nu", "", (uint8_t*)&carMakerInterface.lvmdWarningInput_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.whpDisabled_nu", "", &carMakerInterface.whpDisabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.pdwFailure_nu", "", &carMakerInterface.pdwFailure_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.vedodoActive_nu", "", &carMakerInterface.vedodoActive_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.lscaDisabled_nu", "", &carMakerInterface.lscaDisabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.lscaBrakeDisabled_nu", "", &carMakerInterface.lscaBrakeDisabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.lscaSteeringProposalDisabled_nu", "", &carMakerInterface.lscaSteeringProposalDisabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.lscaVirtualWallDisabled_nu", "", &carMakerInterface.lscaVirtualWallDisabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.lscaFakeStandstillRequest_nu", "", &carMakerInterface.lscaFakeStandstillRequest_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.headUnitVisualizationPort.screen_nu", "", &carMakerInterface.headUnitScreen_nu, DVA_None);
    DDefUChar(NULL, "AP.headUnitVisualizationPort.message_nu", "", &carMakerInterface.headUnitMessage_nu, DVA_None);
    DDefUChar(NULL, "AP.remoteVisualizationPort.screen_nu", "", &carMakerInterface.remoteScreen_nu, DVA_None);
    DDefUChar(NULL, "AP.remoteVisualizationPort.message_nu", "", &carMakerInterface.remoteMessage_nu, DVA_None);
    DDefUChar(NULL, "AP.sceneInterpretationActive_nu", "", &carMakerInterface.sceneInterpretationActive_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.environmentModelActive_nu", "", &carMakerInterface.environmentModelActive_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.odoEstimationPort.overwriteVehVelocityCur_nu", "", (uint8_t*)&carMakerInterface.odoOverwriteVehVelocity_nu, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.overwriteVehVelocityValue_mps", "", &carMakerInterface.odoOverwriteVehVelocityValue_mps, DVA_IO_In);
    DDefUChar(NULL, "AP.odoRecalibratePosition_nu", "", &carMakerInterface.odoRecalibratePosition_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.resetCemSurrogate_nu", "", &carMakerInterface.resetCemSurrogate_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.resetParkingComponents_nu", "", &carMakerInterface.resetParkingComponents_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.enableLimitFieldOfView_nu", "", (uint8_t*)&enableLimitFieldOfView_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.enableLatencyEffect_nu", "", (uint8_t*)&enableLatencyEffect_nu, DVA_IO_In);
    DDefFloat(NULL, "AP.latencyEffectTime_s", "s", &latencyEffectTime_s, DVA_IO_In);
    DDefFloat(NULL, "AP.inflationLength_m", "m", &inflationLength_m, DVA_IO_In);
    DDefUChar(NULL, "AP.doorStatusPort.status.driver_nu", "", (uint8_t*)&doorStatusPort.status.driver_nu, DVA_IO_In);
    registerSignalHeaderToDVA("AP.doorStatusPort.sSigHeader", doorStatusPort.sSigHeader);
    DDefUChar(NULL, "AP.vehicleOccupancyStatusPort.beltBuckleStatus.driver_nu", "", (uint8_t*)&vehicleOccupancyStatusPort.beltBuckleStatus.driver_nu, DVA_IO_In);
    registerSignalHeaderToDVA("AP.vehicleOccupancyStatusPort.sSigHeader", vehicleOccupancyStatusPort.sSigHeader);
    DDefFloat(NULL, "AP.cemSurrogateConfig.cornerDetectionError_m", "m", &cemSurrogateConfig.cornerDetectionError_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.flankKinkWidth_m", "m", &cemSurrogateConfig.flankKinkWidth_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.flankKinkCorrectionDistance_m", "m", &cemSurrogateConfig.flankKinkCorrectionDistance_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.flankKinkCorrectionTimeSpan_s", "s", &cemSurrogateConfig.flankKinkCorrectionTimeSpan_s, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.garageDetectionDistance_m", "m", &cemSurrogateConfig.garageDetectionDistance_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.smallObjectDetectionThreshold_m", "m", &cemSurrogateConfig.smallObjectDetectionThreshold_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.smallObjectDetectionThresholdNotMainAxis_m", "m", &cemSurrogateConfig.smallObjectDetectionThresholdNotMainAxis_m, DVA_IO_In);
    DDefUChar(NULL, "AP.cemSurrogateConfig.smallObjectDetectionDelay_nu", "", &cemSurrogateConfig.smallObjectDetectionDelay_nu, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.misclassificationDistanceThreshold_m", "m", &cemSurrogateConfig.misclassificationDistanceThreshold_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.cameraUncertainty_m", "", &cemSurrogateConfig.cameraUncertainty_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.fragmentObjectLength", "", &cemSurrogateConfig.fragmentObjectLength, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.fragmentGapLength", "", &cemSurrogateConfig.fragmentGapLength, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.fragmentMergeTime_s", "s", &cemSurrogateConfig.fragmentMergeTime_s, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.lineOrientationAngleError_deg", "deg", &cemSurrogateConfig.lineOrientationAngleError_deg, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.lineOrientationCorrectionTimeSpan_s", "s", &cemSurrogateConfig.lineOrientationCorrectionTimeSpan_s, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.lineOrientationCorrectionDistance_m", "m", &cemSurrogateConfig.lineOrientationCorrectionDistance_m, DVA_IO_In);
    DDefUChar(NULL, "AP.cemSurrogateConfig.pmdSurrogateEnabled_nu", "", &cemSurrogateConfig.pmdSurrogateEnabled_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.cemSurrogateConfig.disableCameras_nu", "", &cemSurrogateConfig.disableCameras_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.cemSurrogateConfig.enableDynamicObjectMovement_nu", "", &cemSurrogateConfig.enableDynamicObjectMovement_nu, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.disappearedObjectRemovalTime_s", "s", &cemSurrogateConfig.disappearedObjectRemovalTime_s, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.objectThicknessDuringScanning_m", "m", &cemSurrogateConfig.objectThicknessDuringScanning_m, DVA_IO_In);
    DDefFloat(NULL, "AP.cemSurrogateConfig.odSlotOverlapThreshold", "", &cemSurrogateConfig.odSlotOverlapThreshold, DVA_IO_In);
    cemSurrogate.registerCarMakerDVAs();
    //remoteHMIOutputPort
    DDefUChar(NULL, "AP.remoteHMIOutputPort.paired_nu", "", (uint8_t*)&remDevPaired_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.remoteHMIOutputPort.connected_nu", "", (uint8_t*)&remDevConnected_nu, DVA_IO_In);
    DDefUChar(NULL, "AP.remoteHMIOutputPort.userActionRemoteDeviceCMQuant_nu", "", &userActionRemoteDeviceCMQuant_nu, DVA_IO_In);
    DDefULong(NULL, "AP.remoteHMIOutputPort.fingerPositionX_px", "", &fingerPositionXCMQuant_px, DVA_IO_In);
    DDefULong(NULL, "AP.remoteHMIOutputPort.fingerPositionY_px", "", &fingerPositionYCMQuant_px, DVA_IO_In);
    DDefUChar(NULL, "AP.remoteHMIOutputPort.aliveCounter_nu", "", &remoteHMIOutputPort.aliveCounter_nu, DVA_None);
    registerSignalHeaderToDVA("AP.remoteHMIOutputPort.sSigHeader", remoteHMIOutputPort.sSigHeader);

    // error test inputs
    DDefFloat(NULL, "AP.escInformationPort->brakePressureDriver_bar", "", &escInformationPort.brakePressureDriver_bar, DVA_IO_In);
    registerSignalHeaderToDVA("AP.escInformationPort.sSigHeader", escInformationPort.sSigHeader);

    //GearboxCtrlRequestPort
    DDefUChar(NULL, "AP.gearBoxCtrlRequestPort.gearboxCtrlRequest_nu", "", (uint8_t*)&gearBoxCtrlRequestPort.gearboxCtrlRequest_nu, DVA_None);
    registerSignalHeaderToDVA("AP.gearBoxCtrlRequestPort.sSigHeader", gearBoxCtrlRequestPort.sSigHeader);

    //LaDMCStatusPort
    DDefUChar(NULL, "AP.LaDMCStatusPort.driverIntervention_nu", "", (uint8_t*)&laDMCStatusPort.driverIntervention_nu, DVA_IO_In);
    registerSignalHeaderToDVA("AP.LaDMCStatusPort.sSigHeader", laDMCStatusPort.sSigHeader);

    //Steer control
    DDefFloat(NULL, "AP.steerCtrlRequestPort.frontSteerAngReq_rad", "rad", &laDMCCtrlRequestPort.frontSteerAngReq_rad, DVA_None);
    DDefFloat(NULL, "AP.steerCtrlRequestPort.steerAngReq_rad", "rad", &laDMCCtrlRequestPort.steerWheelAngReq_rad, DVA_None);
    DDefFloat(NULL, "AP.steerCtrlRequestPort.steerAngReqLaDmcIN_rad", "rad", &steerAngReqIn, DVA_None);
    DDefFloat(NULL, "AP.steerCtrlRequestPort.steerAngReqLaDmcOUT_rad", "rad", &steerAngReqOut, DVA_None);
    DDefFloat(NULL, "AP.steerCtrlRequestPort.rearSteerAngReq_rad", "rad", &laDMCCtrlRequestPort.rearSteerAngReq_rad, DVA_None);
    DDefFloat(NULL, "AP.steerCtrlRequestPort.curvatureReq_1pm", "1/m", &laDMCCtrlRequestPort.curvatureReq_1pm, DVA_None);
    DDefUChar(NULL, "AP.steerCtrlRequestPort.laDMCCtrlRequestInterface_nu", "", (uint8_t*)&laDMCCtrlRequestPort.laDMCCtrlRequestInterface_nu, DVA_None);
    DDefUChar(NULL, "AP.steerCtrlRequestPort.laDMCCtrlRequestSource_nu", "", (uint8_t*)&laDMCCtrlRequestPort.laDMCCtrlRequestSource_nu, DVA_None);
    DDefUChar(NULL, "AP.steerCtrlRequestPort.laDMCCtrlRequest_nu", "", (uint8_t*)&laDMCCtrlRequestPort.laDMCCtrlRequest_nu, DVA_None);
    DDefFloat(NULL, "AP.steerCtrlRequestPort.frontSteerTorqueReq_Nm", "Nm", &laDMCCtrlRequestPort.frontSteerTorqueReq_Nm, DVA_None);
    DDefFloat(NULL, "AP.steerCtrlRequestPort.rearSteerTorqueReq_Nm", "Nm", &laDMCCtrlRequestPort.rearSteerTorqueReq_Nm, DVA_None);
    registerSignalHeaderToDVA("AP.steerCtrlRequestPort.sSigHeader", laDMCCtrlRequestPort.sSigHeader);

    //TRJCTLGeneralInputPort
    DDefFloat(NULL, "AP.trjctlGeneralInputPort.trjctlSampleTime_s", "s", &trjctlGeneralInputPort.trjctlSampleTime_s, DVA_None);
    registerSignalHeaderToDVA("AP.trjctlGeneralInputPort.sSigHeader", trjctlGeneralInputPort.sSigHeader);

    //mfControlConfig
    registerSignalHeaderToDVA("AP.mfControlConfig.mfcParams.sSigHeader", carMakerInterface.mfControlConfigMfcParamsSigHeader);
    registerSignalHeaderToDVA("AP.mfControlConfig.sysFuncParams.sSigHeader", carMakerInterface.mfControlConfigSysFuncParamsSigHeader);
    registerSignalHeaderToDVA("AP.mfControlConfig.vehicleParams.sSigHeader", carMakerInterface.mfControlConfigVehicleParamsSigHeader);

    //LaDMCCtrlRequestPort
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.frontSteerAngReq_rad", "rad", &laDMCCtrlRequestPort.frontSteerAngReq_rad, DVA_IO_In);
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.steerWheelAngReq_rad", "rad", &laDMCCtrlRequestPort.steerWheelAngReq_rad, DVA_IO_In);
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.steerAngReqLaDmcIN_rad", "rad", &steerAngReqIn, DVA_None);
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.steerAngReqLaDmcOUT_rad", "rad", &steerAngReqOut, DVA_None);
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.rearSteerAngReq_rad", "rad", &laDMCCtrlRequestPort.rearSteerAngReq_rad, DVA_None);
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.curvatureReq_1pm", "1/m", &laDMCCtrlRequestPort.curvatureReq_1pm, DVA_None);
    DDefUChar(NULL, "AP.laDMCCtrlRequestPort.laDMCCtrlRequestInterface_nu", "", (uint8_t*)&laDMCCtrlRequestPort.laDMCCtrlRequestInterface_nu, DVA_None);
    DDefUChar(NULL, "AP.laDMCCtrlRequestPort.laDMCCtrlRequestSource_nu", "", (uint8_t*)&laDMCCtrlRequestPort.laDMCCtrlRequestSource_nu, DVA_None);
    DDefUChar(NULL, "AP.laDMCCtrlRequestPort.laDMCCtrlRequest_nu", "", (uint8_t*)&laDMCCtrlRequestPort.laDMCCtrlRequest_nu, DVA_None);
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.frontSteerTorqueReq_Nm", "Nm", &laDMCCtrlRequestPort.frontSteerTorqueReq_Nm, DVA_None);
    DDefFloat(NULL, "AP.laDMCCtrlRequestPort.rearSteerTorqueReq_Nm", "Nm", &laDMCCtrlRequestPort.rearSteerTorqueReq_Nm, DVA_None);
    registerSignalHeaderToDVA("AP.laDMCCtrlRequestPort.sSigHeader", laDMCCtrlRequestPort.sSigHeader);

    //LoDMCCtrlRequestPort
    DDefFloat(NULL, "AP.LoDMC.velocityRequestLoDMCIntern_mps", "m/s", &velocityRequestIntern_mps, DVA_None);
    DDefFloat(NULL, "AP.LoDMCCtrlRequestPort.velocityLimitReq_mps", "m/s", &loDMCCtrlRequestPort.velocityLimitReq_mps, DVA_None);
    DDefFloat(NULL, "AP.LoDMCCtrlRequestPort.distanceToStopReq_m", "m", &loDMCCtrlRequestPort.distanceToStopReq_m, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.holdReq_nu", "", (uint8_t*)&loDMCCtrlRequestPort.holdReq_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.secureReq_nu", "", (uint8_t*)&loDMCCtrlRequestPort.secureReq_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.loDMCCtrlRequest_nu", "", (uint8_t*)&loDMCCtrlRequestPort.loDMCCtrlRequest_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.emergencyHoldReq_nu", "", (uint8_t*)&loDMCCtrlRequestPort.emergencyHoldReq_nu, DVA_None);
    DDefFloat(NULL, "AP.LoDMCCtrlRequestPort.accelerationReq_mps2", "m/s^2", &loDMCCtrlRequestPort.accelerationReq_mps2, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.drivingForwardReq_nu", "", (uint8_t*)&loDMCCtrlRequestPort.drivingForwardReq_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.loDMCCtrlRequestInterface_nu", "", (uint8_t*)&loDMCCtrlRequestPort.loDMCCtrlRequestInterface_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.loDMCCtrlRequestSource_nu", "", (uint8_t*)&loDMCCtrlRequestPort.loDMCCtrlRequestSource_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCCtrlRequestPort.trajectoryReset_nu", "", (uint8_t*)&loDMCCtrlRequestPort.trajectoryReset_nu, DVA_None);
    registerSignalHeaderToDVA("AP.LoDMCCtrlRequestPort.sSigHeader", loDMCCtrlRequestPort.sSigHeader);

#ifdef MOCO_REPLACES_LODMC
    //mocoCarmakerInput
    DDefDouble(NULL, "MOCO.mocoCarmakerInput.engineMinTrq_Nm", "Nm", &mocoCarmakerInput.engineMinTrq_Nm, DVA_None);
    DDefDouble(NULL, "MOCO.mocoCarmakerInput.engineMaxTrq_Nm", "Nm", &mocoCarmakerInput.engineMaxTrq_Nm, DVA_None);
    DDefDouble(NULL, "MOCO.mocoCarmakerInput.engineTrq_Nm", "Nm", &mocoCarmakerInput.engineTrq_Nm, DVA_None);
    DDefDouble(NULL, "MOCO.mocoCarmakerInput.engineTrqDriverReq_Nm", "Nm", &mocoCarmakerInput.engineTrqDriverReq_Nm, DVA_None);
    DDefUChar(NULL, "MOCO.mocoCarmakerInput.overrideAccel", "", (uint8_t*)&mocoCarmakerInput.overrideAccel, DVA_None);
    DDefUChar(NULL, "MOCO.mocoCarmakerInput.overrideDecel", "", (uint8_t*)&mocoCarmakerInput.overrideDecel, DVA_None);
    DDefULLong(NULL, "MOCO.mocoCarmakerInput.timestamp_ms", "ms", &mocoCarmakerInput.timestamp_ms, DVA_None);
    DDefFloat(NULL, "MOCO.mocoCarmakerInput.steerAngleFront_rad", "rad", &mocoCarmakerInput.steerAngleFront_rad, DVA_None);
    DDefUChar(NULL, "MOCO.mocoCarmakerInput.vehicleWasSecuredMoCo_nu", "", (uint8_t*)&mocoCarmakerInput.vehicleWasSecuredMoCo_nu, DVA_None);

    //longManeuverReq
    DDefUChar(NULL, "AP.LongManeuverRequestPort.activateCtrl", "", (uint8_t*)&longManeuverRequestPort.activateCtrl, DVA_None);
    DDefFloat(NULL, "AP.LongManeuverRequestPort.ctrlReq.accelReq_mps2", "m/s^2", &longManeuverRequestPort.ctrlReq.accelReq_mps2, DVA_None);
    DDefFloat(NULL, "AP.LongManeuverRequestPort.ctrlReq.distanceToStopReq_m", "m", &longManeuverRequestPort.ctrlReq.distanceToStopReq_m, DVA_None);
    DDefFloat(NULL, "AP.LongManeuverRequestPort.ctrlReq.veloLimMax_mps", "m/s", &longManeuverRequestPort.ctrlReq.veloLimMax_mps, DVA_None);
    DDefUChar(NULL, "AP.LongManeuverRequestPort.ctrlReq.longCtrlMode_nu", "", (uint8_t*)&longManeuverRequestPort.ctrlReq.longCtrlMode_nu, DVA_None);
    DDefUChar(NULL, "AP.LongManeuverRequestPort.drivingDirReq", "", (uint8_t*)&longManeuverRequestPort.drivingDirReq, DVA_None);
    DDefUChar(NULL, "AP.LongManeuverRequestPort.dynMode", "", (uint8_t*)&longManeuverRequestPort.dynMode, DVA_None);
    DDefUChar(NULL, "AP.LongManeuverRequestPort.funMode", "", (uint8_t*)&longManeuverRequestPort.funMode, DVA_None);
    DDefUChar(NULL, "AP.LongManeuverRequestPort.motReq", "", (uint8_t*)&longManeuverRequestPort.motReq, DVA_None);
    DDefUChar(NULL, "AP.LongManeuverRequestPort.fullBrakingReq", "", (uint8_t*)&longManeuverRequestPort.fullBrakingReq, DVA_None);
    DDefFloat(NULL, "AP.LongManeuverRequestPort.lims.longAccelMax", "m/s^2", &longManeuverRequestPort.lims.longAccelMax, DVA_None);
    DDefFloat(NULL, "AP.LongManeuverRequestPort.lims.longAccelMin", "m/s^2", &longManeuverRequestPort.lims.longAccelMin, DVA_None);
    DDefFloat(NULL, "AP.LongManeuverRequestPort.lims.longJerkMax", "m/s^3", &longManeuverRequestPort.lims.longJerkMax, DVA_None);
    DDefFloat(NULL, "AP.LongManeuverRequestPort.lims.longJerkMin", "m/s^3", &longManeuverRequestPort.lims.longJerkMin, DVA_None);
#endif

    //DrivingResistancePort
    DDefFloat(NULL, "AP.DrivingResistancePort.distanceFL", "m", &drivingResistancePort.drivingResistance_FL.distance_m, DVA_None);
    DDefFloat(NULL, "AP.DrivingResistancePort.distanceRL", "m", &drivingResistancePort.drivingResistance_RL.distance_m, DVA_None);
    DDefFloat(NULL, "AP.DrivingResistancePort.distanceRR", "m", &drivingResistancePort.drivingResistance_RR.distance_m, DVA_None);
    DDefFloat(NULL, "AP.DrivingResistancePort.distanceFR", "m", &drivingResistancePort.drivingResistance_FR.distance_m, DVA_None);
    DDefUChar(NULL, "AP.DrivingResistancePort.typeFL", "", (uint8_t*)&drivingResistancePort.drivingResistance_FL.type_nu, DVA_None);
    DDefUChar(NULL, "AP.DrivingResistancePort.typeRL", "", (uint8_t*)&drivingResistancePort.drivingResistance_RL.type_nu, DVA_None);
    DDefUChar(NULL, "AP.DrivingResistancePort.typeRR", "", (uint8_t*)&drivingResistancePort.drivingResistance_RR.type_nu, DVA_None);
    DDefUChar(NULL, "AP.DrivingResistancePort.typeFR", "", (uint8_t*)&drivingResistancePort.drivingResistance_FR.type_nu, DVA_None);
    registerSignalHeaderToDVA("AP.DrivingResistancePort.sSigHeader", drivingResistancePort.sSigHeader);

    //LoDMCStatusPort
    DDefUChar(NULL, "AP.LoDMCStatusPort.loDMCSystemState_nu", "", (uint8_t*)&loDMCStatusPort.loDMCSystemState_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCStatusPort.standstillHoldCur_nu", "", (uint8_t*)&loDMCStatusPort.standstillHoldCur_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCStatusPort.maneuveringFinished_nu", "", (uint8_t*)&loDMCStatusPort.maneuveringFinished_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCStatusPort.standstillSecureCur_nu", "", (uint8_t*)&loDMCStatusPort.standstillSecureCur_nu, DVA_None);
    DDefUChar(NULL, "AP.LoDMCStatusPort.longitudinalControlActiveStatus_nu", "", (uint8_t*)&loDMCStatusPort.longitudinalControlActiveStatus_nu, DVA_None);
    registerSignalHeaderToDVA("AP.LoDMCStatusPort.sSigHeader", loDMCStatusPort.sSigHeader);

#ifdef MOCO_REPLACES_LODMC
    //MOCO inputs
    //MOCO.controlData
    DDefUChar(NULL, "MOCO.controlData_TRATCO.opMode", "", &controlData_TRATCO.opMode, DVA_None);
    DDefUChar(NULL, "MOCO.controlData_VECONA.opMode", "", &controlData_VECONA.opMode, DVA_None);

    //vehParam
    DDefFloat(NULL, "MOCO.vehParam.stat.bumperPosFront", "m", &vehParam.stat.bumperPosFront, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.bumperPosRear", "m", &vehParam.stat.bumperPosRear, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.tireCrnrStiffFront", "N/rad", &vehParam.stat.tireCrnrStiffFront, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.tireCrnrStiffRear", "N/rad", &vehParam.stat.tireCrnrStiffRear, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.tireRadius", "m", &vehParam.stat.tireRadius, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.vehHeightMax", "m", &vehParam.stat.vehHeightMax, DVA_None);
    DDefUShort(NULL, "MOCO.vehParam.stat.vehMassEmpty", "kg", &vehParam.stat.vehMassEmpty, DVA_None);
    DDefUShort(NULL, "MOCO.vehParam.stat.vehMassMean", "kg", &vehParam.stat.vehMassMean, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.vehOverhangFront", "m", &vehParam.stat.vehOverhangFront, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.vehOverhangRear", "m", &vehParam.stat.vehOverhangRear, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.vehWidthMax", "m", &vehParam.stat.vehWidthMax, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.stat.wheelBase", "m", &vehParam.stat.wheelBase, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.dyn.cntrOfGrav_x", "m", &vehParam.dyn.cntrOfGrav.x, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.dyn.cntrOfGrav_y", "m", &vehParam.dyn.cntrOfGrav.y, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.dyn.cntrOfGrav_z", "m", &vehParam.dyn.cntrOfGrav.z, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.dyn.tireCrnrStiffFront", "N/rad", &vehParam.dyn.tireCrnrStiffFront, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.dyn.tireCrnrStiffRear", "N/rad", &vehParam.dyn.tireCrnrStiffRear, DVA_None);
    DDefFloat(NULL, "MOCO.vehParam.dyn.tireRadius", "m", &vehParam.dyn.tireRadius, DVA_None);
    DDefUChar(NULL, "MOCO.vehParam.dyn.trailerAttached", "m", (uint8_t*)&vehParam.dyn.trailerAttached, DVA_None);
    DDefUShort(NULL, "MOCO.vehParam.dyn.vehMass", "kg", &vehParam.dyn.vehMass, DVA_None);

    //MOCO.vehDynFcu
    DDefFloat(NULL, "MOCO.vehDynFcu.bankAgl", "rad", &vehDynFcu.bankAgl, DVA_None);
    DDefFloat(NULL, "MOCO.vehDynFcu.crv", "1/m^2", &vehDynFcu.crv, DVA_None);
    DDefFloat(NULL, "MOCO.vehDynFcu.frontSteerAgl", "rad", &vehDynFcu.frontSteerAgl, DVA_None);
    DDefULLong(NULL, "MOCO.vehDynFcu.imageProcTime", "", (uint64_t*)&vehDynFcu.imageProcTime, DVA_None);
    DDefFloat(NULL, "MOCO.vehDynFcu.rearSteerAgl", "rad", &vehDynFcu.rearSteerAgl, DVA_None);
    DDefFloat(NULL, "MOCO.vehDynFcu.roadSlope", "", &vehDynFcu.roadSlope, DVA_None);
    DDefFloat(NULL, "MOCO.vehDynFcu.sideSlpAgl", "rad", &vehDynFcu.sideSlpAgl, DVA_None);

    //MOCO.brakeFb
    DDefUChar(NULL, "MOCO.brakeFb.available", "", (uint8_t*)&brakeFb.available, DVA_None);
    DDefFloat(NULL, "MOCO.brakeFb.axleTrqSumCur", "Nm", &brakeFb.axleTrqSumCur, DVA_None);
    DDefUChar(NULL, "MOCO.brakeFb.driverBraking", "", (uint8_t*)&brakeFb.driverBraking, DVA_None);
    DDefUChar(NULL, "MOCO.brakeFb.executedFunMode", "", (uint8_t*)&brakeFb.executedFunMode, DVA_None);
    DDefUChar(NULL, "MOCO.brakeFb.failure", "", (uint8_t*)&brakeFb.failure, DVA_None);
    DDefUChar(NULL, "MOCO.brakeFb.parkingBrkOpen", "", (uint8_t*)&brakeFb.parkingBrkOpen, DVA_None);
    DDefUChar(NULL, "MOCO.brakeFb.ssmStatus", "", (uint8_t*)&brakeFb.ssmStatus, DVA_None);

    //MOCO.powertrainFb
    DDefUChar(NULL, "MOCO.powertrainFb.accelPdlGrd", "", (uint8_t*)&powertrainFb.accelPdlGrd, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.accelPdlPos", "", (uint8_t*)&powertrainFb.accelPdlPos, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.arbitrationWinner", "", (uint8_t*)&powertrainFb.arbitrationWinner, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.available", "", (uint8_t*)&powertrainFb.available, DVA_None);
    DDefFloat(NULL, "MOCO.powertrainFb.axleTrqDriverReq", "Nm", &powertrainFb.axleTrqDriverReq, DVA_None);
    DDefFloat(NULL, "MOCO.powertrainFb.axleTrqSumCur", "Nm", &powertrainFb.axleTrqSumCur, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.axleTrqSumDistrib.frontLeftRight", "Nm", (uint8_t*)&powertrainFb.axleTrqSumDistrib.frontLeftRight, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.axleTrqSumDistrib.frontRear", "Nm", (uint8_t*)&powertrainFb.axleTrqSumDistrib.frontRear, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.axleTrqSumDistrib.rearLeftRight", "Nm", (uint8_t*)&powertrainFb.axleTrqSumDistrib.rearLeftRight, DVA_None);
    DDefFloat(NULL, "MOCO.powertrainFb.axleTrqSumFastMax", "Nm", &powertrainFb.axleTrqSumFastMax, DVA_None);
    DDefFloat(NULL, "MOCO.powertrainFb.axleTrqSumFastMin", "Nm", &powertrainFb.axleTrqSumFastMin, DVA_None);
    DDefFloat(NULL, "MOCO.powertrainFb.axleTrqSumMax", "Nm", &powertrainFb.axleTrqSumMax, DVA_None);
    DDefFloat(NULL, "MOCO.powertrainFb.axleTrqSumMin", "Nm", &powertrainFb.axleTrqSumMin, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.executedFunMode", "", (uint8_t*)&powertrainFb.executedFunMode, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.failure", "", (uint8_t*)&powertrainFb.failure, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.gearCur", "", (uint8_t*)&powertrainFb.gearCur, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.gearReqDriver", "", (uint8_t*)&powertrainFb.gearReqDriver, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.gearShiftActive", "", (uint8_t*)&powertrainFb.gearShiftActive, DVA_None);
    DDefUChar(NULL, "MOCO.powertrainFb.sailingStatus", "", (uint8_t*)&powertrainFb.sailingStatus, DVA_None);

    //MOCO.steeringFrontFb
    DDefUChar(NULL, "MOCO.steeringFrontFb.available", "", (uint8_t*)&steeringFrontFb.available, DVA_None);
    DDefFloat(NULL, "MOCO.steeringFrontFb.columnTrqCur", "Nm", &steeringFrontFb.columnTrqCur, DVA_None);
    DDefUChar(NULL, "MOCO.steeringFrontFb.executedFunMode", "", (uint8_t*)&steeringFrontFb.executedFunMode, DVA_None);
    DDefUChar(NULL, "MOCO.steeringFrontFb.failure", "", (uint8_t*)&steeringFrontFb.failure, DVA_None);
    DDefFloat(NULL, "MOCO.steeringFrontFb.steerAglCur", "rad", &steeringFrontFb.steerAglCur, DVA_None);
    DDefFloat(NULL, "MOCO.steeringFrontFb.steerAglVeloCur", "rad/s", &steeringFrontFb.steerAglVeloCur, DVA_None);
    DDefFloat(NULL, "MOCO.steeringFrontFb.steerTrqCur", "Nm", &steeringFrontFb.steerTrqCur, DVA_None);

    //MOCO.steeringFrontFb
    DDefUChar(NULL, "MOCO.steeringRearFb.available", "", (uint8_t*)&steeringRearFb.available, DVA_None);
    DDefUChar(NULL, "MOCO.steeringRearFb.executedFunMode", "", (uint8_t*)&steeringRearFb.executedFunMode, DVA_None);
    DDefUChar(NULL, "MOCO.steeringRearFb.failure", "", (uint8_t*)&steeringRearFb.failure, DVA_None);
    DDefFloat(NULL, "MOCO.steeringRearFb.steerAglCur", "rad", &steeringRearFb.steerAglCur, DVA_None);
    DDefFloat(NULL, "MOCO.steeringRearFb.steerAglVeloCur", "rad/s", &steeringRearFb.steerAglVeloCur, DVA_None);


    //TRATCO
    //TRATCO.tratcoStatusPort
    DDefUChar(NULL, "TRATCO.tratcoStatusPort.ctrlState", "", &tratcoStatusPort.ctrlState, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.crvMax", "", &tratcoStatusPort.latCtrlLimtd.crvMax, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.crvMin", "", &tratcoStatusPort.latCtrlLimtd.crvMin, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.crvRateFall", "", &tratcoStatusPort.latCtrlLimtd.crvRateFall, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.crvRateRise", "", &tratcoStatusPort.latCtrlLimtd.crvRateRise, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.sideSlpAglMax", "", &tratcoStatusPort.latCtrlLimtd.sideSlpAglMax, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.sideSlpAglMin", "", &tratcoStatusPort.latCtrlLimtd.sideSlpAglMin, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.sideSlpAglRateFall", "", &tratcoStatusPort.latCtrlLimtd.sideSlpAglRateFall, DVA_None);
    //DDefUChar(NULL, "TRATCO.tratcoStatusPort.latCtrlLimtd.sideSlpAglRateRise", "", &tratcoStatusPort.latCtrlLimtd.sideSlpAglRateRise, DVA_None);
    DDefUChar(NULL, "TRATCO.tratcoStatusPort.longCtrlLimtd.accelLow", "", &tratcoStatusPort.longCtrlLimtd.accelLow, DVA_None);
    DDefUChar(NULL, "TRATCO.tratcoStatusPort.longCtrlLimtd.accelUp", "", &tratcoStatusPort.longCtrlLimtd.accelUp, DVA_None);
    DDefUChar(NULL, "TRATCO.tratcoStatusPort.longCtrlLimtd.jerkLow", "", &tratcoStatusPort.longCtrlLimtd.jerkLow, DVA_None);
    DDefUChar(NULL, "TRATCO.tratcoStatusPort.longCtrlLimtd.jerkUp", "", &tratcoStatusPort.longCtrlLimtd.jerkUp, DVA_None);
    DDefUChar(NULL, "TRATCO.tratcoStatusPort.maneuveringFinished", "", &tratcoStatusPort.maneuveringFinished, DVA_None);

    //TRATCO.longAccelReq
    DDefFloat(NULL, "TRATCO.longAccelReq.accelReq", "m/s^2", &longAccelReq.accelReq, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.activateCtrl", "", &longAccelReq.activateCtrl, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.drivingDirReq", "", &longAccelReq.drivingDirReq, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.dynMode", "", &longAccelReq.dynMode, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.fullBrakingReq", "", &longAccelReq.fullBrakingReq, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.funMode", "", &longAccelReq.funMode, DVA_None);
    DDefFloat(NULL, "TRATCO.longAccelReq.lims.longAccelMax", "m/s^2", &longAccelReq.lims.longAccelMax, DVA_None);
    DDefFloat(NULL, "TRATCO.longAccelReq.lims.longAccelMin", "m/s^2", &longAccelReq.lims.longAccelMin, DVA_None);
    DDefFloat(NULL, "TRATCO.longAccelReq.lims.longJerkMax", "m/s^3", &longAccelReq.lims.longJerkMax, DVA_None);
    DDefFloat(NULL, "TRATCO.longAccelReq.lims.longJerkMin", "m/s^3", &longAccelReq.lims.longJerkMin, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.motReq", "", &longAccelReq.motReq, DVA_None);
    DDefULLong(NULL, "TRATCO.longAccelReq.sigHeader.uiTimeStamp", "", &longAccelReq.sigHeader.uiTimeStamp, DVA_None);
    DDefUShort(NULL, "TRATCO.longAccelReq.sigHeader.uiMeasurementCounter", "", &longAccelReq.sigHeader.uiMeasurementCounter, DVA_None);
    DDefUShort(NULL, "TRATCO.longAccelReq.sigHeader.uiCycleCounter", "", &longAccelReq.sigHeader.uiCycleCounter, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.sigHeader.eSigStatus", "", &longAccelReq.sigHeader.eSigStatus, DVA_None);
    DDefULong(NULL, "TRATCO.longAccelReq.versionNumber", "", &longAccelReq.versionNumber, DVA_None);
    DDefUChar(NULL, "TRATCO.longAccelReq.warningJerkReq", "", &longAccelReq.warningJerkReq, DVA_None);

    //TRATCO.Debugs
    // longCtrlIn
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.aEgo", "m/s^2", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.accelEgo), DVA_None);
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.aReq", "m/s^2", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.accelReq), DVA_None);
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.distErr", "m", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.distErr), DVA_None);
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.initReq", "", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.initReq), DVA_None);
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.motDirEgo", "", (uint8*)&(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.motDirEgo), DVA_None);
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.veloEgo", "m/s", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.veloEgo), DVA_None);
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.veloReq", "m/s", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.veloReq), DVA_None);
    DDef(NULL, "TRATCO.Debugs.longCtrlIn.veloMax", "m/s", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.longCtrlIn.veloMax), DVA_None);
    // trade
    //DDef(NULL, "TRATCO.Debugs.trade.dummy", "", &(tratcoProcMem.p_inter->tradeInter.tadprkInter.dummy), DVA_None);
    //DDef(NULL, "TRATCO.Debugs.trade.addPrevTime", "m/s^2", &(tratcoProcMem.p_interMeas->tradeInterMeas.tadreqInterMeas.addPrevTime), DVA_None);
    //DDef(NULL, "TRATCO.Debugs.trade.distErrNoBlnd", "m", &(tratcoProcMem.p_intraMeas->tradeIntraMeas.tadreqIntraMeas.distErrNoBlnd), DVA_None);
    //DDef(NULL, "TRATCO.Debugs.trade.distErrBlndStart", "m", &(tratcoProcMem.p_interMeas->tradeInterMeas.tadreqInterMeas.distErrBlndStart), DVA_None);
    //DDef(NULL, "TRATCO.Debugs.trade.ctrlPrioPrv", "-", (uint8*)&(tratcoProcMem.p_inter->tradeInter.tadreqInter.ctrlPrioPrv), DVA_None);
    //DDef(NULL, "TRATCO.Debugs.trade.blndFacPrv", "-", &(tratcoProcMem.p_interMeas->tradeInterMeas.tadreqInterMeas.blndFacPrv), DVA_None);
    // lotco
    DDef(NULL, "TRATCO.Debugs.lotco.ctrlStatus", "", (uint8*)&(tratcoProcMem.p_inter->lotcoInter.ctrlStatus), DVA_None);
    DDef(NULL, "TRATCO.Debugs.lotco.limtrActive", "", (uint8*)&(tratcoProcMem.p_inter->lotcoInter.limtrActive), DVA_None);
    DDef(NULL, "TRATCO.Debugs.lotco.accelReqPrv", "m/s^2", &(tratcoProcMem.p_inter->lotcoInter.ctrlStatus.accelReqPrv), DVA_None);
    DDef(NULL, "TRATCO.Debugs.loccon.distErr", "m", &(tratcoProcMem.p_inter->lotcoInter.locconInter.distErr), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.auxLimsCur.longAccelMax", "m/s^2", &(tratcoProcMem.p_inter->lotcoInter.locstmInter.auxLimsCur.longAccelMax), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.auxLimsCur.longAccelMin", "m/s^2", &(tratcoProcMem.p_inter->lotcoInter.locstmInter.auxLimsCur.longAccelMin), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.auxLimsCur.longJerkMax", "m/s^3", &(tratcoProcMem.p_inter->lotcoInter.locstmInter.auxLimsCur.longJerkMax), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.auxLimsCur.longJerkMin", "m/s^3", &(tratcoProcMem.p_inter->lotcoInter.locstmInter.auxLimsCur.longJerkMin), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.funModeLotco", "", (uint8*)&(tratcoProcMem.p_inter->lotcoInter.locstmInter.funModeLotco), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.jerkMax", "m/s^3", &(tratcoProcMem.p_inter->lotcoInter.locstmInter.jerkMax), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.jerkMin", "m/s^3", &(tratcoProcMem.p_inter->lotcoInter.locstmInter.jerkMin), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.motReq", "", (uint8*)&(tratcoProcMem.p_inter->lotcoInter.locstmInter.motReqPrv), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.overrideAccel", "", &(tratcoProcMem.p_inter->lotcoInter.locstmInter.overrideAccel), DVA_None);
    DDef(NULL, "TRATCO.Debugs.loccon.veloCtrlErr", "m", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locconInterMeas.veloCtrlErr), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.auxAccelFeedthrough", "m/s^2", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.auxAccelFeedthrough), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.distKP", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.outer.distKP), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.distKD", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.outer.distKD), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.distTD", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.outer.distTD), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.veloKFF", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.inner.veloKFF), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.veloKP", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.inner.veloKP), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.veloKD", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.inner.veloKD), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.veloTD", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.inner.veloTD), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.accelKFF", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.ctrlParamsCur.inner.accelKFF), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.activateFeedthroughFlag", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.feedthroughData.activateFeedthroughFlag), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.aFeedthrough", "m/s^2", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.feedthroughData.aFeedthrough), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.longAccelMax", "m/s^2", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.limtrParams.longAccelMax), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.longAccelMin", "m/s^2", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.limtrParams.longAccelMin), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.longJerkMax", "m/s^3", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.limtrParams.longJerkMax), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.longJerkMin", "m/s^3", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmCtrl.limtrParams.longJerkMin), DVA_None);
    DDef(NULL, "TRATCO.Debugs.lotcoParams.accelStandstill", "m/s^2", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.lotcoParams.accelStandstill), DVA_None);
    DDef(NULL, "TRATCO.Debugs.lotcoParams.accelStandStillGrd", "m/s^3", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.lotcoParams.accelStandStillGrd), DVA_None);
    DDef(NULL, "TRATCO.Debugs.lotcoParams.activeAccelLimtr", "", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.lotcoParams.activeAccelLimtr), DVA_None);
    DDef(NULL, "TRATCO.Debugs.lotcoParams.activeJerkLimtr", "", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.lotcoParams.activeJerkLimtr), DVA_None);
    DDef(NULL, "TRATCO.Debugs.lotcoParams.standStillSpdThres", "m/s", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.lotcoParams.standStillSpdThres), DVA_None);
    DDef(NULL, "TRATCO.Debugs.loccon.accelCtrl", "m/s^2", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.locconIntraMeas.accelCtrl), DVA_None);
    DDef(NULL, "TRATCO.Debugs.loccon.accelLimtd", "m/s^2", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.locconIntraMeas.accelLimtd), DVA_None);
    DDef(NULL, "TRATCO.Debugs.loccon.accelReq", "m/s^2", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.locconIntraMeas.accelReq), DVA_None);
    DDef(NULL, "TRATCO.Debugs.loccon.veloCascadeInputReq", "m/s", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.locconIntraMeas.veloCascadeInputReq), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.auxAccelCtrl", "m/s^2", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.locstmIntraMeas.auxAccelCtrl), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.auxAccelLimtd", "m/s^2", &(tratcoProcMem.p_intraMeas->lotcoIntraMeas.locstmIntraMeas.auxAccelLimtd), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.driveOffEndOfStroke.skipDriveOff", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmInterMeas.driveOffEndOfStroke.skipDriveOff), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.driveOffEndOfStroke.tryDriveOff", "", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmInterMeas.driveOffEndOfStroke.tryDriveOffPrv), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.driveOffEndOfStroke.minDistToStopEstimated", "m", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmInterMeas.driveOffEndOfStroke.minDistToStopEstimated), DVA_None);
    DDef(NULL, "TRATCO.Debugs.locstm.driveOffEndOfStroke.timeInStandstill", "s", &(tratcoProcMem.p_interMeas->lotcoInterMeas.locstmInterMeas.driveOffEndOfStroke.timeInStandstillPrv), DVA_None);

    //VECONA
    //VECONA.longVeconaStatusPort
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.ctrlState", "", (uint8_t*)&longVeconaStatusPort.ctrlState, DVA_None);
    DDefFloat(NULL, "VECONA.longVeconaStatusPort.capabilityFb.maxAccel.val_x", "m/s^2", &longVeconaStatusPort.capabilityFb.maxAccel.val[0], DVA_None);
    DDefFloat(NULL, "VECONA.longVeconaStatusPort.capabilityFb.maxAccel.val_y", "m/s^2", &longVeconaStatusPort.capabilityFb.maxAccel.val[1], DVA_None);
    DDefFloat(NULL, "VECONA.longVeconaStatusPort.capabilityFb.maxAccel.val_z", "m/s^2", &longVeconaStatusPort.capabilityFb.maxAccel.val[2], DVA_None);
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.ctrlLimtd.accelLow", "", (uint8_t*)&longVeconaStatusPort.ctrlLimtd.accelLow, DVA_None);
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.ctrlLimtd.accelUp", "", (uint8_t*)&longVeconaStatusPort.ctrlLimtd.accelUp, DVA_None);
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.ctrlLimtd.jerkLow", "", (uint8_t*)&longVeconaStatusPort.ctrlLimtd.jerkLow, DVA_None);
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.ctrlLimtd.jerkUp", "", (uint8_t*)&longVeconaStatusPort.ctrlLimtd.jerkUp, DVA_None);
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.executedFunMode", "", (uint8_t*)&longVeconaStatusPort.executedFunMode, DVA_None);
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.motReqResponse", "", (uint8_t*)&longVeconaStatusPort.motReqResponse, DVA_None);
    DDefUChar(NULL, "VECONA.longVeconaStatusPort.standstillSecured", "", (uint8_t*)&longVeconaStatusPort.standstillSecured, DVA_None);
    DDefFloat(NULL, "VECONA.longVeconaStatusPort.vehAccelExpected", "m/s^2", &longVeconaStatusPort.vehAccelExpected, DVA_None);

    //VECONA.veconaBrakeRequest
    DDefULong(NULL, "VECONA.veconaBrakeRequest.versionNumber", "", &veconaBrakeRequest.versionNumber, DVA_None);
    DDefULLong(NULL, "VECONA.veconaBrakeRequest.sigHeader.uiTimeStamp", "", &veconaBrakeRequest.sigHeader.uiTimeStamp, DVA_None);
    DDefUShort(NULL, "VECONA.veconaBrakeRequest.sigHeader.uiMeasurementCounter", "", &veconaBrakeRequest.sigHeader.uiMeasurementCounter, DVA_None);
    DDefUShort(NULL, "VECONA.veconaBrakeRequest.sigHeader.uiCycleCounter", "", &veconaBrakeRequest.sigHeader.uiCycleCounter, DVA_None);
    DDefUChar(NULL, "VECONA.veconaBrakeRequest.sigHeader.eSigStatus", "", &veconaBrakeRequest.sigHeader.eSigStatus, DVA_None);
    DDefFloat(NULL, "VECONA.veconaBrakeRequest.axleTrqSumReq", "Nm", &veconaBrakeRequest.axleTrqSumReq, DVA_None);
    DDefUChar(NULL, "VECONA.veconaBrakeRequest.funMode", "", &veconaBrakeRequest.funMode, DVA_None);
    DDefUChar(NULL, "VECONA.veconaBrakeRequest.ssmReq", "", &veconaBrakeRequest.ssmReq, DVA_None);
    DDefUChar(NULL, "VECONA.veconaBrakeRequest.activateCtrl", "", &veconaBrakeRequest.activateCtrl, DVA_None);
    DDefUChar(NULL, "VECONA.veconaBrakeRequest.secureStandstill", "", &veconaBrakeRequest.secureStandstill, DVA_None);

    //VECONA.veconaPowertrainRequest
    DDefULong(NULL, "VECONA.veconaPowertrainRequest.versionNumber", "", &veconaPowertrainRequest.versionNumber, DVA_None);
    DDefULLong(NULL, "VECONA.veconaPowertrainRequest.sigHeader.uiTimeStamp", "", &veconaPowertrainRequest.sigHeader.uiTimeStamp, DVA_None);
    DDefUShort(NULL, "VECONA.veconaPowertrainRequest.sigHeader.uiMeasurementCounter", "", &veconaPowertrainRequest.sigHeader.uiMeasurementCounter, DVA_None);
    DDefUShort(NULL, "VECONA.veconaPowertrainRequest.sigHeader.uiCycleCounter", "", &veconaPowertrainRequest.sigHeader.uiCycleCounter, DVA_None);
    DDefUChar(NULL, "VECONA.veconaPowertrainRequest.sigHeader.eSigStatus", "", &veconaPowertrainRequest.sigHeader.eSigStatus, DVA_None);
    DDefFloat(NULL, "VECONA.veconaPowertrainRequest.axleTrqSumReq", "Nm", &veconaPowertrainRequest.axleTrqSumReq, DVA_None);
    DDefUChar(NULL, "VECONA.veconaPowertrainRequest.funMode", "", &veconaPowertrainRequest.funMode, DVA_None);
    DDefUChar(NULL, "VECONA.veconaPowertrainRequest.dynMode", "", &veconaPowertrainRequest.dynMode, DVA_None);
    DDefUChar(NULL, "VECONA.veconaPowertrainRequest.arbMode", "", &veconaPowertrainRequest.arbMode, DVA_None);
    DDefUChar(NULL, "VECONA.veconaPowertrainRequest.gearReq", "", &veconaPowertrainRequest.gearReq, DVA_None);
    DDefUChar(NULL, "VECONA.veconaPowertrainRequest.activateCtrl", "", &veconaPowertrainRequest.activateCtrl, DVA_None);
    DDefUChar(NULL, "VECONA.veconaPowertrainRequest.reduceTrqInStandstill", "", &veconaPowertrainRequest.reduceTrqInStandstill, DVA_None);

    //VECONA.longDriverFeedback
    DDefUChar(NULL, "VECONA.longDriverFeedback.overrideAccel", "", (uint8_t*)&longDriverFeedback.overrideAccel, DVA_None);
    DDefUChar(NULL, "VECONA.longDriverFeedback.axleTrqSumReq", "", (uint8_t*)&longDriverFeedback.overrideDecel, DVA_None);

    //VECONA.vehDynVecona
    DDefFloat(NULL, "VECONA.vehDynVecona.bankAgl", "rad", &vehDynVecona.bankAgl, DVA_None);
    DDefFloat(NULL, "VECONA.vehDynVecona.crv", "1/m^2", &vehDynVecona.crv, DVA_None);
    DDefFloat(NULL, "VECONA.vehDynVecona.frontSteerAgl", "rad", &vehDynVecona.frontSteerAgl, DVA_None);
    DDefULLong(NULL, "VECONA.vehDynVecona.imageProcTime", "", &vehDynVecona.imageProcTime, DVA_None);
    DDefFloat(NULL, "VECONA.vehDynVecona.rearSteerAgl", "rad", &vehDynVecona.rearSteerAgl, DVA_None);
    DDefFloat(NULL, "VECONA.vehDynVecona.roadSlope", "", &vehDynVecona.roadSlope, DVA_None);
    DDefFloat(NULL, "VECONA.vehDynVecona.sideSlpAgl", "rad", &vehDynVecona.sideSlpAgl, DVA_None);

    //VECONA.Debugs
    // p_inter->chacoInter
    // - chalitInter
    DDefUChar(NULL, "VECONA.Debugs.chacoInter.chalitInter.activateCtrlLatPrv", "", &veconaProcMem.p_inter->chacoInter.chalitInter.activateCtrlLatPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.chacoInter.chalitInter.crvBacFiltPrv", "1/m", &veconaProcMem.p_inter->chacoInter.chalitInter.crvBacFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.chacoInter.chalitInter.crvRateReqPrv", "1/ms", &veconaProcMem.p_inter->chacoInter.chalitInter.crvRateReqPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.chacoInter.chalitInter.crvReqFiltPrv", "1/m", &veconaProcMem.p_inter->chacoInter.chalitInter.crvReqFiltPrv, DVA_None);

    // - chapasInter
    DDefUChar(NULL, "VECONA.Debugs.chacoInter.chapasInter.activateCtrlLatPrv", "", &veconaProcMem.p_inter->chacoInter.chapasInter.activateCtrlLatPrv, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.chacoInter.chapasInter.isTimerFinishedPrv", "", &veconaProcMem.p_inter->chacoInter.chapasInter.isTimerFinishedPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.chacoInter.chapasInter.timerValPrv", "s", &veconaProcMem.p_inter->chacoInter.chapasInter.timerValPrv, DVA_None);

    // - chasarInter
    DDefUChar(NULL, "VECONA.Debugs.chacoInter.chasarInter.activateCtrlLatPrv", "", &veconaProcMem.p_inter->chacoInter.chasarInter.activateCtrlLatPrv, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.chacoInter.chasarInter.isTimerFinished", "", &veconaProcMem.p_inter->chacoInter.chasarInter.isTimerFinished, DVA_None);
    DDef(NULL, "VECONA.Debugs.chacoInter.chasarInter.timerVal", "s", &veconaProcMem.p_inter->chacoInter.chasarInter.timerVal, DVA_None);

    // - chasgeCtrlAccelXInter
    DDef(NULL, "VECONA.Debugs.chacoInter.chasgeCtrlAccelXInter.veloXVCSReq", "m/s", &veconaProcMem.p_inter->chacoInter.chasgeCtrlAccelXInter.veloXVCSReq, DVA_None);

    // - chasipInter
    DDef(NULL, "VECONA.Debugs.chacoInter.chasipInter.accelXVCSMdlDel.curSample", "m/s^2", &veconaProcMem.p_inter->chacoInter.chasipInter.accelXVCSMdlDel.curSample, DVA_None);

    // p_inter->wetcoInter
    // - actuatorReqs
    DDefUChar(NULL, "VECONA.Debugs.wetcoInter.actuatorReqs.brkReqActivePrv", "", &veconaProcMem.p_inter->wetcoInter.actuatorReqs.brkReqActivePrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoInter.actuatorReqs.brkTrqReqPrv", "Nm", &veconaProcMem.p_inter->wetcoInter.actuatorReqs.brkTrqReqPrv, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoInter.actuatorReqs.powReqActivePrv", "", &veconaProcMem.p_inter->wetcoInter.actuatorReqs.powReqActivePrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoInter.actuatorReqs.powTrqReqPrv", "Nm", &veconaProcMem.p_inter->wetcoInter.actuatorReqs.powTrqReqPrv, DVA_None);

    // - wetcapInter
    DDefUChar(NULL, "VECONA.Debugs.wetcoInter.wetcapInter.initFlag", "", &veconaProcMem.p_inter->wetcoInter.wetcapInter.initFlag, DVA_None);

    // - wetstmInter
    DDefUChar(NULL, "VECONA.Debugs.wetcoInter.wetstmInter.accelOverridePrv", "", &veconaProcMem.p_inter->wetcoInter.wetstmInter.accelOverridePrv, DVA_None);

    // p_intraMeas->chacoIntraMeas
    // - chadisDisturbances
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chadisDisturbances.bankAgl", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chadisDisturbances.bankAgl, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chadisDisturbances.frcDriver", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chadisDisturbances.frcDriver, DVA_None); // invMdlInputs: distStates: F_driver
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chadisDisturbances.roadSlope", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chadisDisturbances.roadSlope, DVA_None);

    // - chainvLatWhlDynDebug
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvLatWhlDynDebug.slpAglFrontInvCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvLatWhlDynDebug.slpAglFrontInvCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvLatWhlDynDebug.slpAglRearInvCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvLatWhlDynDebug.slpAglRearInvCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvLatWhlDynDebug.strAglFrontInvCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvLatWhlDynDebug.strAglFrontInvCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvLatWhlDynDebug.strAglRearInvCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvLatWhlDynDebug.strAglRearInvCur, DVA_None);

    // - chainvVehDynDebug
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvVehDynDebug.frcsTCSInv", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvVehDynDebug.frcsTCSInv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvVehDynDebug.frcsYVcsInv", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvVehDynDebug.frcsYVcsInv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvVehDynDebug.frcXVcsDebug", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvVehDynDebug.frcXVcsDebug, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chainvVehDynDebug.frcYVcsDebug", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chainvVehDynDebug.frcYVcsDebug, DVA_None);

    // - chalicLims
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.accelXVCSReqMin", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.accelXVCSReqMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.crvAccelFallMax", "1/m^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.crvAccelFallMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.crvAccelRiseMax", "1/m^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.crvAccelRiseMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.crvMax", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.crvMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.crvMin", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.crvMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.crvRateFallMax", "1/ms", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.crvRateFallMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.crvRateRiseMax", "1/ms", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.crvRateRiseMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcFrontCapabilityMax", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcFrontCapabilityMax, DVA_None); // invMdlInputs: meas: frontFrcCapabilityMax
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcFrontCapabilityMin", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcFrontCapabilityMin, DVA_None); // invMdlInputs: meas: frontFrcCapabilityMin
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcPowMax", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcPowMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcPowMaxFilt", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcPowMaxFilt, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcPowMin", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcPowMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcPowMinFilt", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcPowMinFilt, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcRearCapabilityMax", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcRearCapabilityMax, DVA_None); // invMdlInputs: meas: rearFrcCapabilityMax
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcRearCapabilityMin", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcRearCapabilityMin, DVA_None); //invMdlInputs: meas: rearFrcCapabilityMin
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcXTCSFrontMin", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcXTCSFrontMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.frcXTCSRearMin", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.frcXTCSRearMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.sideSlpAglMax", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.sideSlpAglMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.sideSlpAglMin", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.sideSlpAglMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.sideSlpRateFallMax", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.sideSlpRateFallMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.sideSlpRateRiseMax", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.sideSlpRateRiseMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglFrontMax", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglFrontMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglFrontMin", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglFrontMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglRateFrontMax", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglRateFrontMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglRateFrontMin", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglRateFrontMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglRateRearMax", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglRateRearMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglRateRearMin", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglRateRearMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglRearMax", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglRearMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalicLims.strAglRearMin", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalicLims.strAglRearMin, DVA_None);

    // - chalitDebug
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalitDebug.crvReqBac", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalitDebug.crvReqBac, DVA_None);

    // - chalitIntraMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalitIntraMeas.crvRateReqGrdLim", "1/ms", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalitIntraMeas.crvRateReqGrdLim, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalitIntraMeas.crvReqBac", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalitIntraMeas.crvReqBac, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chalitIntraMeas.crvReqFilt", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chalitIntraMeas.crvReqFilt, DVA_None);

    // - chamodLatWhlDynDebug
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodLatWhlDynDebug.frcYTCSFrontMdlPred", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodLatWhlDynDebug.frcYTCSFrontMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodLatWhlDynDebug.frcYTCSRearMdlPred", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodLatWhlDynDebug.frcYTCSRearMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodLatWhlDynDebug.slipAglFrontMdlCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodLatWhlDynDebug.slipAglFrontMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodLatWhlDynDebug.slipAglRearMdlCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodLatWhlDynDebug.slipAglRearMdlCur, DVA_None);

    // - chamodVehDynIntraMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodVehDynIntraMeas.accelXVCSMdlPred", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodVehDynIntraMeas.accelXVCSMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodVehDynIntraMeas.accelYVCSMdlPred", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodVehDynIntraMeas.accelYVCSMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodVehDynIntraMeas.crvMdlPred", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodVehDynIntraMeas.crvMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodVehDynIntraMeas.sideSlpAglMdlPred", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodVehDynIntraMeas.sideSlpAglMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodVehDynIntraMeas.veloRateXVCSMdlPred", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodVehDynIntraMeas.veloRateXVCSMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chamodVehDynIntraMeas.yawAccelMdlPred", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chamodVehDynIntraMeas.yawAccelMdlPred, DVA_None);

    // - chapasReqs
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chapasReqs.precisionReq", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chapasReqs.precisionReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chapasReqs.stiffnessReq", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chapasReqs.stiffnessReq, DVA_None);

    // - chasgeCtrlAccelXIntraMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlAccelXIntraMeas.veloXVCSInvCur", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlAccelXIntraMeas.veloXVCSInvCur, DVA_None); // invMdlInputs: inputs: v_x_input_kp1
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlAccelXIntraMeas.veloXVCSInvPrv", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlAccelXIntraMeas.veloXVCSInvPrv, DVA_None); // invMdlInputs: inputs: v_x_input_k
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlAccelXIntraMeas.accelAccSafetyFiltCur", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlAccelXIntraMeas.accelACCSafetyFiltCur, DVA_None); // invMdlInputs: inputs: safetyBrkAccel

    // - chasgeCtrlVeloYOut
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlVeloYOut.sideSlipAglRateReq", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlVeloYOut.sideSlipAglRateReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlVeloYOut.veloReq", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlVeloYOut.veloReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlVeloYOut.veloYVCSInvCur", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlVeloYOut.veloYVCSInvCur, DVA_None); // invMdlInputs: inputs: v_y_input_kp1
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlVeloYOut.veloYVCSReq", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlVeloYOut.veloYVCSReq, DVA_None);

    // - chasgeCtrlYawRateOut
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlYawRateOut.yawRateInvCur", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlYawRateOut.yawRateInvCur, DVA_None); // invMdlInputs: inputs: Psi_dot_input_kp1
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasgeCtrlYawRateOut.yawRateReq", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasgeCtrlYawRateOut.yawRateReq, DVA_None); // invMdlInputs: req: Psi_dot_req

    // - meas
    // DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipLims.trqPowMax", "N", &veconaProcMem.p_intra->chacoIntra.chasipLims.trqPowMax, DVA_None); // invMdlInputs: meas: axleFrcSumMax
    // DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipLims.trqPowMin", "N", &veconaProcMem.p_intra->chacoIntra.chasipLims.trqPowMin, DVA_None); // invMdlInputs: meas: axleFrcSumMin

    // - chasipFbGain
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFbGain.fbGainLat", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFbGain.fbGainLat, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFbGain.fbGainLong", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFbGain.fbGainLong, DVA_None);

    // - chasipFiltSigs
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.accelXVCSMeasFiltCur", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.accelXVCSMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.frcBrkMeasFiltCur", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.frcBrkMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.frcPowMeasFiltCur", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.frcPowMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.sideSlpAglMeasFiltCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.sideSlpAglMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.sideSlpAglRateMeasFiltCur", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.sideSlpAglRateMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.strAglFrontMeasFiltCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.strAglFrontMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.strAglRearMeasFiltCur", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.strAglRearMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.veloMeasFiltCur", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.veloMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.veloXVCSMeasFiltCur", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.veloXVCSMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.veloYVCSMeasFiltCur", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.veloYVCSMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.sens.yawRateMeasFiltCur", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.sens.yawRateMeasFiltCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipFiltSigs.veloYVCSMdlFiltCur", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipFiltSigs.veloYVCSMdlFiltCur, DVA_None);

    // - chasipReqs
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.accelXVCSReq", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.accelXVCSReq, DVA_None); // req: a_x_req
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.activateCtrlLat", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.activateCtrlLat, DVA_None); // req: a_x_req
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.activateCtrlLong", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.activateCtrlLong, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.crvReq", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.crvReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.drivingDirReqSign", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.drivingDirReqSign, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.dynModeLat", "", (uint8_t*)&veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.dynModeLat, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.dynModeLong", "", (uint8_t*)&veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.dynModeLong, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.funModeLat", "", (uint8_t*)&veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.funModeLat, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.funModeLong", "", (uint8_t*)&veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.funModeLong, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipReqs.sideSlpAglReq", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipReqs.sideSlpAglReq, DVA_None); // invMdlInputs: req: beta_req

    // - chasipSens
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.accelXVCSMeas", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.accelXVCSMeas, DVA_None); // invMdlInputs: meas: a_x_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.accelYVCSMeas", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.accelYVCSMeas, DVA_None); // invMdlInputs: meas: a_y_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.bankAglMeas", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.bankAglMeas, DVA_None); // invMdlInputs: distStates: roll_angle
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.crvMeas", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.crvMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.frcBrkMeas", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.frcBrkMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.frcCrossWindMeas", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.frcCrossWindMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.frcDriverMeas", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.frcDriverMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.frcPowMeas", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.frcPowMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.roadSlopeMeas", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.roadSlopeMeas, DVA_None); // invMdlInputs: distStates: roadSlope
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.sideSlpAglMeas", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.sideSlpAglMeas, DVA_None); // invMdlInputs: meas: beta_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.strAglFrontMeas", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.strAglFrontMeas, DVA_None); // invMdlInputs: meas: delta_f_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.strAglRearMeas", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.strAglRearMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.strTrqColumnMeas", "Nm", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.strTrqColumnMeas, DVA_None); // invMdlInputs: meas: Trq_col_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.trqBrkMeas", "Nm", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.trqBrkMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.trqPowMeas", "Nm", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.trqPowMeas, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.veloMeas", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.veloMeas, DVA_None); // invMdlInputs: meas: v_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.veloXVCSMeas", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.veloXVCSMeas, DVA_None); // invMdlInputs: meas: v_x_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.veloYVCSMeas", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.veloYVCSMeas, DVA_None); // invMdlInputs: meas: v_y_sys
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipSens.yawRateMeas", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipSens.yawRateMeas, DVA_None); // invMdlInputs: meas: Psi_dot_sys

    // - chasipStrAglsInv
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipStrAglsInv.strAglFrontInvPrv", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipStrAglsInv.strAglFrontInvPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chasipStrAglsInv.strAglRearInvPrv", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.chasipStrAglsInv.strAglRearInvPrv, DVA_None);

    // - chastmIntraMeas
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.chastmIntraMeas.resetFlag", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.chastmIntraMeas.resetFlag, DVA_None);

    // - crvReqBac
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.crvReqBac", "1/m", &veconaProcMem.p_intraMeas->chacoIntraMeas.crvReqBac, DVA_None);

    // - frontSteerAglReq
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.frontSteerAglReq.accuracyReq", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.frontSteerAglReq.accuracyReq, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.frontSteerAglReq.activateCtrl", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.frontSteerAglReq.activateCtrl, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.frontSteerAglReq.funMode", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.frontSteerAglReq.funMode, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.frontSteerAglReq.steerAglReq", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.frontSteerAglReq.steerAglReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.frontSteerAglReq.stiffnessReq", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.frontSteerAglReq.stiffnessReq, DVA_None);

    // - fsmConditionsLat
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLat.hasFuncFault", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLat.hasFuncFault, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLat.isActivatable", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLat.isActivatable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLat.isAvailable", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLat.isAvailable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLat.isRampedIn", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLat.isRampedIn, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLat.isRampedOut", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLat.isRampedOut, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLat.shallDeactivate", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLat.shallDeactivate, DVA_None);

    // - fsmConditionsLong
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLong.hasFuncFault", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLong.hasFuncFault, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLong.isActivatable", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLong.isActivatable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLong.isAvailable", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLong.isAvailable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLong.isRampedIn", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLong.isRampedIn, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLong.isRampedOut", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLong.isRampedOut, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.fsmConditionsLong.shallDeactivate", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.fsmConditionsLong.shallDeactivate, DVA_None);

    // - latFrcReq
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.latFrcReq.latFrontWhlFrc", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.latFrcReq.latFrontWhlFrc, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.latFrcReq.latRearWhlFrc", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.latFrcReq.latRearWhlFrc, DVA_None);

    // - longFrcReq
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.longFrcReq.activateCtrl", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.longFrcReq.activateCtrl, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.longFrcReq.drivingDirReq", "", (uint8_t*)&veconaProcMem.p_intraMeas->chacoIntraMeas.longFrcReq.drivingDirReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.longFrcReq.frontFrcReq", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.longFrcReq.frontFrcReq, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.longFrcReq.funMode", "", (uint8_t*)&veconaProcMem.p_intraMeas->chacoIntraMeas.longFrcReq.funMode, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.longFrcReq.motReq", "", (uint8_t*)&veconaProcMem.p_intraMeas->chacoIntraMeas.longFrcReq.motReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.longFrcReq.rearFrcReq", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.longFrcReq.rearFrcReq, DVA_None);

    // - rearSteerAglReq
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.rearSteerAglReq.accuracyReq", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.rearSteerAglReq.accuracyReq, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.rearSteerAglReq.activateCtrl", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.rearSteerAglReq.activateCtrl, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.rearSteerAglReq.funMode", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.rearSteerAglReq.funMode, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.rearSteerAglReq.steerAglReq", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.rearSteerAglReq.steerAglReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoIntraMeas.rearSteerAglReq.stiffnessReq", "", &veconaProcMem.p_intraMeas->chacoIntraMeas.rearSteerAglReq.stiffnessReq, DVA_None);

    // p_intraMeas->veconaCond
    // - condLat
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.extCond.shallEnterStandby", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.extCond.shallEnterStandby, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.extCond.shallGetReady", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.extCond.shallGetReady, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.extCond.shallResetFuncFault", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.extCond.shallResetFuncFault, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.intCond.hasFuncFault", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.intCond.hasFuncFault, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.intCond.isActivatable", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.intCond.isActivatable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.intCond.isAvailable", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.intCond.isAvailable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.intCond.isRampedIn", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.intCond.isRampedIn, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.intCond.isRampedOut", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.intCond.isRampedOut, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLat.intCond.shallDeactivate", "", &veconaProcMem.p_intraMeas->veconaCond.condLat.intCond.shallDeactivate, DVA_None);

    // - condLong
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.extCond.shallEnterStandby", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.extCond.shallEnterStandby, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.extCond.shallGetReady", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.extCond.shallGetReady, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.extCond.shallResetFuncFault", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.extCond.shallResetFuncFault, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.intCond.hasFuncFault", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.intCond.hasFuncFault, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.intCond.isActivatable", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.intCond.isActivatable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.intCond.isAvailable", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.intCond.isAvailable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.intCond.isRampedIn", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.intCond.isRampedIn, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.intCond.isRampedOut", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.intCond.isRampedOut, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.veconaCond.condLong.intCond.shallDeactivate", "", &veconaProcMem.p_intraMeas->veconaCond.condLong.intCond.shallDeactivate, DVA_None);

    // p_intraMeas->wetcoIntraMeas
    // - debugSignals
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.accIsoAccelLimActive", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.accIsoAccelLimActive, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.accIsoDecelLimActive", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.accIsoDecelLimActive, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.accIsoJerkLimActive", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.accIsoJerkLimActive, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.brkTrqReqDriveOff", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.brkTrqReqDriveOff, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.capabilitySumTrqFastMax", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.capabilitySumTrqFastMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.capabilitySumTrqFastMin", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.capabilitySumTrqFastMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.capabilitySumTrqMax", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.capabilitySumTrqMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.capabilitySumTrqMin", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.capabilitySumTrqMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.disturbanceTrqCur", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.disturbanceTrqCur, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.debugSignals.longForceReqIsLimited", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.debugSignals.longForceReqIsLimited, DVA_None);

    // - fsmConditions
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.fsmConditions.hasFuncFault", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.fsmConditions.hasFuncFault, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.fsmConditions.isActivatable", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.fsmConditions.isActivatable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.fsmConditions.isAvailable", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.fsmConditions.isAvailable, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.fsmConditions.isRampedIn", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.fsmConditions.isRampedIn, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.fsmConditions.isRampedOut", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.fsmConditions.isRampedOut, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.fsmConditions.shallDeactivate", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.fsmConditions.shallDeactivate, DVA_None);

    // - wetcapLimits
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.wetcapLimits.rampoutTrqGrd", "Nm/s", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetcapLimits.rampoutTrqGrd, DVA_None);

    // - wetconActReqInternal
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.wetconActReqInternal.brkReqActiveInternal", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetconActReqInternal.brkReqActiveInternal, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.wetconActReqInternal.brkTrqReqInternal", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetconActReqInternal.brkTrqReqInternal, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.wetconActReqInternal.powReqActiveInternal", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetconActReqInternal.powReqActiveInternal, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoIntraMeas.wetconActReqInternal.powTrqReqInternal", "Nm", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetconActReqInternal.powTrqReqInternal, DVA_None);

    // - wetstmCtrlPrecond
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.wetstmCtrlPrecond.brkAvailableForCtrl", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetstmCtrlPrecond.brkAvailableForCtrl, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.wetstmCtrlPrecond.brkAvailableForRampout", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetstmCtrlPrecond.brkAvailableForRampout, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.wetstmCtrlPrecond.powAvailableForCtrl", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetstmCtrlPrecond.powAvailableForCtrl, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.wetstmCtrlPrecond.powAvailableForRampout", "", &veconaProcMem.p_intraMeas->wetcoIntraMeas.wetstmCtrlPrecond.powAvailableForRampout, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoIntraMeas.wetstmCtrlPrecond.ssmReq", "", (uint8_t*)&veconaProcMem.p_intraMeas->wetcoIntraMeas.wetstmCtrlPrecond.ssmReq, DVA_None);

    // p_interMeas->chacoInterMeas
    // - chadisInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chadisInterMeas.bankAglMdlPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chadisInterMeas.bankAglMdlPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chadisInterMeas.bankAglMdlPrvTwoCyc", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chadisInterMeas.bankAglMdlPrvTwoCyc, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chadisInterMeas.frcDriverPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chadisInterMeas.frcDriverPrv, DVA_None);

    // - chaestInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chaestInterMeas.variancesXVCSMdlCur", "", &veconaProcMem.p_interMeas->chacoInterMeas.chaestInterMeas.variancesXVCSMdlCur, DVA_None);

    // - chaestMdlStates
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chaestMdlStates.veloXVCSMdlCorCur", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chaestMdlStates.veloXVCSMdlCorCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chaestMdlStates.veloYVCSMdlCur", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chaestMdlStates.veloYVCSMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chaestMdlStates.yawRateMdlCur", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chaestMdlStates.yawRateMdlCur, DVA_None);

    // - chainvActDynXTCS
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvActDynXTCS.frcSumXTCSFrontInvReq", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvActDynXTCS.frcSumXTCSFrontInvReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvActDynXTCS.frcSumXTCSRearInvReq", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvActDynXTCS.frcSumXTCSRearInvReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvActDynXTCS.trqBrkXTCSInvReq", "Nm", &veconaProcMem.p_interMeas->chacoInterMeas.chainvActDynXTCS.trqBrkXTCSInvReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvActDynXTCS.trqPowXTCSInvReq", "Nm", &veconaProcMem.p_interMeas->chacoInterMeas.chainvActDynXTCS.trqPowXTCSInvReq, DVA_None);

    // - chainvActDynYTCS
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvActDynYTCS.strAglFrontInvReq", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chainvActDynYTCS.strAglFrontInvReq, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvActDynYTCS.strAglRearInvReq", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chainvActDynYTCS.strAglRearInvReq, DVA_None);

    // - chainvInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcBrkXTCSCompLimInvPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcBrkXTCSCompLimInvPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcPowXTCSInvCurDelayed", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcPowXTCSInvCurDelayed, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcPowXTCSInvPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcPowXTCSInvPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcPowXTCSInvPrvDelayed", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcPowXTCSInvPrvDelayed, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcPowXTCSWindUpInvPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcPowXTCSWindUpInvPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcRateBrkXTCSCompLimInvPrv", "N/s", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcRateBrkXTCSCompLimInvPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcYTCSFrontInvPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcYTCSFrontInvPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvInterMeas.frcYTCSRearInvPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chainvInterMeas.frcYTCSRearInvPrv, DVA_None);

    // - chainvStrAgls
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvStrAgls.strAglFrontInvPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chainvStrAgls.strAglFrontInvPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chainvStrAgls.strAglRearInvPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chainvStrAgls.strAglRearInvPrv, DVA_None);

    // - chalicInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chalicInterMeas.frcPowMaxPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chalicInterMeas.frcPowMaxPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chalicInterMeas.frcPowMinPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chalicInterMeas.frcPowMinPrv, DVA_None);

    // - chalimInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chalimInterMeas.crvVehDynFiltPrv", "1/m", &veconaProcMem.p_interMeas->chacoInterMeas.chalimInterMeas.crvVehDynFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chalimInterMeas.sideSlipAglVehDynFiltPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chalimInterMeas.sideSlipAglVehDynFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chalimInterMeas.strAglFrontLimPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chalimInterMeas.strAglFrontLimPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chalimInterMeas.strAglRearLimPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chalimInterMeas.strAglRearLimPrv, DVA_None);

    // - chamodInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.frcBrkXTCSMdlCur", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.frcBrkXTCSMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.frcBrkXTCSReqDelayed", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.frcBrkXTCSReqDelayed, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.frcPowXTCSMdlCur", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.frcPowXTCSMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.frcPowXTCSReqDelayed", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.frcPowXTCSReqDelayed, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.frcRateBrkXTMdlCur", "N/s", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.frcRateBrkXTMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.frcYTCSFrontMdlCur", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.frcYTCSFrontMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.frcYTCSRearMdlCur", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.frcYTCSRearMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.sideSlpAglMdlCur", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.sideSlpAglMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.strAglFrontMdlCur", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.strAglFrontMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.strAglFrontReqDelayed", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.strAglFrontReqDelayed, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.strAglRearMdlCur", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.strAglRearMdlCur, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodInterMeas.strAglRearReqDelayed", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chamodInterMeas.strAglRearReqDelayed, DVA_None);

    // - chamodVehDynInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodVehDynInterMeas.sideSlpAglRateMdlPred", "N/s", &veconaProcMem.p_interMeas->chacoInterMeas.chamodVehDynInterMeas.sideSlpAglRateMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodVehDynInterMeas.veloMdlPred", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chamodVehDynInterMeas.veloMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodVehDynInterMeas.veloXVCSMdlPred", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chamodVehDynInterMeas.veloXVCSMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodVehDynInterMeas.veloYVCSMdlPred", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chamodVehDynInterMeas.veloYVCSMdlPred, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chamodVehDynInterMeas.yawRateMdlPred", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chamodVehDynInterMeas.yawRateMdlPred, DVA_None);

    // - chasgeCtrlLatStates
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeCtrlLatStates.veloYVCSInvPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeCtrlLatStates.veloYVCSInvPrv, DVA_None); // invMdlInputs: inputs: v_y_input_k
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeCtrlLatStates.yawRateInvPrv", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeCtrlLatStates.yawRateInvPrv, DVA_None); // invMdlInputs: inputs: Psi_dot_input_k

    // - chasgeInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.accelACCSafetyFiltPrv", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.accelACCSafetyFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.accelXVCSFiltPrv", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.accelXVCSFiltPrv, DVA_None); // invMdlInputs: distStates: a_x_meas_filt
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.accelXVCSInvPrv", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.accelXVCSInvPrv, DVA_None); // invMdlInputs: distStates: delta_a_x
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.accelXVCSMdlErrFiltPrv", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.accelXVCSMdlErrFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.accelXVCSMdlFiltPrv", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.accelXVCSMdlFiltPrv, DVA_None); // invMdlInputs: distStates: a_x_mdl_filt
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.sideSlpAglReqFiltPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.sideSlpAglReqFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.veloYVCSFiltPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.veloYVCSFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.veloYVCSMdlErrFiltPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.veloYVCSMdlErrFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.veloYVCSMdlFiltPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.veloYVCSMdlFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.yawRateFiltPrv", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.yawRateFiltPrv, DVA_None); // invMdlInputs: distStates: psi_dot_meas_filt
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.yawRateMdlErrFiltPrv", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.yawRateMdlErrFiltPrv, DVA_None); // invMdlInputs: distStates: delta_Psi_dot_filt
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasgeInterMeas.yawRateMdlFiltPrv", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasgeInterMeas.yawRateMdlFiltPrv, DVA_None); // invMdlInputs: distStates: psi_dot_mdl_filt
    //DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdlInputs.errors.delta_a_x_filt", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.  accelXVCSInvCur, DVA_None); // invMdlInputs: distStates: delta_a_x_filt
    //DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdlInputs.errors.delta_Psi_dot", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdlInputs.errors.delta_Psi_dot, DVA_None); // invMdlInputs: distStates: delta_Psi_dot

    // - chasipInterMeas
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.accelXVCSMeasFiltPrv", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.accelXVCSMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.frcBrkMeasFiltPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.frcBrkMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.frcPowMeasFiltPrv", "N", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.frcPowMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.sideSlpAglMeasFiltPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.sideSlpAglMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.strAglFrontMeasFiltPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.strAglFrontMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.strAglRearMeasFiltPrv", "rad", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.strAglRearMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.veloMeasFiltPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.veloMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.veloXVCSMeasFiltPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.veloXVCSMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.veloYVCSMdlFiltPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.veloYVCSMdlFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.veloYVCSMeasFiltPrv", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.veloYVCSMeasFiltPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipInterMeas.yawRateMeasFiltPrv", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasipInterMeas.yawRateMeasFiltPrv, DVA_None);

    // - chasipMdlOutDelayed
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipMdlOutDelayed.accelXVCSMdlDelayed", "m/s^2", &veconaProcMem.p_interMeas->chacoInterMeas.chasipMdlOutDelayed.accelXVCSMdlDelayed, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipMdlOutDelayed.veloYVCSMdlDelayed", "m/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasipMdlOutDelayed.veloYVCSMdlDelayed, DVA_None);
    DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInterMeas.chasipMdlOutDelayed.yawRateMdlDelayed", "rad/s", &veconaProcMem.p_interMeas->chacoInterMeas.chasipMdlOutDelayed.yawRateMdlDelayed, DVA_None);

    // p_interMeas...
    DDefUChar(NULL, "VECONA.Debugs.firstCycleDone", "", &veconaProcMem.p_interMeas->firstCycleDone, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.motReqResponse", "", &veconaProcMem.p_interMeas->motReqResponse, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.standstillSecured", "", &veconaProcMem.p_interMeas->standstillSecured, DVA_None);

    // p_interMeas->frcCapabilityFb
    DDef(NULL, "VECONA.Debugs.frcCapabilityFb.frontFrcCapabilityMax", "N", &veconaProcMem.p_interMeas->frcCapabilityFb.frontFrcCapabilityMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.frcCapabilityFb.frontFrcCapabilityMin", "N", &veconaProcMem.p_interMeas->frcCapabilityFb.frontFrcCapabilityMin, DVA_None);
    DDef(NULL, "VECONA.Debugs.frcCapabilityFb.rearFrcCapabilityMax", "N", &veconaProcMem.p_interMeas->frcCapabilityFb.rearFrcCapabilityMax, DVA_None);
    DDef(NULL, "VECONA.Debugs.frcCapabilityFb.rearFrcCapabilityMin", "N", &veconaProcMem.p_interMeas->frcCapabilityFb.rearFrcCapabilityMin, DVA_None);

    // p_interMeas->fsmLat
    DDefUChar(NULL, "VECONA.Debugs.fsmLat.status.curState", "", (uint8_t*)&veconaProcMem.p_interMeas->fsmLat.status.curState, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.fsmLat.status.funcFaultStatus", "", (uint8_t*)&veconaProcMem.p_interMeas->fsmLat.status.funcFaultStatus, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.fsmLat.status.transition2Active", "", &veconaProcMem.p_interMeas->fsmLat.status.transition2Active, DVA_None);

    // p_interMeas->fsmLong
    DDefUChar(NULL, "VECONA.Debugs.fsmLong.status.curState", "", (uint8_t*)&veconaProcMem.p_interMeas->fsmLong.status.curState, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.fsmLong.status.funcFaultStatus", "", (uint8_t*)&veconaProcMem.p_interMeas->fsmLong.status.funcFaultStatus, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.fsmLong.status.transition2Active", "", &veconaProcMem.p_interMeas->fsmLong.status.transition2Active, DVA_None);

    // p_interMeas->ssmHandlingStatus
    DDefUChar(NULL, "VECONA.Debugs.ssmHandlingStatus.ssmReqActive", "", &veconaProcMem.p_interMeas->ssmHandlingStatus.ssmReqActive, DVA_None);

    // p_interMeas->usedActuatorsFb
    DDefUChar(NULL, "VECONA.Debugs.usedActuatorsFb.brkReqActive", "", &veconaProcMem.p_interMeas->usedActuatorsFb.brkReqActive, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.usedActuatorsFb.powReqActive", "", &veconaProcMem.p_interMeas->usedActuatorsFb.powReqActive, DVA_None);

    // p_interMeas->wetcoInterMeas
    // - ctrlStatus
    DDef(NULL, "VECONA.Debugs.wetcoInterMeas.ctrlStatus.sumTrqReqPrv", "Nm", &veconaProcMem.p_interMeas->wetcoInterMeas.ctrlStatus.sumTrqReqPrv, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoInterMeas.ctrlStatus.sumTrqReqUnlimPrv", "Nm", &veconaProcMem.p_interMeas->wetcoInterMeas.ctrlStatus.sumTrqReqUnlimPrv, DVA_None);

    // - wetstmInterMeas
    DDef(NULL, "VECONA.Debugs.wetcoInterMeas.wetstmInterMeas.accelOverrideTimer", "s", &veconaProcMem.p_interMeas->wetcoInterMeas.wetstmInterMeas.accelOverrideTimer, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoInterMeas.wetstmInterMeas.isRampedOutCond", "", &veconaProcMem.p_interMeas->wetcoInterMeas.wetstmInterMeas.isRampedOutCond, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoInterMeas.wetstmInterMeas.ssmHandlingReleaseTimer", "s", &veconaProcMem.p_interMeas->wetcoInterMeas.wetstmInterMeas.ssmHandlingReleaseTimer, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoInterMeas.wetstmInterMeas.ssmHandlingSecureSuccessTimer", "s", &veconaProcMem.p_interMeas->wetcoInterMeas.wetstmInterMeas.ssmHandlingSecureSuccessTimer, DVA_None);
    DDef(NULL, "VECONA.Debugs.wetcoInterMeas.wetstmInterMeas.ssmHandlingSecureTimer", "s", &veconaProcMem.p_interMeas->wetcoInterMeas.wetstmInterMeas.ssmHandlingSecureTimer, DVA_None);
    DDefUChar(NULL, "VECONA.Debugs.wetcoInterMeas.wetstmInterMeas.ssmHandlingState", "", (uint8_t*)&veconaProcMem.p_interMeas->wetcoInterMeas.wetstmInterMeas.ssmHandlingState, DVA_None);

    /*  Old CHACO simulink
        // invMdlInputs
        // - req
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.chacoInter.chasgeCtrlAccelXInter.veloXVCSReq", "m/s", &veconaProcMem.p_inter->chacoInter.chasgeCtrlAccelXInter.veloXVCSReq, DVA_None); // invMdlInputs: req: v_x_req
        // invMdl
        // - vehDyn
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.longDyn.F_air_x_k", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.longDyn.F_air_x_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.longDyn.F_x_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.longDyn.F_x_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.latDyn.F_air_y_k", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.latDyn.F_air_y_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.latDyn.F_y_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.latDyn.F_y_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.yawDyn.F_Y_f_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.yawDyn.F_Y_f_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.yawDyn.F_Y_r_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.yawDyn.F_Y_r_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.yawDyn.Psi_dot_k", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.yawDyn.Psi_dot_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.whlFrcs.F_YT_f_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.whlFrcs.F_YT_f_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.whlFrcs.F_XT_f_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.whlFrcs.F_XT_f_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.whlFrcs.F_YT_r_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.whlFrcs.F_YT_r_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.vehDyn.whlFrcs.F_XT_r_kp1", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.vehDyn.whlFrcs.F_XT_r_kp1, DVA_None);
        // - latWhlDyn
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.latWhlDyn.delta_f_kp1", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.latWhlDyn.delta_f_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.latWhlDyn.delta_r_kp1", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.latWhlDyn.delta_r_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.latWhlDyn.alpha_f_kp1", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.latWhlDyn.alpha_f_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.latWhlDyn.alpha_r_kp1", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.latWhlDyn.alpha_r_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.latWhlDyn.F_YT_f_k", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.latWhlDyn.F_YT_f_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.latWhlDyn.F_YT_r_k", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.latWhlDyn.F_YT_f_k, DVA_None);
        // - longWhlDyn
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.longWhlDyn.TrqEng ", "Nm", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.longWhlDyn.TrqEng, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.longWhlDyn.TrqBrk ", "Nm", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.longWhlDyn.TrqBrk, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.longWhlDyn.F_XT_f_req_k ", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.longWhlDyn.F_XT_f_req_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.longWhlDyn.F_XT_f_k", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.longWhlDyn.F_XT_f_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.longWhlDyn.F_XT_r_req_k", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.longWhlDyn.F_XT_r_req_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.longWhlDyn.F_XT_r_k", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.longWhlDyn.F_XT_r_k, DVA_None);
        // - epsDyn
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.epsDyn.delta_f_k", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.epsDyn.delta_f_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.epsDyn.delta_f_req_k", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.epsDyn.delta_f_req_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.epsDyn.delta_r_k", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.epsDyn.delta_r_k, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.invMdl.epsDyn.delta_r_req_k", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.invMdl.epsDyn.delta_r_req_k, DVA_None);
        // mdl
        // - steerAgls
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.steerAgls.delta_acker", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.steerAgls.delta_acker, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.steerAgls.delta_f", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.steerAgls.delta_f, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.steerAgls.delta_r", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.steerAgls.delta_r, DVA_None);
        // - states
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.alpha_f", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.alpha_f, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.alpha_r", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.alpha_r, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.a_x_kp1", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.a_x_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.a_y_kp1", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.a_y_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.beta_dot_kp1", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.beta_dot_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.beta_kp1", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.beta_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.kappa_kp1", "rad", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.kappa_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.Psi_dot_kp1", "rad/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.Psi_dot_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.Psi_ddot_kp1", "rad/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.Psi_ddot_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.v_kp1", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.v_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.v_x_kp1", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.v_x_kp1, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.v_x_dot", "m/s^2", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.v_x_dot, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.states.v_y_kp1", "m/s", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.states.v_y_kp1, DVA_None);
        // - frcs
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.F_XT_f", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.F_XT_f, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.F_XT_r", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.F_XT_r, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.F_YT_f", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.F_YT_f, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.F_YT_r", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.F_YT_r, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.TrqEng", "Nm", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.TrqEng, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.TrqBrk", "Nm", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.TrqBrk, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.F_XT_f_lim", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.F_XT_f_lim, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.F_XT_r_lim", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.F_XT_r_lim, DVA_None);
        DDef(NULL, "VECONA.Debugs.ChacoDebugs.mdl.frcs.F_XT_lim", "N", &veconaProcMem.p_intraMeas->chacoIntraMeas.mdl.frcs.F_XT_lim, DVA_None);
        // mdl errors
        //DDef(NULL, "VECONA.Debugs.ChacoDebugs.error.Delta_a_x", "m/s^2", &veconaProcMem.p_intraMeas->delta_a_x, DVA_None);
    */
#endif

    ////ALbu
    /*
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_0.x()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[0].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_0.y()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[0].y(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_1.x()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[1].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_1.y()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[1].y(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_2.x()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[2].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_2.y()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[2].y(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_3.x()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[3].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.pos_m_3.y()", "m", &tempEnvModelPort.staticStructures[0].shape.pos_m[3].y(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_0.x()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[0].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_0.y()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[0].y(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_1.x()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[1].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_1.y()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[1].y(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_2.x()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[2].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_2.y()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[2].y(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_3.x()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[3].x(), DVA_None);
    DDefFloat(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.pos_m_3.y()", "m", &tempEnvModelPort.staticStructures[1].shape.pos_m[3].y(), DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.existenceProb_perc", "", &tempEnvModelPort.staticStructures[0].existenceProb_perc, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.existenceProb_perc", "", &tempEnvModelPort.staticStructures[1].existenceProb_perc, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_2.existenceProb_perc", "", &tempEnvModelPort.staticStructures[2].existenceProb_perc, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.readFromNVRAM_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[0].readFromNVRAM_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.readFromNVRAM_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[1].readFromNVRAM_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.refObjID_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[0].refObjID_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.refObjID_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[1].refObjID_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.refObjClass_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[0].refObjClass_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.refObjClass_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[1].refObjClass_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.numValidPoints_nu", "", &tempEnvModelPort.staticStructures[0].shape.numValidPoints_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.numValidPoints_nu", "", &tempEnvModelPort.staticStructures[1].shape.numValidPoints_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.detectingSensors_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[0].detectingSensors_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.detectingSensors_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[1].detectingSensors_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.refObjOrientationTowardsRoad_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[0].refObjOrientationTowardsRoad_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.refObjOrientationTowardsRoad_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[1].refObjOrientationTowardsRoad_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.heightClassConfidence_perc", "", &tempEnvModelPort.staticStructures[0].shape.heightClassConfidence_perc, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.heightClassConfidence_perc", "", &tempEnvModelPort.staticStructures[1].shape.heightClassConfidence_perc, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_0.shape.heightClass_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[0].shape.heightClass_nu, DVA_None);
    DDefUChar(NULL, "AP.tempEnvModelPort.staticStructures_1.shape.heightClass_nu", "", (uint8_t*)&tempEnvModelPort.staticStructures[1].shape.heightClass_nu, DVA_None);
    DDefUInt(NULL, "AP.tempEnvModelPort.numValidStaticObj", "", (unsigned int *)&tempEnvModelPort.numValidStaticObj, DVA_None);
    */

    registerSignalHeaderToDVA("AP.memoryParkingStatusPort.sSigHeader", memoryParkingStatusPort.sSigHeader);
    DDefUChar(NULL, "AP.memoryParkingStatusPort.mpStatus.memoryParkingState", "", (uint8_t*)&memoryParkingStatusPort.mpStatus.memoryParkingState, DVA_IO_In);
    DDefUChar(NULL, "AP.memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu", "", &memoryParkingStatusPort.mpStatus.numStoredMemoryParkingSlots_nu, DVA_IO_In);

    registerSignalHeaderToDVA("AP.perceptionAvailabilityPort.sSigHeader", perceptionAvailabilityPort.sSigHeader);
    DDefUChar(NULL, "AP.perceptionAvailabilityPort.statusEnvModel_nu", "", (uint8_t*)&perceptionAvailabilityPort.statusEnvModel_nu, DVA_IO_In);
    tDDefault *percAvailPrefix = DDefaultCreate("AP.perceptionAvailabilityPort.statusUSSensors_nu");
    for (uint8_t i{ 0U }; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_US_SENSORS_NU; i++) {
        DDefPrefix(percAvailPrefix, "AP.perceptionAvailabilityPort.statusUSSensors_nu_%d", i);
        DDefUChar(percAvailPrefix, "", "", (uint8_t*)&perceptionAvailabilityPort.statusUSSensors_nu[i], DVA_IO_In);
    }
    DDefaultDelete(percAvailPrefix);

#ifndef VARIANT_CUS_ONLY
    mdl_APCtrl_DeclQuants_CollEnvModelPort(MP);
#endif

    //envModelPort
    const unsigned int maxPointsStaticStructure{ ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU };
    DDefUChar(NULL, "AP.envModelPort.overwritePathBeforeFuncActivation_nu", "", &overwritePathBeforeFuncActivation_nu, DVA_IO_In);
    DDefFloat(NULL, "AP.envModelPort.pathBeforeFuncActivation0PoseX_m", "", &pathBeforeFuncActivation0Pose.Pos().x(), DVA_IO_In);
    DDefFloat(NULL, "AP.envModelPort.pathBeforeFuncActivation0PoseY_m", "", &pathBeforeFuncActivation0Pose.Pos().y(), DVA_IO_In);
    DDefFloat(NULL, "AP.envModelPort.pathBeforeFuncActivation0PoseYaw_rad", "", &pathBeforeFuncActivation0Pose.Yaw_rad(), DVA_IO_In);
    tDDefault *envMP = DDefaultCreate("AP.envModelPort.");
    for (unsigned int j = 0U; j < maxNumStaticStructureForDVA; j++) {
        for (unsigned int i = 0U; i < maxPointsStaticStructure; i++) {
            DDefPrefix(envMP, "AP.envModelPort.staticObjects_%d.objShape_m_%d.x", j, i);
            DDefFloat(envMP, "", "m", &envModelPort.staticObjects[j].objShape_m.array[i].x_dir, DVA_None);
            DDefPrefix(envMP, "AP.envModelPort.staticObjects_%d.objShape_m_%d.y", j, i);
            DDefFloat(envMP, "", "m", &envModelPort.staticObjects[j].objShape_m.array[i].y_dir, DVA_None);
        }
        DDefPrefix(envMP, "AP.envModelPort.staticObjects_%d.", j);
#ifndef VARIANT_CUS_ONLY
        DDefUInt(envMP, "refObjID_nu", "", &envModelPort.staticObjects[j].refObjID_nu, DVA_None);
        DDefUChar(envMP, "refObjClass_nu", "", (uint8_t*)&envModelPort.staticObjects[j].refObjClass_nu, DVA_None);
#else
        DDefUShort(envMP, "refObjID_nu", "", &envModelPort.staticObjects[j].refObjID_nu, DVA_None);
        DDefUChar(envMP, "refObjClass_nu", "", &variant_refObjClass_nu, DVA_None);
#endif
        DDefUChar(envMP, "existenceProb_perc", "", &envModelPort.staticObjects[j].existenceProb_perc, DVA_None);
        DDefUShort(envMP, "objAgeInCycles_nu", "", &envModelPort.staticObjects[j].objAgeInCycles_nu, DVA_None);
        DDefUShort(envMP, "objMeasLastUpdateInCycles_nu", "", &envModelPort.staticObjects[j].objMeasLastUpdateInCycles_nu, DVA_None);
        DDefUShort(envMP, "objTrendLastUpdateInCycles_nu", "", &envModelPort.staticObjects[j].objTrendLastUpdateInCycles_nu, DVA_None);
        DDefUChar(envMP, "objTrend_nu", "", (uint8_t*)&envModelPort.staticObjects[j].objTrend_nu, DVA_None);
        DDefUChar(envMP, "readFromNVRAM_nu", "", (uint8_t*)&envModelPort.staticObjects[j].readFromNVRAM_nu, DVA_None);
        DDefUChar(envMP, "objHeightClass_nu", "", (uint8_t*)&envModelPort.staticObjects[j].objHeightClass_nu, DVA_None);
        DDefUChar(envMP, "objHeightClassConfidence_perc", "", &envModelPort.staticObjects[j].objHeightClassConfidence_perc, DVA_None);
    }
    //envModelPort.dynamicObject
    const uint8_t maxPointsDynamicStructure = 4u;
    DDefUChar(NULL, "AP.envModelPort.dynamicObjects_0.existenceProb_perc", "", &envModelPort.dynamicObjects[0].existenceProb_perc, DVA_None);
    DDefUChar(NULL, "AP.envModelPort.dynamicObjects_0.measurementState_nu", "", (uint8_t*)&envModelPort.dynamicObjects[0].measurementState_nu, DVA_None);
    for (unsigned int i = 0; i < maxPointsDynamicStructure; i++) {
        DDefPrefix(envMP, "AP.envModelPort.dynamicObjects_0.objShape_%d.x", i);
        DDefFloat(envMP, "", "m", &envModelPort.dynamicObjects[0].objShape_m.array[i].x_dir, DVA_None);
        DDefPrefix(envMP, "AP.envModelPort.dynamicObjects_0.objShape_%d.y", i);
        DDefFloat(envMP, "", "m", &envModelPort.dynamicObjects[0].objShape_m.array[i].y_dir, DVA_None);
    }
    DDefFloat(NULL, "AP.envModelPort.dynamicObjects_0.vel_mps_0", "m/s", &envModelPort.dynamicObjects[0].vel_mps.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.dynamicObjects_0.vel_mps_1", "m/s", &envModelPort.dynamicObjects[0].vel_mps.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.dynamicObjects_0.accel_mps2_0", "m/s^2", &envModelPort.dynamicObjects[0].accel_mps2.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.dynamicObjects_0.accel_mps2_1", "m/s^2", &envModelPort.dynamicObjects[0].accel_mps2.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.dynamicObjects_0.headingAngle_rad", "rad", &envModelPort.dynamicObjects[0].headingAngle_rad, DVA_None);
#ifndef VARIANT_CUS_ONLY
    DDefUInt(NULL, "AP.envModelPort.dynamicObjects_0.refObjID_nu", "", &envModelPort.dynamicObjects[0].refObjID_nu, DVA_None);
#else
    DDefUShort(NULL, "AP.envModelPort.dynamicObjects_0.refObjID_nu", "", &envModelPort.dynamicObjects[0].refObjID_nu, DVA_None);
#endif
    //envModelPort.resetOriginResult + .egoVehiclePoseForAP
    DDefUChar(NULL, "AP.envModelPort.resetOriginResult.resetCounter_nu", "", &envModelPort.resetOriginResult.resetCounter_nu, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.resetOriginResult.originTransformation.pos_x_m", "m", &envModelPort.resetOriginResult.originTransformation.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.resetOriginResult.originTransformation.pos_y_m", "m", &envModelPort.resetOriginResult.originTransformation.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.resetOriginResult.originTransformation.yaw_rad", "rad", &envModelPort.resetOriginResult.originTransformation.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.egoVehiclePoseForAP.pos_x_m", "m", &envModelPort.egoVehiclePoseForAP.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.egoVehiclePoseForAP.pos_y_m", "m", &envModelPort.egoVehiclePoseForAP.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPort.egoVehiclePoseForAP.yaw_rad", "rad", &envModelPort.egoVehiclePoseForAP.yaw_rad, DVA_None);
    registerSignalHeaderToDVA("AP.envModelPort.sSigHeader", envModelPort.sSigHeader);
    //envModelPortCMOrigin
    DDefUChar(NULL, "AP.envModelPortCMOrigin.overwritePathBeforeFuncActivation_nu", "", &overwritePathBeforeFuncActivation_nu, DVA_IO_In);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.pathBeforeFuncActivation0PoseX_m", "", &pathBeforeFuncActivation0Pose.Pos().x(), DVA_IO_In);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.pathBeforeFuncActivation0PoseY_m", "", &pathBeforeFuncActivation0Pose.Pos().y(), DVA_IO_In);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.pathBeforeFuncActivation0PoseYaw_rad", "", &pathBeforeFuncActivation0Pose.Yaw_rad(), DVA_IO_In);
    tDDefault *envMPCMOrigin = DDefaultCreate("AP.envModelPortCMOrigin.");
    // In case of CUS-only, number of static object points for visualization is larger than maxPointsStaticStructure.
    const unsigned int numPointsStaticStructureForTclgeo{ std::max(16U, maxPointsStaticStructure) };
    for (unsigned int j = 0; j < maxNumStaticStructureForDVA; j++) {
        for (unsigned int i = 0; i < numPointsStaticStructureForTclgeo; i++) {
            DDefPrefix(envMPCMOrigin, "AP.envModelPortCMOrigin.staticObjects_%d.objShape_m_%d.x", j, i);
            DDefFloat(envMPCMOrigin, "", "m", (i < maxPointsStaticStructure ? &envModelPortCMOrigin.staticObjects[j].objShape_m.array[i].x_dir : &dummyStaticObjectShapePoint), DVA_None);
            DDefPrefix(envMPCMOrigin, "AP.envModelPortCMOrigin.staticObjects_%d.objShape_m_%d.y", j, i);
            DDefFloat(envMPCMOrigin, "", "m", (i < maxPointsStaticStructure ? &envModelPortCMOrigin.staticObjects[j].objShape_m.array[i].y_dir : &dummyStaticObjectShapePoint), DVA_None);
        }
        DDefPrefix(envMPCMOrigin, "AP.envModelPortCMOrigin.staticObjects_%d.", j);
#ifndef VARIANT_CUS_ONLY
        DDefUInt(envMPCMOrigin, "refObjID_nu", "", &envModelPortCMOrigin.staticObjects[j].refObjID_nu, DVA_None);
        DDefUChar(envMPCMOrigin, "refObjClass_nu", "", (uint8_t*)&envModelPortCMOrigin.staticObjects[j].refObjClass_nu, DVA_None);
#else
        DDefUShort(envMPCMOrigin, "refObjID_nu", "", &envModelPortCMOrigin.staticObjects[j].refObjID_nu, DVA_None);
        DDefUChar(envMPCMOrigin, "refObjClass_nu", "", &variant_refObjClass_nu, DVA_None);
#endif
        DDefUChar(envMPCMOrigin, "existenceProb_perc", "", &envModelPortCMOrigin.staticObjects[j].existenceProb_perc, DVA_None);
        DDefUShort(envMPCMOrigin, "objAgeInCycles_nu", "", &envModelPortCMOrigin.staticObjects[j].objAgeInCycles_nu, DVA_None);
        DDefUShort(envMPCMOrigin, "objMeasLastUpdateInCycles_nu", "", &envModelPortCMOrigin.staticObjects[j].objMeasLastUpdateInCycles_nu, DVA_None);
        DDefUShort(envMPCMOrigin, "objTrendLastUpdateInCycles_nu", "", &envModelPortCMOrigin.staticObjects[j].objTrendLastUpdateInCycles_nu, DVA_None);
        DDefUChar(envMPCMOrigin, "objTrend_nu", "", (uint8_t*)&envModelPortCMOrigin.staticObjects[j].objTrend_nu, DVA_None);
        DDefUChar(envMPCMOrigin, "readFromNVRAM_nu", "", (uint8_t*)&envModelPortCMOrigin.staticObjects[j].readFromNVRAM_nu, DVA_None);
        DDefUChar(envMPCMOrigin, "objShape.numValidPoints_nu", "", &numValidPointsStaticStructure[j], DVA_None);
        DDefUChar(envMPCMOrigin, "objHeightClass_nu", "", (uint8_t*)&envModelPortCMOrigin.staticObjects[j].objHeightClass_nu, DVA_None);
        DDefUChar(envMPCMOrigin, "objHeightClassConfidence_perc", "", &envModelPortCMOrigin.staticObjects[j].objHeightClassConfidence_perc, DVA_None);
    }

    //envModelPortCMOrigin.dynamicObject
    DDefUChar(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.existenceProb_perc", "", &envModelPortCMOrigin.dynamicObjects[0].existenceProb_perc, DVA_None);
    DDefUChar(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.measurementState_nu", "", (uint8_t*)&envModelPortCMOrigin.dynamicObjects[0].measurementState_nu, DVA_None);
    for (unsigned int i = 0; i < maxPointsDynamicStructure; i++) {
        DDefPrefix(envMPCMOrigin, "AP.envModelPortCMOrigin.dynamicObjects_0.objShape_%d.x", i);
        DDefFloat(envMPCMOrigin, "", "m", &envModelPortCMOrigin.dynamicObjects[0].objShape_m.array[i].x_dir, DVA_None);
        DDefPrefix(envMPCMOrigin, "AP.envModelPortCMOrigin.dynamicObjects_0.objShape_%d.y", i);
        DDefFloat(envMPCMOrigin, "", "m", &envModelPortCMOrigin.dynamicObjects[0].objShape_m.array[i].y_dir, DVA_None);
    }
    DDefFloat(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.vel_mps_0", "m/s", &envModelPortCMOrigin.dynamicObjects[0].vel_mps.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.vel_mps_1", "m/s", &envModelPortCMOrigin.dynamicObjects[0].vel_mps.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.accel_mps2_0", "m/s^2", &envModelPortCMOrigin.dynamicObjects[0].accel_mps2.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.accel_mps2_1", "m/s^2", &envModelPortCMOrigin.dynamicObjects[0].accel_mps2.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.headingAngle_rad", "rad", &envModelPortCMOrigin.dynamicObjects[0].headingAngle_rad, DVA_None);
#ifndef VARIANT_CUS_ONLY
    DDefUInt(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.refObjID_nu", "", &envModelPortCMOrigin.dynamicObjects[0].refObjID_nu, DVA_None);
#else
    DDefUShort(NULL, "AP.envModelPortCMOrigin.dynamicObjects_0.refObjID_nu", "", &envModelPortCMOrigin.dynamicObjects[0].refObjID_nu, DVA_None);
#endif
    //envModelPortCMOrigin.resetOriginResult + .egoVehiclePoseForAP
    DDefUChar(NULL, "AP.envModelPortCMOrigin.resetOriginResult.resetCounter_nu", "", &envModelPortCMOrigin.resetOriginResult.resetCounter_nu, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.resetOriginResult.originTransformation.pos_x_m", "m", &envModelPortCMOrigin.resetOriginResult.originTransformation.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.resetOriginResult.originTransformation.pos_y_m", "m", &envModelPortCMOrigin.resetOriginResult.originTransformation.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.resetOriginResult.originTransformation.yaw_rad", "rad", &envModelPortCMOrigin.resetOriginResult.originTransformation.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.egoVehiclePoseForAP.pos_x_m", "m", &envModelPortCMOrigin.egoVehiclePoseForAP.x_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.egoVehiclePoseForAP.pos_y_m", "m", &envModelPortCMOrigin.egoVehiclePoseForAP.y_dir, DVA_None);
    DDefFloat(NULL, "AP.envModelPortCMOrigin.egoVehiclePoseForAP.yaw_rad", "rad", &envModelPortCMOrigin.egoVehiclePoseForAP.yaw_rad, DVA_None);
    registerSignalHeaderToDVA("AP.envModelPortCMOrigin.sSigHeader", envModelPortCMOrigin.sSigHeader);

    //parkingBoxPort -> parkingBoxes
    tDDefault *dfParkingOnLeftSide = DDefaultCreate("AP.parkingOnLeftSide_nu.");
    for (unsigned int iPB = 0; iPB < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; ++iPB) {
        DDefPrefix(dfParkingOnLeftSide, "AP.parkingOnLeftSide_nu_%d", iPB);
        DDefUChar(dfParkingOnLeftSide, "", "", (uint8_t*)&parkingOnLeftSide_nu[iPB], DVA_None);
    }
    DDefUChar(NULL, "AP.parkingBoxPort.numValidParkingBoxes_nu", "", (uint8_t*)&gParkingBoxPort.numValidParkingBoxes_nu, DVA_None);
    registerSignalHeaderToDVA("AP.parkingBoxPort.sSigHeader", gParkingBoxPort.sSigHeader);
    tDDefault *dfParkingBoxPort = DDefaultCreate("AP.parkingBoxPort.parkingBoxes_%d.");
    for (unsigned int iPB = 0; iPB < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; ++iPB) {

        DDefPrefix(dfParkingBoxPort, "AP.parkingBoxPort.parkingBoxes_%d.", iPB);
        DDefUChar(dfParkingBoxPort, "parkingBoxID_nu", "", (uint8_t*)&gParkingBoxPort.parkingBoxes[iPB].parkingBoxID_nu, DVA_None);
        DDefUChar(dfParkingBoxPort, "existenceProb_perc", "", (uint8_t*)&gParkingBoxPort.parkingBoxes[iPB].existenceProb_perc, DVA_None);
        DDefUChar(dfParkingBoxPort, "parkingScenario_nu", "", (uint8_t*)&gParkingBoxPort.parkingBoxes[iPB].parkingScenario_nu, DVA_None);
        DDefUChar(dfParkingBoxPort, "numValidDelimiters_nu", "", (uint8_t*)&gParkingBoxPort.parkingBoxes[iPB].numValidDelimiters_nu, DVA_None);
        DDefUChar(dfParkingBoxPort, "numVirtualLines_nu", "", (uint8_t*)&gParkingBoxPort.parkingBoxes[iPB].numVirtualLines_nu, DVA_None);

        if (parkingOnLeftSide_nu[iPB] == false) {
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontLeft_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[0].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontLeft_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[0].y_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearLeft_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[1].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearLeft_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[1].y_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearRight_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[2].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearRight_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[2].y_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontRight_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[3].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontRight_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[3].y_dir, DVA_None);
        }
        else {
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearRight_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[0].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearRight_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[0].y_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontRight_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[1].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontRight_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[1].y_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontLeft_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[2].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_FrontLeft_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[2].y_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearLeft_x", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[3].x_dir, DVA_None);
            DDefFloat(dfParkingBoxPort, "slotCoordinates_RearLeft_y", "", &gParkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[3].y_dir, DVA_None);
        }
    }
    //parkingBoxPortCMOrigin -> parkingBoxes
    DDefUChar(NULL, "AP.parkingBoxPortCMOrigin.numValidParkingBoxes_nu", "", (uint8_t*)&parkingBoxPortCMOrigin.numValidParkingBoxes_nu, DVA_None);
    registerSignalHeaderToDVA("AP.parkingBoxPortCMOrigin.sSigHeader", parkingBoxPortCMOrigin.sSigHeader);
    tDDefault *dfParkingBoxCMOrigin = DDefaultCreate("AP.parkingBoxPortCMOrigin.parkingBoxes_");
    for (int iPB = 0; iPB < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; iPB++) {
        DDefPrefix(dfParkingBoxCMOrigin, "AP.parkingBoxPortCMOrigin.parkingBoxes_%d.", iPB);
        DDefUChar(dfParkingBoxCMOrigin, "parkingBoxID_nu", "", (uint8_t*)&parkingBoxPortCMOrigin.parkingBoxes[iPB].parkingBoxID_nu, DVA_None);
        DDefUChar(dfParkingBoxCMOrigin, "existenceProb_perc", "", (uint8_t*)&parkingBoxPortCMOrigin.parkingBoxes[iPB].existenceProb_perc, DVA_None);
        DDefUChar(dfParkingBoxCMOrigin, "parkingScenario_nu", "", (uint8_t*)&parkingBoxPortCMOrigin.parkingBoxes[iPB].parkingScenario_nu, DVA_None);
        //DDefUChar(dfParkingBoxCMOrigin, "parkingScenario_nu", "", (uint8_t*)&parkingBoxPortCMOrigin.parkingBoxes[iPB].parkingScenario_nu, DVA_None);
        DDefUChar(dfParkingBoxCMOrigin, "numValidDelimiters_nu", "", (uint8_t*)&parkingBoxPortCMOrigin.parkingBoxes[iPB].numValidDelimiters_nu, DVA_None);
        DDefUChar(dfParkingBoxCMOrigin, "numVirtualLines_nu", "", (uint8_t*)&parkingBoxPortCMOrigin.parkingBoxes[iPB].numVirtualLines_nu, DVA_None);
        // In case of CUS-only, number of virtual lines for visualization is larger than AP_G_MAX_NUM_VIRTUAL_LINES_NU.
        const unsigned numVirtualLinesForTclgeo{ std::max(8U, static_cast<unsigned>(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU)) };
        for (int iVL = 0; iVL < numVirtualLinesForTclgeo; iVL++) {
            const bool isDummyVirtualLine = iVL >= ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU;
            DDefPrefix(dfParkingBoxCMOrigin, "AP.parkingBoxPortCMOrigin.parkingBoxes_%d.virtualLines_%d.", iPB, iVL);
            DDefFloat(dfParkingBoxCMOrigin, "virtLineVertices_0_x", "m", (isDummyVirtualLine ? &dummyVirtualLinePoint
                : &parkingBoxPortCMOrigin.parkingBoxes[iPB].virtualLines[iVL].virtLineVertices_m.array[0].x_dir), DVA_None);
            DDefFloat(dfParkingBoxCMOrigin, "virtLineVertices_0_y", "m", (isDummyVirtualLine ? &dummyVirtualLinePoint
                : &parkingBoxPortCMOrigin.parkingBoxes[iPB].virtualLines[iVL].virtLineVertices_m.array[0].y_dir), DVA_None);
            DDefFloat(dfParkingBoxCMOrigin, "virtLineVertices_1_x", "m", (isDummyVirtualLine ? &dummyVirtualLinePoint
                : &parkingBoxPortCMOrigin.parkingBoxes[iPB].virtualLines[iVL].virtLineVertices_m.array[1].x_dir), DVA_None);
            DDefFloat(dfParkingBoxCMOrigin, "virtLineVertices_1_y", "m", (isDummyVirtualLine ? &dummyVirtualLinePoint
                : &parkingBoxPortCMOrigin.parkingBoxes[iPB].virtualLines[iVL].virtLineVertices_m.array[1].y_dir), DVA_None);
        }

        DDefPrefix(dfParkingBoxCMOrigin, "AP.parkingBoxPortCMOrigin.parkingBoxes_%d.slotCoordinates_", iPB);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "RearRight_x" : "FrontLeft_x"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[0].x_dir, DVA_None);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "RearRight_y" : "FrontLeft_y"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[0].y_dir, DVA_None);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "FrontRight_x" : "RearLeft_x"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[1].x_dir, DVA_None);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "FrontRight_y" : "RearLeft_y"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[1].y_dir, DVA_None);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "FrontLeft_x" : "RearRight_x"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[2].x_dir, DVA_None);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "FrontLeft_y" : "RearRight_y"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[2].y_dir, DVA_None);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "RearLeft_x" : "FrontRight_x"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[3].x_dir, DVA_None);
        DDefFloat(dfParkingBoxCMOrigin, (parkingOnLeftSide_nu[iPB] ? "RearLeft_y" : "FrontRight_y"), "m", &parkingBoxPortCMOrigin.parkingBoxes[iPB].slotCoordinates_m.array[3].y_dir, DVA_None);
    }
    DDefaultDelete(dfParkingBoxCMOrigin);

    //egoMotionPort
    DDefFloat(NULL, "AP.egoMotionPort.accel_mps2", "m/s^2", &egoMotionPort.accel_mps2, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPort.drivenDistance_m", "m", &egoMotionPort.drivenDistance_m, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPort.frontWheelAngle_rad", "rad", &egoMotionPort.frontWheelAngle_rad, DVA_None);
    DDefChar(NULL, "AP.egoMotionPort.motionState_nu", "", (char *)&egoMotionPort.motionState_nu, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPort.pitch_rad", "rad", &egoMotionPort.pitch_rad, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPort.rearWheelAngle_rad", "rad", &egoMotionPort.rearWheelAngle_rad, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPort.roll_rad", "rad", &egoMotionPort.roll_rad, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPort.vel_mps", "m/s", &egoMotionPort.vel_mps, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPort.yawRate_radps", "rad/s", &egoMotionPort.yawRate_radps, DVA_None);
    registerSignalHeaderToDVA("AP.egoMotionPort.sSigHeader", egoMotionPort.sSigHeader);
    // Modified yaw rate
    DDefFloat(NULL, "yawRateEgoCurMod_rad", "rad/s", &yawRateEgoCurMod_rad, DVA_None);


    //egoMotionPortCM
    DDefFloat(NULL, "AP.egoMotionPortCM.accel_mps2", "m/s^2", &egoMotionPortCM.accel_mps2, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPortCM.drivenDistance_m", "m", &egoMotionPortCM.drivenDistance_m, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPortCM.frontWheelAngle_rad", "rad", &egoMotionPortCM.frontWheelAngle_rad, DVA_None);
    DDefChar(NULL, "AP.egoMotionPortCM.motionState_nu", "", (char *)&egoMotionPortCM.motionState_nu, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPortCM.pitch_rad", "rad", &egoMotionPortCM.pitch_rad, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPortCM.rearWheelAngle_rad", "rad", &egoMotionPortCM.rearWheelAngle_rad, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPortCM.roll_rad", "rad", &egoMotionPortCM.roll_rad, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPortCM.vel_mps", "m/s", &egoMotionPortCM.vel_mps, DVA_None);
    DDefFloat(NULL, "AP.egoMotionPortCM.yawRate_radps", "rad/s", &egoMotionPortCM.yawRate_radps, DVA_None);
    registerSignalHeaderToDVA("AP.egoMotionPortCM.sSigHeader", egoMotionPortCM.sSigHeader);

    //odoInputPort
    DDefUChar(NULL, "AP.odoInputPort.odoExtCtrlPort.resetPoseEstimation_nu", "", (unsigned char*)&odoExtCtrlPort.resetPoseEstimation_nu, DVA_None);
    registerSignalHeaderToDVA("AP.odoInputPort.odoExtCtrlPort.sSigHeader", odoExtCtrlPort.sSigHeader);
    registerSignalHeaderToDVA("AP.odoInputPort.odoSystemTimePort.sSigHeader", systemTimePort.sSigHeader);

    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.gearboxCtrlStatus.gearCur_nu", "", (unsigned char*)&gearboxCtrlStatusPort.gearInformation.gearCur_nu, DVA_None);
    registerSignalHeaderToDVA("AP.odoInputPort.odoSigFcanPort.steerCtrlStatus.sSigHeader", steerCtrlStatusPort.sSigHeader);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.steerCtrlStatus.steeringWheelAngle_rad", "rad", &steerCtrlStatusPort.steeringWheelAngle_rad, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.steerCtrlStatus.steeringWheelAngle_QF_nu", "", (unsigned char*)&steerCtrlStatusPort.steeringWheelAngle_QF_nu, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.steerCtrlStatus.steeringWheelAngleVelocity_radps", "radps", &steerCtrlStatusPort.steeringWheelAngleVelocity_radps, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.steerCtrlStatus.steeringWheelAngleVelocity_QF_nu", "", (unsigned char*)&steerCtrlStatusPort.steeringWheelAngleVelocity_QF_nu, DVA_None);

    // Variable name AP.odoInputPort.odoSigFcanPort.vehDynamics.timestamp_us expected by STET KPI tooling.
    DDefULLong(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.timestamp_us", "us", &vehDynamicsPort.sSigHeader.uiTimeStamp, DVA_None);
    registerSignalHeaderToDVA("AP.odoInputPort.odoSigFcanPort.vehDynamics.sSigHeader", vehDynamicsPort.sSigHeader);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.lateralAcceleration_mps2", "mps2", &vehDynamicsPort.lateralAcceleration_mps2, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.lateralAcceleration_QF_nu", "", (unsigned char*)&vehDynamicsPort.lateralAcceleration_QF_nu, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.longitudinalAcceleration_mps2", "m/s^2", &vehDynamicsPort.longitudinalAcceleration_mps2, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.longitudinalAcceleration_QF_nu", "", (unsigned char*)&vehDynamicsPort.longitudinalAcceleration_QF_nu, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.yawRate_radps", "rad/s", &vehDynamicsPort.yawRate_radps, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.pitchRate_radps", "rad/s", &vehDynamicsPort.pitchRate_radps, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.rollRate_radps", "rad/s", &vehDynamicsPort.rollRate_radps, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.lateralAcceleration_mps2", "m/s^2", &vehDynamicsPort.lateralAcceleration_mps2, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.longitudinalAcceleration_mps2", "m/s^2", &vehDynamicsPort.longitudinalAcceleration_mps2, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.verticalAcceleration_mps2", "m/s^2", &vehDynamicsPort.verticalAcceleration_mps2, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.yawRate_QF_nu", "", (unsigned char*)&vehDynamicsPort.yawRate_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.pitchRate_QF_nu", "", (unsigned char*)&vehDynamicsPort.pitchRate_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.rollRate_QF_nu", "", (unsigned char*)&vehDynamicsPort.rollRate_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.longitudinalAcceleration_QF_nu", "", (unsigned char*)&vehDynamicsPort.longitudinalAcceleration_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.lateralAcceleration_QF_nu", "", (unsigned char*)&vehDynamicsPort.lateralAcceleration_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.vehDynamics.verticalAcceleration_QF_nu", "", (unsigned char*)&vehDynamicsPort.verticalAcceleration_QF_nu, DVA_None);

    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesFL_QF_nu", "", (unsigned char*)&wheelPulsePort.wheelPulsesFL_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesFR_QF_nu", "", (unsigned char*)&wheelPulsePort.wheelPulsesFR_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesRL_QF_nu", "", (unsigned char*)&wheelPulsePort.wheelPulsesRL_QF_nu, DVA_None);
    DDefUChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesRR_QF_nu", "", (unsigned char*)&wheelPulsePort.wheelPulsesRR_QF_nu, DVA_None);
    DDefUShort(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesFL_nu", "", &wheelPulsePort.wheelPulsesFL_nu, DVA_None);
    DDefUShort(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesFR_nu", "", &wheelPulsePort.wheelPulsesFR_nu, DVA_None);
    DDefUShort(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesRL_nu", "", &wheelPulsePort.wheelPulsesRL_nu, DVA_None);
    DDefUShort(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.wheelPulsesRR_nu", "", &wheelPulsePort.wheelPulsesRR_nu, DVA_None);
    registerSignalHeaderToDVA("AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelPulse.sSigHeader", wheelPulsePort.sSigHeader);

    DDefChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelDrivingDirections.wheelDrivingDirection_FL_nu", "", (char*)&wheelDrivingDirectionsPort.wheelDrivingDirection_FL_nu, DVA_None);
    DDefChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelDrivingDirections.wheelDrivingDirection_FR_nu", "", (char*)&wheelDrivingDirectionsPort.wheelDrivingDirection_FR_nu, DVA_None);
    DDefChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelDrivingDirections.wheelDrivingDirection_RL_nu", "", (char*)&wheelDrivingDirectionsPort.wheelDrivingDirection_RL_nu, DVA_None);
    DDefChar(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelDrivingDirections.wheelDrivingDirection_RR_nu", "", (char*)&wheelDrivingDirectionsPort.wheelDrivingDirection_RR_nu, DVA_None);
    registerSignalHeaderToDVA("AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelDrivingDirections.sSigHeader", wheelDrivingDirectionsPort.sSigHeader);

    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelSpeed.wheelSpeedFL_radps", "rad/s", &wheelSpeedPort.wheelRotSpeedFL_radps, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelSpeed.wheelSpeedFR_radps", "rad/s", &wheelSpeedPort.wheelRotSpeedFR_radps, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelSpeed.wheelSpeedRL_radps", "rad/s", &wheelSpeedPort.wheelRotSpeedRL_radps, DVA_None);
    DDefFloat(NULL, "AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelSpeed.wheelSpeedRR_radps", "rad/s", &wheelSpeedPort.wheelRotSpeedRR_radps, DVA_None);
    registerSignalHeaderToDVA("AP.odoInputPort.odoSigFcanPort.wheelMeasurements.wheelSpeed.sSigHeader", wheelSpeedPort.sSigHeader);

    //gearboxCtrlStatusPort
    DDefUChar(NULL, "AP.gearboxCtrlStatusPort.gearInformation.gearCur_nu", "", (unsigned char*)&gearboxCtrlStatusPort.gearInformation.gearCur_nu, DVA_None);

    //odoEstimationPort
    DDefFloat(NULL, "AP.odoEstimationPort.longiVelocity_mps", "m/s", &odoEstimationOutputPort.odoEstimation.longiVelocity_mps, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.longiAcceleration_mps2", "m/s^2", &odoEstimationOutputPort.odoEstimation.longiAcceleration_mps2, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.lateralAcceleration_mps2", "m/s^2", &odoEstimationOutputPort.odoEstimation.lateralAcceleration_mps2, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.xPosition_m", "m", &odoEstimationOutputPort.odoEstimation.xPosition_m, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.yPosition_m", "m", &odoEstimationOutputPort.odoEstimation.yPosition_m, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.xVelocity_mps", "m/s", &odoEstimationOutputPort.odoEstimation.xVelocity_mps, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.yVelocity_mps", "m/s", &odoEstimationOutputPort.odoEstimation.yVelocity_mps, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.yawAngle_rad", "rad", &odoEstimationOutputPort.odoEstimation.yawAngle_rad, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.yawRate_radps", "rad/s", &odoEstimationOutputPort.odoEstimation.yawRate_radps, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.steerAngFrontAxle_rad", "rad", &odoEstimationOutputPort.odoEstimation.steerAngFrontAxle_rad, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.drivenDistance_m", "m", &odoEstimationOutputPort.odoEstimation.drivenDistance_m, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.rollAngle_rad", "rad", &odoEstimationOutputPort.odoEstimation.rollAngle_rad, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.rollRate_radps", "rad/s", &odoEstimationOutputPort.odoEstimation.rollRate_radps, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.pitchAngle_rad", "rad", &odoEstimationOutputPort.odoEstimation.pitchAngle_rad, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPort.pitchRate_radps", "rad/s", &odoEstimationOutputPort.odoEstimation.pitchRate_radps, DVA_IO_In);
    DDefUChar(NULL, "AP.odoEstimationPort.motionStatus_nu", "", (uint8_t*)&odoEstimationOutputPort.odoEstimation.motionStatus_nu, DVA_None);
    DDefChar(NULL, "AP.odoEstimationPort.drivingDirection_nu", "", (char*)&odoEstimationOutputPort.odoEstimation.drivingDirection_nu, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.sideSlipAngle_rad", "rad", &odoEstimationOutputPort.odoEstimation.sideSlipAngle_rad, DVA_None);
    registerSignalHeaderToDVA("AP.odoEstimationPort.sSigHeader", odoEstimationOutputPort.sSigHeader);

    //OdoPersDataPort
    registerSignalHeaderToDVA("AP.OdoPersDataPort.sSigHeader", odoPersistentDataPort.sSigHeader);
    DDefFloat(NULL, "AP.OdoPersDataPort.SensorOffsetLateralAcceleration_mps", "", &odoPersistentDataPort.LatAcc.ZeroAccel, DVA_None);
    DDefFloat(NULL, "AP.OdoPersDataPort.SensorOffsetYawRate_radps", "", &odoPersistentDataPort.YwRate.ZeroRate, DVA_None);
    DDefFloat(NULL, "AP.OdoPersDataPort.SensorOffsetSteeringWheelAngle_rad", "", &odoPersistentDataPort.StWhlAng.ZeroAngle, DVA_None);

    //odoEstimationPortCM
    DDefFloat(NULL, "AP.odoEstimationPortCM.xPosition_m", "m", &odoEstimationPortCM.xPosition_m, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPortCM.yPosition_m", "m", &odoEstimationPortCM.yPosition_m, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPortCM.yawAngle_rad", "rad", &odoEstimationPortCM.yawAngle_rad, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPortCM.longiVelocity_mps", "m/s", &odoEstimationPortCM.longiVelocity_mps, DVA_IO_In);
    DDefFloat(NULL, "AP.odoEstimationPortCM.steerAngFrontAxle_rad", "rad", &odoEstimationPortCM.steerAngFrontAxle_rad, DVA_IO_In);
    registerSignalHeaderToDVA("AP.odoEstimationPortCM.sSigHeader", odoEstimationOutputPortCM.sSigHeader);

    //odoDebugPort
    DDefFloat(NULL, "AP.odoDebugPort.odoAccelByWheel_mps2", "m/s^2", &odoDebugPort.odoAccelByWheel_mps2, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.odoCmRefxPosition_m", "m", &odoEstimationPortCM.xPosition_m, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.odoCmRefyPosition_m", "m", &odoEstimationPortCM.yPosition_m, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.odoCmRefyawAngEgoRaCur_rad", "rad", &odoEstimationPortCM.yawAngle_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.steeringWheelAngleOffset_rad", "rad", &odoDebugPort.steeringWheelAngleOffset_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.steeringWheelAngleOffsetCalibrated_rad", "rad", &odoDebugPort.steeringWheelAngleOffsetCalibrated_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.yawRateOffset_radps", "rad/s", &odoDebugPort.yawRateOffset_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.yawRateOffsetCalibrated_radps", "rad/s", &odoDebugPort.yawRateOffsetCalibrated_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distancePerStepRA_m", "m", &odoDebugPort.distancePerStepRA_m, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distancePerStepFA_m", "m", &odoDebugPort.distancePerStepFA_m, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distancePerStepWheel_FL_m", "m", &odoDebugPort.distancePerStepWheel_m[0], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distancePerStepWheel_FR_m", "m", &odoDebugPort.distancePerStepWheel_m[1], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distancePerStepWheel_RL_m", "m", &odoDebugPort.distancePerStepWheel_m[2], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distancePerStepWheel_RR_m", "m", &odoDebugPort.distancePerStepWheel_m[3], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimVelRearAxleFLFR_mps", "m/s", &odoDebugPort.estimVelRearAxleFLFR_mps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimVelRearAxleFLRL_mps", "m/s", &odoDebugPort.estimVelRearAxleFLRL_mps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimVelRearAxleFLRR_mps", "m/s", &odoDebugPort.estimVelRearAxleFLRR_mps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimVelRearAxleFRRL_mps", "m/s", &odoDebugPort.estimVelRearAxleFRRL_mps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimVelRearAxleFLFR_mps", "m/s", &odoDebugPort.estimVelRearAxleFRRR_mps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimVelRearAxleRLRR_mps", "m/s", &odoDebugPort.estimVelRearAxleRLRR_mps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimYawRateFLFR_radps", "rad/s", &odoDebugPort.estimYawRateFLFR_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimYawRateFLRL_radps", "rad/s", &odoDebugPort.estimYawRateFLRL_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimYawRateFLRR_radps", "rad/s", &odoDebugPort.estimYawRateFLRR_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimYawRateFRRL_radps", "rad/s", &odoDebugPort.estimYawRateFRRL_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimYawRateFRRR_radps", "rad/s", &odoDebugPort.estimYawRateFRRR_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.estimYawRateRLRR_radps", "rad/s", &odoDebugPort.estimYawRateRLRR_radps, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.odoAccelByWheel_mps2", "rad", &odoDebugPort.steerAngleCTR, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.steerAngleFL", "rad", &odoDebugPort.steerAngleFL, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.steerAngleFR", "rad", &odoDebugPort.steerAngleFR, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.yawAnglePerStepAckermann_rad", "rad", &odoDebugPort.yawAnglePerStepAckermann_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.yawAnglePerStepYawRate_rad", "rad", &odoDebugPort.yawAnglePerStepYawRate_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.yawAnglePerStepStandstillSteer_rad", "rad", &odoDebugPort.yawAnglePerStepStandstillSteer_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.yawAnglePerStepWhlDistFront_rad", "rad", &odoDebugPort.yawAnglePerStepWhlDistFront_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.yawAnglePerStepWhlDistRear_rad", "rad", &odoDebugPort.yawAnglePerStepWhlDistRear_rad, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distanceFA_m", "m", &odoDebugPort.distanceFA_m, DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.distanceRA_m", "m", &odoDebugPort.distanceRA_m, DVA_None);
    //DDefFloat(NULL, "AP.odoDebugPort.odoKalman_x_m", "m", &odoDebugPort.odoKalman_x_m, DVA_None);
    //DDefFloat(NULL, "AP.odoDebugPort.odoKalman_y_m", "m", &odoDebugPort.odoKalman_y_m, DVA_None);
    //DDefFloat(NULL, "AP.odoDebugPort.odoKalman_psi_rad", "rad", &odoDebugPort.odoKalman_psi_rad, DVA_None);
    DDefChar(NULL, "AP.odoDebugPort.drivingDirectionGear_nu", "", (char*)&odoDebugPort.drivingDirectionGear_nu, DVA_None);
    DDefChar(NULL, "AP.odoDebugPort.ticsIncrement_FL_nu", "", (char*)&odoDebugPort.ticsIncrement_4_nu[0], DVA_None);
    DDefChar(NULL, "AP.odoDebugPort.ticsIncrement_FR_nu", "", (char*)&odoDebugPort.ticsIncrement_4_nu[1], DVA_None);
    DDefChar(NULL, "AP.odoDebugPort.ticsIncrement_RL_nu", "", (char*)&odoDebugPort.ticsIncrement_4_nu[2], DVA_None);
    DDefChar(NULL, "AP.odoDebugPort.ticsIncrement_RR_nu", "", (char*)&odoDebugPort.ticsIncrement_4_nu[3], DVA_None);

    DDefInt(NULL, "AP.odoDebugPort.debugInt_00", "", &odoDebugPort.debugInt[0], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_01", "", &odoDebugPort.debugInt[1], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_02", "", &odoDebugPort.debugInt[2], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_03", "", &odoDebugPort.debugInt[3], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_04", "", &odoDebugPort.debugInt[4], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_05", "", &odoDebugPort.debugInt[5], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_06", "", &odoDebugPort.debugInt[6], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_07", "", &odoDebugPort.debugInt[7], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_08", "", &odoDebugPort.debugInt[8], DVA_None);
    DDefInt(NULL, "AP.odoDebugPort.debugInt_09", "", &odoDebugPort.debugInt[9], DVA_None);

    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_00", "", &odoDebugPort.debugFloat[0], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_01", "", &odoDebugPort.debugFloat[1], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_02", "", &odoDebugPort.debugFloat[2], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_03", "", &odoDebugPort.debugFloat[3], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_04", "", &odoDebugPort.debugFloat[4], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_05", "", &odoDebugPort.debugFloat[5], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_06", "", &odoDebugPort.debugFloat[6], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_07", "", &odoDebugPort.debugFloat[7], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_08", "", &odoDebugPort.debugFloat[8], DVA_None);
    DDefFloat(NULL, "AP.odoDebugPort.debugFloat_09", "", &odoDebugPort.debugFloat[9], DVA_None);
    registerSignalHeaderToDVA("AP.odoDebugPort.sSigHeader", odoDebugPort.sSigHeader);

    // suspension
    pSuspFL = DDictGetEntry("Car.DampFL.l_kin");  // handles to Carmaker quantities
    pSuspFR = DDictGetEntry("Car.DampFR.l_kin");
    pSuspRL = DDictGetEntry("Car.DampRL.l_kin");
    pSuspRR = DDictGetEntry("Car.DampRR.l_kin");

    DDefFloat(NULL, "AP.Susp.groundHeightCalcFL.pos", "m", &groundHeightCalcFL, DVA_None);
    DDefFloat(NULL, "AP.Susp.groundHeightCalcFR.pos", "m", &groundHeightCalcFR, DVA_None);
    DDefFloat(NULL, "AP.Susp.groundHeightCalcRL.pos", "m", &groundHeightCalcRL, DVA_None);
    DDefFloat(NULL, "AP.Susp.groundHeightCalcRR.pos", "m", &groundHeightCalcRR, DVA_None);
    DDefFloat(NULL, "AP.Susp.pitch_rad", "rad", &cmSuspPitch_rad, DVA_None);
    DDefFloat(NULL, "AP.Susp.roll_rad", "rad", &cmSuspRoll_rad, DVA_None);
    DDefFloat(NULL, "AP.Susp.height_m", "m", &cmSuspHeight_m, DVA_None);

    DDefFloat(NULL, "AP.odoEstimationPort.suspHeight_m", "m", &odoEstimationOutputPort.odoEstimation.suspHeight_m, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.suspPitch_rad", "rad", &odoEstimationOutputPort.odoEstimation.suspPitch_rad, DVA_None);
    DDefFloat(NULL, "AP.odoEstimationPort.suspRoll_rad", "rad", &odoEstimationOutputPort.odoEstimation.suspRoll_rad, DVA_None);

    //tceEstimationPort
    registerSignalHeaderToDVA("AP.tceEstimationPort.sSigHeader", tceEstimationPort.sSigHeader);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircFL_m", "m", &tceEstimationPort.tireCircFL_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircFR_m", "m", &tceEstimationPort.tireCircFR_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircRL_m", "m", &tceEstimationPort.tireCircRL_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircRR_m", "m", &tceEstimationPort.tireCircRR_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircStdvFL_m", "m", &tceEstimationPort.tireCircStdvFL_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircStdvFR_m", "m", &tceEstimationPort.tireCircStdvFR_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircStdvRL_m", "m", &tceEstimationPort.tireCircStdvRL_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.tireCircStdvRR_m", "m", &tceEstimationPort.tireCircStdvRR_m, DVA_None);
    DDefUChar(NULL, "AP.tceEstimationPort.tireCircFLValid", "", (uint8_t*)&tceEstimationPort.tireCircFLValid, DVA_None);
    DDefUChar(NULL, "AP.tceEstimationPort.tireCircFRValid", "", (uint8_t*)&tceEstimationPort.tireCircFRValid, DVA_None);
    DDefUChar(NULL, "AP.tceEstimationPort.tireCircRLValid", "", (uint8_t*)&tceEstimationPort.tireCircRLValid, DVA_None);
    DDefUChar(NULL, "AP.tceEstimationPort.tireCircRRValid", "", (uint8_t*)&tceEstimationPort.tireCircRRValid, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.rearTrackWidth_m", "m", &tceEstimationPort.rearTrackWidth_m, DVA_None);
    DDefFloat(NULL, "AP.tceEstimationPort.rearTrackWidthStdv_m", "m", &tceEstimationPort.rearTrackWidthStdv_m, DVA_None);
    DDefUChar(NULL, "AP.tceEstimationPort.rearTrackWidthValid", "", (uint8_t*)&tceEstimationPort.rearTrackWidthValid, DVA_None);

    //tcePersDataPort
    registerSignalHeaderToDVA("AP.tcePersDataPort.sSigHeader", tcePersDataPort.sSigHeader);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircFL_0p1mm", "", &tcePersDataPort.tireCircFL_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircFR_0p1mm", "", &tcePersDataPort.tireCircFR_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircRL_0p1mm", "", &tcePersDataPort.tireCircRL_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircRR_0p1mm", "", &tcePersDataPort.tireCircRR_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircStdvFL_0p1mm", "", &tcePersDataPort.tireCircStdvFL_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircStdvFR_0p1mm", "", &tcePersDataPort.tireCircStdvFR_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircStdvRL_0p1mm", "", &tcePersDataPort.tireCircStdvRL_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.tireCircStdvRR_0p1mm", "", &tcePersDataPort.tireCircStdvRR_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.rearTrackWidth_0p1mm", "", &tcePersDataPort.rearTrackWidth_0p1mm, DVA_None);
    DDefUShort(NULL, "AP.tcePersDataPort.rearTrackWidthStdv_0p1mm", "", &tcePersDataPort.rearTrackWidthStdv_0p1mm, DVA_None);
    DDefUChar(NULL, "AP.tcePersDataPort.dataChanged", "", (uint8_t*)&tcePersDataPort.dataChanged, DVA_None);

    //tceDebugPort
    registerSignalHeaderToDVA("AP.tceDebugPort.sSigHeader", tceDebugPort.sSigHeader);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_00", "", &tceDebugPort.debugInt[0], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_01", "", &tceDebugPort.debugInt[1], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_02", "", &tceDebugPort.debugInt[2], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_03", "", &tceDebugPort.debugInt[3], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_04", "", &tceDebugPort.debugInt[4], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_05", "", &tceDebugPort.debugInt[5], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_06", "", &tceDebugPort.debugInt[6], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_07", "", &tceDebugPort.debugInt[7], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_08", "", &tceDebugPort.debugInt[8], DVA_None);
    DDefInt(NULL, "AP.tceDebugPort.debugInt_09", "", &tceDebugPort.debugInt[9], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_00", "", &tceDebugPort.debugFloat[0], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_01", "", &tceDebugPort.debugFloat[1], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_02", "", &tceDebugPort.debugFloat[2], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_03", "", &tceDebugPort.debugFloat[3], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_04", "", &tceDebugPort.debugFloat[4], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_05", "", &tceDebugPort.debugFloat[5], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_06", "", &tceDebugPort.debugFloat[6], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_07", "", &tceDebugPort.debugFloat[7], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_08", "", &tceDebugPort.debugFloat[8], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.debugFloat_09", "", &tceDebugPort.debugFloat[9], DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.tireCircInternalFL_m", "m", &tceDebugPort.tireCircInternalFL_m, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.tireCircInternalFR_m", "m", &tceDebugPort.tireCircInternalFR_m, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.tireCircInternalRL_m", "m", &tceDebugPort.tireCircInternalRL_m, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.tireCircInternalRR_m", "m", &tceDebugPort.tireCircInternalRR_m, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.gpsDisplacements_m", "m", &tceDebugPort.gpsDisplacements_m, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.vehDrivenDistanceWhl_m", "m", &tceDebugPort.vehDrivenDistanceWhl_m, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.vehDrivenDistanceGPS_m", "m", &tceDebugPort.vehDrivenDistanceGPS_m, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.sumWheelRotations_rad", "rad", &tceDebugPort.sumWheelRotations_rad, DVA_None);
    DDefFloat(NULL, "AP.tceDebugPort.earthEllipsoidRadius_m", "m", &tceDebugPort.earthEllipsoidRadius_m, DVA_None);
    DDefUChar(NULL, "AP.tceDebugPort.gpsUpdateFlag", "", (uint8_t*)&tceDebugPort.gpsUpdateFlag, DVA_None);

    //Dummy change for CI Trigger
    //planningCtrlCommands
    registerSignalHeaderToDVA("AP.slotCtrlPort.sSigHeader", slotCtrlPort.sSigHeader);
    DDefUChar(NULL, "AP.planningCtrlPort.apChosenTargetPoseId_nu", "", &slotCtrlPort.planningCtrlCommands.apChosenTargetPoseId_nu, DVA_None);
    DDefUChar(NULL, "AP.planningCtrlPort.apStates", "", (uint8_t*)&slotCtrlPort.planningCtrlCommands.apState, DVA_None);
    DDefUChar(NULL, "AP.planningCtrlPort.raStates", "", (uint8_t*)&slotCtrlPort.planningCtrlCommands.raState, DVA_None);
    DDefUChar(NULL, "AP.planningCtrlPort.mpStates", "", (uint8_t*)&slotCtrlPort.planningCtrlCommands.mpState, DVA_None);
    DDefUChar(NULL, "AP.planningCtrlPort.gpState", "", (uint8_t*)&slotCtrlPort.planningCtrlCommands.gpState, DVA_None);
    DDefUChar(NULL, "AP.planningCtrlPort.rmState", "", (uint8_t*)&slotCtrlPort.planningCtrlCommands.rmState, DVA_None);
    DDefUChar(NULL, "AP.planningCtrlPort.tpState", "", (uint8_t*)&slotCtrlPort.planningCtrlCommands.tpState, DVA_None);
    DDefUChar(NULL, "AP.resetOriginRequestPort.resetCounter_nu", "", (uint8_t*)&slotCtrlPort.resetOriginRequestPort.resetCounter_nu, DVA_None);
    DDefUChar(NULL, "AP.resetOriginRequestPort.resetOrigin_nu", "", (uint8_t*)&slotCtrlPort.resetOriginRequestPort.resetOrigin_nu, DVA_None);
    registerSignalHeaderToDVA("AP.CtrlCommandPort.sSigHeader", gCtrlCommandPort.sSigHeader);
    DDefUChar(NULL, "AP.CtrlCommandPort.ppcParkingMode_nu", "", (uint8_t*)&gCtrlCommandPort.ppcParkingMode_nu, DVA_None);
    DDefUChar(NULL, "AP.CtrlCommandPort.stdRequest_nu", "", (uint8_t*)&gCtrlCommandPort.stdRequest_nu, DVA_None);
    DDefUChar(NULL, "AP.CtrlCommandPort.selectedTargetPoseId_nu", "", &gCtrlCommandPort.selectedTargetPoseId_nu, DVA_None);

    //PARKSMCoreStatusPort
    registerSignalHeaderToDVA("AP.PARKSMCoreStatusPort.sSigHeader", gPARKSMCoreStatusPort.sSigHeader);
    DDefUChar(NULL, "AP.PARKSMCoreStatusPort.parksmCoreState_nu", "", (uint8_t*)&gPARKSMCoreStatusPort.parksmCoreState_nu, DVA_None);
    DDefUChar(NULL, "AP.PARKSMCoreStatusPort.coreStopReason_nu", "", (uint8_t*)&gPARKSMCoreStatusPort.coreStopReason_nu, DVA_None);

    //trajCtrlRequestPort
    registerSignalHeaderToDVA("AP.trajCtrlRequestPort.sSigHeader", trajCtrlRequestPort.sSigHeader);
    DDefUChar(NULL, "AP.trajCtrlRequestPort.drivingModeReq_nu", "", (uint8_t*)&trajCtrlRequestPort.drivingModeReq_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlRequestPort.trajCtrlActive_nu", "", (uint8_t*)&trajCtrlRequestPort.trajCtrlActive_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlRequestPort.remoteReq_nu", "", (uint8_t*)&trajCtrlRequestPort.emergencyBrakeRequest, DVA_None);

    //laCtrlRequestPort
    registerSignalHeaderToDVA("AP.laCtrlRequestPort.sSigHeader", laCtrlRequestPort.sSigHeader);
    DDefUChar(NULL, "AP.laCtrlRequestPort.activateLaCtrl", "", (uint8_t*)&laCtrlRequestPort.activateLaCtrl, DVA_None);
    DDefUChar(NULL, "AP.laCtrlRequestPort.laCtrlRequestType", "", (uint8_t*)&laCtrlRequestPort.laCtrlRequestType, DVA_None);
    DDefFloat(NULL, "AP.laCtrlRequestPort.curvatureReq_1pm", "1/m", &laCtrlRequestPort.curvatureReq_1pm, DVA_None);
    DDefFloat(NULL, "AP.laCtrlRequestPort.steerAngReqFront_rad", "rad", &laCtrlRequestPort.steerAngReqFront_rad, DVA_None);
    DDefFloat(NULL, "AP.laCtrlRequestPort.steerAngReqRear_rad", "rad", &laCtrlRequestPort.steerAngReqRear_rad, DVA_None);
    DDefFloat(NULL, "AP.laCtrlRequestPort.steerTorqueReq_Nm", "Nm", &laCtrlRequestPort.steerTorqueReq_Nm, DVA_None);

    //loCtrlRequestPort
    registerSignalHeaderToDVA("AP.loCtrlRequestPort.sSigHeader", loCtrlRequestPort.sSigHeader);
    DDefUChar(NULL, "AP.loCtrlRequestPort.activateLoCtrl", "", (uint8_t*)&loCtrlRequestPort.activateLoCtrl, DVA_None);
    DDefUChar(NULL, "AP.loCtrlRequestPort.loCtrlRequestType", "", (uint8_t*)&loCtrlRequestPort.loCtrlRequestType, DVA_None);
    DDefUChar(NULL, "AP.loCtrlRequestPort.comfortStopRequest", "", (uint8_t*)&loCtrlRequestPort.comfortStopRequest, DVA_None);
    DDefUChar(NULL, "AP.loCtrlRequestPort.secureInStandstill", "", (uint8_t*)&loCtrlRequestPort.secureInStandstill, DVA_None);
    DDefFloat(NULL, "AP.loCtrlRequestPort.accelerationReq_mps2.requestedValue", "m/s^2", &loCtrlRequestPort.accelerationReq_mps2.requestedValue, DVA_None);
    DDefUChar(NULL, "AP.loCtrlRequestPort.accelerationReq_mps2.drivingDirRequest", "", (uint8_t*)&loCtrlRequestPort.accelerationReq_mps2.drivingDirRequest, DVA_None);
    DDefFloat(NULL, "AP.loCtrlRequestPort.distanceToStopReq_m.requestedValue", "m", &loCtrlRequestPort.distanceToStopReq_m.requestedValue, DVA_None);
    DDefUChar(NULL, "AP.loCtrlRequestPort.distanceToStopReq_m.drivingDirRequest", "", (uint8_t*)&loCtrlRequestPort.distanceToStopReq_m.drivingDirRequest, DVA_None);
    DDefFloat(NULL, "AP.loCtrlRequestPort.velocityReq_mps.requestedValue", "m/s", &loCtrlRequestPort.velocityReq_mps.requestedValue, DVA_None);
    DDefUChar(NULL, "AP.loCtrlRequestPort.velocityReq_mps.drivingDirRequest", "", (uint8_t*)&loCtrlRequestPort.velocityReq_mps.drivingDirRequest, DVA_None);

    //trajRequestPort
    registerSignalHeaderToDVA("AP.trajRequestPort.sSigHeader", trajRequestPort.sSigHeader);
    DDefUChar(NULL, "AP.trajRequestPort.trajType_nu", "", (uint8_t*)&trajRequestPort.trajType_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.numValidCtrlPoints_nu", "", &trajRequestPort.numValidCtrlPoints_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.drivingForwardReq_nu", "", (uint8_t*)&trajRequestPort.drivingForwardReq_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.trajValid_nu", "", (uint8_t*)&trajRequestPort.trajValid_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.newSegmentStarted_nu", "", (uint8_t*)&trajRequestPort.newSegmentStarted_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.isLastSegment_nu", "", (uint8_t*)&trajRequestPort.isLastSegment_nu, DVA_None);
    DDefFloat(NULL, "AP.trajRequestPort.drivingResistance_0.distance_m", "m", &trajRequestPort.drivingResistance[0].distance_m, DVA_None);
    DDefFloat(NULL, "AP.trajRequestPort.drivingResistance_1.distance_m", "m", &trajRequestPort.drivingResistance[1].distance_m, DVA_None);
    DDefFloat(NULL, "AP.trajRequestPort.drivingResistance_2.distance_m", "m", &trajRequestPort.drivingResistance[2].distance_m, DVA_None);
    DDefFloat(NULL, "AP.trajRequestPort.drivingResistance_3.distance_m", "m", &trajRequestPort.drivingResistance[3].distance_m, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.drivingResistance_0.type_nu", "", (uint8_t*)&trajRequestPort.drivingResistance[0].type_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.drivingResistance_1.type_nu", "", (uint8_t*)&trajRequestPort.drivingResistance[1].type_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.drivingResistance_2.type_nu", "", (uint8_t*)&trajRequestPort.drivingResistance[2].type_nu, DVA_None);
    DDefUChar(NULL, "AP.trajRequestPort.drivingResistance_3.type_nu", "", (uint8_t*)&trajRequestPort.drivingResistance[3].type_nu, DVA_None);

    tDDefault *trajRequestplannedTrajDefault = DDefaultCreate("AP.trajRequestPort.plannedTraj");

    for (unsigned int i = 0; i < mf_manager::MF_MANAGER_Consts::AP_M_MAX_NUM_TRAJ_CTRL_POINTS; i++) {
        DDefPrefix(trajRequestplannedTrajDefault, "AP.trajRequestPort.plannedTraj_%d.xTrajRAReq_m", i);
        DDefFloat(trajRequestplannedTrajDefault, "", "m", &trajRequestPort.plannedTraj[i].xTrajRAReq_m, DVA_None);
        DDefPrefix(trajRequestplannedTrajDefault, "AP.trajRequestPort.plannedTraj_%d.yTrajRAReq_m", i);
        DDefFloat(trajRequestplannedTrajDefault, "", "m", &trajRequestPort.plannedTraj[i].yTrajRAReq_m, DVA_None);
        DDefPrefix(trajRequestplannedTrajDefault, "AP.trajRequestPort.plannedTraj_%d.crvRARReq_1pm", i);
        DDefFloat(trajRequestplannedTrajDefault, "", "1/m", &trajRequestPort.plannedTraj[i].crvRAReq_1pm, DVA_None);
        DDefPrefix(trajRequestplannedTrajDefault, "AP.trajRequestPort.plannedTraj_%d.velocityLimitReq_mps", i);
        DDefFloat(trajRequestplannedTrajDefault, "", "m/s", &trajRequestPort.plannedTraj[i].velocityLimitReq_mps, DVA_None);
        DDefPrefix(trajRequestplannedTrajDefault, "AP.trajRequestPort.plannedTraj_%d.distanceToStopReq_m", i);
        DDefFloat(trajRequestplannedTrajDefault, "", "m", &trajRequestPort.plannedTraj[i].distanceToStopReq_m, DVA_None);
        DDefPrefix(trajRequestplannedTrajDefault, "AP.trajRequestPort.plannedTraj_%d.yawReq_rad", i);
        DDefFloat(trajRequestplannedTrajDefault, "", "rad", &trajRequestPort.plannedTraj[i].yawReq_rad, DVA_None);
    }

    //trajCtrlDebugPort
    DDefUChar(NULL, "AP.trajCtrlDebugPort.free1", "", (uint8_t*)&trajCtrlDebugPort.free1, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.curvaturePreviewed_1pm", "1/m", &trajCtrlDebugPort.curvaturePreviewed_1pm, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.currentDeviation_m", "m", &trajCtrlDebugPort.currentDeviation_m, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.currentTrajectoryIndex_nu", "", &trajCtrlDebugPort.currentTrajectoryIndex_nu, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.curvatureInterpolTraj_1pm", "1/m", &trajCtrlDebugPort.curvatureInterpolTraj_1pm, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m", "m", &trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.driverInterventionDetected_nu", "", (uint8_t*)&trajCtrlDebugPort.driverInterventionDetected_nu, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.filteredSteerIntervention_nu", "", &trajCtrlDebugPort.filteredSteerIntervention_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.finalVehicleStateReached_nu", "", (uint8_t*)&trajCtrlDebugPort.finalVehicleStateReached_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.gearCorrect_nu", "", (uint8_t*)&trajCtrlDebugPort.gearCorrect_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.free2", "", (uint8_t*)&trajCtrlDebugPort.free2, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.driverInterventionDetected_nu", "", (uint8_t*)&trajCtrlDebugPort.driverInterventionDetected_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.standstillSteeringDesired_nu", "", (uint8_t*)&trajCtrlDebugPort.standstillSteeringDesired_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.comfStandstillSteeringExtrapolationMode_nu", "", &trajCtrlDebugPort.comfStandstillSteeringExtrapolationMode_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.distanceControlFinished_nu", "", (uint8_t*)&trajCtrlDebugPort.distanceControlFinished_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.outsideTrajectoryEnd_nu", "", (uint8_t*)&trajCtrlDebugPort.outsideTrajectoryEnd_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.outsideTrajectoryStart_nu", "", (uint8_t*)&trajCtrlDebugPort.outsideTrajectoryStart_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.comfStandStillSteeringMode_nu", "", &trajCtrlDebugPort.comfStandStillSteeringMode_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.rateAndAccelerationLimitatMode", "", &trajCtrlDebugPort.rateAndAccelerationLimitatMode, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.rawSteerAngleRequestMode_nu", "", &trajCtrlDebugPort.rawSteerAngleRequestMode_nu, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.pathControlRequestMode_nu", "", &trajCtrlDebugPort.pathControlRequestMode_nu, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.orientationError_rad", "rad", &trajCtrlDebugPort.orientationError_rad, DVA_None);
    DDefUChar(NULL, "AP.trajCtrlDebugPort.standstillHoldCur_nu", "", (uint8_t*)&trajCtrlDebugPort.standstillHoldCur_nu, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.steerAngReqCurvature_rad", "rad", &trajCtrlDebugPort.steerAngReqCurvature_rad, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.steerAngReqLateralDeviation_rad", "rad", &trajCtrlDebugPort.steerAngReqLateralDeviation_rad, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.steerAngReqRaw_rad", "rad", &trajCtrlDebugPort.steerAngReqRaw_rad, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.steerAngReqYawDeviation_rad", "rad", &trajCtrlDebugPort.steerAngReqYawDeviation_rad, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.trajIntermediateValueRaw_perc", "", &trajCtrlDebugPort.trajIntermediateValueRaw_perc, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.velocityLimitReqInterpolTraj_mps", "m/s", &trajCtrlDebugPort.velocityLimitReqInterpolTraj_mps, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.xInterpolTraj_m", "m", &trajCtrlDebugPort.xInterpolTraj_m, DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.yInterpolTraj_m", "m", &trajCtrlDebugPort.yInterpolTraj_m, DVA_None);

    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_00", "", &trajCtrlDebugPort.debugInt[0], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_01", "", &trajCtrlDebugPort.debugInt[1], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_02", "", &trajCtrlDebugPort.debugInt[2], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_03", "", &trajCtrlDebugPort.debugInt[3], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_04", "", &trajCtrlDebugPort.debugInt[4], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_05", "", &trajCtrlDebugPort.debugInt[5], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_06", "", &trajCtrlDebugPort.debugInt[6], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_07", "", &trajCtrlDebugPort.debugInt[7], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_08", "", &trajCtrlDebugPort.debugInt[8], DVA_None);
    DDefInt(NULL, "AP.trajCtrlDebugPort.debugInt_09", "", &trajCtrlDebugPort.debugInt[9], DVA_None);

    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_00", "rad", &trajCtrlDebugPort.debugFloat[0], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_01", "rad", &trajCtrlDebugPort.debugFloat[1], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_02", "rad", &trajCtrlDebugPort.debugFloat[2], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_03", "", &trajCtrlDebugPort.debugFloat[3], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_04", "", &trajCtrlDebugPort.debugFloat[4], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_05", "", &trajCtrlDebugPort.debugFloat[5], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_06", "", &trajCtrlDebugPort.debugFloat[6], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_07", "", &trajCtrlDebugPort.debugFloat[7], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_08", "", &trajCtrlDebugPort.debugFloat[8], DVA_None);
    DDefFloat(NULL, "AP.trajCtrlDebugPort.debugFloat_09", "", &trajCtrlDebugPort.debugFloat[9], DVA_None);
    registerSignalHeaderToDVA("AP.trajCtrlDebugPort.sSigHeader", trajCtrlDebugPort.sSigHeader);

    //mfControlStatusPort
    DDefUChar(NULL, "AP.mfControlStatusPort.driverSteerIntervDetected_nu", "", (uint8_t*)&mfControlStatusPort.driverSteerIntervDetected_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.numUsedCtrlPoints_nu", "", (uint8_t*)&mfControlStatusPort.numUsedCtrlPoints_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.lateralControlFailed_nu", "", (uint8_t*)&mfControlStatusPort.lateralControlFailed_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.lateralControlFinished_nu", "", (uint8_t*)&mfControlStatusPort.lateralControlFinished_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.lateralPathControlFailed_nu", "", (uint8_t*)&mfControlStatusPort.lateralPathControlFailed_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.longitudinalPathControlFailed_nu", "", (uint8_t*)&mfControlStatusPort.longitudinalPathControlFailed_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.longitudinalControlFailed_nu", "", (uint8_t*)&mfControlStatusPort.longitudinalControlFailed_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.longitudinalControlFinished_nu", "", (uint8_t*)&mfControlStatusPort.longitudinalControlFinished_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.vehStandstillHold_nu", "", (uint8_t*)&mfControlStatusPort.vehStandstillHold_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.vehStandstillSecured_nu", "", (uint8_t*)&mfControlStatusPort.vehStandstillSecured_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.ladmcHandshakeFailedStatus_nu", "", (uint8_t*)&mfControlStatusPort.ladmcHandshakeFailedStatus_nu, DVA_None);
    DDefUChar(NULL, "AP.mfControlStatusPort.lodmcHandshakeFailedStatus_nu", "", (uint8_t*)&mfControlStatusPort.lodmcHandshakeFailedStatus_nu, DVA_None);
    registerSignalHeaderToDVA("AP.mfControlStatusPort.sSigHeader", mfControlStatusPort.sSigHeader);

    //trjplaDebugPort
    registerSignalHeaderToDVA("AP.trajPlanDebugPort.sSigHeader", trjplaDebugPort.sSigHeader);
    DDefUChar(NULL, "AP.trajPlanDebugPort.mTrajPlanState", "", &trjplaDebugPort.mTrajPlanState, DVA_None);
    DDefUChar(NULL, "AP.trajPlanDebugPort.mStateEntry_nu", "", (uint8_t*)&trjplaDebugPort.mStateEntry_nu, DVA_None);
    DDefUChar(NULL, "AP.trajPlanDebugPort.mReplanSuccessful_nu", "", (uint8_t*)&trjplaDebugPort.mReplanSuccessful_nu, DVA_None);
    DDefUChar(NULL, "AP.trajPlanDebugPort.mNumOfReplanCalls", "", &trjplaDebugPort.mNumOfReplanCalls, DVA_None);
    DDefUShort(NULL, "AP.trajPlanDebugPort.numValidPoses_nu", "", &trjplaVisuPort.numValidPoses_nu, DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_00", "", &trjplaDebugPort.debugInt[0], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_01", "", &trjplaDebugPort.debugInt[1], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_02", "", &trjplaDebugPort.debugInt[2], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_03", "", &trjplaDebugPort.debugInt[3], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_04", "", &trjplaDebugPort.debugInt[4], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_05", "", &trjplaDebugPort.debugInt[5], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_06", "", &trjplaDebugPort.debugInt[6], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_07", "", &trjplaDebugPort.debugInt[7], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_08", "", &trjplaDebugPort.debugInt[8], DVA_None);
    DDefInt(NULL, "AP.trajPlanDebugPort.debugInt_09", "", &trjplaDebugPort.debugInt[9], DVA_None);
    DDefFloat(NULL, "AP.trajPlanDebugPort.debugFloat_00", "", &trjplaDebugPort.debugFloat[0], DVA_None);

    //uspWrapperInput
    DDefUInt(NULL, "AP.UsDrvDetectionList.uiVersionNumber", "", &usDrvDetectionListCopy.uiVersionNumber, DVA_None);
    registerSignalHeaderToDVA("AP.UsDrvDetectionList.sSigHeader", usDrvDetectionListCopy.sSigHeader);
    DDefUShort(NULL, "AP.UsDrvDetectionList.numDetections", "", &usDrvDetectionListCopy.numDetections, DVA_None);

    tDDefault *usData = DDefaultCreate("AP.UsDrvDetectionList.");
    for (uint8_t iSens{ 0U }; iSens < us_drv::US_DRV_Consts::US_DRV_MAX_NUM_SENSORS; ++iSens) {
        DDefPrefix(usData, "AP.UsDrvDetectionList.sensorState[%d]", iSens);
        DDefUChar(usData, "", "", (uint8*)&(usDrvDetectionListCopy.sensorState[iSens]), DVA_None);
    }

    for (uint16_t idx_usData = 0; idx_usData < us_drv::US_DRV_Consts::US_DRV_MAX_NUM_DETECTIONS; idx_usData++) {
        DDefPrefix(usData, "AP.UsDrvDetectionList.detections[%d].", idx_usData);
        DDefUChar(usData, "sensorId", "", &usDrvDetectionListCopy.detections[idx_usData].sensorId, DVA_None);
        DDefUChar(usData, "detectionType", "", (uint8*)&(usDrvDetectionListCopy.detections[idx_usData].detectionType), DVA_None);
        DDefUShort(usData, "amplitude", "", &usDrvDetectionListCopy.detections[idx_usData].amplitude, DVA_None);
        DDefChar(usData, "phaseDerivative", "", (char*)&usDrvDetectionListCopy.detections[idx_usData].phaseDerivative, DVA_None);
        DDefUChar(usData, "syncCnt_ms", "ms", &usDrvDetectionListCopy.detections[idx_usData].syncCnt_ms, DVA_None);
        DDefUShort(usData, "sensorTimestamp_us", "us", &usDrvDetectionListCopy.detections[idx_usData].sensorTimestamp_us, DVA_None);
        DDefInt(usData, "relEcuTimestamp_us", "us", &usDrvDetectionListCopy.detections[idx_usData].relEcuTimestamp_us, DVA_None);
    }
    DDefaultDelete(usData);

    //uspWrapperOutput
    DDefUInt(NULL, "AP.uspPointListOutput.uiVersionNumber", "", &uspPointListOutput.uiVersionNumber, DVA_None);
    registerSignalHeaderToDVA("AP.uspPointListOutput.sSigHeader", uspPointListOutput.sSigHeader);
    DDefUInt(NULL, "AP.uspPointListOutput.numberOfPoints", "", &uspPointListOutput.numberOfPoints, DVA_None);
    tDDefault *pointListPrefix = DDefaultCreate("AP.uspPointListOutput.points");
    for (uint8_t iP = 0U; iP < us_processing::US_PROCESSING_Consts::US_PROCESSING_MAX_NUM_POINTS; ++iP) {
        DDefPrefix(pointListPrefix, "AP.uspPointListOutput.points[%d].", iP);
        DDefFloat(pointListPrefix, "directionVariance", "m", &uspPointListOutput.points[iP].directionVariance, DVA_None);
        DDefFloat(pointListPrefix, "direction", "m", &uspPointListOutput.points[iP].direction, DVA_None);
        DDefFloat(pointListPrefix, "heightConfidence", "", &uspPointListOutput.points[iP].heightConfidence, DVA_None);
        DDefUInt(pointListPrefix, "pointCountInTrack", "", &uspPointListOutput.points[iP].pointCountInTrack, DVA_None);
        DDefFloat(pointListPrefix, "probabilityHigh", "", &uspPointListOutput.points[iP].probabilityHigh, DVA_None);
        DDefFloat(pointListPrefix, "rawMeasRange_m", "m", &uspPointListOutput.points[iP].rawMeasRange_m, DVA_None);
        DDefFloat(pointListPrefix, "sensorDirection", "rad", &uspPointListOutput.points[iP].sensorDirection, DVA_None);
        DDefUInt(pointListPrefix, "sensorMask", "", &uspPointListOutput.points[iP].sensorMask, DVA_None);
        DDefULLong(pointListPrefix, "timestamp_us", "us", &uspPointListOutput.points[iP].timestamp_us, DVA_None);
        DDefFloat(pointListPrefix, "trackCurvature", "", &uspPointListOutput.points[iP].trackCurvature, DVA_None);
        DDefUInt(pointListPrefix, "trackId", "", &uspPointListOutput.points[iP].trackId, DVA_None);
        DDefFloat(pointListPrefix, "varTrackCurvature", "", &uspPointListOutput.points[iP].varTrackCurvature, DVA_None);
        DDefFloat(pointListPrefix, "varXposition_m", "m", &uspPointListOutput.points[iP].varXposition_m, DVA_None);
        DDefFloat(pointListPrefix, "varYposition_m", "m", &uspPointListOutput.points[iP].varYposition_m, DVA_None);
        DDefFloat(pointListPrefix, "xPosition_m", "m", &uspPointListOutput.points[iP].xPosition_m, DVA_None);
        DDefFloat(pointListPrefix, "xSensorPos_m", "m", &uspPointListOutput.points[iP].xSensorPos_m, DVA_None);
        DDefFloat(pointListPrefix, "xyPositionCovar_m", "m", &uspPointListOutput.points[iP].xyPositionCovar_m, DVA_None);
        DDefFloat(pointListPrefix, "yPosition_m", "m", &uspPointListOutput.points[iP].yPosition_m, DVA_None);
        DDefFloat(pointListPrefix, "ySensorPos_m", "m", &uspPointListOutput.points[iP].ySensorPos_m, DVA_None);
    }
    DDefaultDelete(pointListPrefix);
    DDefUInt(NULL, "AP.usFilteredEchoOutput.uiVersionNumber", "", &usFilteredEchoOutput.uiVersionNumber, DVA_None);
    DDefUShort(NULL, "AP.usFilteredEchoOutput.numEchoes", "", &usFilteredEchoOutput.numEchoes, DVA_None);
    registerSignalHeaderToDVA("AP.usFilteredEchoOutput.sSigHeader", usFilteredEchoOutput.sSigHeader);

    tDDefault *usFilteredPrefix = DDefaultCreate("AP.usFilteredEchoOutput");

    for (uint16_t iE = 0U; iE < us_processing::US_PROCESSING_Consts::US_PROCESSING_MAX_NUM_ECHOES; ++iE) {
        DDefPrefix(usFilteredPrefix, "AP.usFilteredEchoOutput.echoes[%d].", iE);
        DDefUChar(usFilteredPrefix, "rxSensorId", "", &usFilteredEchoOutput.echoes[iE].rxSensorId, DVA_None);
        DDefUChar(usFilteredPrefix, "txSensorId", "", &usFilteredEchoOutput.echoes[iE].txSensorId, DVA_None);
        DDefUShort(usFilteredPrefix, "trackId", "", &usFilteredEchoOutput.echoes[iE].trackId, DVA_None);
        DDefInt(usFilteredPrefix, "relEcuTimestamp_us", "us", &usFilteredEchoOutput.echoes[iE].relEcuTimestamp_us, DVA_None);
        DDefUShort(usFilteredPrefix, "tof_us", "us", &usFilteredEchoOutput.echoes[iE].tof_us, DVA_None);
        DDefUShort(usFilteredPrefix, "amplitude", "", &usFilteredEchoOutput.echoes[iE].amplitude, DVA_None);
        DDefChar(usFilteredPrefix, "phaseDerivative", "", (char*)&usFilteredEchoOutput.echoes[iE].phaseDerivative, DVA_None);
        DDefUChar(usFilteredPrefix, "codingConfidence", "", &usFilteredEchoOutput.echoes[iE].codingConfidence, DVA_None);
        DDefUChar(usFilteredPrefix, "timeConfidence", "", &usFilteredEchoOutput.echoes[iE].timeConfidence, DVA_None);
    }
    DDefaultDelete(usFilteredPrefix);

    tDDefault* uspDistListPrefix = DDefaultCreate("AP.uspDistanceList.");
    registerSignalHeaderToDVA("AP.uspDistanceList.sSigHeader", uspDistListOutput.sSigHeader);
    DDefUChar(uspDistListPrefix, "numberOfSensors", "", &uspDistListOutput.numberOfSensors, DVA_None);
    for (unsigned iSen{ 0U }; iSen < us_processing::US_PROCESSING_Consts::US_PROCESSING_MAX_NUM_SENSORS; ++iSen) {
        DDefPrefix(uspDistListPrefix, "AP.uspDistanceList.distdir_m_%d", iSen);
        DDefFloat(uspDistListPrefix, "", "m", &(uspDistListOutput.distdir_m[iSen]), DVA_None);
        DDefPrefix(uspDistListPrefix, "AP.uspDistanceList.distDirQuality_%d", iSen);
        DDefFloat(uspDistListPrefix, "", "", &(uspDistListOutput.distDirQuality[iSen]), DVA_None);
        DDefPrefix(uspDistListPrefix, "AP.uspDistanceList.distCross_m_%d", iSen);
        DDefFloat(uspDistListPrefix, "", "m", &(uspDistListOutput.distCross_m[iSen]), DVA_None);
        DDefPrefix(uspDistListPrefix, "AP.uspDistanceList.distCrossQuality_%d", iSen);
        DDefFloat(uspDistListPrefix, "", "", &(uspDistListOutput.distCrossQuality[iSen]), DVA_None);
    }
    DDefaultDelete(uspDistListPrefix);

    tDDefault* uspDiagOutputPrefix = DDefaultCreate("AP.uspDiagOutput.");
    registerSignalHeaderToDVA("AP.uspDiagOutput.sSigHeader", uspDiagOutput.sSigHeader);
    for (unsigned iSen{ 0U }; iSen < us_processing::US_PROCESSING_Consts::US_PROCESSING_MAX_NUM_SENSORS; ++iSen) {
        DDefPrefix(uspDiagOutputPrefix, "AP.uspDiagOutput.sensorStatus_%d", iSen);
        DDefUChar(uspDiagOutputPrefix, "", "", (uint8_t*)&(uspDiagOutput.sensorStatus[iSen]), DVA_None);
    }
    DDefaultDelete(uspDiagOutputPrefix);

    registerSignalHeaderToDVA("AP.usEmDebugOutputPort.sSigHeader", usEmDebugPort.sSigHeader);
    tDDefault* usEmDebugOutputPrefix = DDefaultCreate("AP.usEmDebugOutputPort.");
    DDefUShort(usEmDebugOutputPrefix, "numberOfPoints", "", &usEmDebugPort.numberOfPoints, DVA_None);
    DDefUInt(usEmDebugOutputPrefix, "cycleCounter", "", &usEmDebugPort.cycleCounter, DVA_None);
    for (unsigned iPt{ 0U }; iPt < us_em::US_EM_Consts::US_EM_MAX_NUM_MAP_POINTS; ++iPt) {
        DDefPrefix(usEmDebugOutputPrefix, "AP.usEmDebugOutputPort.usEmPointList_%d.", iPt);
        DDefFloat(usEmDebugOutputPrefix, "pointPos.xPosition_m", "m", &(usEmDebugPort.usEmPointList[iPt].pointPos.xPosition_m), DVA_None);
        DDefFloat(usEmDebugOutputPrefix, "pointPos.yPosition_m", "m", &(usEmDebugPort.usEmPointList[iPt].pointPos.yPosition_m), DVA_None);
        DDefFloat(usEmDebugOutputPrefix, "probHigh", "", &(usEmDebugPort.usEmPointList[iPt].probHigh), DVA_None);
    }
    for (unsigned iV{ 0U }; iV < 4U; ++iV) {
        DDefPrefix(usEmDebugOutputPrefix, "AP.usEmDebugOutputPort.vehicleBoundingBoxes_%d.", iV);
        DDefFloat(usEmDebugOutputPrefix, "xPosition_m", "m", &(usEmDebugPort.vehicleBoundingBoxes[iV].xPosition_m), DVA_None);
        DDefFloat(usEmDebugOutputPrefix, "yPosition_m", "m", &(usEmDebugPort.vehicleBoundingBoxes[iV].yPosition_m), DVA_None);
    }
    for (unsigned iObj{ 0U }; iObj < us_em::US_EM_Consts::US_EM_MAX_NUM_STATIC_OBJ; ++iObj) {
        DDefPrefix(usEmDebugOutputPrefix, "AP.usEmDebugOutputPort.decayData_%d.", iObj);
        DDefFloat(usEmDebugOutputPrefix, "objDecFactors", "", &(usEmDebugPort.decayData[iObj].objDecFactors), DVA_None);
        DDefFloat(usEmDebugOutputPrefix, "alphaProj", "", &(usEmDebugPort.decayData[iObj].alphaProj), DVA_None);
        DDefUChar(usEmDebugOutputPrefix, "sensorId", "", &(usEmDebugPort.decayData[iObj].sensorId), DVA_None);
        DDefUChar(usEmDebugOutputPrefix, "visibility", "", &(usEmDebugPort.decayData[iObj].visibility), DVA_None);
    }
    DDefaultDelete(usEmDebugOutputPrefix);

    registerSignalHeaderToDVA("AP.usEmEnvModelPort.sSigHeader", usEnvModelPort.sSigHeader);
    tDDefault* usEmEnvModePrefix = DDefaultCreate("AP.usEmEnvModelPort.");
    DDefUInt(usEmEnvModePrefix, "uiVersionNumber", "", &usEnvModelPort.uiVersionNumber, DVA_None);
    DDefUChar(usEmEnvModePrefix, "numberOfStaticObjects", "", &usEnvModelPort.numberOfStaticObjects_u8, DVA_None);
    DDefUChar(usEmEnvModePrefix, "numberOfDynamicObjects", "", &usEnvModelPort.numberOfDynamicObjects_u8, DVA_None);
    DDefUChar(usEmEnvModePrefix, "firstStatObjOutDetZoneIdx", "", &usEnvModelPort.firstStatObjOutDetZoneIdx_u8, DVA_None);
    DDefUChar(usEmEnvModePrefix, "firstDynObjOutDetZoneIdx", "", &usEnvModelPort.firstDynObjOutDetZoneIdx_u8, DVA_None);
    for (unsigned iObjDyn{ 0U }; iObjDyn < us_em::US_EM_Consts::US_EM_MAX_NUM_DYN_OBJ; ++iObjDyn) {
        DDefPrefix(usEmEnvModePrefix, "AP.usEmEnvModelPort.dynamicObjects_%d.", iObjDyn);
        DDefUChar(usEmEnvModePrefix, "existenceProb_perc", "", &usEnvModelPort.dynamicObjects[iObjDyn].existenceProb_perc, DVA_None);
        DDefUInt(usEmEnvModePrefix, "objShape_m.actualSize", "m", &usEnvModelPort.dynamicObjects[iObjDyn].objShape_m.actualSize, DVA_None);
        DDefFloat(usEmEnvModePrefix, "vel_mps.x_dir", "mps", &usEnvModelPort.dynamicObjects[iObjDyn].vel_mps.x_dir, DVA_None);
        DDefFloat(usEmEnvModePrefix, "vel_mps.y_dir", "mps", &usEnvModelPort.dynamicObjects[iObjDyn].vel_mps.y_dir, DVA_None);
        DDefFloat(usEmEnvModePrefix, "accel_mps2.x_dir", "mps2", &usEnvModelPort.dynamicObjects[iObjDyn].accel_mps2.x_dir, DVA_None);
        DDefFloat(usEmEnvModePrefix, "accel_mps2.y_dir", "mps2", &usEnvModelPort.dynamicObjects[iObjDyn].accel_mps2.y_dir, DVA_None);
        DDefFloat(usEmEnvModePrefix, "headingAngle_rad", "rad", &usEnvModelPort.dynamicObjects[iObjDyn].headingAngle_rad, DVA_None);
        DDefUShort(usEmEnvModePrefix, "refObjID", "", &usEnvModelPort.dynamicObjects[iObjDyn].refObjID_nu, DVA_None);
        DDefUChar(usEmEnvModePrefix, "measurementState", "", (uint8_t*)&usEnvModelPort.dynamicObjects[iObjDyn].measurementState_nu, DVA_None);
        for (unsigned iObjDyn_iPt{ 0U }; iObjDyn_iPt < us_em::US_EM_Consts::US_EM_MAX_NUM_DYN_OBJ_PTS; ++iObjDyn_iPt) {
            DDefPrefix(usEmEnvModePrefix, "AP.usEmEnvModelPort.dynamicObjects_%d.objShape_m.array_%d.", iObjDyn, iObjDyn_iPt);
            DDefFloat(usEmEnvModePrefix, "x_dir", "m", &usEnvModelPort.dynamicObjects[iObjDyn].objShape_m.array[iObjDyn_iPt].x_dir, DVA_None);
            DDefFloat(usEmEnvModePrefix, "y_dir", "m", &usEnvModelPort.dynamicObjects[iObjDyn].objShape_m.array[iObjDyn_iPt].y_dir, DVA_None);
        }
    }
    for (unsigned iObjStatic{ 0U }; iObjStatic < us_em::US_EM_Consts::US_EM_MAX_NUM_STATIC_OBJ; ++iObjStatic) {
        DDefPrefix(usEmEnvModePrefix, "AP.usEmEnvModelPort.staticObjects_%d.", iObjStatic);
        DDefUShort(usEmEnvModePrefix, "refObjID", "", &usEnvModelPort.staticObjects[iObjStatic].refObjID_nu, DVA_None);
        DDefUChar(usEmEnvModePrefix, "existenceProb_perc", "", &usEnvModelPort.staticObjects[iObjStatic].existenceProb_perc, DVA_None);
        DDefUShort(usEmEnvModePrefix, "objAgeInCycles", "", &usEnvModelPort.staticObjects[iObjStatic].objAgeInCycles_nu, DVA_None);
        DDefUShort(usEmEnvModePrefix, "objMeasLastUpdateInCycles", "", &usEnvModelPort.staticObjects[iObjStatic].objMeasLastUpdateInCycles_nu, DVA_None);
        DDefUShort(usEmEnvModePrefix, "objTrendLastUpdateInCycles", "", &usEnvModelPort.staticObjects[iObjStatic].objTrendLastUpdateInCycles_nu, DVA_None);
        DDefUChar(usEmEnvModePrefix, "objTrend", "", (uint8_t*)&usEnvModelPort.staticObjects[iObjStatic].objTrend_nu, DVA_None);
        DDefUChar(usEmEnvModePrefix, "readFromNVRAM", "", (uint8_t*)&usEnvModelPort.staticObjects[iObjStatic].readFromNVRAM_nu, DVA_None);
        DDefUChar(usEmEnvModePrefix, "objHeightClass", "", (uint8_t*)&usEnvModelPort.staticObjects[iObjStatic].objHeightClass_nu, DVA_None);
        DDefUChar(usEmEnvModePrefix, "objHeightClassConfidence_perc", "", &usEnvModelPort.staticObjects[iObjStatic].objHeightClassConfidence_perc, DVA_None);
        DDefUInt(usEmEnvModePrefix, "objShape_m.actualSize", "m", &usEnvModelPort.staticObjects[iObjStatic].objShape_m.actualSize, DVA_None);
        for (unsigned iObjSt_iPt{ 0U }; iObjSt_iPt < us_em::US_EM_Consts::US_EM_MAX_NUM_STATIC_OBJ_PTS; ++iObjSt_iPt) {
            DDefPrefix(usEmEnvModePrefix, "AP.usEmEnvModelPort.staticObjects_%d.objShape_m.array_%d", iObjStatic, iObjSt_iPt);
            DDefFloat(usEmEnvModePrefix, "x_dir", "m", &usEnvModelPort.staticObjects[iObjStatic].objShape_m.array[iObjSt_iPt].x_dir, DVA_None);
            DDefFloat(usEmEnvModePrefix, "y_dir", "m", &usEnvModelPort.staticObjects[iObjStatic].objShape_m.array[iObjSt_iPt].y_dir, DVA_None);
        }
    }
    DDefaultDelete(usEmEnvModePrefix);

    registerSignalHeaderToDVA("AP.usEmPerceptionAvailabilityPort.sSigHeader", usPercAvailPort.sSigHeader);
    tDDefault* usEmPrecAvailPrefix = DDefaultCreate("AP.usEmPerceptionAvailabilityPort.");
    DDefUInt(usEmPrecAvailPrefix, "uiVersionNumber", "", &usPercAvailPort.uiVersionNumber, DVA_None);
    DDefUChar(usEmPrecAvailPrefix, "statusEnvModel", "", (uint8_t*)&usPercAvailPort.statusEnvModel_nu, DVA_None);
    for (uint8_t iSen{ 0U }; iSen < us_em::US_EM_Consts::US_EM_MAX_NUM_OF_SENSORS; ++iSen) {
        DDefPrefix(usEmPrecAvailPrefix, "AP.usEmPerceptionAvailabilityPort.statusUSSensors_nu_%d", iSen);
        DDefUChar(usEmPrecAvailPrefix, "", "", (uint8_t*)&usPercAvailPort.statusUSSensors_nu[iSen], DVA_None);
    }
    DDefaultDelete(usEmPrecAvailPrefix);


    //S curve length
    DDefFloat(NULL, "SCurveLength_m", "m", &globalSCurveLength_m, DVA_None);
    //Add signals for plot the Planned Trajectory Path in IPG Movie
    registerSignalHeaderToDVA("trjplaVisuPort.trjplaVisuPort.sSigHeader", trjplaVisuPort.sSigHeader);
    DDefUInt(NULL, "trjplaVisuPort.uiVersionNumber", "", &trjplaVisuPort.uiVersionNumber, DVA_None);
    DDefUShort(NULL, "trjplaVisuPort.currentPoseIdx_nu", "", &trjplaVisuPort.currentPoseIdx_nu, DVA_None);
    DDefUShort(NULL, "trjplaVisuPort.numValidPoses_nu", "", &trjplaVisuPort.numValidPoses_nu, DVA_None);
    DDefUChar(NULL, "trjplaVisuPort.numValidSegments", "", &trjplaVisuPort.numValidSegments, DVA_None);
    tDDefault *df = DDefaultCreate("trjplaVisuPort.");
    for (unsigned int i = 0U; i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_POSES_IN_PATH_NU; i++) {
        DDefPrefix(df, "trjplaVisuPort.plannedPathXPos_m_%d", i);
        DDefFloat(df, "", "m", &trjplaVisuPort.plannedPathXPos_m[i], DVA_None);
        DDefPrefix(df, "trjplaVisuPort.plannedPathYPos_m_%d", i);
        DDefFloat(df, "", "m", &trjplaVisuPort.plannedPathYPos_m[i], DVA_None);
    }

    for (unsigned int i = 0U; i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_SEGMENTS_IN_PATH_NU; i++) {
        DDefPrefix(df, "trjplaVisuPort.plannedGeometricPath_%d.", i);

        DDefFloat(df, "startPose_0", "", &trjplaVisuPort.plannedGeometricPath[i].startPose[0], DVA_None);
        DDefFloat(df, "startPose_1", "", &trjplaVisuPort.plannedGeometricPath[i].startPose[1], DVA_None);
        DDefFloat(df, "startPose_2", "", &trjplaVisuPort.plannedGeometricPath[i].startPose[2], DVA_None);

        DDefFloat(df, "endPose_0", "", &trjplaVisuPort.plannedGeometricPath[i].endPose[0], DVA_None);
        DDefFloat(df, "endPose_1", "", &trjplaVisuPort.plannedGeometricPath[i].endPose[1], DVA_None);
        DDefFloat(df, "endPose_2", "", &trjplaVisuPort.plannedGeometricPath[i].endPose[2], DVA_None);

        DDefFloat(df, "turnRadius_m", "m", &trjplaVisuPort.plannedGeometricPath[i].turnRadius_m, DVA_None);
        DDefFloat(df, "turnRadiusSecond_m", "m", &trjplaVisuPort.plannedGeometricPath[i].turnRadiusSecond_m, DVA_None);
        DDefUChar(df, "drvDir", "", &trjplaVisuPort.plannedGeometricPath[i].drvDir, DVA_None);
        DDefUChar(df, "steerDir", "", &trjplaVisuPort.plannedGeometricPath[i].steerDir, DVA_None);
        DDefFloat(df, "longVel_mps", "m/s", &trjplaVisuPort.plannedGeometricPath[i].longVel_mps, DVA_None);
        DDefFloat(df, "length_m", "m", &trjplaVisuPort.plannedGeometricPath[i].length_m, DVA_None);

        DDefFloat(df, "rotationCenter_0_m", "m", &trjplaVisuPort.plannedGeometricPath[i].rotationCenter_m[0], DVA_None);
        DDefFloat(df, "rotationCenter_1_m", "m", &trjplaVisuPort.plannedGeometricPath[i].rotationCenter_m[1], DVA_None);

        DDefFloat(df, "rotationCenterSecond_0_m", "m", &trjplaVisuPort.plannedGeometricPath[i].rotationCenterSecond_m[0], DVA_None);
        DDefFloat(df, "rotationCenterSecond_1_m", "m", &trjplaVisuPort.plannedGeometricPath[i].rotationCenterSecond_m[1], DVA_None);

        DDefUChar(df, "planPhase", "", &trjplaVisuPort.plannedGeometricPath[i].planPhase, DVA_None);
    }

    //Add transformed signals for plot the Planned Trajectory Path in IPG Movie
    registerSignalHeaderToDVA("trjplaVisuPortCMOrigin.sSigHeader", trjplaVisuPortCMOrigin.sSigHeader);
    DDefUInt(NULL, "trjplaVisuPortCMOrigin.uiVersionNumber", "", &trjplaVisuPortCMOrigin.uiVersionNumber, DVA_None);
    DDefUShort(NULL, "trjplaVisuPortCMOrigin.currentPoseIdx_nu", "", &trjplaVisuPortCMOrigin.currentPoseIdx_nu, DVA_None);
    DDefUShort(NULL, "trjplaVisuPortCMOrigin.numValidPoses_nu", "", &trjplaVisuPortCMOrigin.numValidPoses_nu, DVA_None);
    DDefUChar(NULL, "trjplaVisuPortCMOrigin.numValidSegments", "", &trjplaVisuPortCMOrigin.numValidSegments, DVA_None);
    tDDefault *df2 = DDefaultCreate("trjplaVisuPortCMOrigin.");

    for (unsigned int i = 0U; i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_POSES_IN_PATH_NU; i++) {
        DDefPrefix(df2, "trjplaVisuPortCMOrigin.plannedPathXPos_m_%d", i);
        DDefFloat(df2, "", "m", &trjplaVisuPortCMOrigin.plannedPathXPos_m[i], DVA_None);
        DDefPrefix(df2, "trjplaVisuPortCMOrigin.plannedPathYPos_m_%d", i);
        DDefFloat(df2, "", "m", &trjplaVisuPortCMOrigin.plannedPathYPos_m[i], DVA_None);
    }
    for (unsigned int i = 0U; i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_SEGMENTS_IN_PATH_NU; i++) {
        DDefPrefix(df2, "trjplaVisuPortCMOrigin.plannedGeometricPath_%d.", i);

        DDefFloat(df2, "startPose_0", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].startPose[0], DVA_None);
        DDefFloat(df2, "startPose_1", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].startPose[1], DVA_None);
        DDefFloat(df2, "startPose_2", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].startPose[2], DVA_None);

        DDefFloat(df2, "endPose_0", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].endPose[0], DVA_None);
        DDefFloat(df2, "endPose_1", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].endPose[1], DVA_None);
        DDefFloat(df2, "endPose_2", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].endPose[2], DVA_None);

        DDefFloat(df2, "turnRadius_m", "m", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].turnRadius_m, DVA_None);
        DDefFloat(df2, "turnRadiusSecond_m", "m", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].turnRadiusSecond_m, DVA_None);
        DDefUChar(df2, "drvDir", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].drvDir, DVA_None);
        DDefUChar(df2, "steerDir", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].steerDir, DVA_None);
        DDefFloat(df2, "longVel_mps", "m/s", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].longVel_mps, DVA_None);
        DDefFloat(df2, "length_m", "m", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].length_m, DVA_None);

        DDefFloat(df2, "rotationCenter_0_m", "m", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].rotationCenter_m[0], DVA_None);
        DDefFloat(df2, "rotationCenter_1_m", "m", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].rotationCenter_m[1], DVA_None);

        DDefFloat(df2, "rotationCenterSecond_0_m", "m", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].rotationCenterSecond_m[0], DVA_None);
        DDefFloat(df2, "rotationCenterSecond_1_m", "m", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].rotationCenterSecond_m[1], DVA_None);

        DDefUChar(df2, "planPhase", "", &trjplaVisuPortCMOrigin.plannedGeometricPath[i].planPhase, DVA_None);
    }

    registerSignalHeaderToDVA("AP.reverseAssistAvailabilityPort.sSigHeader", reverseAssistAvailabilityPort.sSigHeader);
    DDefUChar(NULL, "AP.reverseAssistAvailabilityPort.pathAvailable", "", &reverseAssistAvailabilityPort.pathAvailable, DVA_None);
    DDefFloat(NULL, "AP.reverseAssistAvailabilityPort.pathLength_m", "m", &reverseAssistAvailabilityPort.pathLength_m, DVA_None);

    //Add signals for plot the replan position points in IPG Movie
    tDDefault *dfReplan = DDefaultCreate("trjplaDebugPort.");
    const unsigned int maxNumReplanPoints{ sizeof(replanPositionCMOrigin) / sizeof(replanPositionCMOrigin[0]) };
    for (unsigned int i = 0U; i < maxNumReplanPoints; i++) {
        DDefPrefix(dfReplan, "replanPositionCMOrigin.replanXPos_m_%d", i);
        DDefFloat(dfReplan, "", "m", &replanPositionCMOrigin[i].Pos().x(), DVA_None);
        DDefPrefix(dfReplan, "replanPositionCMOrigin.replanYPos_m_%d", i);
        DDefFloat(dfReplan, "", "m", &replanPositionCMOrigin[i].Pos().y(), DVA_None);
    }

    //Add transformed signals for plot the previous planned Trajectory Path in IPG Movie
    registerSignalHeaderToDVA("AP.previousTrajectoryPortCMOrigin.sSigHeader", previousTrajectoryPortCMOrigin.sSigHeader);
    DDefUShort(NULL, "AP.previousTrajectoryPortCMOrigin.numValidPoses_nu", "", &previousTrajectoryPortCMOrigin.numValidPoses_nu, DVA_None);
    tDDefault *dfPrevTraj = DDefaultCreate("AP.previousTrajectoryPortCMOrigin.");
    for (unsigned int i = 0U; i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_POSES_IN_PATH_NU; i++) {
        DDefPrefix(dfPrevTraj, "AP.previousTrajectoryPortCMOrigin.plannedPathXPos_m_%d", i);
        DDefFloat(dfPrevTraj, "", "m", &previousTrajectoryPortCMOrigin.plannedPathXPos_m[i], DVA_None);
        DDefPrefix(dfPrevTraj, "AP.previousTrajectoryPortCMOrigin.plannedPathYPos_m_%d", i);
        DDefFloat(dfPrevTraj, "", "m", &previousTrajectoryPortCMOrigin.plannedPathYPos_m[i], DVA_None);
    }
    //signals to plot the previous target pose for IPG Movie replanning visualization
    DDefFloat(NULL, "AP.previousTargetPose.pose.pos.x", "m", &previousTargetPose.pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.previousTargetPose.pose.pos.y", "m", &previousTargetPose.pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.previousTargetPose.pose.yaw_rad", "rad", &previousTargetPose.pose.yaw_rad, DVA_None);

    //plannedTrajPort
    registerSignalHeaderToDVA("AP.plannedTrajPort.sSigHeader", plannedTrajPort.sSigHeader);
    DDefUChar(NULL, "AP.plannedTrajPort.trajType_nu", "", (uint8_t*)&plannedTrajPort.trajType_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.numValidCtrlPoints_nu", "", &plannedTrajPort.numValidCtrlPoints_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.drivingForwardReq_nu", "", (uint8_t*)&plannedTrajPort.drivingForwardReq_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.trajValid_nu", "", (uint8_t*)&plannedTrajPort.trajValid_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.newSegmentStarted_nu", "", (uint8_t*)&plannedTrajPort.newSegmentStarted_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.isLastSegment_nu", "", (uint8_t*)&plannedTrajPort.isLastSegment_nu, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajPort.drivingResistance_0.distance_m", "m", &plannedTrajPort.drivingResistance[0].distance_m, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajPort.drivingResistance_1.distance_m", "m", &plannedTrajPort.drivingResistance[1].distance_m, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajPort.drivingResistance_2.distance_m", "m", &plannedTrajPort.drivingResistance[2].distance_m, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajPort.drivingResistance_3.distance_m", "m", &plannedTrajPort.drivingResistance[3].distance_m, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.drivingResistance_0.type_nu", "", (uint8_t*)&plannedTrajPort.drivingResistance[0].type_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.drivingResistance_1.type_nu", "", (uint8_t*)&plannedTrajPort.drivingResistance[1].type_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.drivingResistance_2.type_nu", "", (uint8_t*)&plannedTrajPort.drivingResistance[2].type_nu, DVA_None);
    DDefUChar(NULL, "AP.plannedTrajPort.drivingResistance_3.type_nu", "", (uint8_t*)&plannedTrajPort.drivingResistance[3].type_nu, DVA_None);

    tDDefault *plannedTrajDefault = DDefaultCreate("AP.plannedTrajPort.plannedTraj");

    for (unsigned int i = 0; i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_TRAJ_CTRL_POINTS; i++) {
        DDefPrefix(plannedTrajDefault, "AP.plannedTrajPort.plannedTraj_%d.xTrajRAReq_m", i);
        DDefFloat(plannedTrajDefault, "", "m", &plannedTrajPort.plannedTraj[i].xTrajRAReq_m, DVA_None);
        DDefPrefix(plannedTrajDefault, "AP.plannedTrajPort.plannedTraj_%d.yTrajRAReq_m", i);
        DDefFloat(plannedTrajDefault, "", "m", &plannedTrajPort.plannedTraj[i].yTrajRAReq_m, DVA_None);
        DDefPrefix(plannedTrajDefault, "AP.plannedTrajPort.plannedTraj_%d.crvRARReq_1pm", i);
        DDefFloat(plannedTrajDefault, "", "1/m", &plannedTrajPort.plannedTraj[i].crvRAReq_1pm, DVA_None);
        DDefPrefix(plannedTrajDefault, "AP.plannedTrajPort.plannedTraj_%d.distanceToStopReq_m", i);
        DDefFloat(plannedTrajDefault, "", "m", &plannedTrajPort.plannedTraj[i].distanceToStopReq_m, DVA_None);
        DDefPrefix(plannedTrajDefault, "AP.plannedTrajPort.plannedTraj_%d.velocityLimitReq_mps", i);
        DDefFloat(plannedTrajDefault, "", "m/s", &plannedTrajPort.plannedTraj[i].velocityLimitReq_mps, DVA_None);
        DDefPrefix(plannedTrajDefault, "AP.plannedTrajPort.plannedTraj_%d.yawReq_rad", i);
        DDefFloat(plannedTrajDefault, "", "rad", &plannedTrajPort.plannedTraj[i].yawReq_rad, DVA_None);
    }

    // targetPosesPort
    registerSignalHeaderToDVA("AP.targetPosesPort.sSigHeader", targetPosesPort.sSigHeader);
    tDDefault *targetPoses = DDefaultCreate("AP.targetPosesPort.targetPoses");
    for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU; i++) {
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.pose_ID", i);
        DDefUChar(targetPoses, "", "", &targetPosesPort.targetPoses[i].pose_ID, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.relatedParkingBoxID", i);
        DDefUShort(targetPoses, "", "", &targetPosesPort.targetPoses[i].relatedParkingBoxID, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.reachableStatus", i);
        DDefUChar(targetPoses, "", "", (uint8_t*)&targetPosesPort.targetPoses[i].reachableStatus, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.poseFailReason", i);
        DDefUChar(targetPoses, "", "", (uint8_t*)&targetPosesPort.targetPoses[i].poseFailReason, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.type", i);
        DDefUChar(targetPoses, "", "", (uint8_t*)&targetPosesPort.targetPoses[i].type, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.targetSide", i);
        DDefUChar(targetPoses, "", "", (uint8_t*)&targetPosesPort.targetPoses[i].targetSide, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.pose.pos.x", i);
        DDefFloat(targetPoses, "", "m", &targetPosesPort.targetPoses[i].pose.x_dir, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.pose.pos.y", i);
        DDefFloat(targetPoses, "", "m", &targetPosesPort.targetPoses[i].pose.y_dir, DVA_None);
        DDefPrefix(targetPoses, "AP.targetPosesPort.targetPoses_%d.pose.yaw_rad", i);
        DDefFloat(targetPoses, "", "rad", &targetPosesPort.targetPoses[i].pose.yaw_rad, DVA_None);
    }
    DDefUChar(NULL, "AP.targetPosesPort.numValidPoses", "", &targetPosesPort.numValidPoses, DVA_None);
    DDefUChar(NULL, "AP.targetPosesPort.selectedPoseData.reachedStatus", "", (uint8_t*)&targetPosesPort.selectedPoseData.reachedStatus, DVA_None);
    DDefUChar(NULL, "AP.targetPosesPort.anyPathFound", "", &targetPosesPort.anyPathFound, DVA_None);
    DDefUChar(NULL, "AP.targetPosesPort.failReason", "", (uint8_t*)&targetPosesPort.failReason, DVA_None);
    DDefUChar(NULL, "AP.targetPosesPort.resetCounter", "", (uint8_t*)&targetPosesPort.resetCounter, DVA_None);

    // targetPosesPortCMOrigin
    registerSignalHeaderToDVA("AP.targetPosesPortCMOrigin.sSigHeader", targetPosesPortCMOrigin.sSigHeader);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_0.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[0].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_0.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[0].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_0.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[0].pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_1.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[1].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_1.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[1].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_1.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[1].pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_2.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[2].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_2.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[2].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_2.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[2].pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_3.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[3].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_3.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[3].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_3.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[3].pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_4.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[4].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_4.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[4].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_4.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[4].pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_5.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[5].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_5.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[5].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_5.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[5].pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_6.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[6].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_6.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[6].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_6.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[6].pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_7.pose.pos.x", "m", &targetPosesPortCMOrigin.targetPoses[7].pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_7.pose.pos.y", "m", &targetPosesPortCMOrigin.targetPoses[7].pose.y_dir, DVA_None);
    DDefFloat(NULL, "AP.targetPosesPortCMOrigin.targetPoses_7.pose.yaw_rad", "rad", &targetPosesPortCMOrigin.targetPoses[7].pose.yaw_rad, DVA_None);

    //headUnitVisualizationPort
    DDefUChar(NULL, "AP.headUnitVisualizationPort.message_nu", "", &carMakerInterface.headUnitMessage_nu, DVA_None);

    //hmiInputPort
    registerSignalHeaderToDVA("AP.hmiInputPort.sSigHeader", gHmiInputPort.sSigHeader);
    DDefUChar(NULL, "AP.hmiInputPort.left.egoRelativePos_nu", "", &gHmiInputPort.parkingSituation.left.egoRelativePos_nu, DVA_None);
    DDefUChar(NULL, "AP.hmiInputPort.right.egoRelativePos_nu", "", &gHmiInputPort.parkingSituation.right.egoRelativePos_nu, DVA_None);
    DDefUChar(NULL, "AP.hmiInputPort.reverseAssistAvailabilityPort_nu.pathAvailable", "", (uint8_t*)&gHmiInputPort.reverseAssistAvailabilityPort_nu.pathAvailable, DVA_None);
    tDDefault *hmiInputPort = DDefaultCreate("AP.hmiInputPort");
    for (unsigned int i = 0; i < mf_hmih::MF_HMIH_Consts::AP_H_MAX_NUM_SLOTS_SIDE_NU; i++) {
        DDefPrefix(hmiInputPort, "AP.hmiInputPort.left.poseID_nu_%d", i);
        DDefUChar(hmiInputPort, "", "", &gHmiInputPort.parkingSpaces.left.poseID_nu[i], DVA_None);
        DDefPrefix(hmiInputPort, "AP.hmiInputPort.right.poseID_nu_%d", i);
        DDefUChar(hmiInputPort, "", "", &gHmiInputPort.parkingSpaces.right.poseID_nu[i], DVA_None);
    }

    //apUserInteractionPort
    DDefUChar(NULL, "AP.apUserInteractionPort.selectedTPID_nu", "", &apUserInteractionPort.selectedTPID_nu, DVA_None);
    DDefUChar(NULL, "AP.apUserInteractionPort.headUnitInteraction", "", (uint8_t*)&apUserInteractionPort.headUnitInteraction, DVA_IO_In);

    //userDefinedSlotPort
    registerSignalHeaderToDVA("AP.userDefinedSlotPort.sSigHeader", userDefinedSlotPort.sSigHeader);
    DDefUChar(NULL, "AP.userDefinedSlotPort.slotSide_nu", "", (uint8_t*)&userDefinedSlotPort.slotSide_nu, DVA_None);
    DDefUChar(NULL, "AP.userDefinedSlotPort.slotType_nu", "", (uint8_t*)&userDefinedSlotPort.slotType_nu, DVA_None);
    DDefUChar(NULL, "AP.userDefinedSlotPort.userDefined_nu", "", (uint8_t*)&userDefinedSlotPort.userDefined_nu, DVA_None);
    DDefFloat(NULL, "AP.userDefinedSlotPort.pose.yaw_rad", "rad", &userDefinedSlotPort.pose.yaw_rad, DVA_None);
    DDefFloat(NULL, "AP.userDefinedSlotPort.pose.x", "m", &userDefinedSlotPort.pose.x_dir, DVA_None);
    DDefFloat(NULL, "AP.userDefinedSlotPort.pose.y", "m", &userDefinedSlotPort.pose.y_dir, DVA_None);

    //hmihDebugPort
    tDDefault *hmihDebugPort = DDefaultCreate("AP.hmihDebugPort");
    for (unsigned int i = 0; i < mf_hmih::MF_HMIH_Consts::NUM_MTS_DEBUG_FREESPACE_HMIH; i++) {
        DDefPrefix(hmihDebugPort, "AP.hmihDebugPort.debugInt_%d", i);
        DDefUChar(hmihDebugPort, "", "", (uint8_t*)&gMFHmiHDebugPort.debugInt[i], DVA_None);
        DDefPrefix(hmihDebugPort, "AP.hmihDebugPort.debugFloat_%d", i);
        DDefFloat(hmihDebugPort, "", "", &gMFHmiHDebugPort.debugFloat[i], DVA_None);
    }

    //taposdDebugPort
    registerSignalHeaderToDVA("AP.taposdDebugPort.sSigHeader", taposdDebugPort.sSigHeader);
    DDefFloat(NULL, "AP.taposdDebugPort.latDistToTarget_m", "m", &taposdDebugPort.latDistToTarget_m, DVA_None);
    DDefFloat(NULL, "AP.taposdDebugPort.longDistToTarget_m", "m", &longDistToTarget_m, DVA_None);
    DDefFloat(NULL, "AP.taposdDebugPort.yawDiffToTarget_rad", "rad", &taposdDebugPort.yawDiffToTarget_rad, DVA_None);

    unsigned int maxPointsSimpleBox = 4U;
    unsigned int maxNumSimpleBoxes = 8U;
    tDDefault *taposdDP = DDefaultCreate("AP.taposdDebugPort.");
    for (unsigned int j = 0; j < maxNumSimpleBoxes; j++) {
        DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugBackwards_%d.comfParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPort.pbDebugBackwards[j].comfParkingBox.numValidPoints_nu, DVA_None);
        DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugBackwards_%d.maxParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPort.pbDebugBackwards[j].maxParkingBox.numValidPoints_nu, DVA_None);
        for (unsigned int i = 0; i < maxPointsSimpleBox; i++) {
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugBackwards_%d.comfParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugBackwards[j].comfParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugBackwards_%d.comfParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugBackwards[j].comfParkingBox.posY_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugBackwards_%d.maxParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugBackwards[j].maxParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugBackwards_%d.maxParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugBackwards[j].maxParkingBox.posY_m[i], DVA_None);
        }
    }
    for (unsigned int j = 0; j < maxNumSimpleBoxes; j++) {
        DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugForwards_%d.comfParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPort.pbDebugForwards[j].comfParkingBox.numValidPoints_nu, DVA_None);
        DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugForwards_%d.maxParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPort.pbDebugForwards[j].maxParkingBox.numValidPoints_nu, DVA_None);
        for (unsigned int i = 0; i < maxPointsSimpleBox; i++) {
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugForwards_%d.comfParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugForwards[j].comfParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugForwards_%d.comfParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugForwards[j].comfParkingBox.posY_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugForwards_%d.maxParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugForwards[j].maxParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPort.pbDebugForwards_%d.maxParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPort.pbDebugForwards[j].maxParkingBox.posY_m[i], DVA_None);
        }
    }

    //taposdDebugPortCMOrigin
    registerSignalHeaderToDVA("AP.taposdDebugPortCMOrigin.sSigHeader", taposdDebugPortCMOrigin.sSigHeader);
    for (unsigned int j = 0; j < maxNumSimpleBoxes; j++) {
        DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugBackwards_%d.comfParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPortCMOrigin.pbDebugBackwards[j].comfParkingBox.numValidPoints_nu, DVA_None);
        DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugBackwards_%d.maxParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPortCMOrigin.pbDebugBackwards[j].maxParkingBox.numValidPoints_nu, DVA_None);
        for (unsigned int i = 0; i < maxPointsSimpleBox; i++) {
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugBackwards_%d.comfParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugBackwards[j].comfParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugBackwards_%d.comfParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugBackwards[j].comfParkingBox.posY_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugBackwards_%d.maxParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugBackwards[j].maxParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugBackwards_%d.maxParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugBackwards[j].maxParkingBox.posY_m[i], DVA_None);
        }
    }
    for (unsigned int j = 0; j < maxNumSimpleBoxes; j++) {
        DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugForwards_%d.comfParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPortCMOrigin.pbDebugForwards[j].comfParkingBox.numValidPoints_nu, DVA_None);
        DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugForwards_%d.maxParkingBox.numValidPoints_nu", j);
        DDefUChar(taposdDP, "", "", &taposdDebugPortCMOrigin.pbDebugForwards[j].maxParkingBox.numValidPoints_nu, DVA_None);
        for (unsigned int i = 0; i < maxPointsSimpleBox; i++) {
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugForwards_%d.comfParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugForwards[j].comfParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugForwards_%d.comfParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugForwards[j].comfParkingBox.posY_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugForwards_%d.maxParkingBox.posX_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugForwards[j].maxParkingBox.posX_m[i], DVA_None);
            DDefPrefix(taposdDP, "AP.taposdDebugPortCMOrigin.pbDebugForwards_%d.maxParkingBox.posY_m_%d", j, i);
            DDefFloat(taposdDP, "", "m", &taposdDebugPortCMOrigin.pbDebugForwards[j].maxParkingBox.posY_m[i], DVA_None);
        }
    }

    //LSCADataPort

    // SteeringPort
    DDefFloat(NULL, "LSCA.staticSteering.currentSteering", "deg", &(lscaPlotDataPort.staticSteering.currentSteering_deg), DVA_None);
    DDefUChar(NULL, "LSCA.staticSteering.doPlot", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.doPlot_nu), DVA_None);
    DDefUChar(NULL, "LSCA.staticSteering.movingForward_nu", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.movingForward_nu), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steeringAngleLimits_lower", "deg", &(lscaPlotDataPort.staticSteering.steeringAngleLimits_deg[0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steeringAngleLimits_upper", "deg", &(lscaPlotDataPort.staticSteering.steeringAngleLimits_deg[1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steeringTorqueLimits_lower", "Nm", &(lscaPlotDataPort.staticSteering.steeringTorqueLimits_Nm[0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steeringTorqueLimits_upper", "Nm", &(lscaPlotDataPort.staticSteering.steeringTorqueLimits_Nm[1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steeringTorqueRequestFunction", "Nm", &(lscaPlotDataPort.staticSteering.steeringTorqueRequestFunction_Nm), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steeringTorqueRequestLsca", "Nm", &(lscaPlotDataPort.staticSteering.steeringTorqueRequestLsca_Nm), DVA_None);

    // virtWall
    DDefUChar(NULL, "LSCA.staticSteering.virtWall.doPlot", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.resist.doPlot_nu), DVA_None);
    DDefUChar(NULL, "LSCA.staticSteering.virtWall.isOn_nu", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.resist.isOn_nu), DVA_None);
    DDefUChar(NULL, "LSCA.staticSteering.virtWall.isActive_nu", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.resist.isActive_nu), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.proposedSteering", "deg", &(lscaPlotDataPort.staticSteering.resist.proposedSteering_deg), DVA_None);

    //ROI
    lscaPlotDataPort.staticSteering.resist.roiPlot.setSize(lscaPlotDataPort.staticSteering.resist.roiPlot.getMaxSize()); // Force the size to 4 in case the data was not set
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.rightTop_x", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[0][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.rightTop_y", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[0][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.leftTop_x", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[1][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.leftTop_y", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[1][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.leftBottom_x", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[2][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.leftBottom_y", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[2][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.rightBottom_x", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[3][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.virtWall.roiPlot.rightBottom_y", "m", &(lscaPlotDataPort.staticSteering.resist.roiPlot[3][1]), DVA_None);
    lscaPlotDataPort.staticSteering.resist.roiPlot.setSize(0U);

    tDDefault *lscaspecsteering = DDefaultCreate("LSCA.staticSteering.virtWall.");
    lscaPlotDataPort.staticSteering.resist.tracesToBePlotted.setSize(lscaPlotDataPort.staticSteering.resist.tracesToBePlotted.getMaxSize());
    for (unsigned int idx = 0; idx < lscaPlotDataPort.staticSteering.resist.tracesToBePlotted.getMaxSize(); idx++) {
        DDefPrefix(lscaspecsteering, "LSCA.staticSteering.virtWall.Traces.TracesToBePlotted_%d.right", idx);
        DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.resist.tracesToBePlotted[idx][0], DVA_None);
        DDefPrefix(lscaspecsteering, "LSCA.staticSteering.virtWall.Traces.TracesToBePlotted_%d.left", idx);
        DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.resist.tracesToBePlotted[idx][1], DVA_None);
    }
    lscaPlotDataPort.staticSteering.resist.tracesToBePlotted.setSize(0U);
    DDefUChar(NULL, "LSCA.staticSteering.virtWall.Traces.TracesActualSize", "", (uint8_t*)&lscaPlotDataPort.staticSteering.resist.tracesToBePlotted, DVA_None);

    // Objects in the ROI:
    lscaPlotDataPort.staticSteering.resist.objectsAfterRoi.setSize(lscaPlotDataPort.staticSteering.resist.objectsAfterRoi.getMaxSize());
    for (lsm_geoml::size_type idxObj = 0U; idxObj < lscaPlotDataPort.staticSteering.resist.objectsAfterRoi.getMaxSize(); ++idxObj)
    {
        lscaPlotDataPort.staticSteering.resist.objectsAfterRoi[idxObj].setSize(lscaPlotDataPort.staticSteering.resist.objectsAfterRoi[idxObj].getMaxSize());
        for (lsm_geoml::size_type idxCoord = 0U; idxCoord < lscaPlotDataPort.staticSteering.resist.objectsAfterRoi[idxObj].getMaxSize(); ++idxCoord)
        {
            DDefPrefix(lscaspecsteering, "LSCA.staticSteering.virtWall.Objects.objectsAfterRoi_%d.coord_%d.x", idxObj, idxCoord);
            DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.resist.objectsAfterRoi[idxObj][idxCoord][0], DVA_None);
            DDefPrefix(lscaspecsteering, "LSCA.staticSteering.virtWall.Objects.objectsAfterRoi_%d.coord_%d.y", idxObj, idxCoord);
            DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.resist.objectsAfterRoi[idxObj][idxCoord][1], DVA_None);

        }
        lscaPlotDataPort.staticSteering.resist.objectsAfterRoi[idxObj].setSize(0U);
        DDefPrefix(lscaspecsteering, "LSCA.staticSteering.virtWall.Objects.objectsAfterRoi_%d.coordsActualSize", idxObj);
        DDefUChar(lscaspecsteering, "", "", (uint8_t*)&lscaPlotDataPort.staticSteering.resist.objectsAfterRoiCM_numOfCoord[idxObj], DVA_None);
    }
    lscaPlotDataPort.staticSteering.resist.objectsAfterRoi.setSize(0U);
    // copy actual number of valid objects:
    DDefUChar(NULL, "LSCA.staticSteering.virtWall.Objects.objectsActualSize", "", (uint8_t*)&lscaPlotDataPort.staticSteering.resist.objectsAfterRoiCM_numOfObj_nu, DVA_None);

    // SteerProposol
    DDefUChar(NULL, "LSCA.staticSteering.steerProp.doPlot", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.propose.doPlot_nu), DVA_None);
    DDefUChar(NULL, "LSCA.staticSteering.steerProp.isOn_nu", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.propose.isOn_nu), DVA_None);
    DDefUChar(NULL, "LSCA.staticSteering.steerProp.isActive_nu", "", (uint8_t*)&(lscaPlotDataPort.staticSteering.propose.isActive_nu), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.proposedSteering", "deg", &(lscaPlotDataPort.staticSteering.propose.proposedSteering_deg), DVA_None);

    lscaPlotDataPort.staticSteering.propose.roiPlot.setSize(lscaPlotDataPort.staticSteering.propose.roiPlot.getMaxSize()); // Force the size to 4 in case the data was not set
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.rightTop_x", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[0][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.rightTop_y", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[0][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.leftTop_x", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[1][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.leftTop_y", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[1][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.leftBottom_x", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[2][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.leftBottom_y", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[2][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.rightBottom_x", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[3][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticSteering.steerProp.roiPlot.rightBottom_y", "m", &(lscaPlotDataPort.staticSteering.propose.roiPlot[3][1]), DVA_None);
    lscaPlotDataPort.staticSteering.propose.roiPlot.setSize(0U);

    lscaPlotDataPort.staticSteering.propose.tracesToBePlotted.setSize(lscaPlotDataPort.staticSteering.propose.tracesToBePlotted.getMaxSize());
    for (unsigned int idx = 0; idx < lscaPlotDataPort.staticSteering.propose.tracesToBePlotted.getMaxSize(); idx++) {
        DDefPrefix(lscaspecsteering, "LSCA.staticSteering.steerProp.Traces.TracesToBePlotted_%d.right", idx);
        DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.propose.tracesToBePlotted[idx][0], DVA_None);
        DDefPrefix(lscaspecsteering, "LSCA.staticSteering.steerProp.Traces.TracesToBePlotted_%d.left", idx);
        DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.propose.tracesToBePlotted[idx][1], DVA_None);
    }
    lscaPlotDataPort.staticSteering.propose.tracesToBePlotted.setSize(0U);

    DDefUChar(NULL, "LSCA.staticSteering.steerProp.Traces.TracesActualSize", "", (uint8_t*)&lscaPlotDataPort.staticSteering.propose.tracesToBePlotted, DVA_None);

    // Objects in the ROI:
    lscaPlotDataPort.staticSteering.propose.objectsAfterRoi.setSize(lscaPlotDataPort.staticSteering.propose.objectsAfterRoi.getMaxSize());
    for (lsm_geoml::size_type idxObj = 0U; idxObj < lscaPlotDataPort.staticSteering.propose.objectsAfterRoi.getMaxSize(); ++idxObj)
    {
        lscaPlotDataPort.staticSteering.propose.objectsAfterRoi[idxObj].setSize(lscaPlotDataPort.staticSteering.propose.objectsAfterRoi[idxObj].getMaxSize());
        for (lsm_geoml::size_type idxCoord = 0U; idxCoord < lscaPlotDataPort.staticSteering.propose.objectsAfterRoi[idxObj].getMaxSize(); ++idxCoord)
        {
            DDefPrefix(lscaspecsteering, "LSCA.staticSteering.steerProp.Objects.objectsAfterRoi_%d.coord_%d.x", idxObj, idxCoord);
            DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.propose.objectsAfterRoi[idxObj][idxCoord][0], DVA_None);
            DDefPrefix(lscaspecsteering, "LSCA.staticSteering.steerProp.Objects.objectsAfterRoi_%d.coord_%d.y", idxObj, idxCoord);
            DDefFloat(lscaspecsteering, "", "", &lscaPlotDataPort.staticSteering.propose.objectsAfterRoi[idxObj][idxCoord][1], DVA_None);

        }
        lscaPlotDataPort.staticSteering.propose.objectsAfterRoi[idxObj].setSize(0U);
        DDefPrefix(lscaspecsteering, "LSCA.staticSteering.steerProp.Objects.objectsAfterRoi_%d.coordsActualSize", idxObj);
        DDefUChar(lscaspecsteering, "", "", (uint8_t*)&lscaPlotDataPort.staticSteering.propose.objectsAfterRoiCM_numOfCoord[idxObj], DVA_None);
    }
    lscaPlotDataPort.staticSteering.propose.objectsAfterRoi.setSize(0U);
    // copy actual number of valid objects:
    DDefUChar(NULL, "LSCA.staticSteering.steerProp.Objects.objectsActualSize", "", (uint8_t*)&lscaPlotDataPort.staticSteering.propose.objectsAfterRoiCM_numOfObj_nu, DVA_None);

    ////////////////

    // LSCA
    // BrakingPort
    DDefUChar(NULL, "LSCA.staticBraking.doPlot", "", (uint8_t*)&(lscaPlotDataPort.staticBraking.doPlot_nu), DVA_None);
    DDefUChar(NULL, "LSCA.staticBraking.EbaActive", "", (uint8_t*)&(lscaPlotDataPort.staticBraking.intervention_nu), DVA_None);
    DDefUChar(NULL, "LSCA.staticBraking.EbaOn", "", (uint8_t*)&(lscaPlotDataPort.staticBraking.on_nu), DVA_None);

    DDefFloat(NULL, "LSCA.staticBraking.ICR_x", "deg", &(lscaPlotDataPort.staticBraking.icr[0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.ICR_y", "deg", &(lscaPlotDataPort.staticBraking.icr[1]), DVA_None);

    lscaPlotDataPort.staticBraking.roi.setSize(lscaPlotDataPort.staticBraking.roi.getMaxSize()); // Force the size to 4 in case the data was not set
    DDefFloat(NULL, "LSCA.staticBraking.roi.rightTop_x", "m", &(lscaPlotDataPort.staticBraking.roi[0][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.roi.rightTop_y", "m", &(lscaPlotDataPort.staticBraking.roi[0][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.roi.leftTop_x", "m", &(lscaPlotDataPort.staticBraking.roi[1][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.roi.leftTop_y", "m", &(lscaPlotDataPort.staticBraking.roi[1][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.roi.leftBottom_x", "m", &(lscaPlotDataPort.staticBraking.roi[2][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.roi.leftBottom_y", "m", &(lscaPlotDataPort.staticBraking.roi[2][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.roi.rightBottom_x", "m", &(lscaPlotDataPort.staticBraking.roi[3][0]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.roi.rightBottom_y", "m", &(lscaPlotDataPort.staticBraking.roi[3][1]), DVA_None);
    DDefFloat(NULL, "LSCA.staticBraking.rotationAngleToBrake", "rad", &(lscaPlotDataPort.staticBraking.rotationAngleToBrake), DVA_None);
    lscaPlotDataPort.staticBraking.roi.setSize(0U);
    //    tDDefault *lscabraking = DDefaultCreate("LSCA.staticbraking.");




    //for (unsigned int index_nu = 0U; index_nu < maxWheelIndexSize; ++index_nu) {

    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelFrontLeft_%d.posX", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelFrontLeft[lscaPlotDataPort.staticBraking.IndicesFrontLeft[index_nu]].x(), DVA_None);
    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelFrontLeft_%d.posY", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelFrontLeft[lscaPlotDataPort.staticBraking.IndicesFrontLeft[index_nu]].y(), DVA_None);
    //}

    //for (unsigned int index_nu = 0U; index_nu < maxWheelIndexSize; ++index_nu) {

    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelFrontRight_%d.posX", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelFrontRight[lscaPlotDataPort.staticBraking.IndicesFrontRight[index_nu]].x(), DVA_None);
    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelFrontRight_%d.posY", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelFrontRight[lscaPlotDataPort.staticBraking.IndicesFrontRight[index_nu]].y(), DVA_None);
    //}


    //for (unsigned int index_nu = 0U; index_nu < maxWheelIndexSize; ++index_nu) {

    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelRearLeft_%d.posX", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelRearLeft[lscaPlotDataPort.staticBraking.IndicesRearLeft[index_nu]].x(), DVA_None);
    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelRearLeft_%d.posY", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelRearLeft[lscaPlotDataPort.staticBraking.IndicesRearLeft[index_nu]].y(), DVA_None);
    //}



    //for (unsigned int index_nu = 0U; index_nu < maxWheelIndexSize; ++index_nu) {

    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelRearRight_%d.posX", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelRearRight[lscaPlotDataPort.staticBraking.IndicesRearRight[index_nu]].x(), DVA_None);
    //    DDefPrefix(lscabraking, "LSCA.staticbraking.WheelRearRight_%d.posY", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.WheelRearRight[lscaPlotDataPort.staticBraking.IndicesRearRight[index_nu]].y(), DVA_None);
    //}


    //for (unsigned int index_nu = 0U; index_nu < maxEgoShapeSize; ++index_nu) {
    //
    //    DDefPrefix(lscabraking, "LSCA.staticbraking.bodyShapePoints_%d.posX", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.bodyShapePoints[lscaPlotDataPort.staticBraking.bodyShapeIndices[index_nu]].x(), DVA_None);
    //    DDefPrefix(lscabraking, "LSCA.staticbraking.bodyShapePoints_%d.posY", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.bodyShapePoints[lscaPlotDataPort.staticBraking.bodyShapeIndices[index_nu]].y(), DVA_None);

    //}


    //for (unsigned int index_nu = 0U; index_nu < maxEgoShapeSize; ++index_nu) {

    //    DDefPrefix(lscabraking, "LSCA.staticbraking.coreShapePoints_%d.posX", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.coreShapePoints[lscaPlotDataPort.staticBraking.coreShapeIndices[index_nu]].x(), DVA_None);
    //    DDefPrefix(lscabraking, "LSCA.staticbraking.coreShapePoints_%d.posY", index_nu);
    //    DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.coreShapePoints[lscaPlotDataPort.staticBraking.coreShapeIndices[index_nu]].y(), DVA_None);

    //}


    //for (unsigned int objectNu = 0U; objectNu < maxStaticObjectSize; ++objectNu)
    //{

    //    for (unsigned int coordinateNu = 0U; coordinateNu < maxStaticObjShapePointsSize; ++coordinateNu)
    //    {
    //        DDefPrefix(lscabraking, "LSCA.staticbraking.objectsAfterRoi_%d.posX_%d", objectNu, coordinateNu);
    //        DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.objectsAfterRoi[objectNu][coordinateNu].x(), DVA_None);
    //        DDefPrefix(lscabraking, "LSCA.staticbraking.objectsAfterRoi_%d.posY_%d", objectNu, coordinateNu);
    //        DDefFloat(lscabraking, "", "", &lscaPlotDataPort.staticBraking.objectsAfterRoi[objectNu][coordinateNu].y(), DVA_None);
    //    }
    //}


    //lscaPlotDataPort.staticBraking.bodyShapePoints;
    //lscaPlotDataPort.staticBraking.bodyShapeIndices;
    //lscaPlotDataPort.staticBraking.coreShapeIndices;
    //lscaPlotDataPort.staticBraking.coreShapePoints;


    //lscaPlotDataPort.staticBraking.WheelFrontLeft;
    //lscaPlotDataPort.staticBraking.WheelFrontRight;
    //lscaPlotDataPort.staticBraking.WheelRearLeft;
    //lscaPlotDataPort.staticBraking.WheelRearRight;


    //lscaPlotDataPort.staticBraking.IndicesFrontLeft;
    //lscaPlotDataPort.staticBraking.IndicesFrontRight;
    //lscaPlotDataPort.staticBraking.IndicesRearLeft;
    //lscaPlotDataPort.staticBraking.IndicesRearRight;

    /////////////////////////////////////////To be designed ////////////////////

    //lscaPlotDataPort.staticBraking.objectsAfterRoi;

    ///////////////////////////////////////////////////////////////////////////

    /*LscaBrakeport*/
    registerSignalHeaderToDVA("LSCA.brakePort.sSigHeader", lscaPlotDataPort.brakePort.sSigHeader);
    DDefUChar(NULL, "LSCA.brakePort.requestMode", "", (uint8_t*)&lscaPlotDataPort.brakePort.requestMode, DVA_None);
    DDefUChar(NULL, "LSCA.brakePort.holdInStandstill_nu", "", (uint8_t*)&lscaPlotDataPort.brakePort.holdInStandstill_nu, DVA_None);
    DDefFloat(NULL, "LSCA.brakePort.distanceToStop_m", "", &lscaPlotDataPort.brakePort.distanceToStop_m, DVA_None);

    /*LscaStatusPort*/
    registerSignalHeaderToDVA("LSCA.statusPort.sSigHeader", lscaPlotDataPort.statusPort.sSigHeader);
    DDefUChar(NULL, "LSCA.statusPort.brakingModuleState_nu", "", (uint8_t*)&lscaPlotDataPort.statusPort.brakingModuleState_nu, DVA_None);
    DDefUChar(NULL, "LSCA.statusPort.doorProtectionModuleState_nu", "", (uint8_t*)&lscaPlotDataPort.statusPort.doorProtectionModuleState_nu, DVA_None);
    DDefUChar(NULL, "LSCA.statusPort.rctaModuleState_nu", "", (uint8_t*)&lscaPlotDataPort.statusPort.rctaModuleState_nu, DVA_None);
    DDefUChar(NULL, "LSCA.statusPort.PmpModuleState_nu", "", (uint8_t*)&lscaPlotDataPort.statusPort.PmpModuleState_nu, DVA_None);
    DDefUChar(NULL, "LSCA.statusPort.steeringResistanceModuleState_nu", "", (uint8_t*)&lscaPlotDataPort.statusPort.steeringResistanceModuleState_nu, DVA_None);
    DDefUChar(NULL, "LSCA.statusPort.steeringProposalModuleState_nu", "", (uint8_t*)&lscaPlotDataPort.statusPort.steeringProposalModuleState_nu, DVA_None);
    DDefUChar(NULL, "LSCA.statusPort.lscaOverallMode_nu", "", (uint8_t*)&lscaPlotDataPort.statusPort.lscaOverallMode_nu, DVA_None);

    //LVMDStausport
    registerSignalHeaderToDVA("LVMD.StatusPort.sSigHeader", lvmdStatusPort.sSigHeader);
    DDefUChar(NULL, "LVMD.StatusPort.lvmdSystemStatus_nu", "", (uint8_t*)&lvmdStatusPort.lvmdSystemStatus_nu, DVA_None);
    DDefUChar(NULL, "LVMD.StatusPort.warningStatus", "", (uint8_t*)&lvmdStatusPort.lvmdWarningStatus_nu, DVA_None);
    DDefUChar(NULL, "LVMD.StatusPort.warningTriger", "", (uint8_t*)&lvmdStatusPort.lvmdWarningTrigger_nu, DVA_None);

    //psmDebugPort
    registerSignalHeaderToDVA("AP.psmDebugPort.sSigHeader", psmDebugPort.sSigHeader);
    DDefUChar(NULL, "AP.psmDebugPort.stateVarPPC_nu", "", (uint8_t*)&psmDebugPort.stateVarPPC_nu, DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.stateVarDM_nu", "", (uint8_t*)&psmDebugPort.stateVarDM_nu, DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.stateVarESM_nu", "", (uint8_t*)&psmDebugPort.stateVarESM_nu, DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.stateVarVSM_nu", "", (uint8_t*)&psmDebugPort.stateVarVSM_nu, DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.stateVarRDM_nu", "", (uint8_t*)&psmDebugPort.stateVarRDM_nu, DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.debugInt[0]", "", (uint8_t*)&psmDebugPort.debugInt[0], DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.debugInt[1]", "", (uint8_t*)&psmDebugPort.debugInt[1], DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.debugInt[2]", "", (uint8_t*)&psmDebugPort.debugInt[2], DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.debugInt[3]", "", (uint8_t*)&psmDebugPort.debugInt[3], DVA_None);
    DDefUChar(NULL, "AP.psmDebugPort.debugInt[4]", "", (uint8_t*)&psmDebugPort.debugInt[4], DVA_None);

    //gMFHmiHDebugPort
    registerSignalHeaderToDVA("AP.gMFHmiHDebugPort.sSigHeader", gMFHmiHDebugPort.sSigHeader);
    DDefUChar(NULL, "AP.gMFHmiHDebugPort.stateVarHMI_nu", "", (uint8_t*)&gMFHmiHDebugPort.stateVarHMI_nu, DVA_None);

    //odoGPSPort
    DDefFloat(NULL, "odoGpsPort.gpsData.gpsAntennaHeight_m", "", &odoGpsPort.gpsData.gpsAntennaHeight_m, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.gpsCourseOverGround", "", &odoGpsPort.gpsData.gpsCourseOverGround, DVA_None);
    DDefLong(NULL, "odoGpsPort.gpsData.gpsLatitude_dd", "", (signed long*)&odoGpsPort.gpsData.gpsLatitude_dd, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.gpsLatitude_mm", "", &odoGpsPort.gpsData.gpsLatitude_mm, DVA_None);
    DDefLong(NULL, "odoGpsPort.gpsData.gpsLongitude_dd", "", (signed long*)&odoGpsPort.gpsData.gpsLongitude_dd, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.gpsLongitude_mm", "", &odoGpsPort.gpsData.gpsLongitude_mm, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.gpsSpeed_mps", "", &odoGpsPort.gpsData.gpsSpeed_mps, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.gpsR32SpeedOverGround_mps", "", &odoGpsPort.gpsData.gpsR32SpeedOverGround_mps, DVA_None);
    DDefUInt(NULL, "odoGpsPort.gpsData.gpsUtcTime_hh", "", (unsigned int*)&odoGpsPort.gpsData.gpsUtcTime_hh, DVA_None);
    DDefUInt(NULL, "odoGpsPort.gpsData.gpsUtcTime_mm", "", (unsigned int*)&odoGpsPort.gpsData.gpsUtcTime_mm, DVA_None);
    DDefUInt(NULL, "odoGpsPort.gpsData.gpsUtcTime_ss", "", (unsigned int*)&odoGpsPort.gpsData.gpsUtcTime_ss, DVA_None);
    DDefUChar(NULL, "odoGpsPort.gpsData.gpsLatitudeHemisphere_nu", "", (uint8_t*)&odoGpsPort.gpsData.gpsLatitudeHemisphere_nu, DVA_None);
    DDefUChar(NULL, "odoGpsPort.gpsData.gpsLongitudeHemisphere_nu", "", (uint8_t*)&odoGpsPort.gpsData.gpsLongitudeHemisphere_nu, DVA_None);
    DDefUInt(NULL, "odoGpsPort.gpsData.gpsDateDay_dd", "", (unsigned int*)&odoGpsPort.gpsData.gpsDateDay_dd, DVA_None);
    DDefUInt(NULL, "odoGpsPort.gpsData.gpsDateMonth_mm", "", (unsigned int*)&odoGpsPort.gpsData.gpsDateMonth_mm, DVA_None);
    DDefUInt(NULL, "odoGpsPort.gpsData.gpsDateYear_yy", "", (unsigned int*)&odoGpsPort.gpsData.gpsDateYear_yy, DVA_None);
    DDefUInt(NULL, "odoGpsPort.gpsData.gpsNoOfSatellites", "", (unsigned int*)&odoGpsPort.gpsData.gpsNoOfSatellites, DVA_None);
    DDefUChar(NULL, "odoGpsPort.gpsData.ReceiverStatus_nu", "", (uint8_t*)&odoGpsPort.gpsData.ReceiverStatus_nu, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.geometricDOP", "", &odoGpsPort.gpsData.geometricDOP, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.timeDOP", "", &odoGpsPort.gpsData.timeDOP, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.positionDOP", "", &odoGpsPort.gpsData.positionDOP, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.horizontalDOP", "", &odoGpsPort.gpsData.horizontalDOP, DVA_None);
    DDefFloat(NULL, "odoGpsPort.gpsData.verticalDOP", "", &odoGpsPort.gpsData.verticalDOP, DVA_None);
    registerSignalHeaderToDVA("odoGpsPort.sSigHeader", odoGpsPort.sSigHeader);

    //ambientDataPort
    DDefFloat(NULL, "ambientDataPort.Ambient_pressure", "", &ambientDataPort.Ambient_pressure, DVA_None);
    DDefFloat(NULL, "ambientDataPort.Ambient_temperature", "", &ambientDataPort.Ambient_temperature, DVA_None);
    registerSignalHeaderToDVA("ambientDataPort.sSigHeader", ambientDataPort.sSigHeader);

    //debugSignals
    DDefFloat(NULL, "AP.maxCycleTimeOfAUPStep_ms", "", &maxCycleTimeOfAUPStep_ms, DVA_None);
    /*DDefFloat(NULL, "AP.steeringWheelAngleAcceleration", "", &steeringWheelAngleAcceleration, DVA_None);*/
    steeringWheelAngleAcceleration = 0;
    steeringAngleAcceleration = 0;
    DDefFloat(NULL, "AP.steeringWheelAngleAcceleration", "", &steeringWheelAngleAcceleration, DVA_None);
    DDefFloat(NULL, "AP.steeringAngleAcceleration", "", (&steeringAngleAcceleration), DVA_None);
    DDefUChar(NULL, "AP.TCEActive_nu", "", (uint8_t*)&TCEActive_nu, DVA_IO_In);

    testEvaluation.registerCarMakerDVAs();
    odometryDataManger.registerDVAVariables();

    // environment variables
    auto genJsonVar = std::getenv("CARMAKER_GENERATE_JSON");
    if (genJsonVar) {
        std::string genJsonVarStr{ genJsonVar };
        std::transform(genJsonVarStr.begin(), genJsonVarStr.end(), genJsonVarStr.begin(),
            [](unsigned char character) { return static_cast<unsigned char>(::tolower(character)); });


        if ((genJsonVarStr.find("no") != std::string::npos) || (genJsonVarStr.find("off") != std::string::npos) || (genJsonVarStr == "0")) {
            jsonExportType_nu = JsonExportType::NO_EXPORT;
        }
        else if (genJsonVarStr.find("finished") != std::string::npos) {
            jsonExportType_nu = JsonExportType::EXPORT_WHEN_FINISHED;
        }
        else if (genJsonVarStr.find("selected") != std::string::npos) {
            jsonExportType_nu = JsonExportType::EXPORT_WHEN_TARGET_POSE_SELECTED;
        }
        else if (genJsonVarStr.find("replan") != std::string::npos) {
            jsonExportType_nu = JsonExportType::EXPORT_WHEN_REPLAN_TRIGGERED;
        }
        else { // default behavior
            jsonExportType_nu = JsonExportType::EXPORT_WHEN_FINISHED;
        }

        auto root = std::getenv("CARMAKER_GENERATE_JSON_ROOT");
        jsonFileGenerationRoot_nu = root ? root : "S:\\CarMakerExport";
    }
    else {
        jsonExportType_nu = JsonExportType::NO_EXPORT;
    }

    /*ThGo Start*/
    DDefFloat(NULL, "AP.FakeLoDMC.Car_ax_filt", "m/s^2", &Acc_y_filt, DVA_None);
    DDefFloat(NULL, "AP.FakeLoDMC.Car_ax_req", "m/s^2", &accelReq_mps2, DVA_None);
    DDefDouble(NULL, "curAccel_mps2", "m/s^2", &curAccel_mps2, DVA_None);
    DDefDouble(NULL, "AP.FakeLoDMC.Car_delta_ax", "m/s^2", &delta_ax, DVA_None);
    DDefDouble(NULL, "AP.FakeLoDMC.c", "", &c, DVA_None);
    registerSignalHeaderToDVA("lscaStatusPort.sSigHeader", lscaStatusPort.sSigHeader);
    DDefUChar(NULL, "lscaStatusPort.braking_enabled_nu", "", (uint8_t*)&lscaStatusPort.brakingModuleState_nu, DVA_None); // this is destroyed for now since no more providing bool!
    DDefUChar(NULL, "test_bool", "", (uint8_t*)&test_bool, DVA_None);
    /*ThGo End*/

    // DWF
    //drvWarnStatusPort
    registerSignalHeaderToDVA("AP.drvWarnStatus.sSigHeader", drvWarnStatusPort.sSigHeader);
    DDefUChar(NULL, "AP.drvWarnStatus.pdwState_nu", "", (uint8_t*)&drvWarnStatusPort.pdwState_nu, DVA_None);
    DDefUChar(NULL, "AP.drvWarnStatus.pdwSystemState_nu", "", (uint8_t*)&drvWarnStatusPort.pdwSystemState_nu, DVA_None);
    DDefUChar(NULL, "AP.drvWarnStatus.pdwShutdownCause_nu", "", (uint8_t*)&drvWarnStatusPort.pdwShutdownCause_nu, DVA_None);
    DDefUChar(NULL, "AP.drvWarnStatus.whpState_nu", "", (uint8_t*)&drvWarnStatusPort.whpState_nu, DVA_None);
    DDefUChar(NULL, "AP.drvWarnStatus.whpDisplayReq_nu", "", (uint8_t*)&drvWarnStatusPort.whpDisplayReq_nu, DVA_None);
    DDefUChar(NULL, "AP.drvWarnStatus.reduceToMuteSoundReq_nu", "", (uint8_t*)&drvWarnStatusPort.reduceToMuteSoundReq_nu, DVA_None);

    //drvWarnCoreStatusPort
    registerSignalHeaderToDVA("AP.drvWarnCoreStatusPort.sSigHeader", drvWarnCoreStatusPort.sSigHeader);
    DDefUChar(NULL, "AP.drvWarnCoreStatusPort.whpCoreState_nu", "", (uint8_t*)&drvWarnCoreStatusPort.whpCoreState_nu, DVA_None);
    DDefUChar(NULL, "AP.drvWarnCoreStatusPort.pdwCoreState_nu", "", (uint8_t*)&drvWarnCoreStatusPort.pdwCoreState_nu, DVA_None);
    DDefUChar(NULL, "AP.drvWarnCoreStatusPort.pdwActiveState_nu", "", (uint8_t*)&drvWarnCoreStatusPort.pdwActiveState_nu, DVA_None);

    //appToCoreSMPort
    registerSignalHeaderToDVA("AP.appToCoreSMPort.sSigHeader", appToCoreSMPort.sSigHeader);
    DDefUChar(NULL, "AP.appToCoreSMPort.pdwRequestMode_nu", "", (uint8_t*)&appToCoreSMPort.pdwRequestMode_nu, DVA_None);
    DDefUChar(NULL, "AP.appToCoreSMPort.whpRequestMode_nu", "", (uint8_t*)&appToCoreSMPort.whpRequestMode_nu, DVA_None);

    //logicToProcPort
    registerSignalHeaderToDVA("AP.logicToProcPort.sSigHeader", logicToProcPort.sSigHeader);
    DDefUChar(NULL, "AP.logicToProcPort.drvTubeDisplayReq_nu", "", (uint8_t*)&logicToProcPort.drvTubeDisplayReq_nu, DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.sideResetReq_nu_Front", "", (uint8_t*)&logicToProcPort.sideResetReq_nu[0], DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.sideResetReq_nu_Rear", "", (uint8_t*)&logicToProcPort.sideResetReq_nu[1], DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.sideResetReq_nu_Left", "", (uint8_t*)&logicToProcPort.sideResetReq_nu[2], DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.sideResetReq_nu_Right", "", (uint8_t*)&logicToProcPort.sideResetReq_nu[3], DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.whlWarningType_nu_FL", "", (uint8_t*)&logicToProcPort.whlWarningType_nu[0], DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.whlWarningType_nu_FR", "", (uint8_t*)&logicToProcPort.whlWarningType_nu[1], DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.whlWarningType_nu_RL", "", (uint8_t*)&logicToProcPort.whlWarningType_nu[2], DVA_None);
    DDefUChar(NULL, "AP.logicToProcPort.whlWarningType_nu_RR", "", (uint8_t*)&logicToProcPort.whlWarningType_nu[3], DVA_None);

    //drvWarnDebugPort
    registerSignalHeaderToDVA("AP.drvWarnDebugPort.sSigHeader", drvWarnDebugPort.sSigHeader);
    DDefInt(NULL, "AP.drvWarnDebugPort.autoActivResetCond", "", &drvWarnDebugPort.debugInt[1], DVA_None);
    DDefInt(NULL, "AP.drvWarnDebugPort.pdwPrevCondMask", "", &drvWarnDebugPort.debugInt[7], DVA_None);
    DDefInt(NULL, "AP.drvWarnDebugPort.whpPrevCondMask", "", &drvWarnDebugPort.debugInt[8], DVA_None);
    DDefInt(NULL, "AP.drvWarnDebugPort.whpRstCond", "", &drvWarnDebugPort.debugInt[9], DVA_None);

    DDefFloat(NULL, "AP.drvWarnDebugPort.minDistFrontSensors", "", &drvWarnDebugPort.debugFloat[4], DVA_None);
    DDefFloat(NULL, "AP.drvWarnDebugPort.rollBackDist", "", &drvWarnDebugPort.debugFloat[5], DVA_None);
    DDefFloat(NULL, "AP.drvWarnDebugPort.rollForwardDist", "", &drvWarnDebugPort.debugFloat[6], DVA_None);
    DDefFloat(NULL, "AP.drvWarnDebugPort.whpIntState", "", &drvWarnDebugPort.debugFloat[7], DVA_None);

    //toneOutputPort
    registerSignalHeaderToDVA("AP.toneOutputPort.sSigHeader", toneOutputPort.sSigHeader);
    DDefUChar(NULL, "AP.toneOutputPort.FrontPitch_nu", "", (uint8_t*)&toneOutputPort.speakerOutput[0].pitch_nu, DVA_None);
    DDefUChar(NULL, "AP.toneOutputPort.RearPitch_nu", "", (uint8_t*)&toneOutputPort.speakerOutput[1].pitch_nu, DVA_None);
    DDefUChar(NULL, "AP.toneOutputPort.FrontVolume_nu", "", (uint8_t*)&toneOutputPort.speakerOutput[0].volume_nu, DVA_None);
    DDefUChar(NULL, "AP.toneOutputPort.RearVolume_nu", "", (uint8_t*)&toneOutputPort.speakerOutput[1].volume_nu, DVA_None);
    DDefUChar(NULL, "AP.toneOutputPort.FrontSoundOn_nu", "", (uint8_t*)&toneOutputPort.speakerOutput[0].soundOn_nu, DVA_None);
    DDefUChar(NULL, "AP.toneOutputPort.RearSoundOn_nu", "", (uint8_t*)&toneOutputPort.speakerOutput[1].soundOn_nu, DVA_None);

    //wheel protection processing
    DDefUChar(NULL, "AP.whpOutputPort.whlWarningLevel_nu_FL", "", (uint8_t*)&whpOutputPort.whlWarningLevel_nu[0], DVA_None);
    DDefUChar(NULL, "AP.whpOutputPort.whlWarningLevel_nu_FR", "", (uint8_t*)&whpOutputPort.whlWarningLevel_nu[1], DVA_None);
    DDefUChar(NULL, "AP.whpOutputPort.whlWarningLevel_nu_RL", "", (uint8_t*)&whpOutputPort.whlWarningLevel_nu[2], DVA_None);
    DDefUChar(NULL, "AP.whpOutputPort.whlWarningLevel_nu_RR", "", (uint8_t*)&whpOutputPort.whlWarningLevel_nu[3], DVA_None);
    DDefUChar(NULL, "AP.whpOutputPort.whlWarningPresent_nu", "", (uint8_t*)&whpOutputPort.whlWarningPresent_nu, DVA_None);
    DDefUChar(NULL, "AP.whpOutputPort.processingError_nu", "", (uint8_t*)&whpOutputPort.processingError_nu, DVA_None);
    registerSignalHeaderToDVA("AP.whpOutputPort.sSigHeader", whpOutputPort.sSigHeader);

    //pdcpSectorsPort
    registerSignalHeaderToDVA("AP.pdcpSectorsPort.sSigHeader", pdcpSectorsPort.sSigHeader);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_0.smallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[0].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_1.smallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[1].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_2.smallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[2].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_3.smallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[3].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_0.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[0].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_1.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[1].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_2.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[2].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsFront_3.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsFront[3].dynamicSmallestDistance_m, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_0.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[0].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_1.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[1].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_2.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[2].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_3.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[3].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_0.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[0].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_1.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[1].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_2.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[2].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_3.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[3].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_0.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[0].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_1.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[1].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_2.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[2].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsFront_3.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsFront[3].intersectsDrvTube_nu, DVA_None);

    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_0.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[0].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_1.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[1].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_2.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[2].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_3.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[3].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_0.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[0].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_1.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[1].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_2.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[2].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRear_3.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRear[3].dynamicSmallestDistance_m, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_0.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[0].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_1.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[1].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_2.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[2].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_3.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[3].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_0.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[0].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_1.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[1].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_2.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[2].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_3.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[3].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_0.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[0].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_1.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[1].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_2.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[2].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRear_3.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRear[3].intersectsDrvTube_nu, DVA_None);

    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_0.smallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[0].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_1.smallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[1].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_2.smallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[2].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_3.smallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[3].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_0.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[0].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_1.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[1].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_2.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[2].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsLeft_3.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsLeft[3].dynamicSmallestDistance_m, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_0.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[0].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_1.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[1].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_2.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[2].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_3.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[3].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_0.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[0].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_1.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[1].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_2.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[2].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_3.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[3].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_0.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[0].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_1.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[1].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_2.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[2].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsLeft_3.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsLeft[3].intersectsDrvTube_nu, DVA_None);

    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_0.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[0].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_1.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[1].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_2.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[2].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_3.smallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[3].smallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_0.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[0].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_1.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[1].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_2.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[2].dynamicSmallestDistance_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpSectorsPort.sectorsRight_3.dynamicSmallestDistance_m", "m", &pdcpSectorsPort.sectorsRight[3].dynamicSmallestDistance_m, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_0.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[0].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_1.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[1].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_2.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[2].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_3.criticalityLevel_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[3].criticalityLevel_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_0.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[0].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_1.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[1].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_2.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[2].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_3.scanned_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[3].scanned_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_0.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[0].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_1.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[1].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_2.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[2].intersectsDrvTube_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpSectorsPort.sectorsRight_3.intersectsDrvTube_nu", "", (uint8_t*)&pdcpSectorsPort.sectorsRight[3].intersectsDrvTube_nu, DVA_None);

    //pdcpDrivingTubePort
    registerSignalHeaderToDVA("AP.pdcpDrivingTubePort.sSigHeader", pdcpDrivingTubePort.sSigHeader);
    DDefFloat(NULL, "AP.pdcpDrivingTubePort.frontRadius_m", "m", &pdcpDrivingTubePort.frontRadius_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpDrivingTubePort.rearRadius_m", "m", &pdcpDrivingTubePort.rearRadius_m, DVA_None);
    DDefFloat(NULL, "AP.pdcpDrivingTubePort.turningCircleCenter_nu.X", "m", &pdcpDrivingTubePort.turningCircleCenter_nu.x_dir, DVA_None);
    DDefFloat(NULL, "AP.pdcpDrivingTubePort.turningCircleCenter_nu.Y", "m", &pdcpDrivingTubePort.turningCircleCenter_nu.y_dir, DVA_None);
    DDefUChar(NULL, "AP.pdcpDrivingTubePort.drvTubeDisplay_nu", "", (uint8_t*)&pdcpDrivingTubePort.drvTubeDisplay_nu, DVA_None);
    DDefUChar(NULL, "AP.pdcpDrivingTubePort.straightDrvTube_nu", "", (uint8_t*)&pdcpDrivingTubePort.straightDrvTube_nu, DVA_None);

    //pdwProcToLogicPort
    registerSignalHeaderToDVA("AP.pdwProcToLogicPort.sSigHeader", pdwProcToLogicPort.sSigHeader);
    DDefFloat(NULL, "AP.pdwProcToLogicPort.minimalDistance_m", "m", &pdwProcToLogicPort.minimalDistance_m, DVA_None);
    DDefUChar(NULL, "AP.pdwProcToLogicPort.processingError_nu", "", (uint8_t*)&pdwProcToLogicPort.processingError_nu, DVA_None);

    //pdcpDebugPort
    registerSignalHeaderToDVA("AP.pdcpDebugPort.sSigHeader", pdcpDebugPort.sSigHeader);
    DDefUChar(NULL, "AP.pdcpDebugPort.validNumOfShapes", "", (uint8_t*)&pdcpDebugPort.debugInt[1], DVA_None);

    DDefFloat(NULL, "SI.trafficObject.Vehicle.sRoad", "m", &sRoadT01_m, DVA_None);
    DDefFloat(NULL, "SI.trafficObject.parkingLaneMarking.sRoad", "m", &sRoadLim01_m, DVA_None);
    DDefFloat(NULL, "SI.trafficObject.furthestObject.sRoad", "m", &sRoadFurthestObject_m, DVA_None);
    DDefFloat(NULL, "SI.trafficObject.furthestObject.yaw", "rad", &yawAngleFurthestObject_rad, DVA_None);

    //avgaPort
    DDefUChar(NULL, "AP.avgaSupervisionRequestLSCA.supervisionRequest", "", &avgaSupervisionRequestLSCA.supervisionRequest, DVA_None);
    DDefUChar(NULL, "AP.avgaSupervisionRequestPDW.supervisionRequest", "", &avgaSupervisionRequestPDW.supervisionRequest, DVA_None);
    DDefUChar(NULL, "AP.automatedVehicleGuidanceStateAUP.selfDrivingState", "", (uint8_t*)&automatedVehicleGuidanceStateAUP.selfDrivingState, DVA_None);
    DDefUChar(NULL, "AP.avgaStopRequestMCRA.stopRequest", "", &avgaStopRequestMCRA.stopRequest, DVA_None);

    //SI delimiter zones
    SiUtility::getInstance().registerDVAVariables();

#ifndef VARIANT_CUS_ONLY
    //cem outputs
    registerCemOutputsAsDvaVariables(
        dynamic_environment_subscriber,
        ego_motion_at_cem_output_subscriber,
        parking_slot_detection_output_subscriber,
        pcl_output_subscriber,
        sgf_output_subscriber,
        stop_Line_output_subscriber,
        pedes_cross_output_subscriber);
    groundTruthEnvironment.registerCarMakerDVAs();

    //SvcModel
    if (!isSvcModelProcessingDataAllocated) {
        svcModelProcessingInput = SvcModelWrapper::Instance().AllocateInput();
        svcModelProcessingOutput = SvcModelWrapper::Instance().AllocateOutput();
        svcModelProcessingSettings = SvcModelWrapper::Instance().AllocateSettings();
        isSvcModelProcessingDataAllocated = true;
    }

    SvcModelUtils::declQuants(svcModelProcessingInput, svcModelProcessingOutput, svcModelProcessingSettings);
#endif

    if (MP == nullptr) return;
}

// The given function is local and not referenced in the body of the module; therefore, the function is dead code.
// Uncomment this function if is need it

//static SI::ApParkingBoxPort transformParkingBoxPort(SI::ApParkingBoxPort parkingBoxPortInput) {
//    if (slotCtrlPort.resetOriginRequestPort.resetCounter_nu != prevResetOriginRequestPort.resetCounter_nu) {
//        prevResetOriginRequestPort = slotCtrlPort.resetOriginRequestPort;
//        switch (prevResetOriginRequestPort.resetOrigin_nu) {
//        case AP_PSM::ResetOriginType::RRT_NONE:
//            //TODO: not expected to have a change in counter but no type
//            assert(0);
//            break;
//        case AP_PSM::ResetOriginType::RRT_RESET_PSI:
//            prevResetOriginRequestPort.transformation.yaw_rad = envModelPort.egoVehiclePoseForAP.yaw_rad;
//            prevResetOriginRequestPort.transformation.pos = { 0.0F, 0.0F };
//            break;
//        case AP_PSM::ResetOriginType::RRT_RESET_XY:
//            prevResetOriginRequestPort.transformation.yaw_rad = 0.0F;
//            prevResetOriginRequestPort.transformation.pos = envModelPort.egoVehiclePoseForAP.pos;
//            break;
//        case AP_PSM::ResetOriginType::RRT_RESET_XY_PSI:
//            prevResetOriginRequestPort.transformation.yaw_rad = envModelPort.egoVehiclePoseForAP.yaw_rad;
//            prevResetOriginRequestPort.transformation.pos = envModelPort.egoVehiclePoseForAP.pos;
//            break;
//        case AP_PSM::ResetOriginType::RRT_RESET_CUSTOM:
//        default:
//            //keep copied origin
//            break;
//        }
//    }
//    if (prevResetOriginRequestPort.resetCounter_nu > 0) {
//        LSM_GEOML::CoordinateTransformer2D transformToNewCS(prevResetOriginRequestPort.transformation);
//        inverseTransformation = transformToNewCS.inverseTransform({ slotCtrlPort.resetOriginRequestPort.transformation.pos, slotCtrlPort.resetOriginRequestPort.transformation.yaw_rad });
//
//        //transform parking box
//        for (unsigned int i = 0; i < parkingBoxPortInput.numValidParkingBoxes_nu; i++) {
//            for (auto& pbpPos : parkingBoxPortInput.parkingBoxes[i].slotCoordinates_m) {
//                pbpPos = transformToNewCS.inverseTransform(pbpPos);
//            }
//        }
//    }
//    return parkingBoxPortInput;
//}

struct JsonDataCollection {
    si::ApEnvModelPort emData;
    si::ApParkingBoxPort parkingBoxPort;
    si::EgoMotionPort egoMotion;
    ap_tp::TargetPosesPort targetPosesPort;
    std::list<LSM_GEOML::Pose> drivenPath;
    std::list<LSM_GEOML::Pose> plannedPath;
};

#ifndef VARIANT_CUS_ONLY
static uint32_t legacyWriteStaticObjectClass(si::StaticObjectClass provisionalValue) {
    switch (provisionalValue) {
    case si::StaticObjectClass::STAT_OBJ_VEHICLE:                return 0U;
    case si::StaticObjectClass::STAT_OBJ_WHEEL_STOPPER:          return 1U;
    case si::StaticObjectClass::STAT_OBJ_POLE:                   return 2U;
    case si::StaticObjectClass::STAT_OBJ_CURB:                   return 3U;
    case si::StaticObjectClass::STAT_OBJ_MAX_NUM_TYPES:          return 5U;
    default /*si::StaticObjectClass::STAT_OBJ_UNCLASSIFIED_STRUCTURE:*/: return 4U;
    }
}
#endif

static uint32_t legacyWriteStaticObjHeigthType(si::StaticObjHeigthType provisionalValue) {
    switch (provisionalValue) {
    case si::StaticObjHeigthType::SO_HI_WHEEL_TRAVERSABLE:      return 0U; /* WHEEL_TRAVERSABLE */
    case si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE:       return 1U; /* BODY_TRAVERSABLE */
    case si::StaticObjHeigthType::SO_HI_DOOR_OPENABLE:          return 2U; /* DOOR_OPENABLE */
    case si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE:          return 3U; /* HIGH_OBSTACLE */
    case si::StaticObjHeigthType::SO_HI_HANGING_OBJECT:         return 4U; /* HANGING_OBJECT */
    default /*si::StaticObjHeigthType::SO_HI_UNKNOWN:*/:        return 5U; /* MAX_NUM_HEIGHT_TYPES */
    }
}
/**
 * Export environment model, target poses and trajectories to json.
 * @param   filename    Filename of the json file.
 * @param   dataList    List of EM data.
 */

static void saveEMToJson(const char* filename,
    const std::list<JsonDataCollection>& dataList) {
    // update internal cache


    // copied from pep_demo/monitormenus.cpp
    //
    // Open file
    //
    FILE * pFile = fopen(filename, "w");
    if (pFile) {
        fprintf(pFile, "{\n  \"Input\": [");

        bool firstData = true;

        for (auto const& data : dataList) {
            auto const& emData = data.emData;
            auto const& parkingBoxPort = data.parkingBoxPort;

            if (firstData) {
                firstData = false;
            }
            else {
                fprintf(pFile, ",");
            }

            fprintf(pFile, "\n  {\n");

            //
            // Write dfiOutput.dfiEnvModel;
            //
            fprintf(pFile, "    \"envModelPort\": {\n");

            // find number of static objects
            uint32_t maxObstacles = 0;
            for (uint32_t iP = 0; ((iP < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU) && (iP < emData.numberOfStaticObjects_u8)); ++iP) {
                if (emData.staticObjects[iP].existenceProb_perc > 0) {
                    maxObstacles = iP + 1;
                }
            }
            fprintf(pFile, "      \"staticStructures\": [");
            for (uint32_t iP = 0; iP < maxObstacles; ++iP) {
                if (iP > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                auto const & p = emData.staticObjects[iP];
                fprintf(pFile, "        {\n");
                fprintf(pFile, "          \"refObjID_nu\": %d,\n", p.refObjID_nu);
#ifndef VARIANT_CUS_ONLY
                fprintf(pFile, "          \"refObjClass_nu\": %d,\n", legacyWriteStaticObjectClass(p.refObjClass_nu));
#endif

                //fprintf(pFile, "        \"refObjOrientationTowardsRoad_nu\": %d,\n", static_cast<uint32_t>(p.refObjOrientationTowardsRoad_nu));

                fprintf(pFile, "          \"existenceProb_perc\": %d,\n", p.existenceProb_perc);

                //fprintf(pFile, "        \"detectingSensors_nu\": [");
                //for (uint32_t i = 0; i < static_cast<uint8_t>(AP_Common::AP_G_MAX_NUM_SENSOR_TYPES_NU); ++i) {
                //    if (i > 0) fprintf(pFile, ", ");
                //    fprintf(pFile, "\n");
                //    fprintf(pFile, "          %s", (p.detectingSensors_nu[i]) ? "true" : "false");
                //}
                //fprintf(pFile, "\n");
                //fprintf(pFile, "        ],\n"); // Closes "detectingSensors_nu": [-

                fprintf(pFile, "          \"readFromNVRAM_nu\": %s,\n", (p.readFromNVRAM_nu) ? "true" : "false");

                fprintf(pFile, "          \"shape\": {\n");
                fprintf(pFile, "            \"pos_m\": [");
                for (uint32_t iV = 0; iV < p.objShape_m.actualSize; ++iV) {
                    if (iV > 0) fprintf(pFile, ",");
                    fprintf(pFile, "\n");
                    fprintf(pFile, "              [%.3f, %.3f]", p.objShape_m.array[iV].x_dir, p.objShape_m.array[iV].y_dir);
                }
                fprintf(pFile, "\n");
                fprintf(pFile, "            ],\n"); // Closes "pos_m": [-
                fprintf(pFile, "            \"numValidPoints_nu\": %d,\n", p.objShape_m.actualSize);
                fprintf(pFile, "            \"heightClass_nu\": %d\n", legacyWriteStaticObjHeigthType(p.objHeightClass_nu));
                fprintf(pFile, "         }\n"); // Closes "shape": {-

                fprintf(pFile, "        }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "      ],\n"); // Closes "staticStructures": [-

#ifndef VARIANT_CUS_ONLY
            // find number of parking space markings
            uint32_t maxParkingSpaceMarkings = 0;
            for (uint32_t iP = 0; ((iP < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_SPACE_MARKINGS_NU) && (iP < emData.numberOfParkMarkings_u8)); ++iP) {
                if (emData.parkingSpaceMarkings[iP].existenceProb_perc > 0) {
                    maxParkingSpaceMarkings = iP + 1;
                }
            }

            fprintf(pFile, "      \"parkingSpaceMarkings\": [");
            for (uint32_t iP = 0; iP < maxParkingSpaceMarkings; ++iP) {
                if (iP > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                auto const & psm = emData.parkingSpaceMarkings[iP];
                fprintf(pFile, "        {\n");

                fprintf(pFile, "            \"pos_m\": [\n");
                fprintf(pFile, "              [%.3f, %.3f],\n", psm.pos_m.array[0].x_dir, psm.pos_m.array[0].y_dir);
                fprintf(pFile, "              [%.3f, %.3f]\n", psm.pos_m.array[1].x_dir, psm.pos_m.array[1].y_dir);
                fprintf(pFile, "            ],\n");
                fprintf(pFile, "          \"existenceProb_perc\": %d\n", psm.existenceProb_perc);
                fprintf(pFile, "        }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "      ]\n"); // Closes "parkingSpaceMarkings": [-
#endif
            fprintf(pFile, "    },\n"); // Closes "EnvModelPort": {-

            // write egoMotionPort
            fprintf(pFile, "    \"egoMotionPort\": {\n");
            fprintf(pFile, "        \"egoVehiclePose\": {\n");
            LSM_GEOML::Pose const & pose = emData.egoVehiclePoseForAP;
            fprintf(pFile, "          \"pos_m\": [%.3f, %.3f],\n", pose.Pos().x(), pose.Pos().y());
            fprintf(pFile, "          \"yaw_rad\": %f\n", pose.Yaw_rad());
            fprintf(pFile, "        }\n"); // Closes "egoVehiclePose"
            fprintf(pFile, "        // Rest of egoMotionPort missing\n");
            fprintf(pFile, "      },\n"); // Closes "egoMotionPort"

            // Write parkingBoxPort;
            fprintf(pFile, "    \"ParkingBoxPort\": {\n");

            fprintf(pFile, "      \"parkingBoxes\": [");
            for (uint32_t iP = 0; iP < parkingBoxPort.numValidParkingBoxes_nu; ++iP) {
                if (iP > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                auto const & p = parkingBoxPort.parkingBoxes[iP];
                fprintf(pFile, "        {\n");
                fprintf(pFile, "          \"parkingBoxID_nu\": %d,\n", p.parkingBoxID_nu);

                fprintf(pFile, "          \"pos_m\": [");
                for (uint32_t iV = 0; iV < 4; ++iV) {
                    if (iV > 0) fprintf(pFile, ",");
                    fprintf(pFile, "\n");
                    fprintf(pFile, "            [%.3f, %.3f]", p.slotCoordinates_m.array[iV].x_dir, p.slotCoordinates_m.array[iV].y_dir);
                }
                fprintf(pFile, "\n");
                fprintf(pFile, "          ],\n"); // Closes "pos_m": [-

                fprintf(pFile, "          \"existenceProb_perc\": %d,\n", p.existenceProb_perc);

                fprintf(pFile, "          \"parkingScenario_nu\": %d,\n", static_cast<uint32_t>(p.parkingScenario_nu));

                fprintf(pFile, "          \"delimiters\": [");
                for (uint32_t i = 0; ((i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_DELIMITERS_NU) && (i < p.numValidDelimiters_nu)); ++i) {
                    if (i > 0) fprintf(pFile, ",");
                    fprintf(pFile, "\n");
                    fprintf(pFile, "            {\n");
                    fprintf(pFile, "              \"indexInList_nu\": %d,\n", p.delimiters[i].indexInList_nu);
#ifndef VARIANT_CUS_ONLY
                    fprintf(pFile, "              \"delimiterType_nu\": %u,\n", (uint8_t)p.delimiters[i].delimiterType_nu);
#endif
                    fprintf(pFile, "              \"delimitingSide_nu\": %u\n", (uint8_t)p.delimiters[i].delimitingSide_nu);
                    fprintf(pFile, "            }");
                }
                fprintf(pFile, "\n");
                fprintf(pFile, "          ],\n"); // Closes "delimiters": [-
                fprintf(pFile, "          \"virtualLines\": [");
                for (uint32_t i = 0; ((i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU) && (i < p.numVirtualLines_nu)); ++i) {
                    auto l = p.virtualLines[i];
                    if (i > 0) fprintf(pFile, ",");
                    fprintf(pFile, "\n");
                    fprintf(pFile, "            {\n");
                    fprintf(pFile, "              \"pos_m\": [ ");
                    fprintf(pFile, "  [%.3f, %.3f], [%.3f, %.3f] ]\n", l.virtLineVertices_m.array[0].x_dir, l.virtLineVertices_m.array[0].y_dir,
                        l.virtLineVertices_m.array[1].x_dir, l.virtLineVertices_m.array[1].y_dir);
                    fprintf(pFile, "            }");
                }
                fprintf(pFile, "\n");
                fprintf(pFile, "          ]\n"); // Closes "virtualLine": [-

#if 0
                fprintf(pFile, "        \"shape\": {  // Does not exist in I/F specification\n");
                fprintf(pFile, "          \"pos_m\": [");
                for (uint32_t iV = 0; iV < 4; ++iV) {
                    if (iV > 0) fprintf(pFile, ",");
                    fprintf(pFile, "\n");
                    fprintf(pFile, "            [%.2f, %.2f]", p.pos_m[iV].x(), p.pos_m[iV].y());
                }
                fprintf(pFile, "\n");
                fprintf(pFile, "          ]\n"); // Closes "pos_m": [-
                fprintf(pFile, "        }\n"); // Closes "shape": {-
#endif
                fprintf(pFile, "        }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "      ],\n"); // Closes "parkingBoxes": [-

            fprintf(pFile, "      \"numValidParkingBoxes_nu\": %d\n", parkingBoxPort.numValidParkingBoxes_nu);

            fprintf(pFile, "    },\n"); // Closes "ParkingBoxPort": {-

            fprintf(pFile, "    \"EgoPath\": [");

            for (auto iter = data.drivenPath.begin(); iter != data.drivenPath.end(); ++iter) {
                if (iter != data.drivenPath.begin()) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                fprintf(pFile, "      {\n");
                fprintf(pFile, "        \"pos_m\": [%.3f, %.3f],\n", iter->Pos().x(), iter->Pos().y());
                fprintf(pFile, "        \"yaw_rad\": %f\n", iter->Yaw_rad());
                fprintf(pFile, "      }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "  ],\n"); // Closes "EgoPath": {

            fprintf(pFile, "    \"PlannedPath\": [");

            for (auto iter = data.plannedPath.begin(); iter != data.plannedPath.end(); ++iter) {
                if (iter != data.plannedPath.begin()) fprintf(pFile, ",");
                fprintf(pFile, "\n");
                fprintf(pFile, "      {\n");
                fprintf(pFile, "        \"pos_m\": [%.3f, %.3f],\n", iter->Pos().x(), iter->Pos().y());
                fprintf(pFile, "        \"yaw_rad\": %f\n", iter->Yaw_rad());
                fprintf(pFile, "      }");
            }
            fprintf(pFile, "\n");
            fprintf(pFile, "  ]\n"); // Closes "PlannedPath": {
            //
            // Close
            //

            fprintf(pFile, "}");

        }
        fprintf(pFile, "\n],");

        fprintf(pFile, "\n  \"ExpectedOutput\": [");

        firstData = true;

        for (auto const& data : dataList) {
            auto const& localTargetPosesPort = data.targetPosesPort;

            if (firstData) {
                firstData = false;
            }
            else {
                fprintf(pFile, ",");
            }

            fprintf(pFile, "\n  {\n");

            // write TargetPosesPort
            fprintf(pFile, "   \"TargetPosesPort\": {\n");
            fprintf(pFile, "        \"targetPoses\": [");

            for (uint32_t iP = 0; iP < localTargetPosesPort.numValidPoses; ++iP) {
                if (iP > 0) fprintf(pFile, ",");
                fprintf(pFile, "\n");

                auto const& tp = localTargetPosesPort.targetPoses[iP];
                fprintf(pFile, "          {\n");
                fprintf(pFile, "            \"pose\": {\n");
                fprintf(pFile, "              \"pos\": {\n");
                fprintf(pFile, "                \"x\": %.3f,\n", tp.pose.x_dir);
                fprintf(pFile, "                \"y\": %.3f\n", tp.pose.y_dir);
                fprintf(pFile, "              },\n"); // Closes pos
                fprintf(pFile, "              \"yaw_rad\": %f\n", tp.pose.yaw_rad);
                fprintf(pFile, "            }, \n"); // Closes pose
                fprintf(pFile, "            \"pose_ID\": %d,\n", static_cast<uint8_t>(tp.pose_ID));
                fprintf(pFile, "            \"relatedParkingBoxID\": %d,\n", static_cast<uint8_t>(tp.relatedParkingBoxID));
                fprintf(pFile, "            \"type\": %d,\n", static_cast<uint8>(tp.type));
                fprintf(pFile, "            \"targetSide\": %d,\n", static_cast<uint8>(tp.targetSide));
                fprintf(pFile, "            \"reachableStatus\": %d\n", static_cast<uint8>(tp.reachableStatus));
                fprintf(pFile, "            \"poseFailReason\": %d\n", static_cast<uint8>(tp.poseFailReason));
                fprintf(pFile, "          }");
            }
            fprintf(pFile, "\n        ],\n"); // Closes "targetPoses": [-
            fprintf(pFile, "        \"numValidPoses\": %d,\n", static_cast<uint8_t>(localTargetPosesPort.numValidPoses));
            fprintf(pFile, "        \"failReason\": %d,\n", static_cast<uint8>(localTargetPosesPort.failReason));
            fprintf(pFile, "        \"anyPathFound\": %s,\n", (localTargetPosesPort.anyPathFound) ? "true" : "false");
            fprintf(pFile, "        \"selectedPoseData\": {\n");
            fprintf(pFile, "          \"selectionStatus\": %d,\n", static_cast<uint8>(localTargetPosesPort.selectedPoseData.selectionStatus));
            fprintf(pFile, "          \"reachedStatus\": %d,\n", static_cast<uint8>(localTargetPosesPort.selectedPoseData.reachedStatus));
            fprintf(pFile, "          \"distanceToStart_m\": %.3f\n", localTargetPosesPort.selectedPoseData.distanceToStart_m);
            fprintf(pFile, "        }\n"); // Closes selectedPoseData
            fprintf(pFile, "    }\n"); // Closes "TargetPosesPort": {-
            fprintf(pFile, "  }");

        }
        // Close file
        fprintf(pFile, "\n  ]\n}\n");
        fclose(pFile);
    }
}

static void saveTrajectoryAsCsv(const uint64_t timeStamp_ms) {
    static bool headerWritten = false;
    const std::string outputDirectory = "SimOutput";
    //Get testrun name from SimCore.TestRun.Name and split by "/" until only the tesrun name remains
    std::string TestRunName = SimCore.TestRun.Name;

    while (TestRunName.find("/") != std::string::npos) {
        TestRunName = TestRunName.substr(TestRunName.find("/") + 1, TestRunName.length() - TestRunName.find("/"));
    }
    std::string outputFileName = outputDirectory + "/" + TestRunName + "_Ego_Trajectory" + ".csv";
    std::ofstream outputFile;

    if (isFirstCycle) {
        outputFile.open(outputFileName.c_str(), std::ios::out | std::ios::trunc);
    }
    else {
        outputFile.open(outputFileName.c_str(), std::ios::app);
    }

    if ((timeStamp_ms % CEM_LSM_CYCLE_TIME) == 0) {
        if (!headerWritten) {
            outputFile << "x, y, z, x_rot, y_rot, z_rot\n";
            headerWritten = true;
        }
        outputFile << Car.Fr1.t_0[0] << ","
            << Car.Fr1.t_0[1] << ","
            << Car.Fr1.t_0[2] << ","
            << Car.Fr1.r_zyx[0] << ","
            << Car.Fr1.r_zyx[1] << ","
            << Car.Fr1.r_zyx[2] << "\n";
        outputFile.close();
    }
}

static bool isScanning(const ap_psm::PlanningCtrlCommands &planningCtrlCommands) {
    return planningCtrlCommands.apState == ap_psm::APState::AP_SCAN_IN || planningCtrlCommands.apState == ap_psm::APState::AP_SCAN_OUT ||
        planningCtrlCommands.gpState == ap_psm::GPState::GP_SCAN_IN || planningCtrlCommands.gpState == ap_psm::GPState::GP_SCAN_OUT ||
        planningCtrlCommands.rmState == ap_psm::RMState::RM_SCANNING;
}

static void resetParkingSoftwareComponents() {
    //ap_common::terminate(); // do not execute as this just kills the MF_Plotter
    ap_common::initialize(carMakerInterface, odoPersistentDataPort,  tcePersDataPort, carMakerSystemServices);
    if (carMakerInterface.environmentModelActive_nu) {
        uspWrapper::getInstance().init();
        UsEmWrapper::getInstance().init();
#ifndef VARIANT_CUS_ONLY
        Reset_CEM();
#endif
    }
    vedodoOffsetAtComponentReset.setRefPose({ odoEstimationPortCM.xPosition_m, odoEstimationPortCM.yPosition_m, odoEstimationPortCM.yawAngle_rad });
#if !defined(VARIANT_CUS_ONLY) && !defined(NO_SCENE_INTERPRETATION) // use SI-high component
    siHighWrapper.initSiAlgorithm(carMakerInterface.vehicle_Params);
    siHighWrapper.resetSiAlgorithm();
    siHighWrapper.addVedodoOffsetTransformation(vedodoOffsetAtComponentReset);
#elif defined(VARIANT_CUS_ONLY) // use SI-low component
    si::SiLowWrapper::getInstance().init(carMakerInterface.vehicle_Params);
    si::SiLowWrapper::getInstance().reset();
    si::SiLowWrapper::getInstance().addVedodoOffsetTransformation(vedodoOffsetAtComponentReset);
#endif
    odoExtCtrlPort.resetPoseEstimation_nu = b_TRUE;
    lastTimeVedodoReset_s = SimCore.Time;
}

static int mdl_APCtrl_Calc(void *MP, double dt)
{
    static std::list<JsonDataCollection> dataCollection;

    static std::list<LSM_GEOML::Pose> drivenPath;

    static std::list<LSM_GEOML::Pose> plannedPath;

    static int egoResetCounter = 0;

    static constexpr float MIN_PATH_POSE_DIST_M = 0.03f; // min distance to previous pose to add path point

#ifdef USE_ENV_PLOTTER
    static MF_Plot::plotterCemObjectList cemObjectsForPlotter{};
    static MF_Plot::DYN_OBJ_LIST_FROM_CEM cemDynObjectsForPlotter{};
    static MF_Plot::OD_SLOT_LIST_FROM_CEM odSlotsForPlotter{};
    static MF_Plot::CusReflectionData cusReflectionData{};
    static MF_Plot::plotterCemLineList cemLinesForPlotter{};
    static si::PlotData siPlotData{};
    static MF_Plot::MemParkData memParkPlotData{};
    static SVCPlotterData svcPlotData{};
#endif

    const uint64_t timeStamp_ms = static_cast<uint64_t>(std::round(SimCore.Time * 1000));

#if defined(_MSC_VER)
    // check if we are about to end the testrun for whatever reason
    if (jsonExportType_nu != JsonExportType::NO_EXPORT) {
        if (dataCollection.empty()) {
            // if the test is about to be stopped, save the EM anyway, even if no target pose was selected
            bool hasValidTargetPose = (SimCore.State >= SCState_EndIdleSet);

            if (jsonExportType_nu == JsonExportType::EXPORT_WHEN_TARGET_POSE_SELECTED) {
                hasValidTargetPose |= slotCtrlPort.planningCtrlCommands.apChosenTargetPoseId_nu < std::numeric_limits<uint8_t>::max()
                    && targetPosesPort.numValidPoses < 3;

                for (int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU; i++) {
                    if (targetPosesPort.targetPoses[i].reachableStatus == ap_tp::PoseReachableStatus::TP_FULLY_REACHABLE) {
                        hasValidTargetPose = true;
                        break;
                    }
                }

            }

            if (jsonExportType_nu == JsonExportType::EXPORT_WHEN_REPLAN_TRIGGERED && trjplaDebugPort.mReplanSuccessful_nu) {
                if (poseBeforePlanning.Pos().norm() > 1e-05) {
                    hasValidTargetPose = cml::Vec2Df(poseBeforePlanning.Pos() - LSM_GEOML::Pose(envModelPort.egoVehiclePoseForAP).Pos()).norm() > LSM_GEOML::MIN_FLT_DIVISOR;
                }
                poseBeforePlanning = envModelPort.egoVehiclePoseForAP;
            }

            if (hasValidTargetPose) {
                dataCollection.push_back(JsonDataCollection{ envModelPort, gParkingBoxPort, egoMotionPort, targetPosesPort, drivenPath, plannedPath });

                // get path
                std::string foldername = jsonFileGenerationRoot_nu + "\\" + SimCore.TestRun.Name;
                std::replace(foldername.begin(), foldername.end(), '/', '\\');

                // recursively create path folders
                // see https://stackoverflow.com/questions/1530760/how-do-i-recursively-create-a-folder-in-win32
                const char *path = foldername.c_str();
                char folder[MAX_PATH];
                const char *end;
                ZeroMemory(folder, sizeof(folder));

                end = strchr(path, '\\');

                while (end != NULL)
                {
                    // Convert folder path to UNICODE before creating directory
                    strncpy(folder, path, end - path + 1);
                    wchar_t wtext[260];
                    mbstowcs(wtext, folder, strlen(folder) + 1);
                    if (!CreateDirectory(wtext, NULL))
                    {
                        DWORD err = GetLastError();

                        if (err != ERROR_ALREADY_EXISTS)
                        {
                            // do whatever handling you'd like
                        }
                    }
                    end = strchr(++end, '\\');
                }

                // finally save parking scene into json
                std::string filename = foldername + SimCore.TestRun.Variation + ".json";
                std::replace(filename.begin(), filename.end(), ',', '_');
                std::replace(filename.begin(), filename.end(), '[', '(');
                std::replace(filename.begin(), filename.end(), ']', ')');

                saveEMToJson(filename.c_str(), dataCollection);

                if (jsonExportType_nu == JsonExportType::EXPORT_WHEN_TARGET_POSE_SELECTED || jsonExportType_nu == JsonExportType::EXPORT_WHEN_REPLAN_TRIGGERED) {
                    SimStop();
                }
            }
        }
    }

#endif

    if (SimCore.State != SCState_Simulate) {
        return 0;
    }

    if (isFirstCycle) {
        //set global parkingOnLeftSide_nu in order to working with SI-Surrogate and SI Component
        // NOTE: For simulation with SI, the parkingOnLeftSide_nu entries can only be correctly related  to the
        // corresponding parking box, if the PB CM traffic objects are defined in the same order as they appear in parkingBoxPort.parkingBoxes
        unsigned numParkingBoxes{ 0U };
        for (unsigned int i = 0; i < (unsigned int)Traffic.nObjs; i++) {
            tTrafficObj* trafficObj = Traffic_GetByTrfId(i);
            if ((trafficObj->Cfg.h < MIN_OBJ_HEIGHT_M) && (trafficObj->Cfg.Name[0] == 'P')) {
                /*only update this on start, since the ego vehicle pose might be on the other side of the PB and we do not want it to toggle*/
                parkingOnLeftSide_nu[numParkingBoxes] = (trafficObj->tRoad > Vehicle.tRoad);
                // HACK: parkingOnLeftSide_nu = false for garage parking, otherwise the curbside zone is at garage entry
                if (strstr(trafficObj->Cfg.Info, "_GP")) { // Garage Parking scenarios are indicated by _GP in ParkingBox description
                    parkingOnLeftSide_nu[numParkingBoxes] = false;
                }
                numParkingBoxes++;
            }
        }
        ap_common::terminate();
        ap_common::initialize(carMakerInterface, odoPersistentDataPort, tcePersDataPort, carMakerSystemServices);

        //////////////////////////////////////////////////////////////
        // start set signal states manually as temporary workaround //
        //////////////////////////////////////////////////////////////

        odoEstimationPortCM.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // Inputs of SI
        //updateSignalHeader(timeStamp_ms * 1000U, odoEstimationOutputPort.odoEstimation.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, odoEstimationOutputPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, slotCtrlPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, slotCtrlPort.planningCtrlCommands.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, userDefinedSlotPort.sSigHeader);

        // Outputs of SI
        //updateSignalHeader(timeStamp_ms * 1000U, envModelPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, collEnvModelPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, egoMotionPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, perceptionAvailabilityPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, memoryParkingStatusPort.sSigHeader);

        // Inputs of TCE & VEDODO
        //updateSignalHeader(timeStamp_ms * 1000U, tceEstimationPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, tcePersDataPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, tceDebugPort.sSigHeader);

#ifdef MOCO_REPLACES_LODMC
        updateSignalHeader(timeStamp_ms * 1000U, tratcoStatusPort.sSigHeader);
        updateSignalHeader(timeStamp_ms * 1000U, longVeconaStatusPort.sSigHeader);
#endif
        // Inputs of ParkSM
        //updateSignalHeader(timeStamp_ms * 1000U, loDMCStatusPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, loDMCCtrlRequestPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, laDMCStatusPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, gearBoxCtrlRequestPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, mfControlStatusPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, lscaStatusPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, targetPosesPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, apUserInteractionPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, drvWarnCoreStatusPort.sSigHeader);

        // Input of MFHMIHandler
        //updateSignalHeader(timeStamp_ms * 1000U, pdcpSectorsPort.sSigHeader);

        // Inputs of DrvWarnSM
        //updateSignalHeader(timeStamp_ms * 1000U, pdwProcToLogicPort.sSigHeader);
        //updateSignalHeader(timeStamp_ms * 1000U, whpOutputPort.sSigHeader);

        //updateSignalHeader(timeStamp_ms * 1000U, gPARKSMCoreStatusPort.sSigHeader);

        ////////////////////////////////////////////////////////////
        // end set signal states manually as temporary workaround //
        ////////////////////////////////////////////////////////////

        SiUtility::getInstance().init(carMakerInterface.vehicle_Params);
        if (carMakerInterface.environmentModelActive_nu)
        {
            uspWrapper::getInstance().init();
			UsEmWrapper::getInstance().init();
        }

        initializeActuators();
        carMakerInterface.headUnitScreen_nu = 0U;
        dataCollection.clear();
        drivenPath.clear();
        plannedPath.clear();
        egoResetCounter = 0;
        lastTransformation = { {0.0F,0.0F},0.0F };
        tapOnStartParkingConfirmed_nu = false;
#ifdef USE_ENV_PLOTTER
        cemObjectsForPlotter.clear();
        cemDynObjectsForPlotter.clear();
        cusReflectionData.clear();
        cemLinesForPlotter.clear();
        siPlotData = {};
        memParkPlotData = {};
        svcPlotData = {};
#endif
        targetPoseKPICheck = {};
        targetPoseReachedStatusSet = false;
    }

    updateOdometryData();

    updateSuspensionData();

#ifdef USE_HMI
    PRIVATE_CAN_RecvLoop(PRIVATE_CAN_Slot_Rx, PRIVATE_CAN_Channel_Rx, timeStamp_ms);
#endif
    updateCANRxData();

    //Fake odometry velocity to make state machine change scanning mode to scanning in during standstill
    //Use in TestRun defined value otherwise default value 250 m/s
    if (carMakerInterface.odoOverwriteVehVelocityValue_mps != 0.0F) carMakerInterface.odoOverwriteVehVelocity_mps = carMakerInterface.odoOverwriteVehVelocityValue_mps;
    else carMakerInterface.odoOverwriteVehVelocity_mps = 250.0f;

    // adjust sceneInterpretationActive_nu according to selected variant and availability of SI
#ifdef VARIANT_CUS_ONLY
    if (!carMakerInterface.sceneInterpretationActive_nu) {
        carMakerInterface.sceneInterpretationActive_nu = true;
        LogWarnStr(EC_General, "For CUS only not supported: AP.sceneInterpretationActive_nu = false!");
    }
#elif defined(NO_SCENE_INTERPRETATION)
    if (carMakerInterface.sceneInterpretationActive_nu) {
        carMakerInterface.sceneInterpretationActive_nu = false;
        LogWarnStr(EC_General, "AP.sceneInterpretationActive_nu = true is not supported since SI is not available!");
    }
#endif
#ifndef VARIANT_CUS_ONLY
    if (!carMakerInterface.sceneInterpretationActive_nu && carMakerInterface.environmentModelActive_nu) {
        carMakerInterface.sceneInterpretationActive_nu = true;
        LogWarnStr(EC_General, "For environment model active (CEM_LSM): AP.sceneInterpretationActive_nu = true is required!");
    }
    // update ground-truth environment that shall be used for CEM_LSM testing
    if ((timeStamp_ms % EnvironmentPerception::SI_HIGH_CYCLE_TIME_MS) == 0U) {
        groundTruthEnvironment.update(parkingOnLeftSide_nu,
            MIN_OBJ_HEIGHT_M,
            pathBeforeActIndex,
            overwritePathBeforeFuncActivation_nu,
            pathBeforeFuncActivation0Pose,
            trafficContour2D_t,
            isFirstCycle,
            odoEstimationPortCM,
            odoEstimationOutputPort);
    }
#endif

    if (carMakerInterface.resetParkingComponents_nu) {
        // TODO do we also have to call resetGlobalsTo0()?
        resetParkingSoftwareComponents();
        carMakerInterface.resetParkingComponents_nu = 0U;
    }

    if (!carMakerInterface.sceneInterpretationActive_nu) {
#ifndef VARIANT_CUS_ONLY
        if ((timeStamp_ms % EnvironmentPerception::SI_HIGH_CYCLE_TIME_MS) == 0U) {
            updateEgoMotionData();
            //Update AUP core interface
            environmentPerception.Init(enableLimitFieldOfView_nu, enableLatencyEffect_nu, latencyEffectTime_s);
            environmentPerception.updateEnvModelData(carMakerInterface.vedodoActive_nu,
                parkingOnLeftSide_nu,
                MIN_OBJ_HEIGHT_M,
                pathBeforeActIndex,
                overwritePathBeforeFuncActivation_nu,
                pathBeforeFuncActivation0Pose,
                trafficContour2D_t,
                inflationLength_m,
                isFirstCycle,
                odoEstimationPortCM,
                odoEstimationOutputPort,
                envModelPort,
                envModelPortCMOrigin,
                collEnvModelPort,
                gParkingBoxPort,
                parkingBoxPortCMOrigin,
                perceptionAvailabilityPort);
            environmentPerception.transformEnvModelData(slotCtrlPort.resetOriginRequestPort,
                prevResetOriginRequestPort,
                lastTransformation,
                inverseTransformation,
                envModelPort,
                envModelPortCMOrigin,
                gParkingBoxPort);
            //        updateNumberOfDelimiters();     // Commented because covered by EnvironmentPerception.updateEnvironmentModelAndParkingBox
        }
#endif
    }
    else { // use Scene Interpretation component (high or low variant)
#ifndef VARIANT_CUS_ONLY
		if (isFirstCycle) {
            MF_Publishers::isPublishingActive = carMakerInterface.environmentModelActive_nu;
            lsm_vedodo::OdoEstimationOutputPort *odoEstimationOTPort2Use = carMakerInterface.vedodoActive_nu ? &odoEstimationOutputPort : &odoEstimationOutputPortCM;
            iniOdoEstimationPort(odoEstimationOTPort2Use);
            //us processing + us_em to eCAL
            iniUssPointListPort(&uspPointListOutput, &uspDistListOutput, &usEnvModelPort);
            //svc to eCal
            iniCemInputDataPort(svcModelProcessingOutput.data);
        }
#endif
        if (isFirstCycle || carMakerInterface.resetCemSurrogate_nu) {
            cemSurrogate.resetHistory();
            cemSurrogateConfig.latencyTime_ms = enableLatencyEffect_nu ? (latencyEffectTime_s*1000.0f) : 0.0f;
            cemSurrogateConfig.staticOffsetX_m = 0.0f;
            cemSurrogateConfig.staticOffsetY_m = 0.0f;
#ifdef VARIANT_CUS_ONLY
            cemSurrogateConfig.disableCameras_nu = 1U;
#endif
            cemSurrogate.init(cemSurrogateConfig);
            carMakerInterface.resetCemSurrogate_nu = 0U;
        }

        // use vCUS-Phen-Model ouput to execute us_processing component
        if (carMakerInterface.environmentModelActive_nu) {
            vCUS_Main(); // execute vCUS-Phen-Model
            usDrvDetectionListCopy = *UsRaw; // use copy of UsRaw to be independent from memory allocation in vCUS-Phen-Model
            if ((timeStamp_ms > 0U) && (timeStamp_ms % US_PROCESSING_SAMPLE_TIME_MS == 0U)) {
                const auto &odoOutputPortToUse = carMakerInterface.vedodoActive_nu ? odoEstimationOutputPort : odoEstimationOutputPortCM;
                uspWrapper::getInstance().run(timeStamp_ms, odoOutputPortToUse, ambientDataPort, usDrvDetectionListCopy,
                    uspPointListOutput, usFilteredEchoOutput, uspDistListOutput, uspDiagOutput, uspIntegrityOutput,
                    usDrvRntConfigRequest);
				UsEmWrapper::getInstance().run(
					// inputs
					odoOutputPortToUse, uspPointListOutput, uspDistListOutput, uspDiagOutput,
					// outputs
					usEnvModelPort, usPercAvailPort, usEmDebugPort);
#ifdef USE_ENV_PLOTTER
#ifdef VARIANT_CUS_ONLY
				UsEmWrapper::convertToCemObjectsForPlotter(usEnvModelPort, cemObjectsForPlotter);
#endif
                /************** vCUS ground truth for mf_plot*/
                cusReflectionData.clear();
                MF_Plot::CusReflectionDataPerID tempStru{};
                for (int iCycle = 0; iCycle < _MF_PLOT_HISTORY_; iCycle++) {
                    if (MfPlotDataStru[iCycle].vCUS_CycleId == -1 || MfPlotDataStru[iCycle].nReflPnts == 0) {
                        continue;
                    }

                    tempStru.id = iCycle;// MfPlotDataStru[iCycle].mfp_CycleId;
                    for (int iPnt = 0; iPnt < MfPlotDataStru[iCycle].nReflPnts; iPnt++) {
                        tempStru.points.append(cml::Vec2Df{ MfPlotDataStru[iCycle].CoordArrayFr1[iPnt][0],
                                                            MfPlotDataStru[iCycle].CoordArrayFr1[iPnt][1] });
                    }
                    cusReflectionData.append(tempStru);
                    tempStru = {};
                }
                /**************/
#endif
            }
        }

#if !defined(VARIANT_CUS_ONLY) && !defined(NO_SCENE_INTERPRETATION) // use SI-high component
        if (carMakerInterface.environmentModelActive_nu) {
            // initialize and execute the phenomenological SVC sensor model
            if (isFirstCycle) {
                auto settingsPath = std::string(PathJoin(2, SimCore.TestRig.AppDir, SvcModelUtils::settingsFilePath));

                if (isSvcModelProcessingDataAllocated) {
                    size_t maxLength = std::min(svcModelProcessingSettings.maxFilenameLength, settingsPath.size() + 1);
                    strcpy_s(svcModelProcessingSettings.configFilename, svcModelProcessingSettings.maxFilenameLength,
                        settingsPath.substr(0, maxLength - 1).c_str());

                    SvcModelWrapper::Instance().init(svcModelProcessingSettings);
                }
            }

            if ((timeStamp_ms > 0U) && (timeStamp_ms % SVC_MODEL_SAMPLE_TIME_MS == 0U))
            {
                const uint64_t timeStamp_us = static_cast<uint64_t>(std::round(1e6 * SimCore.Time));
                if (isSvcModelProcessingDataAllocated) {
                    LSM_GEOML::Pose const egoVehiclePose{ carMakerInterface.vedodoActive_nu ? odometryDataManger.getAbsoluteOdoPose() : getPose(odoEstimationPortCM) };
                    std::vector<VCEM::ODSlot> const& ODSlots{ cemSurrogate.determineODSlots(trafficContour2D_t, egoVehiclePose, true, true) };
                    SvcModelUtils::assignODSlots(ODSlots);  // get read access to the traffic contours in the SVC surrogate model
                    SvcModelWrapper::Instance().ResetInput(svcModelProcessingInput);
                    SvcModelUtils::fillSvcModelInputByTrafficObjects(svcModelProcessingInput, timeStamp_us, svcModelProcessingSettings);
                    SvcModelWrapper::Instance().process(svcModelProcessingInput, svcModelProcessingOutput);
#ifdef USE_ENV_PLOTTER
                    SvcModelUtils::copySVCData(svcModelProcessingOutput, svcPlotData);
#endif
                }
            }
        }

        if (isFirstCycle) {
            siHighWrapper.initSiAlgorithm(carMakerInterface.vehicle_Params);
            siHighWrapper.resetSiAlgorithm();
            slotCtrlPort.planningCtrlCommands.apChosenTargetPoseId_nu = 255U; // initialize with default value
        }
        if ((timeStamp_ms % SiHighWrapper::SI_HIGH_CYCLE_TIME_MS) == 0U) {
            const auto &odoEstimationPortToUse = carMakerInterface.vedodoActive_nu ? odoEstimationOutputPort : odoEstimationOutputPortCM;
            // ego pose used to transform the CEM surrogate output data to ego vehicle coordinates
            const auto egoVehiclePose = carMakerInterface.vedodoActive_nu ? vedodoOffsetAtComponentReset.transform(odometryDataManger.getAbsoluteOdoPose()) : getPose(odoEstimationPortCM);
            const auto& convexHulls = cemSurrogate.determineObstacles(MIN_OBJ_HEIGHT_M, isScanning(slotCtrlPort.planningCtrlCommands), trafficContour2D_t,
                SiHighWrapper::SI_HIGH_CYCLE_TIME_MS, egoVehiclePose);
            const auto& pclDelimiters = cemSurrogate.determineDelimiters(isScanning(slotCtrlPort.planningCtrlCommands), trafficContour2D_t,
                SiHighWrapper::SI_HIGH_CYCLE_TIME_MS, egoVehiclePose);
            const auto& ODSlots = cemSurrogate.determineODSlots(trafficContour2D_t, egoVehiclePose);
            siHighWrapper.runSiAlgorithm(
                systemTimePort.sSigHeader.uiTimeStamp, slotCtrlPort, odoEstimationPortToUse, convexHulls, pclDelimiters, ODSlots, userDefinedSlotPort, targetPosesPort, odoGpsPort, carMakerInterface.environmentModelActive_nu,
                // outputs from SI
                envModelPort, gParkingBoxPort, collEnvModelPort, perceptionAvailabilityPort, egoMotionPort, memoryParkingStatusPort
#ifdef USE_ENV_PLOTTER
                , siPlotData
#endif
            );

            envModelPortCMOrigin = envModelPort;
            parkingBoxPortCMOrigin = gParkingBoxPort;
            // FIXME if carMakerInterface.vedodoActive_nu, call SI a second time with odoEstimationPortCM to fill egoMotionPortCM with SI result based on odoEstimationPortCM
            egoMotionPortCM = egoMotionPort;
#ifdef USE_ENV_PLOTTER
            const auto memParkDebugData{ siHighWrapper.getDebugData() };
            const auto relocDebugData{ siHighWrapper.getRelocPlotData() };
            //Parse MemParkData
            memParkPlotData.relocalizationDelta = { {memParkDebugData.oldPoseToCurrentPose.x, memParkDebugData.oldPoseToCurrentPose.y, memParkDebugData.oldPoseToCurrentPose.phi },
                                                    (b_TRUE == memParkDebugData.oldPoseToCurrentPose.valid) };
            memParkPlotData.userInput = { {memParkDebugData.userDeltaPose.x, memParkDebugData.userDeltaPose.y, memParkDebugData.userDeltaPose.phi },
                                                    (b_TRUE == memParkDebugData.userDeltaPose.valid) };
            memParkPlotData.userDefinedSlotSide = static_cast<mf_hmih::UserDefinedSlotSide>(memParkDebugData.userRequestSide);
            memParkPlotData.userDefinedSlotType = static_cast<mf_hmih::UserDefinedSlotType>(memParkDebugData.userRequestType);

            memParkPlotData.scanRoi = relocDebugData.scanRoi;
            memParkPlotData.adjustedMap = relocDebugData.adjustedMap;
            memParkPlotData.referenceMap = relocDebugData.referenceMap;
            memParkPlotData.candidateMap = relocDebugData.candidateMap;

            if (carMakerInterface.environmentModelActive_nu == 1) {
                CEMSubscribeStaticObjects2Plotter(sgf_output_subscriber, cemObjectsForPlotter);
                CEMSubscribeDynObj2Plotter(dynamic_environment_subscriber, cemDynObjectsForPlotter);
                CEMSubscribeLines2Plotter(pcl_output_subscriber, cemLinesForPlotter);
                CEMSubscribeODSlots2Plotter(parking_slot_detection_output_subscriber, odSlotsForPlotter);
            }
            else {
                VCEM::CemSurrogate::convertToCemObjectsForPlotter(convexHulls, cemObjectsForPlotter, cemDynObjectsForPlotter);
                VCEM::CemSurrogate::convertToCemODSlotsForPlotter(ODSlots, odSlotsForPlotter);
                VCEM::CemSurrogate::convertToCemLinesForPlotter(pclDelimiters, cemLinesForPlotter);
            }
            SiUtility::getInstance().updateDelimiterZones(siPlotData.high.delimiterZones, getPose(odoEstimationPortToUse.odoEstimation));
#endif

            // set inverseTransformation which is later used to transform possibletargetPosesPortCMOrigin, taposdDebugPortCMOrigin and trjplaVisuPortCMOrigin
            inverseTransformation = siHighWrapper.getCoordinateTransformer().getSiToWorldTransformation();
            if (inverseTransformation.Pos().norm1() > 0.0f) {
                siHighWrapper.getCoordinateTransformer().transformToWorldCoordinates(envModelPortCMOrigin);
                siHighWrapper.getCoordinateTransformer().transformToWorldCoordinates(parkingBoxPortCMOrigin);
            }
        }
#elif defined(VARIANT_CUS_ONLY) // use SI-low component
        if (isFirstCycle) {
            si::SiLowWrapper::getInstance().init(carMakerInterface.vehicle_Params);
            si::SiLowWrapper::getInstance().reset();
        }
        if ((timeStamp_ms % si::SI_LOW_CYCLE_TIME_MS) == 0U) {
            // use CEM surrogate model
            if (!carMakerInterface.environmentModelActive_nu) {
                // ego pose used to transform the CEM surrogate output data to ego vehicle coordinates
                const auto egoVehiclePose = carMakerInterface.vedodoActive_nu ? vedodoOffsetAtComponentReset.transform(odometryDataManger.getAbsoluteOdoPose()) : getPose(odoEstimationPortCM);
                const auto& convexHulls = cemSurrogate.determineObstacles(MIN_OBJ_HEIGHT_M, isScanning(slotCtrlPort.planningCtrlCommands), trafficContour2D_t,
                    si::SI_LOW_CYCLE_TIME_MS, egoVehiclePose);
                updateSignalHeader(timeStamp_ms * 1000U, usEnvModelPort.sSigHeader);
                updateSignalHeader(timeStamp_ms * 1000U, usPercAvailPort.sSigHeader);
                si::SiLowWrapper::convertToUsEnvModelPort(systemTimePort.sSigHeader.uiTimeStamp, convexHulls, usEnvModelPort);
                si::SiLowWrapper::update(usPercAvailPort);
#ifdef USE_ENV_PLOTTER
                VCEM::CemSurrogate::convertToCemObjectsForPlotter(convexHulls, cemObjectsForPlotter, cemDynObjectsForPlotter);
#endif
            }
            const auto &odoOutputPortToUse = carMakerInterface.vedodoActive_nu ? odoEstimationOutputPort : odoEstimationOutputPortCM;
            const auto siLowResult = si::SiLowWrapper::getInstance().run(
                odoOutputPortToUse, slotCtrlPort, usEnvModelPort, usPercAvailPort
                // outputs from SI
                , perceptionAvailabilityPort
                , egoMotionPort
                , collEnvModelPort
#ifdef USE_ENV_PLOTTER
                , siPlotData
#endif
            );
            updateEgoMotionPort(odoEstimationPortCM, egoMotionPortCM);
            if (!siLowResult.second.empty()) {
                std::string errorMsg = "Running the SI-low algorithm was not successful:\n\t" + siLowResult.second;
                LogErrStr(EC_General, errorMsg.c_str());
            }
            else {
                const auto &siOutput = siLowResult.first;
                envModelPort = *siOutput.environmentModel;
                envModelPortCMOrigin = *siOutput.environmentModel;
                gParkingBoxPort = *siOutput.parkingBoxes;
                parkingBoxPortCMOrigin = *siOutput.parkingBoxes;

                // set inverseTransformation which is later used to transform possibletargetPosesPortCMOrigin, taposdDebugPortCMOrigin and trjplaVisuPortCMOrigin
                inverseTransformation = si::SiLowWrapper::getInstance().getCoordinateTransformer().getSiToWorldTransformation();
                if (inverseTransformation.Pos().norm1() > 0.0f) {
                    si::SiLowWrapper::getInstance().getCoordinateTransformer().transformToWorldCoordinates(envModelPortCMOrigin);
                    si::SiLowWrapper::getInstance().getCoordinateTransformer().transformToWorldCoordinates(parkingBoxPortCMOrigin);
                }
            }
        }
#endif
    }

    //transform OptimalTargetPose in the SI coordinates
    if (envModelPort.resetOriginResult.resetCounter_nu > 0U) {
        const LSM_GEOML::CoordinateTransformer2D transformToCMOrigin(envModelPort.resetOriginResult.originTransformation);
        optimalTargetPoseSIOrigin.pose.Pos() = transformToCMOrigin.inverseTransform(testEvaluation.evaluationPort.optimalTargetPose.pose.Pos());
    }

    //Set number of points of static structure for tclgeo visualization
    for (unsigned int j = 0; j < maxNumStaticStructureForDVA; j++) {
        numValidPointsStaticStructure[j] = static_cast<uint8_t>(envModelPortCMOrigin.staticObjects[j].objShape_m.actualSize);
    }
    //    updateNumberOfDelimiters();      // Commented because covered by EnvironmentPerception.updateEnvironmentModelAndParkingBox
    clock_t startAUPStep_ms = clock();

    chooseCorrectParkingSlot(gHMIOutputPort);

    checkUserInputPossible(gHMIOutputPort);

    ap_common::step(
        timeStamp_ms,
        starterStatusPort,
        trunkLidStatusPort,
        convertibleTopStatusPort,
        steeringColSwitchesStatusPort,
        trailerStatusPort,
        doorStatusPort,
        vehicleOccupancyStatusPort,
        additionalBCMStatusPort,
        accInformationPort,
        keylessStatusPort,
        authenticationStatusPort,
        apCtrlStatusPort,
        escInformationPort,
        externalFunctionStatusPort,
        odoGpsPort,
        odoExtCtrlPort,
        gearboxCtrlStatusPort,
        suspensionPort,
        engineCtrlStatusPort,
        systemTimePort,
        wheelPulsePort,
        wheelDrivingDirectionsPort,
        wheelSpeedPort,
        vehDynamicsPort,
        steerCtrlStatusPort,
        trjctlGeneralInputPort,
        gHMIOutputPort,
        remoteHMIOutputPort,
        visuInputData,
        gParkingBoxPort,
        odoEstimationPortCM,
        envModelPort,
        collEnvModelPort,
        egoMotionPort,
        perceptionAvailabilityPort,
        memoryParkingStatusPort,
#ifdef USE_ENV_PLOTTER
        cemObjectsForPlotter,
        cemDynObjectsForPlotter,
        odSlotsForPlotter,
        cusReflectionData,
        cemLinesForPlotter,
        siPlotData,
        memParkPlotData,
        optimalTargetPoseSIOrigin,
        svcPlotData,
#endif
        odoEstimationOutputPort,
        odoPersistentDataPort,
        tceEstimationPort,
        tcePersDataPort,
        tceDebugPort,
        loDMCStatusPort,
#ifdef MOCO_REPLACES_LODMC
        tratcoStatusPort,
        longVeconaStatusPort,
#endif
        laDMCStatusPort,
        carMakerInterface,
        laDMCCtrlRequestPort,
        loDMCCtrlRequestPort,
#ifdef MOCO_REPLACES_LODMC
        longManeuverRequestPort,
#endif
        gearBoxCtrlRequestPort,
        brakeCtrlStatusPort,
        odoDebugPort,
        psmDebugPort,
        trajCtrlRequestPort,
        psmToSSPPort,
        slotCtrlPort,
        trajCtrlDebugPort,
        mfControlStatusPort,
        taposdDebugPort,
        lscaStatusPort,
        lscaPlotDataPort,
        targetPosesPort,
        plannedTrajPort,
        trjplaDebugPort,
        trjplaVisuPort,
        reverseAssistAvailabilityPort,
        gHmiInputPort,
        headUnitVisualizationPort,
        remoteVisualizationPort,
        gMFHmiHDebugPort,
        userDefinedSlotPort,
        apUserInteractionPort,
        lvmdUserInteractionPort,
        drivingResistancePort,
        gCtrlCommandPort,
        gPARKSMCoreStatusPort,
        gPARKSMCoreDebugPort,
        laCtrlRequestPort,
        loCtrlRequestPort,
        trajRequestPort,
        drvWarnStatusPort,
        drvWarnDebugPort,
        appToCoreSMPort,
        drvWarnCoreStatusPort,
        logicToProcPort,
        pdcpSectorsPort,
        pdcpDrivingTubePort,
        pdwProcToLogicPort,
        pdcpDebugPort,
        toneOutputPort,
        whpOutputPort,
        lvmdStatusPort,
        avgaSupervisionRequestLSCA,
        avgaSupervisionRequestPDW,
        automatedVehicleGuidanceStateAUP,
        avgaStopRequestMCRA,
        carMakerSystemServices);
    clock_t stopAUPStep_ms = clock();

    //Detect when reachedStatus set first time
    //Store target pose
    if ((targetPosesPort.selectedPoseData.reachedStatus != ap_tp::PoseReachedStatus::NO_TP_REACHED_STATUS) && (!targetPoseReachedStatusSet)) {
        targetPoseKPICheck = targetPosesPort.targetPoses[0].pose;
        targetPoseReachedStatusSet = true;
    }
    else {
        targetPoseReachedStatusSet = false;
    }
    //Determine taposd debug deviation based on stored target pose data and current ego pose
    if (targetPoseReachedStatusSet) {
        const LSM_GEOML::CoordinateTransformer2D transformToTPOrigin(targetPoseKPICheck);
        const LSM_GEOML::Pose egoPoseTPOrigin = transformToTPOrigin.inverseTransform(envModelPort.egoVehiclePoseForAP);
        //Overwrite taposd debug deviation for KPI
        taposdDebugPort.longDistToTarget_m = egoPoseTPOrigin.Pos().x();
        taposdDebugPort.latDistToTarget_m = egoPoseTPOrigin.Pos().y();
        taposdDebugPort.yawDiffToTarget_rad = egoPoseTPOrigin.Yaw_rad();
        //Overwrite target pose for car_outside_PB KPI
        targetPosesPort.targetPoses[0].pose = { targetPoseKPICheck.Pos().x(), targetPoseKPICheck.Pos().y(), targetPoseKPICheck.Yaw_rad() };
    }

#ifdef USE_HMI
    updateCANTxData();
    PRIVATE_CAN_Send(PRIVATE_CAN_Slot_Tx, PRIVATE_CAN_Channel_Tx, timeStamp_ms);
#endif

    odometryDataManger.update(odoEstimationOutputPort.odoEstimation);

    //TODO: Update evaluation of HTML reports to cover ParkOut scenario and not check longitudinal deviation
    ap_tp::PoseType selcectedPoseType_nu = targetPosesPort.targetPoses[0].type;
    if (selcectedPoseType_nu == ap_tp::PoseType::T_PAR_PARKING_OUT ||
        selcectedPoseType_nu == ap_tp::PoseType::T_PERP_PARKING_OUT_BWD ||
        selcectedPoseType_nu == ap_tp::PoseType::T_PERP_PARKING_OUT_FWD) {
        longDistToTarget_m = 0.0f;
    }
    else {
        longDistToTarget_m = taposdDebugPort.longDistToTarget_m;
    }

    maxCycleTimeOfAUPStep_ms = float(stopAUPStep_ms - startAUPStep_ms);

    // update data for replanning visualization in IPG Movie
    if (replanCounter_nu != trjplaDebugPort.mNumOfReplanCalls) {
        replanPositionCMOrigin[replanCounter_nu] = { odoEstimationOutputPort.odoEstimation.xPosition_m,
                                                     odoEstimationOutputPort.odoEstimation.yPosition_m,
                                                     odoEstimationOutputPort.odoEstimation.yawAngle_rad };
        if (replanCounter_nu > 0U) {
            previousTargetPose = targetPosesPortCMOrigin.targetPoses[0];
            previousTrajectoryPortCMOrigin = trjplaVisuPortCMOrigin;
        }
        replanCounter_nu++;
    }

    //Inverse transformation of possibleTargetPosesPort for tclgeo visualization
    if (inverseTransformation.Pos().norm1() > 0.0f) {
        const LSM_GEOML::CoordinateTransformer2D transformToCMOrigin(inverseTransformation);
        targetPosesPortCMOrigin = targetPosesPort;
        for (uint8_t i = 0; i < targetPosesPortCMOrigin.numValidPoses; i++)
        {
            LSM_GEOML::Pose transformedTargetPose = transformToCMOrigin.inverseTransform({ targetPosesPortCMOrigin.targetPoses[i].pose.x_dir, targetPosesPortCMOrigin.targetPoses[i].pose.y_dir , targetPosesPortCMOrigin.targetPoses[i].pose.yaw_rad });
            targetPosesPortCMOrigin.targetPoses[i].pose.x_dir = transformedTargetPose.Pos().x();
            targetPosesPortCMOrigin.targetPoses[i].pose.y_dir = transformedTargetPose.Pos().y();
            targetPosesPortCMOrigin.targetPoses[i].pose.yaw_rad = transformedTargetPose.Yaw_rad();
        }
    }
    else {
        targetPosesPortCMOrigin = targetPosesPort;
    }


    //Inverse transformation of taposdDebugPort for tclgeo visualization
    if (inverseTransformation.Pos().norm1() > 0.0f) {
        const LSM_GEOML::CoordinateTransformer2D transformToCMOrigin(inverseTransformation);
        taposdDebugPortCMOrigin = taposdDebugPort;
        for (auto& pdDebugBackwards : taposdDebugPortCMOrigin.pbDebugBackwards)
        {
            for (uint16_t ii = 0; ii < pdDebugBackwards.comfParkingBox.numValidPoints_nu; ii++) {
                LSM_GEOML::Pose transformedPose = transformToCMOrigin.inverseTransform({ cml::Vec2Df{ pdDebugBackwards.comfParkingBox.posX_m[ii], pdDebugBackwards.comfParkingBox.posY_m[ii] }, 0.0f });
                pdDebugBackwards.comfParkingBox.posX_m[ii] = transformedPose.Pos().x();
                pdDebugBackwards.comfParkingBox.posY_m[ii] = transformedPose.Pos().y();
            }
            for (uint16_t ii = 0; ii < pdDebugBackwards.maxParkingBox.numValidPoints_nu; ii++) {
                LSM_GEOML::Pose transformedPose = transformToCMOrigin.inverseTransform({ cml::Vec2Df{ pdDebugBackwards.maxParkingBox.posX_m[ii], pdDebugBackwards.maxParkingBox.posY_m[ii] }, 0.0f });
                pdDebugBackwards.maxParkingBox.posX_m[ii] = transformedPose.Pos().x();
                pdDebugBackwards.maxParkingBox.posY_m[ii] = transformedPose.Pos().y();
            }
        }

        for (auto& pbDebugForwards : taposdDebugPortCMOrigin.pbDebugForwards)
        {
            for (uint16_t ii = 0; ii < pbDebugForwards.comfParkingBox.numValidPoints_nu; ii++) {
                LSM_GEOML::Pose transformedPose = transformToCMOrigin.inverseTransform({ cml::Vec2Df{ pbDebugForwards.comfParkingBox.posX_m[ii], pbDebugForwards.comfParkingBox.posY_m[ii] }, 0.0f });
                pbDebugForwards.comfParkingBox.posX_m[ii] = transformedPose.Pos().x();
                pbDebugForwards.comfParkingBox.posY_m[ii] = transformedPose.Pos().y();
            }
            for (uint16_t ii = 0; ii < pbDebugForwards.maxParkingBox.numValidPoints_nu; ii++) {
                LSM_GEOML::Pose transformedPose = transformToCMOrigin.inverseTransform({ cml::Vec2Df{ pbDebugForwards.maxParkingBox.posX_m[ii], pbDebugForwards.maxParkingBox.posY_m[ii] }, 0.0f });
                pbDebugForwards.maxParkingBox.posX_m[ii] = transformedPose.Pos().x();
                pbDebugForwards.maxParkingBox.posY_m[ii] = transformedPose.Pos().y();
            }
        }
    }
    else {
        taposdDebugPortCMOrigin = taposdDebugPort;
    }


    //Inverse transformation of trjplaDebugPort for tclgeo visualization
    if (inverseTransformation.Pos().norm1() > 0.0f) {
        const LSM_GEOML::CoordinateTransformer2D transformToCMOrigin(inverseTransformation);
        trjplaVisuPortCMOrigin = trjplaVisuPort;
        for (uint16_t ii = 0; ii < trjplaVisuPort.numValidPoses_nu; ii++)
        {
            cml::Vec2Df transformedPoint = transformToCMOrigin.inverseTransform(cml::Vec2Df{ trjplaVisuPort.plannedPathXPos_m[ii], trjplaVisuPort.plannedPathYPos_m[ii] });
            trjplaVisuPortCMOrigin.plannedPathXPos_m[ii] = transformedPoint[0];
            trjplaVisuPortCMOrigin.plannedPathYPos_m[ii] = transformedPoint[1];
        }



        if (envModelPort.resetOriginResult.resetCounter_nu != egoResetCounter) {
            egoResetCounter = envModelPort.resetOriginResult.resetCounter_nu;

            for (auto& pose : drivenPath) {
                pose.Pos() -= cml::Vec2Df{ envModelPort.resetOriginResult.originTransformation.x_dir, envModelPort.resetOriginResult.originTransformation.y_dir };
                pose.Yaw_rad() -= envModelPort.resetOriginResult.originTransformation.yaw_rad;
            }
        }
    }
    else {
        trjplaVisuPortCMOrigin = trjplaVisuPort;
    }
    //calculate the length of the S curve
    if (trjplaVisuPortCMOrigin.numValidSegments > 0U) {
        float32_t localSCurveLength{ 0.0f };
        for (uint8_t idx = 0U; idx < (trjplaVisuPortCMOrigin.numValidSegments - 1U); ++idx)
        {
            float32_t x1{ trjplaVisuPortCMOrigin.plannedGeometricPath[idx].startPose[0] };
            float32_t y1{ trjplaVisuPortCMOrigin.plannedGeometricPath[idx].startPose[1] };
            float32_t x2{ trjplaVisuPortCMOrigin.plannedGeometricPath[idx + 1].startPose[0] };
            float32_t y2{ trjplaVisuPortCMOrigin.plannedGeometricPath[idx + 1].startPose[1] };
            localSCurveLength += sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
        }
        globalSCurveLength_m = localSCurveLength;
    }

    // add ego pose to driven path if position changed

    if (drivenPath.empty() || cml::Vec2Df(cml::Vec2Df{ envModelPortCMOrigin.egoVehiclePoseForAP.x_dir, envModelPortCMOrigin.egoVehiclePoseForAP.y_dir } -drivenPath.back().Pos()).norm() >= MIN_PATH_POSE_DIST_M) {
        drivenPath.push_back(envModelPortCMOrigin.egoVehiclePoseForAP);
    }

    // update planned path
    if (plannedTrajPort.numValidCtrlPoints_nu >= ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS) {
        auto pathEntry = plannedTrajPort.plannedTraj[ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS];
        LSM_GEOML::Pose pose{ cml::Vec2Df { pathEntry.xTrajRAReq_m, pathEntry.yTrajRAReq_m}, pathEntry.yawReq_rad };
        LSM_GEOML::CoordinateTransformer2D transformToOdometryCSRotation({ 0.0F, 0.0F, envModelPortCMOrigin.transformationToOdometry.yaw_rad });
        LSM_GEOML::CoordinateTransformer2D transformToOdometryCSTranslation({ envModelPortCMOrigin.transformationToOdometry.x_dir, envModelPortCMOrigin.transformationToOdometry.y_dir, 0.0F });
        LSM_GEOML::Pose transformedPose = transformToOdometryCSTranslation.inverseTransform(transformToOdometryCSRotation.inverseTransform(pose));

        if (plannedPath.empty() || cml::Vec2Df{ transformedPose.Pos() - plannedPath.back().Pos() }.norm() >= MIN_PATH_POSE_DIST_M) {
            plannedPath.push_back(transformedPose);
        }
    }

    if (TCEActive_nu == 0) {
        ctrlActuators();
    }

    testEvaluation.run(plannedTrajPort,
        targetPosesPortCMOrigin,
        slotCtrlPort,
        envModelPort,
        envModelPortCMOrigin,
        gParkingBoxPort,
        parkingBoxPortCMOrigin,
        carMakerInterface,
        odoEstimationOutputPort.odoEstimation,
        odoEstimationPortCM,
        gearboxCtrlStatusPort.gearInformation.gearCur_nu,
        drivenPath,
        hadGearReq_nu,
        parkingOnLeftSide_nu,
        egoMotionPort.frontWheelAngle_rad,
        odoEstimationPortCM.longiVelocity_mps,
        isFirstCycle,
        mfControlStatusPort,
        trajCtrlDebugPort,
        trafficContour2D_t);

    if (sideslip_simulation_active) {
        const double front_steer_angle_rad{ 0.5 * (Car.Susp[0].SteerAngle + Car.Susp[1].SteerAngle) };
        if (front_steer_angle_rad != 0.0) {
            //if going backwards shift the turning point in the other direction
            const double direction_sign{ (wheelDrivingDirectionsPort.wheelDrivingDirection_FL_nu == ap_commonvehsigprovider::WheelDrivingDirection::WHEEL_DIRECTION_REVERSE) ? -1.0 :
                ((wheelDrivingDirectionsPort.wheelDrivingDirection_FL_nu == ap_commonvehsigprovider::WheelDrivingDirection::WHEEL_DIRECTION_FORWARD) ? 1.0 : 0.0) };

            const double turn_radius_m{ carMakerInterface.vehicle_Params.AP_V_WHEELBASE_M / tan(front_steer_angle_rad) };
            // side-slip angle model from mf_vedodo https://github-am.geo.conti.de/ADAS/mf_vedodo/blob/master/doc/Sideslip_Kinematic_Estimation.md
            const double sideslip_angle_rad{ -direction_sign * atan(sideslip_lh_m / turn_radius_m) };
            Susp_Ext.Kin_tExt[3][ixrz] = sideslip_angle_rad;
            Susp_Ext.Kin_tExt[2][ixrz] = sideslip_angle_rad;
        }
        else {
            Susp_Ext.Kin_tExt[3][ixrz] = 0.0;
            Susp_Ext.Kin_tExt[2][ixrz] = 0.0;
        }
    }
    if (carMakerInterface.trajExportActive_nu) {
        saveTrajectoryAsCsv(timeStamp_ms);
    }

    isFirstCycle = false;
    if (MP == nullptr && dt == 0.0) return 0;
    return 0;
}


static void *
mdl_APCtrl_New(struct tInfos *Inf, const char *KindKey)
{
    char *key;//, MsgPre[64];
    //const char *ModelKind;
    //int VersionId = 0;

#ifdef USE_HMI
    if (Initialize_Vector() != 0) {
        LogErrF(EC_Init, "Vector driver initialization failed");
    }
    PRIVATE_CAN_Init_First();
    PRIVATE_CAN_CAN_Recv = Vector_Recv;
    PRIVATE_CAN_CAN_RecvFD = Vector_RecvFD;
    PRIVATE_CAN_CAN_Send = Vector_Send;
    PRIVATE_CAN_CAN_SendFD = Vector_SendFD;
    PRIVATE_CAN_Init();
#endif

#ifndef VARIANT_CUS_ONLY
    if (!isSvcModelProcessingDataAllocated) {
        svcModelProcessingOutput = SvcModelWrapper::Instance().AllocateOutput();
        svcModelProcessingInput = SvcModelWrapper::Instance().AllocateInput();
        svcModelProcessingSettings = SvcModelWrapper::Instance().AllocateSettings();
        isSvcModelProcessingDataAllocated = true;
    }

    SvcModelUtils::getVehicleOriginPosition();
    SvcModelUtils::getTrafficObjects2DContureMirroring();
#endif

    resetGlobalsTo0();

    // reset random number generator seed to have reproducible simulation runs
    gRandomEngine.seed();

    // Disable all ObjectSensors by default. Models requiring these sensors should re-enable them (e.g. EnvironmentPerception).
    for (int iSens = 0; iSens < ObjectSensorCount; ++iSens) {
        ObjectSensor_Disable(iSens);
    }

#ifdef MOCO_REPLACES_LODMC
    OCTAGONEngine_MaxTrq_DDictEntry = DDictGetEntry("PT.Engine.MaxTrq");
    OCTAGONEngine_MinTrq_DDictEntry = DDictGetEntry("PT.Engine.MinTrq");
    gasInterpretedTrq_DDictEntry = DDictGetEntry("PT.Control.GasInterpret.Trq_trg");
#endif

    double *temp = NULL;
    int numberOfObject = 0;

    // Set not 0 init evaluation values
    testEvaluation.evaluationPort.parkingManeuver_nu = Parking_Maneuver::NOT_SPECIFIED;

    //Check testDescription
    char **testDescription;
    //search in the testrun
    //SimCore.TestRun.Inf - get the testrun info -
    //search Description of the TestRun
    std::string paramName = "Description";

    if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
        testDescription = iGetTxt(SimCore.TestRun.Inf, paramName.c_str());
    }
    else {
        testDescription = NULL;
    }
    if (testDescription != NULL) {
        uint8_t i = 0;
        //check each line of testDescription
        //test description should look:
        //    TestDescription (e.g: Parking space is on opposite side (mirror whole situation horizontally)
        //
        //    EnvironmentModelActive = 0
        //    --- Evaluation Criteria ---
        //    n_strokes_max = 11.0
        //    v_max = 2.78 m/s
        //    t_sim_max = 200.0 s
        //    Scope Base = 1 (yes)
        //    Non-/Use Case = 1 (Use Case)
        //    Maneuver = backward

        //set testEvaluation.evaluationPort.latMaxDeviation_m to -1 so that if it isn't defined, it has value -1
        testEvaluation.evaluationPort.latMaxDeviation_m = -1.f;

        //Read testDescription
        while (testDescription[i] != NULL) {
            //cast to string the line "i" of testDescription to can use find function
            std::string testDescription_s = std::string(testDescription[i]);
            //search the line that contains "n_strokes_max" and fill evaluationPort.n_strokes_max_nu with the max number of strokes
            if (testDescription_s.find("n_strokes_max") != std::string::npos) {
                testEvaluation.evaluationPort.n_strokes_max_nu = static_cast<uint8_t>(returnEvaluationInfoFromTestrun(testDescription_s));
            }
            //search the line that contains "v_max" and fill evaluationPort.v_max with the max velocity allowed
            else if (testDescription_s.find("v_max") != std::string::npos) {
                testEvaluation.evaluationPort.v_max_mps = returnEvaluationInfoFromTestrun(testDescription_s);
            }
            //search the line that contains "t_sim_max" and fill evaluationPort.t_sim_max_s with the max time of testrun simulation
            else if (testDescription_s.find("t_sim_max") != std::string::npos) {
                testEvaluation.evaluationPort.t_sim_max_s = returnEvaluationInfoFromTestrun(testDescription_s);
            }
            //search the line that contains "Scope Base" and fill data in evaluationPort.scopeBase_nu
            else if (testDescription_s.find("Scope Base") != std::string::npos) {
                testEvaluation.evaluationPort.scopeBase_nu = static_cast<bool>(returnEvaluationInfoFromTestrun(testDescription_s));
            }
            //search the line that contains "Non-/Use Case" and fill data in evaluationPort.useCase_nu
            else if (testDescription_s.find("Non-/Use Case") != std::string::npos) {
                testEvaluation.evaluationPort.useCase_nu = static_cast<bool>(returnEvaluationInfoFromTestrun(testDescription_s));
            }
            //search the line that contains "Maneuver" and fill data in evaluationPort.maneuver_nu
            else if (testDescription_s.find("Maneuver") != std::string::npos) {
                testEvaluation.evaluationPort.parkingManeuver_nu = returnManeuverInfoFromTestrun(testDescription_s);
            }
            //search the line that contains "Optimal Target Pose" and set evaluationPort.isOTPPresent_nu to true
            else if (testDescription_s.find("--- Optimal Target Pose ---") != std::string::npos) {
                testEvaluation.evaluationPort.optimalTargetPose.valid = true;
            }
            //search the line that contains "OTP_x" and set evaluationPort.optimalTargetPose.pos.x to true
            else if (testDescription_s.find("OTP_x") != std::string::npos) {
                testEvaluation.evaluationPort.optimalTargetPose.pose.Pos().x() = returnEvaluationInfoFromTestrun(testDescription_s);
            }
            //search the line that contains "OTP_x" and set evaluationPort.optimalTargetPose.pos.y to true
            else if (testDescription_s.find("OTP_y") != std::string::npos) {
                testEvaluation.evaluationPort.optimalTargetPose.pose.Pos().y() = returnEvaluationInfoFromTestrun(testDescription_s);
            }
            //search the line that contains "OTP_x" and set evaluationPort.optimalTargetPose.yaw_rad to true
            else if (testDescription_s.find("OTP_psi") != std::string::npos) {
                testEvaluation.evaluationPort.optimalTargetPose.pose.Yaw_rad() = returnEvaluationInfoFromTestrun(testDescription_s) * LSM_GEOML::LSM_PI / 180.0f;
            }
            //search the line that contains "latMaxDeviation_m" and fill evaluationPort.latMaxDeviation_m with the max lateral deviation allowed
            else if (testDescription_s.find("latMaxDeviation_m") != std::string::npos) {
                testEvaluation.evaluationPort.latMaxDeviation_m = returnEvaluationInfoFromTestrun(testDescription_s);
            }
            //search the line that contains "EnvironmentModelActive" and set carMakerInterface.environmentModelActive_nu accordingly
            else if (testDescription_s.find("EnvironmentModelActive") != std::string::npos) {
                carMakerInterface.environmentModelActive_nu = static_cast<uint8_t>(returnEvaluationInfoFromTestrun(testDescription_s));
            }
            else {
                //do nothing
            }
            ++i;
        }
    }

    // If not already activated via the TestRun description, check in the SimParameters Infofile whether the environment model (CEM_LSM or us_em) should be active.
    if (carMakerInterface.environmentModelActive_nu == 0U) {
        carMakerInterface.environmentModelActive_nu = iEntryExists(SimCore.TestRig.SimParam.Inf, "EnvironmentModelActive") && iGetInt(SimCore.TestRig.SimParam.Inf, "EnvironmentModelActive");
    }
    if (carMakerInterface.trajExportActive_nu == 0U) {
        carMakerInterface.trajExportActive_nu = iEntryExists(SimCore.TestRig.SimParam.Inf, "trajectory_export_active") && iGetInt(SimCore.TestRig.SimParam.Inf, "trajectory_export_active");
    }

#ifndef VARIANT_CUS_ONLY
    CarMakerNode::getReferenceCarmakerNode()->SetCemSimulationActive(carMakerInterface.environmentModelActive_nu);
#endif

    // this snippet was simply moved from higher up, so that the FSpaceSensor_New call is done once (no confusion as to why it isn't called right after this)
    // Adjust the FreeSpaceSensor update rate and cycle offset so that the sensor calculation is synchronous to the CEM surrogate.
#ifdef VARIANT_CUS_ONLY
    const uint64_t cemSurrogate_cycleTime_ms = si::SI_LOW_CYCLE_TIME_MS;
#else
    const uint64_t cemSurrogate_cycleTime_ms = SiHighWrapper::SI_HIGH_CYCLE_TIME_MS;
#endif
    for (int iSens = 0; iSens < FSpaceSensorCount; ++iSens) {
        const std::string keyUpdateRate = "Sensor.FSpace." + std::to_string(iSens) + ".UpdRate";
        const int updateRate_Hz = static_cast<int>(std::round(1000.0 / cemSurrogate_cycleTime_ms));
        iSetFStr(SimCore.Vhcl.Inf, keyUpdateRate.c_str(), std::to_string(updateRate_Hz).c_str());
        const std::string keyCycleOffset = "Sensor.FSpace." + std::to_string(iSens) + ".nCycleOffset";
        const int cycleOffset_nu = static_cast<int>(cemSurrogate_cycleTime_ms) - 1;
        iSetFStr(SimCore.Vhcl.Inf, keyCycleOffset.c_str(), std::to_string(cycleOffset_nu).c_str());
    }
    //FSpaceSensor_New(); // moved lower to include the new freespace parameter changes

#ifndef VARIANT_CUS_ONLY
    // HACK: the tilt for the side cameras need to be changed at runtime, but ONLY for SVC scenarios
    if (carMakerInterface.environmentModelActive_nu) {
        uint32_t countHits{ 0U };
        char const* numHorSegm{ "36" };

        // search for the 4 CAMs, starting from the end (this is done in case the order is changed in the future, or additional sensors are added)
        for (int32_t iSens{ FSpaceSensorCount - 1 }; iSens >= 0; --iSens) {
            std::string const fsParamBase{ "Sensor.FSpace." + std::to_string(iSens) };
            std::string const fsParamName{ fsParamBase + ".name" };
            std::string const fsParamHorSegm{ fsParamBase + ".nHorSegm" };
            std::string const fsParamRot{ fsParamBase + ".rot" };
            char const* fsName{ iGetStr(SimCore.Vhcl.Inf, fsParamName.c_str()) };

            if ((strncmp(fsName, "CAM_F", 5U) == 0) || (strncmp(fsName, "CAM_B", 5U) == 0) ||
                (strncmp(fsName, "CAM_L", 5U) == 0) || (strncmp(fsName, "CAM_R", 5U) == 0)) {
                std::string rotExistingVal{ std::string(iGetStr(SimCore.Vhcl.Inf, fsParamRot.c_str())) };
                rotExistingVal.replace(rotExistingVal.cbegin() + rotExistingVal.find_first_of(" ") + 1U,
                                       rotExistingVal.cbegin() + rotExistingVal.find_last_of(" "),
                                       "19");
                iSetFStr(SimCore.Vhcl.Inf, fsParamHorSegm.c_str(), numHorSegm);
                iSetFStr(SimCore.Vhcl.Inf, fsParamRot.c_str(), rotExistingVal.c_str());
                ++countHits;
            }

            // when the 4 CAMs are found, do not progress further
            if (countHits == 4U) {
                break;
            }
        }
    }
#endif

    FSpaceSensor_New(); // Reload updated FSpaceSensor configuration

    //set in optimalTargetPoseSIOrigin the value read from testRun (CM Origin)
    optimalTargetPoseSIOrigin = testEvaluation.evaluationPort.optimalTargetPose;

    // configure ObjectSensors for vCUS pheno model + us_processing
    if (carMakerInterface.environmentModelActive_nu) {
        const auto& uspSensorParams{ uspWrapper::getInstance().getSensorParameters() };
        assert(uspSensorParams.sensorParameterCount == ObjectSensorCount);
        const double rearOverhang_m{ iGetDblOpt(SimCore.Vhcl.Inf, "Wheel.rl.pos", 1.096) };
        for (int iSens = 0; iSens < ObjectSensorCount; ++iSens) {
            // enable object sensors for vCUS pheno sensor model (they are disabled by default)
            ObjectSensor_Enable(iSens);
            // Take sensor position and orientation from us_processing
            const std::string keySensorPosition = "Sensor.Object." + std::to_string(iSens) + ".pos";
            iSetFStr(SimCore.Vhcl.Inf, keySensorPosition.c_str(), "%f %f %f", uspSensorParams.sensorParameter[iSens].posX_m + rearOverhang_m,
                uspSensorParams.sensorParameter[iSens].posY_m, uspSensorParams.sensorParameter[iSens].posZ_m);
            const std::string keySensorRotation = "Sensor.Object." + std::to_string(iSens) + ".rot";
            iSetFStr(SimCore.Vhcl.Inf, keySensorRotation.c_str(), "%f %f %f", 0.0, uspSensorParams.sensorParameter[iSens].verRot_deg,
                uspSensorParams.sensorParameter[iSens].horRot_deg);
            // adjust object sensor FOVs to the vCUS pheno sensor needs
            const std::string keySensorRange = "Sensor.Object." + std::to_string(iSens) + ".range";
            iSetFStr(SimCore.Vhcl.Inf, keySensorRange.c_str(), "6");
            const std::string keySensorAzimuth = "Sensor.Object." + std::to_string(iSens) + ".alpha";
            iSetFStr(SimCore.Vhcl.Inf, keySensorAzimuth.c_str(), "120");
            const std::string keySensorElevation = "Sensor.Object." + std::to_string(iSens) + ".theta";
            iSetFStr(SimCore.Vhcl.Inf, keySensorElevation.c_str(), "80");
        }
        ObjectSensor_New(); // Reload updated ObjectSensor configuration
    }

    //search number of traffic Object
    paramName = "Traffic.N";
    if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str()))
    {
        numberOfObject = iGetInt(SimCore.TestRun.Inf, paramName.c_str());
    }
    else
    {
        numberOfObject = 0;
    }
    //search and save name,traffic ID, number of rows of 2D contour and points of 2d contour
    for (uint8_t i = 0; i < numberOfObject; i++)
    {
        paramName = "Traffic." + std::to_string(i) + ".Name";
        if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
            trafficContour2D_t[i].name = iGetStr(SimCore.TestRun.Inf, paramName.c_str());
        }
        else
        {
            trafficContour2D_t[i].name = NULL;
        }

        paramName = "Traffic." + std::to_string(i) + ".Basics.Contour.Mirror";
        if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
            trafficContour2D_t[i].isMirroringConstruction_nu = (iGetInt(SimCore.TestRun.Inf, paramName.c_str()) == 1) ? true : false;
        }
        else
        {
            trafficContour2D_t[i].isMirroringConstruction_nu = false;
        }

        paramName = "Traffic." + std::to_string(i) + ".Basics.Contour";

        if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
            int nRows = 1;
            int nCols = 2;
            temp = iGetTable2(SimCore.TestRun.Inf, paramName.c_str(), nCols, &nRows);
            trafficContour2D_t[i].trafficId = i;
            trafficContour2D_t[i].nrRows = static_cast<uint8_t>(nRows);
            for (uint8_t j = 0; j < nRows; j++)
            {
                if (j < MAX_NUMBER_OF_POINTS_READ_FROM_TESTRUN) {
                    trafficContour2D_t[i].points[0][j] = static_cast<float32_t>(temp[j]);
                    trafficContour2D_t[i].points[1][j] = static_cast<float32_t>(temp[j + nRows]);
                }
                else {
                    //read only the first MAX_NUMBER_OF_POINTS_READ_FROM_TESTRUN(16) points
                    break;
                }
            }
        }
        else {
            memset(&trafficContour2D_t[i].points, 0, MAX_NUMBER_OF_POINTS_READ_FROM_TESTRUN * sizeof(trafficContour2D_t[i].points));
        }

        if (trafficContour2D_t[i].name && (strcmp(trafficContour2D_t[i].name, "T01") == 0)) {

            paramName = "Traffic." + std::to_string(i) + ".Init.Road";

            if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
                const auto roadCoord = iGetFixedTable2(SimCore.TestRun.Inf, paramName.c_str(), 2, 1);
                sRoadT01_m = static_cast<float32_t>(roadCoord[0]);
            }
            else
            {
                sRoadT01_m = .0f;
            }
        }
        else if (trafficContour2D_t[i].name && (strcmp(trafficContour2D_t[i].name, "Lim01") == 0)) {

            paramName = "Traffic." + std::to_string(i) + ".Init.Road";

            if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
                const auto roadCoord = iGetFixedTable2(SimCore.TestRun.Inf, paramName.c_str(), 2, 1);
                sRoadLim01_m = static_cast<float32_t>(roadCoord[0]);
            }
            else
            {
                sRoadLim01_m = .0f;
            }
        }

        tTrafficObj* trafficObj{ Traffic_GetByTrfId(i) };
        bool isRelevantFurthestObjectCandidate{ trafficObj != NULL};
        if ((trafficObj != NULL) && std::string{ trafficObj->Cfg.Name }.find("Lim") == 0)
        {
            // the object name starts with "Lim"
            if (std::string{ trafficObj->Cfg.Info }.find("Parking limiter") != 0)
            {
                // the object name does not start with "Parking limiter" -> ignore it
                isRelevantFurthestObjectCandidate = false;
            }
        }
        if (isRelevantFurthestObjectCandidate)
        {
            // trafficObj->Cfg.sRoad unfortunately yields the sRoad of the center of mass, not the hitch
            paramName = "Traffic." + std::to_string(i) + ".Init.Road";
            if (iEntryExists(SimCore.TestRun.Inf, paramName.c_str())) {
                const auto roadCoord = iGetFixedTable2(SimCore.TestRun.Inf, paramName.c_str(), 2, 1);
                const float32_t sRoadFurthestObjectCandidate_m{ static_cast<float32_t>(roadCoord[0]) };
                if (sRoadFurthestObjectCandidate_m > sRoadFurthestObject_m)
                {
                    sRoadFurthestObject_m = sRoadFurthestObjectCandidate_m;
                    yawAngleFurthestObject_rad = static_cast<float32_t>(trafficObj->Cfg.r_zyx[2]);
                }
            }
        }
    }

    prevResetOriginRequestPort.resetCounter_nu = 0;
    prevResetOriginRequestPort.resetOrigin_nu = ap_psm::ResetOriginType::RRT_NONE;
    prevResetOriginRequestPort.transformation = { 0.0F, 0.0F, 0.0F };
    segment_selector = 0;
    pathBeforeActIndex = 0;
    c_i = 0.0;
    key = "AccelCtrl";
    /* Wheel speed sensor extension use decision*/
    DesicionWhlMdlExt = static_cast<uint16_t>(iGetDbl(SimCore.Vhcl.Inf, "wheelPulsePort.DesicionWhlMdlExt"));
    /*ThGo Start*/
    gain_p_f = iGetDbl(SimCore.Vhcl.Inf, "FakeLoDMC.Ctrl.pf");
    gain_i_f = iGetDbl(SimCore.Vhcl.Inf, "FakeLoDMC.Ctrl.if");
    gain_p_r = iGetDbl(SimCore.Vhcl.Inf, "FakeLoDMC.Ctrl.pr");
    gain_i_r = iGetDbl(SimCore.Vhcl.Inf, "FakeLoDMC.Ctrl.ir");
    cmFiltCoeff_ay = iGetDblOpt(SimCore.Vhcl.Inf, "FakeLoDMC.filt.ay", 1);
    gain_slope = iGetDbl(SimCore.Vhcl.Inf, "FakeLoDMC.Ctrl.gain_slope");
    Acc_y_filt = 0;
    int n;
    GB_Ratios_front = iGetTable2(SimCore.Vhcl.Inf, "PowerTrain.GearBox.iForwardGears", 1, &n);
    GB_Ratios_rear = iGetTable2(SimCore.Vhcl.Inf, "PowerTrain.GearBox.iBackwardGears", 1, &n);
    // Yaw rate
    extension_wanted = iGetDbl(SimCore.Vhcl.Inf, "AP.odoDebugPort.YawRateExtensionWanted_nu");
    /*ThGo End*/
    sideslip_simulation_active = iGetInt(SimCore.Vhcl.Inf, "SideslipSimulation.on");
    sideslip_lh_m = iGetDbl(SimCore.Vhcl.Inf, "SideslipSimulation.lh");
    isFirstCycle = true;
    isFirstGearReq = true;
    lastAng_rad = 0.0;
    hadSteerCtrlRequest_nu = false;
    lastGearReq_nu = -1;
    hadGearReq_nu = false;
    gearChangeRequested_nu = false;
    driverReqToAccelFlag_nu = 0;
    driverReqToBrakeFlag_nu = 0;
    carStopped_nu = true;
    currStroke_nu = 0;
    currGearReq_nu = 0;
    distToStopReq_m = 0.0F;
    undershoot_delay_cntr = 0;
    overwriteVehVelocityCur_nu = 0;
    vehVelocityCurCMQuant_mps = 0.0f;
    for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; i++) {
        gParkingBoxPort.parkingBoxes[i].existenceProb_perc = 0;
    }
    gParkingBoxPort.numValidParkingBoxes_nu = 0;
    wheelSpeedSensorSimulationInit();
    wheelPulsesFL_nu = wheelPulsePort.wheelPulsesFL_nu, wheelPulsesRL_nu = wheelPulsePort.wheelPulsesRL_nu, wheelPulsesFR_nu = wheelPulsePort.wheelPulsesFR_nu, wheelPulsesRR_nu = wheelPulsePort.wheelPulsesRR_nu;
    for (unsigned int i = 0; i < 4u; i++)
    {
        latestWheelSpeed_radps[i] = 0;
        wheelPulsesLast_nu[i] = 0;
        realTimeStampLatestTicks_s[i] = 0.0F;
        TimeStampDelta_s[i] = 0.0F;
        DeltaTimeStampLatestTicks_nu[i] = 0.0F;
        TimeStampLatestTicks_nu[i] = static_cast<float32_t>(SimCore.Time);
        latestWheelSpeedFiltred_radps[i] = 0.0F;
        CountIterationsToZero_nu[i] = 0;
        TimeAfterJitter_s[i] = 0.0F;
        LatestWheelSpeedFilteredJt_radps[i] = 0.0F;
        WheelPulsesJt_nu[i] = 0.0F;
        JitterTime_s[i] = 0.0F;
        JitterTimeLast_s[i] = 0.0F;
        TimeLast20ms_s[i] = static_cast<float32_t>(SimCore.Time);
        Timedifference20ms_s[i] = 0.0F;
        PT1Filter[i].reset();
        diffWheelpulses_nu[i] = 0;
    }

    if (KindKey == nullptr && Inf == nullptr) return (void *)ThisModelKind;
    return (void *)ThisModelKind;
}


static void
mdl_APCtrl_Delete(void *MP)
{
    //struct tmdl_APCtrl *mp = (tmdl_APCtrl*)MP;
    resetGlobalsTo0();
    ap_common::terminate();

#ifndef VARIANT_CUS_ONLY
    if (carMakerInterface.environmentModelActive_nu) {
        SvcModelWrapper::Instance().release();
    }

    if (isSvcModelProcessingDataAllocated) {
        SvcModelWrapper::Instance().ReleaseOutput(svcModelProcessingOutput);
        SvcModelWrapper::Instance().ReleaseInput(svcModelProcessingInput);
        SvcModelWrapper::Instance().ReleaseSettings(svcModelProcessingSettings);
        isSvcModelProcessingDataAllocated = false;
    }
#endif

    for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; i++) {
        gParkingBoxPort.parkingBoxes[i].existenceProb_perc = 0;
    }
#ifdef USE_HMI
    Uninitialize_Vector();
#endif
    /* Park the dict entries for dynamically allocated variables before deleting */
    //mdl_APCtrl_DeclQuants_dyn (mp, 1);
    //mdl_APCtrl_DeclQuants (mp);
    //free (MP);
    if (MP == nullptr) return;
}


int
VehicleControl_Register_mdl_APCtrl(void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.VehicleControl.New = mdl_APCtrl_New;
    m.VehicleControl.Calc = mdl_APCtrl_Calc;
    m.VehicleControl.DeclQuants = mdl_APCtrl_DeclQuants;
    m.VehicleControl.Delete = mdl_APCtrl_Delete;
    /* Should only be used if the model doesn't read params from extra files */
    m.VehicleControl.ParamsChanged = ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_VehicleControl, ThisModelKind, &m);
}