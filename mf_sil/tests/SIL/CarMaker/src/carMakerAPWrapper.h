#pragma once

#include "carMakerAP_DLL_common.h"

// Component output includes
#ifdef MOCO_REPLACES_LODMC
#include "mf_trjctl_types/MoCo_Outputs.h"
#else
#include <ap_lodmc/lo_dmcstatus_port.h>
#endif
#include <ap_ladmc/la_dmcstatus_port.h>
#include "mf_drvwarnsm/mf_drvwarnsm_generated_types.h"
#include "mf_hmih/mf_hmih_generated_types.h"
#include "mf_tonh/tone_output_port.h"
#include "ap_psm_app/ap_psm_app_generated_types.h"
#include "mf_drvwarnsm_core/drv_warn_core_status_port.h"
#include "mf_lsca_types/LSCA360_ExternalTypes.h"
#include "mf_lsca/lsca_status_port.h"
#include "mf_manager/mf_manager_generated_types.h"
#include "ap_psm/ap_psm_generated_types.h"
#include "ap_trjctl/ap_trjctl_generated_types.h"
#include "ap_tp/ap_tp_generated_types.h"
#include "mf_whlprotectproc/mf_whlprotectproc_generated_types.h"
#include "mf_vedodo_types/OdoServiceFunctions.h"
#include "mf_whlprotectproc/whpproc_output_port.h"
#include "si/si_generated_types.h"
#include "mf_mempark/memory_parking_status_port.h"
#include "tce/tce_generated_types.h"
#include "ap_lodmc/lo_dmcstatus_port.h"
#include "ap_ladmc/la_dmcstatus_port.h"
#include "pdcp/pdcp_generated_types.h"
#include "ap_vehstatesigprovider/ap_vehstatesigprovider_generated_types.h"
#include "ap_commonvehsigprovider/ap_commonvehsigprovider_generated_types.h"
#include "ap_hmitoap/hmioutput_port.h"
#include "ap_hmitoap/remote_hmioutput_port.h"
#include "mf_lvmd/lvmd_status_port.h"
#include "ap_hmitoap/visu_input_data.h"
#include "avga_swc/avga_supervision_request.h"
#include "avga_swc/avga_automated_vehicle_guidance_state.h"
#include "avga_swc/avga_stop_request_port.h"
#include "MfSilTypes.h"

#ifdef USE_ENV_PLOTTER
#include "mf_plot/MF_PlotterSi.h"
#include "mf_plot/MF_PlotterCem.h"
#include "mf_plot/MF_PlotterCus.h"
#endif

#include "CarMakerSystemServices.h"

// Component parameter includes
#include "ap_common/vehicle_params.h"

// Other includes
#include "TestEvaluationStruct.h"

#ifdef USE_ENV_PLOTTER
struct SVCPlotterData
{
    MF_Plot::sensorPointList gdrFront;
    MF_Plot::sensorPointList gdrRear;
    MF_Plot::sensorPointList gdrRight;
    MF_Plot::sensorPointList gdrLeft;

    MF_Plot::markerList pmdRight;
    MF_Plot::markerList pmdLeft;
    MF_Plot::markerList pmdFront;
    MF_Plot::markerList pmdRear;

    MF_Plot::markerList wheelStopperListRight;
    MF_Plot::markerList wheelStopperListLeft;
    MF_Plot::markerList wheelStopperListFront;
    MF_Plot::markerList wheelStopperListRear;
};
#endif

namespace ap_common {

    struct CarMakerInterface {
        unsigned char headUnitScreen_nu;
        unsigned char headUnitMessage_nu;
        unsigned char remoteScreen_nu;
        unsigned char remoteMessage_nu;
        unsigned char vedodoActive_nu;
        unsigned char lscaDisabled_nu;
        unsigned char lscaBrakeDisabled_nu;
        unsigned char lscaVirtualWallDisabled_nu;
        unsigned char lscaSteeringProposalDisabled_nu;
        unsigned char lscaFakeStandstillRequest_nu;
        unsigned char lvmdDisabled_nu;
        unsigned char lvmdWarningInput_nu;
        Vehicle_Params vehicle_Params;
        bool odoOverwriteVehVelocity_nu;
        float32_t odoOverwriteVehVelocityValue_mps;
        float odoOverwriteVehVelocity_mps;
        unsigned char odoRecalibratePosition_nu;
        bool variantSemiAPActive_nu;
        unsigned char sceneInterpretationActive_nu; // Activates the si_core component (use CEM surrogate)
        unsigned char environmentModelActive_nu{ 0U };  // Activates the CEM_LSM or us_em component (use vCUS model + us_processing, SVC model)
        unsigned char trajExportActive_nu{ 0U }; // Activates the trajectory export process
        unsigned char pdwDisabled_nu;
        unsigned char whpDisabled_nu;
        unsigned char pdwFailure_nu;
        struct ap_common::FC_TRJPLA_Sys_Func_Params trajplaSysFuncParams;
        unsigned char resetCemSurrogate_nu{ 0U }; // Reset CemSurrogate history and reconfigure CemSurrogate
        unsigned char resetParkingComponents_nu{ 0U }; // Reset the parking software components
    };

    DllExport void initialize(CarMakerInterface& carMakerInterface, lsm_vedodo::OdoNVMData& odoPersistentDataPort, tce::TcePersDataPort& tcePersDataPort, eco::CarMakerSystemServices& carMakerSystemServices);

    DllExport void step(uint64_t timeStamp_ms,
        const ap_vehstatesigprovider::StarterStatusPort& starterStatusPort,
        const ap_vehstatesigprovider::TrunkLidStatusPort& trunkLidStatusPort,
        const ap_vehstatesigprovider::ConvertibleTopStatusPort& convertibleTopStatusPort,
        const ap_vehstatesigprovider::SteeringColSwitchesStatusPort& steeringColSwitchesStatusPort,
        const ap_vehstatesigprovider::TrailerStatusPort& trailerStatusPort,
        const ap_vehstatesigprovider::DoorStatusPort& doorStatusPort,
        const ap_vehstatesigprovider::VehicleOccupancyStatusPort& vehicleOccupancyStatusPort,
        const ap_vehstatesigprovider::AdditionalBCMStatusPort& additionalBCMStatusPort,
        const ap_vehstatesigprovider::ACCInformationPort& accInformationPort,
        const ap_vehstatesigprovider::KeylessStatusPort& keylessStatusPort,
        const ap_vehstatesigprovider::AuthenticationStatusPort& authenticationStatusPort,
        const ap_vehstatesigprovider::APCtrlStatusPort& apCtrlStatusPort,
        const ap_vehstatesigprovider::ESCInformationPort& escInformationPort,
        const ap_vehstatesigprovider::ExternalFunctionStatusPort& externalFunctionStatusPort,
        const ap_commonvehsigprovider::OdoGpsPort & odoGpsPort,
        const ap_commonvehsigprovider::OdoExtCtrlPort& odoExtCtrlPort,
        const ap_commonvehsigprovider::GearboxCtrlStatusPort& gearboxCtrlStatusPort,
        const ap_commonvehsigprovider::SuspensionPort& suspensionPort,
        const ap_commonvehsigprovider::EngineCtrlStatusPort& engineCtrlStatusPort,
        const ap_commonvehsigprovider::SystemTimePort& systemTimePort,
        const ap_commonvehsigprovider::WheelPulsePort& wheelPulsePort,
        const ap_commonvehsigprovider::WheelDrivingDirectionsPort& wheelDrivingDirectionsPort,
        const ap_commonvehsigprovider::WheelSpeedPort& wheelSpeedPort,
        const ap_commonvehsigprovider::VehDynamicsPort& vehDynamicsPort,
        const ap_commonvehsigprovider::SteerCtrlStatusPort& steerCtrlStatusPort,
        const ap_hmitoap::HMIOutputPort& hmiOutputPort,
        const ap_hmitoap::RemoteHMIOutputPort& remoteHMIOutputPort,
        const ap_hmitoap::VisuInputData& visuInputData,
        const si::ApParkingBoxPort& parkingBoxPort,
        const lsm_vedodo::OdoEstimation& odoEstimationPortCM,
        const si::ApEnvModelPort& envModelPort,
        const si::CollEnvModelPort& collEnvModelPort,
        const si::EgoMotionPort& egoMotionPort,
        const si::PerceptionAvailabilityPort& perceptionAvailabilityPort,
        const mf_mempark::MemoryParkingStatusPort& memoryParkingStatusPort,
#ifdef USE_ENV_PLOTTER
        const MF_Plot::plotterCemObjectList& cemObjectsForPlotter,
        const MF_Plot::DYN_OBJ_LIST_FROM_CEM& cemDynObjectsForPlotter,
        const MF_Plot::OD_SLOT_LIST_FROM_CEM& odSlotsForPlotter,
        const MF_Plot::CusReflectionData& cusReflectionData,
        const MF_Plot::plotterCemLineList& cemLinesForPlotter,
        const si::PlotData& siPlotData,
        const MF_Plot::MemParkData& memParkPlotData,
        const OptimalTargetPose& optimalTargetPose,
        const SVCPlotterData& svcPlotData,
#endif
        lsm_vedodo::OdoEstimationOutputPort& odoEstimationOutputPort,
        lsm_vedodo::OdoNVMData &odoPersDataPort,
        tce::TceEstimationPort& tceEstimationPort,
        tce::TcePersDataPort& tcePersDataPort,
        tce::TceDebugPort& tceDebugPort,
        ap_lodmc::LoDMCStatusPort& loDMCStatusPort,
#ifdef MOCO_REPLACES_LODMC
        const TRATCO_t_CpldTratcoStatus& tratcoStatusPort,
        const VECONA_t_LongVeconaStatus& longVeconaStatusPort,
#endif
        ap_ladmc::LaDMCStatusPort& laDMCStatusPort,
        CarMakerInterface& carMakerInterface,
        ap_trjctl::LaDMCCtrlRequestPort& laDMCCtrlRequestPort,
        ap_trjctl::LoDMCCtrlRequestPort& loDMCCtrlRequestPort,
#ifdef MOCO_REPLACES_LODMC
        AP_TRJCTL::MF_CONTROL_t_LongManeuverRequestPort& longManeuverRequestPort,
#endif
        ap_trjctl::GearboxCtrlRequestPort& gearBoxCtrlRequestPort,
        ap_commonvehsigprovider::BrakeCtrlStatusPort& brakeCtrlStatusPort,
        lsm_vedodo::OdoDebugPort& odoDebugPort,
        ap_psm_app::PSMDebugPort& psmDebugPort,
        ap_psm::TrajCtrlRequestPort& trajCtrlRequestPort,
        ap_psm_app::PSMToSSPPort& psmToSSPPort,
        ap_psm::SlotCtrlPort& slotCtrlPort,
        ap_trjctl::TrajCtrlDebugPort& trajCtrlDebugPort,
        ap_trjctl::MFControlStatusPort& mfControlStatusPort,
        ap_tp::TAPOSDDebugPort& taposdDebugPort,
        mf_lsca::LscaStatusPort& lscaStatusPort,
        mf_lsca::structs::plot_t& gLscaPlotDataPort,
        ap_tp::TargetPosesPort &trjplaTPPort,
        ap_tp::PlannedTrajPort &trjplaPlannedTrajPort,
        ap_tp::TrajPlanDebugPort &trjplaDebugPort,
        ap_tp::TrajPlanVisuPort &trjplaVisuPort,
        ap_tp::ReverseAssistAvailabilityPort reverseAssistAvailabilityPort,
        mf_hmih::HMIGeneralInputPort& hmiInputPort,
        mf_hmih::HeadUnitVisualizationPort& headUnitVisualizationPort,
        mf_hmih::RemoteVisualizationPort& remoteVisualizationPort,
        mf_hmih::MFHmiHDebugPort &mfHmiHDebugPort,
        mf_hmih::UserDefinedSlotPort &userDefinedSlotPort,
        mf_hmih::APUserInteractionPort &apUserInteractionPort,
        mf_hmih::LVMDUserInteractionPort &lvmdUserInteractionPort,
        ap_trjctl::DrivingResistancePort& drivingResistancePort,
        ap_psm_app::CtrlCommandPort& ctrlCommandPort,
        ap_psm::PARKSMCoreStatusPort& parksmCoreStatusPort,
        ap_psm::PARKSMCoreDebugPort& parksmoreDebugPort,
        mf_manager::LaCtrlRequestPort& laCtrlRequestPort,
        mf_manager::LoCtrlRequestPort& loCtrlRequestPort,
        mf_manager::TrajRequestPort& trajRequestPort,
        mf_drvwarnsm::DrvWarnStatusPort& drvWarnStatusPort,
        mf_drvwarnsm::DrvWarnDebugPort& drvWarnDebugPort,
        mf_drvwarnsm::AppToCoreSMPort& appToCoreSMPort,
        mf_drvwarnsm_core::DrvWarnCoreStatusPort& drvWarnCoreStatusPort,
        mf_drvwarnsm::LogicToProcPort& logicToProcPort,
        pdcp::PDCPSectorsPort& pdcpSectorsPort,
        pdcp::PDCPDrivingTubePort& pdcpDrivingTubePort,
        pdcp::ProcToLogicPort& pdwProcToLogicPort,
        pdcp::PDCPDebugPort& pdcpDebugPort,
        mf_tonh::ToneOutputPort& toneOutputPort,
        mf_whlprotectproc::WHPProcOutputPort& whpProcOutputPort,
        mf_lvmd::LvmdStatusPort& lvmdStatusPort, 
        avga_swc::AVGA_SupervisionRequest& avgaSupervisionRequestLSCA,
        avga_swc::AVGA_SupervisionRequest& avgaSupervisionRequestPDW,
        avga_swc::AVGA_AutomatedVehicleGuidanceState& automatedVehicleGuidanceStateAUP,
        avga_swc::AvgaStopRequestPort& avgaStopRequestMCRA,
        eco::CarMakerSystemServices& carMakerSystemServices);

    DllExport void terminate(void);

};