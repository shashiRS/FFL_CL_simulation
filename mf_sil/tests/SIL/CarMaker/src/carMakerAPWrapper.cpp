////////////////////////////////////
///////////// Includes /////////////
////////////////////////////////////

// General includes
#include <sstream>
#include <fstream>
#include "carMakerAPWrapper.h"
#include "MfSilTypes.h"
#ifndef VARIANT_CUS_ONLY
#include <plp_parameter_handler/plp_parameter_handler_algo_interface.h>
#include "mf_lvmd/lvmd_params.h"
#include "mf_lvmd/ReadJsonLVMDParams.h"
#else
// Component parameter includes
#include "mf_drvwarnsm/fc_drv_warn_sm_params.h"
#include "mf_hmih/fc_mfhmih_params.h"
#include "ap_psm_app/fc_parksm_params.h"
#include "mf_tonh/fc_mf_tone_handler_params.h"
#include "ap_common/sys_func_params.h"
#include "ap_common/vehicle_params.h"
#include "mf_drvwarnsm_core/fc_drv_warn_smcore_params.h"
#include "mf_manager/fc_mf_manager_params.h"
#include "ap_psm/fc_parksm_core_params.h"
#include "pdcp/fc_pdcp_params.h"
#include "ap_trjctl/fc_trjctl_params.h"
#include "mf_trjpla_types/FC_TRJPLA_Params.h"
#include "mf_whlprotectproc/fc_mf_whl_protect_proc_params.h"
#endif

// Component output includes
#include "mf_tonh/mf_tonh_generated_types.h"
#include "mf_drvwarnsm_core/mf_drvwarnsm_core_generated_types.h"
#include "pdcp/pdcp_generated_types.h"
#include "mf_whlprotectproc/whpproc_output_port.h"
#include "mf_whlprotectproc/whpproc_debug_port.h"

// Component interface includes
#include "appdemo_drvwarnsm/DrvWarnSM_Interface.h"
#include "appdemo_hmih/MFHMIH_Interface.h"
#include "appdemo_parksm/PARKSM_Interface.h"
#include "appdemo_tonh/MF_ToneHandler_Interface.h"
#include "mf_drvwarnsm_core/DrvWarnSMCore_Interface.h"
#include "mf_lsca/LSCA360_Interface.h"
#include "mf_manager/MF_Manager_Interface.h"
#include "mf_parksm_core/PARKSM_Core_Interface.h"
#include "mf_pdwarnproc/PDCP_Interface.h"
#include <mf_trjctl/MF_CONTROL_Interface.h>
#ifndef NO_TAPOSD_TRJPLA
#include "mf_trjpla/TRJPLA_Interface.h"
#endif
#include "mf_vedodo/VEDODO_Interface.h"
#include "mf_whlprotectproc/MF_WhlProtectProc_Interface.h"
#include "tce/tce_component_interface.h"
#include "avga_swc/avga_swc_interface.h"
#ifndef VARIANT_CUS_ONLY
#include "mf_lvmd/LVMD_Interface.h"
#endif

// Other includes
#include "vc/vc_generated_types.h"
#include "geoml/CoordinateTransformer2D.h"
#include "avga_swc/avga_swc_generated_types.h"
#include <mf_lvmd/mf_lvmd_generated_types.h>
#include <hpsd/health_vector_port.h>

// Includes for PARKSM HMI visualization
#ifdef USE_HMI_VIS
    #include "AP_HMI_Plotter.h"
    #include <thread>
#endif

// Includes for mf_plot
#ifdef USE_ENV_PLOTTER
    #include "mf_plot/MF_Plotter.h"
    #include <mutex>
    #include <iostream>
#ifndef USE_HMI_VIS
    #include <thread>
#endif
#endif

// Includes for MF_TRJPLA visualization tool
#ifdef AP_DEBUG_DRAW_ENABLE
#include <parkingvisuclient.h>
#endif

/////////////////////////////////////
///////////// Parameter /////////////
/////////////////////////////////////

// Parking Base:       Define path to parameter json file
    // Bricks:         Based on component bricks package
    // VS solution:    Based on component GitHub repository structure
// Parking Entry:      Include parameter header
#ifndef VARIANT_CUS_ONLY
#ifdef CIP_BRICKS_BUILD
    #define LVMD_PARAMS_FILE_NAME       "data/mf_lvmd/LVMD_Params_config.json"
#else
    #define LVMD_PARAMS_FILE_NAME       "mf_lvmd/src/platform/data/LVMD_Params_config.json"
#endif

#else
#include "mf_vedodo/FC_VEDODO_Params_default_set.h"
#include "tce/fc_tce_params_default_set.h"
#include "appdemo_parksm/FC_PARKSM_Params_default_set.h"
#include "mf_parksm_core/FC_PARKSM_Core_Params_default_set.h"
#include "mf_trjpla/FC_TRJPLA_Params_default_set.h"
#include "mf_trjctl/FC_TRJCTL_Params_default_set.h"
#include "mf_common/Sys_Func_Params_default_set.h"
#include "mf_common/Vehicle_Params_default_set.h"
#include "mf_lsca/LSCA360_Params_default_set.h"
#include "appdemo_hmih/FC_MFHMIH_Params_default_set.h"
#include "mf_pdwarnproc/FC_PDCP_Params_default_set.h"
#include "mf_drvwarnsm_core/FC_DrvWarnSMCore_Params_default_set.h"
#include "appdemo_drvwarnsm/FC_DrvWarnSM_Params_default_set.h"
#include "appdemo_tonh/FC_MF_ToneHandler_Params_default_set.h"
#include "mf_whlprotectproc/FC_MF_WhlProtectProc_Params_default_set.h"
//TODO: MF_Manager?
#endif

// Parameter structs
static ap_common::Sys_Func_Params sys_FUNC_PARA_Params{};
static ap_common::Vehicle_Params vehicle_Params{};
static ap_psm::FC_PARKSM_Core_Params fc_PARKSM_Core_Params{};
static ap_psm_app::FC_PARKSM_Params fc_PARKSM_Params{};
static ap_tp::FC_TRJPLA_Params fc_trjpla_params{};
static ap_tp::FC_TAPOSD_Params fc_taposd_params{};
static ap_trjctl::FC_TRJCTL_Params fc_mfControl_params{};
static lsm_vedodo::FC_VEDODO_Params fc_VEDODO_Params{};
static mf_drvwarnsm::FC_DrvWarnSM_Params fc_DrvWarnSM_Params{};
static mf_drvwarnsm_core::FC_DrvWarnSMCore_Params fc_DrvWarnSMCore_Params{};
static mf_hmih::FC_MFHMIH_Params fc_MFHMIH_Params{};
static mf_lsca::LscaFunctionConfig lsca_Params{};
static mf_manager::FC_MF_Manager_Params fc_MF_MANAGER_Params{};
static mf_tonh::FC_MF_ToneHandler_Params fc_toneHandler_Params{};
static mf_whlprotectproc::FC_MF_WhlProtectProc_Params fc_WhlProtectProc_Params{};
static pdcp::FC_PDCP_Params fc_PDCP_Params{};
static tce::FC_TCE_Params fc_TCE_Params{};
static avga_swc::AVGA_params avga_Params{};
static mf_lvmd::LvmdParams lvmd_Params{};

#ifndef VARIANT_CUS_ONLY
static plp_parameter_handler::ParameterHandlerOutput parameterHandlerOutput
{
        /* lsm_vedodo::FC_VEDODO_Params                                 */&fc_VEDODO_Params,
        /* ap_trjctl::FC_TRJCTL_Params                                  */&fc_mfControl_params,
        /* tce::FC_TCE_Params                                           */&fc_TCE_Params,
        /* ap_psm::FC_PARKSM_Core_Params                                */&fc_PARKSM_Core_Params,
        /* ap_psm_app::FC_PARKSM_Params                                 */&fc_PARKSM_Params,
        /* mf_hmih::FC_MFHMIH_Params                                    */&fc_MFHMIH_Params,
        /* mf_manager::FC_MF_Manager_Params                             */&fc_MF_MANAGER_Params,
        /* mf_drvwarnsm_core::FC_DrvWarnSMCore_Params                   */&fc_DrvWarnSMCore_Params,
        /* mf_drvwarnsm::FC_DrvWarnSM_Params                            */&fc_DrvWarnSM_Params,
        /* pdcp::FC_PDCP_Params                                         */&fc_PDCP_Params,
        /* mf_whlprotectproc::FC_MF_WhlProtectProc_Params               */&fc_WhlProtectProc_Params,
        /* mf_tonh::FC_MF_ToneHandler_Params                            */&fc_toneHandler_Params,
        /* ap_common::Vehicle_Params                                    */&vehicle_Params,
        /* ap_common::Sys_Func_Params                                   */&sys_FUNC_PARA_Params,
        /* ap_tp::FC_TRJPLA_Params                                      */&fc_trjpla_params,
        /* ap_tp::FC_TAPOSD_Params                                      */&fc_taposd_params,
        /* mf_lsca::LscaFunctionConfig                                  */&lsca_Params
};
#endif

// Parameter configs
static mf_psm::PARKSMCore_Config psmCoreConfig{};
static ap_psm_app::PARKSM_Config psmConfig{};
#ifndef NO_TAPOSD_TRJPLA
static ap_tp::TRJPLA_Config trjplaConfig{};
#endif
static ap_trjctl::MF_Control_Config mfControlConfig{};
static lsm_vedodo::VEDODO_Config vedodoConfig{};
static mf_drvwarnsm::DrvWarnSM_Config drvWarnSMConfig{};
static mf_drvwarnsm_core::DrvWarnSMCore_Config drvWarnSMCoreConfig{};
static mf_hmih::MFHMIH_Config mfhmihConfig{};
static mf_lsca::LSCA_Config lscaConfig{};
static mf_manager::MFManager_Config mfManagerConfig{};
static mf_tonh::MFToneHandler_Config tonhConfig{};
static mf_whlprotectproc::MFWhlProtectProc_Config whpConfig{};
static pdcp::PDCP_Config pdcpConfig{};
static tce::TceConfig tceConfig{};
static avga_swc::AvgaConfig avgaConfig{};
#ifndef VARIANT_CUS_ONLY
static mf_lvmd::LVMD_Config lvmdConfig{};
#endif


////////////////////////////////////
//////////////  Ports  /////////////
////////////////////////////////////
static ap_commonvehsigprovider::EngineCtrlStatusPort gEngineCtrlStatusPort{};
static ap_commonvehsigprovider::GearboxCtrlStatusPort gGearboxCtrlStatusPort{};
static ap_vehstatesigprovider::TrailerStatusPort gTrailerStatusPort{};
static ap_vehstatesigprovider::DoorStatusPort gDoorStatusPort{};
static ap_vehstatesigprovider::VehicleOccupancyStatusPort gVehicleOccupancyStatusPort{};
static ap_vehstatesigprovider::TrunkLidStatusPort gTrunkLidStatusPort{};
static ap_psm::SlotCtrlPort gSlotCtrlPort{};
static ap_psm_app::OverrideLSCAPort gOverrideLSCAPort{};
static ap_psm_app::APUserInformationPort gAPUserInformationPort{};
static ap_tp::TargetPosesPort gTargetPosesPort{};
static ap_tp::TAPOSDDebugPort gTaposdDebugPort{};
static ap_tp::PlannedTrajPort gPlannedTrajPort_AP_Transformed{};
static ap_tp::TrajPlanVisuPort gTrajPlanVisuPort{};
static mf_drvwarnsm_core::DrvWarnCoreDebugPort gDrvWarnCoreDebugPort{};
static mf_hmih::HeadUnitVisualizationPort gHeadUnitVisualizationPort{};
static mf_hmih::HMIGeneralInputPort gHmiInputPort{};
static mf_hmih::RemoteVisualizationPort gRemoteVisualizationPort{};
static mf_hmih::SurroundViewRequestPort gSurroundViewRequestPort{};
static mf_hmih::VisuInputPort gVisuInputPort{};
static mf_hmih::APUserInteractionPort gAPUserInteractionPort{};
static mf_hmih::PDCUserInteractionPort gPdcUserInteractionPort{};
static mf_hmih::LVMDUserInteractionPort gLvmdUserInteractionPort{};
static mf_hmih::UserDefinedSlotPort gUserDefinedSlotPort{};
static mf_lsca::LscaBrakePort gLscaBrakePort{};
static mf_lsca::LscaSteerPort gLscaSteerPort{};
static mf_lsca::LscaHMIPort gLscaHmiPort{};
static mf_lsca::structs::plot_t gLscaPlotDataPort{};
static mf_manager::ActiveManeuveringFunctionPort gActiveManeuveringFunctionPort{};
static mf_tonh::ToneHandlerDebugPort gToneHandlerDebugPort{};
static mf_whlprotectproc::WHPProcDebugPort  gWhpProcDebugPort{};
static si::ApEnvModelPort gEnvModelPort{};
static si::CollEnvModelPort gCollEnvModelPort{};
static si::ApParkingBoxPort gParkingBoxPort{};
static si::EgoMotionPort gEgoMotionPort{};
static vc::ScreenSwitchDataPort gScreenSwitchDataPort{};

// Ports for PARKSM HMI visualization
#ifdef USE_HMI_VIS
AP_Utils::AP_HMIPlotter gAPHMIPlotter{};
MF_HMIH::HMIGeneralInputPort gOldHmiInputPort{};
MF_HMIH::HeadUnitVisualizationPort gOldHeadUnitVisualizationPort{};
MF_HMIH::RemoteVisualizationPort gOldRemoteVisualizationPort{};
static bool userHMIAction{ false };
#endif

////////////////////////////////////////
/////////////  Sample time  ////////////
////////////////////////////////////////

//Short sample time
const uint64_t TCE_SAMPLE_TIME_MS{ SHORT_SAMPLE_TIME_MS };
const uint64_t VEDODO_SAMPLE_TIME_MS{ SHORT_SAMPLE_TIME_MS };
const uint64_t TRJCTL_SAMPLE_TIME_MS{ SHORT_SAMPLE_TIME_MS };
//Long sample time
const uint64_t PARKSM_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t TRJPLA_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t LSCA_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t MF_MANAGER_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t DRVWARNSM_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t PDWPROC_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t HMIH_SAMPLE_TIMES_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t TONH_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t WHP_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t AVGA_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
const uint64_t LVMD_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };

////////////////////////////////////
/////////////  Objects  ////////////
////////////////////////////////////

// Visualization of TRJPLA replanning
static uint8_t gPreviousNumOfReplanCalls{};
static ap_tp::TrajPlanVisuPort gPreviousTrajPlanVisuPort{};
static ap_tp::TargetPosesPort gPreviousTargetPosesPort{};

// Odometry position transformation (transform VEDODO in CarMaker origin)
static bool gOdometryWasReseted{ false };
static bool gOdoPositionWasRecalibrated{ false };
static LSM_GEOML::Pose gOdometryOffset{ 0.0f,0.0f,0.0f };

// Objects for mf_plot
#ifdef USE_ENV_PLOTTER
static MF_Plot::DynamicReplanningVisuData gReplanningData{};
static MF_Plot::plotterCemObjectList gCemObjectsForPlotter{};
static MF_Plot::DYN_OBJ_LIST_FROM_CEM gCemDynObjectsForPlotter{};
static MF_Plot::OD_SLOT_LIST_FROM_CEM gCemODSlotsForPlotter{};
static MF_Plot::CusReflectionData gCusReflectionData{};
static MF_Plot::plotterCemLineList gCemLinesForPlotter{};
static si::PlotData gSiPlotData{};
static MF_Plot::MemParkData gMemParkPlotData{};
static OptimalTargetPose gOptimalTargetPose{};
static SVCPlotterData gSvcPlotData{};
std::thread guiThread{};
namespace AP_Utils {
    struct PlotWindow {
        std::mutex          mutex{};            // mutex to protect threaded access
        MF_Plot::MF_Plotter plotter{};          // the plotter class
        bool                newDataFlag{false}; // new data flag fro the drawing thread
    };
    PlotWindow gVisu{};

    // check the IPC for the new data flag
    void checkVisuData(void) {
        bool newdata{false};
        {
            std::lock_guard<std::mutex> locker(gVisu.mutex);
            newdata = gVisu.newDataFlag;
            gVisu.newDataFlag = false;
        }
        if (newdata) {
            // in case of new data trigger a redraw
            glutPostRedisplay();
        }
        // in order not to constantly cycle this thread without reason, put in a
        // small sleep to release system resources and not lock-unlock-lock the mutex the whole time
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // set plot data
    bool setEnvPlotterGlutData() {

        MF_Plot::PlotterInput plotterInput{};

        plotterInput.possibleTargetPosesPort    = &gTargetPosesPort;
        plotterInput.taposdDebugPort            = &gTaposdDebugPort;
        plotterInput.envModelPort               = &gEnvModelPort;
        plotterInput.envModelPortLsca           = &gCollEnvModelPort;
        plotterInput.parkingBoxPort             = &gParkingBoxPort;
        plotterInput.egoMotionPort              = &gEgoMotionPort;
        plotterInput.plannedTrajPort            = &gPlannedTrajPort_AP_Transformed;
        plotterInput.trajPlanVisuPort           = &gTrajPlanVisuPort;
        plotterInput.vehicleParams              = &vehicle_Params;
        plotterInput.trailerStatusPort          = &gTrailerStatusPort;
        plotterInput.doorStatusPort             = &gDoorStatusPort;
        plotterInput.vehicleOccupancyStatusPort = &gVehicleOccupancyStatusPort;
        plotterInput.trunkLidStatusPort         = &gTrunkLidStatusPort;
        plotterInput.engineCtrlStatusPort       = &gEngineCtrlStatusPort;
        plotterInput.gearBoxCtrlStatusPort      = &gGearboxCtrlStatusPort;
        plotterInput.lscaPlotPort               = &gLscaPlotDataPort;
        plotterInput.slotCtrlPort               = &gSlotCtrlPort;
        plotterInput.replanningData             = &gReplanningData;
        plotterInput.cemObjects                 = &gCemObjectsForPlotter;
        plotterInput.dynObjFromCem              = &gCemDynObjectsForPlotter;
        plotterInput.odSlotListFromCEM          = &gCemODSlotsForPlotter;
        plotterInput.cemLines                   = &gCemLinesForPlotter;
        plotterInput.siData                     = &gSiPlotData;
        plotterInput.memParkData                = &gMemParkPlotData;
        plotterInput.cusReflectionData          = &gCusReflectionData;

        plotterInput.sfmRight                   = &gSvcPlotData.gdrRight;
        plotterInput.sfmFront                   = &gSvcPlotData.gdrFront;
        plotterInput.sfmRear                    = &gSvcPlotData.gdrRear;
        plotterInput.sfmLeft                    = &gSvcPlotData.gdrLeft;

        plotterInput.pmdRight                   = &gSvcPlotData.pmdRight;
        plotterInput.pmdLeft                    = &gSvcPlotData.pmdLeft;
        plotterInput.pmdRear                    = &gSvcPlotData.pmdRear;
        plotterInput.pmdFront                   = &gSvcPlotData.pmdFront;

        plotterInput.palincaWheelStopperListFront   = &gSvcPlotData.wheelStopperListFront;
        plotterInput.palincaWheelStopperListRear    = &gSvcPlotData.wheelStopperListRear;
        plotterInput.palincaWheelStopperListRight   = &gSvcPlotData.wheelStopperListRight;
        plotterInput.palincaWheelStopperListLeft    = &gSvcPlotData.wheelStopperListLeft;

        //Bit Hacky, but seems like plotter uses a parallel structure to the target one -> topic for SIL team
        MF_Plot::OptimalTargetPose temp{ gOptimalTargetPose.valid,gOptimalTargetPose.pose };
        plotterInput.optimalTargetPose = &temp;

        gVisu.plotter.setData(plotterInput);
        return true;
    }

    // the actual plot function
    void plotEnv() {
        {
            std::lock_guard<std::mutex> locker(gVisu.mutex);
            gVisu.newDataFlag = setEnvPlotterGlutData();
            gVisu.plotter.plot();
        }
    }

    void mousePosHandler(int x, int y)
    {
        gVisu.plotter.mIOManager.readMousePosition(x, y);
    }
    void mouseClickHandler(int button, int state, int x, int y)
    {
        gVisu.plotter.mIOManager.readMouseButtons(button, state == 0, x, y);
    }
    void mouseShiftHandler(int x, int y)
    {
        gVisu.plotter.mIOManager.readMouseShiftPosition(x, y);
    }
    void keyboardHandler(int key, int x, int y)
    {
        gVisu.plotter.mIOManager.readKeyboardSpecial(key, x, y, false);
    }

} // Namespace
#endif

// TRJPLA visualization tool
#ifdef AP_DEBUG_DRAW_ENABLE
// visualization connection back end for VISU_DRAW_... calls
VisuClient gVisuClient{};
#endif

namespace ap_common {

   DllExport void initialize(CarMakerInterface& carMakerInterface, lsm_vedodo::OdoNVMData& odoPersistentDataPort, tce::TcePersDataPort& tcePersDataPort, eco::CarMakerSystemServices& carMakerSystemServices)
    {
       ////////////////////////////////////////////
       /////////////  Read parameters  ////////////
       ////////////////////////////////////////////
#ifndef VARIANT_CUS_ONLY

#ifdef CIP_BRICKS_BUILD
       const std::string paramPath{ "../../../conf/package/" };
#else
       const std::string paramPath{ "../../../../" };
#endif
       plp_parameter_handler::Parameter_Handler_Algo_Interface &parameterHandler{ plp_parameter_handler::Parameter_Handler_Algo_Interface::getInstance() };
       parameterHandler.init(parameterHandlerOutput, paramPath);

       ap_read_params::ReadJsonLVMDParams readJsonLVMDParams{ ap_read_params::ReadJsonLVMDParams::getInstance() };
       readJsonLVMDParams.readJsonLVMDParams(paramPath + LVMD_PARAMS_FILE_NAME, lvmd_Params);
#else
       initSys_Func_Params(&sys_FUNC_PARA_Params);
       initVehicle_Params(&vehicle_Params);
       overwriteVehicle_Params_VariantAP_Sim(&vehicle_Params);
       initFC_PARKSM_Core_Params(&fc_PARKSM_Core_Params);
       initFC_VEDODO_Params(&fc_VEDODO_Params);
       overwriteFC_VEDODO_Params_VariantAP_Sim(&fc_VEDODO_Params);
       initFC_TCE_Params(&fc_TCE_Params);
       initFC_TRJCTL_Params(&fc_mfControl_params);
#ifndef NO_TAPOSD_TRJPLA
       initFC_TRJPLA_Params(&fc_trjpla_params);
       overwriteFC_TRJPLA_Params_VariantAP_Sim(&fc_trjpla_params);
#endif
       initFC_PDCP_Params(&fc_PDCP_Params);
       initFC_DrvWarnSM_Params(&fc_DrvWarnSM_Params);
       initFC_DrvWarnSMCore_Params(&fc_DrvWarnSMCore_Params);
       initFC_PARKSM_Params(&fc_PARKSM_Params);
       initFC_MFHMIH_Params(&fc_MFHMIH_Params);
       initLSCAParams(&lsca_Params);
       initFC_MF_ToneHandler_Params(&fc_toneHandler_Params);
       initFC_MF_WhlProtectProc_Params(&fc_WhlProtectProc_Params);
#endif
       ///////////////////////////////////////////
        //////////  Extern configuration  /////////
        ///////////////////////////////////////////
        //Extern deactivation of PDW (e.g. via CarMaker testrun)
        if (carMakerInterface.pdwDisabled_nu) {
            fc_DrvWarnSMCore_Params.DWF_C_PDW_ENABLED_NU = false;
        }
        //Extern deactivation of WHP (e.g. via CarMaker testrun)
        if (carMakerInterface.whpDisabled_nu) {
            fc_DrvWarnSMCore_Params.DWF_C_WHP_ENABLED_NU = false;
        }

        //Explicit scanning feature deactivated in SIL
        fc_MFHMIH_Params.AP_H_EXPLICIT_SCAN_ENABLED_NU = false;

        ////////////////////////////////////////////
        /////////////  Init Components  ////////////
        ////////////////////////////////////////////

#ifndef NO_TAPOSD_TRJPLA
        //Init TRJPLA
        trjplaConfig.trjplaParams = &fc_trjpla_params;
        trjplaConfig.vehicleParams= &vehicle_Params;
        trjplaConfig.sysFuncParams= &sys_FUNC_PARA_Params;
        ap_tp::TRJPLA_Interface::getInstance().init(trjplaConfig, &carMakerSystemServices);
#endif
        //Init VEDODO
        vedodoConfig.vedodoParams = &fc_VEDODO_Params;
        static constexpr float32 SWA_ANG_OFFS_DEFAULT { 0.0F };
        static constexpr float32 AY_OFFS_DEFAULT { 0.0F };
        odoPersistentDataPort.LatAcc.ZeroAccel = AY_OFFS_DEFAULT;
        odoPersistentDataPort.LatAcc.CalStatus = 0xFFFFFFFF;
        odoPersistentDataPort.YwRate.ZeroRate = 0.0F;
        odoPersistentDataPort.YwRate.ZeroRateMin = -3.5F;
        odoPersistentDataPort.YwRate.ZeroRateMax = 3.5F;
        odoPersistentDataPort.YwRate.CalStatus = 0xFFFFFFFF;
        odoPersistentDataPort.StWhlAng.ZeroAngle = SWA_ANG_OFFS_DEFAULT;
        vedodoConfig.odoPersData = &odoPersistentDataPort;
        lsm_vedodo::VEDODO_Interface::getInstance().init(vedodoConfig);

        //Init TCE
        tceConfig.tceParams = &fc_TCE_Params;
        tcePersDataPort.tireCircFL_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_FL_M * 1e4F);
        tcePersDataPort.tireCircFR_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_FR_M * 1e4F);
        tcePersDataPort.tireCircRL_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_RL_M * 1e4F);
        tcePersDataPort.tireCircRR_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_RR_M * 1e4F);
        tcePersDataPort.tireCircStdvFL_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_FL_M * 0.04F * 1e4F);
        tcePersDataPort.tireCircStdvFR_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_FR_M * 0.04F * 1e4F);
        tcePersDataPort.tireCircStdvRL_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_RL_M * 0.04F * 1e4F);
        tcePersDataPort.tireCircStdvRR_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TYRE_CIRCUMFERENCE_RR_M * 0.04F * 1e4F);
        tcePersDataPort.rearTrackWidth_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TRACK_REAR_M * 1e4F);
        tcePersDataPort.rearTrackWidthStdv_0p1mm = static_cast<uint16_t>(fc_TCE_Params.TCE_TRACK_REAR_M * 0.04F * 1e4F);
        tcePersDataPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        tcePersDataPort.sSigHeader.uiTimeStamp = 1U;
        tceConfig.tcePersData = &tcePersDataPort;
        tce::Tce_Interface::getInstance().init(tceConfig);

        //Init PARKSM_Core
        psmCoreConfig.fc_PARKSM_Core_Params = &fc_PARKSM_Core_Params;
        mf_psm::PARKSM_Core_Interface::getInstance().init(psmCoreConfig);

        //Init PARKSM
        psmConfig.fc_PARKSM_Params = &fc_PARKSM_Params;
        psmConfig.sysFuncParams = &sys_FUNC_PARA_Params;
        psmConfig.vehicleParams = &vehicle_Params;
        ap_psm_app::PARKSM_Interface::getInstance().init(psmConfig, &carMakerSystemServices);

        //Init MF_MANAGER
        mfManagerConfig.fc_MF_Manager_Params = &fc_MF_MANAGER_Params;
        mf_manager::MF_Manager_Interface::getInstance().init(mfManagerConfig);

        //Init MF Control
        mfControlConfig.sysFuncParams = &sys_FUNC_PARA_Params;
        mfControlConfig.mfcParams = &fc_mfControl_params;
        mfControlConfig.vehicleParams = &vehicle_Params;
        ap_trjctl::MF_Control_Interface::getInstance().init(mfControlConfig, &carMakerSystemServices);
        carMakerInterface.mfControlConfigMfcParamsSigHeader = mfControlConfig.mfcParams->sSigHeader;
        carMakerInterface.mfControlConfigSysFuncParamsSigHeader = mfControlConfig.sysFuncParams->sSigHeader;
        carMakerInterface.mfControlConfigVehicleParamsSigHeader = mfControlConfig.vehicleParams->sSigHeader;

        //Init MFHMIH
        mfhmihConfig.fc_MFHMIH_Params = &fc_MFHMIH_Params;
        mfhmihConfig.vehicleParams = &vehicle_Params;
        mfhmihConfig.fc_PDCP_Params = &fc_PDCP_Params;
        mf_hmih::MFHMIH_Interface::getInstance().init(mfhmihConfig, &carMakerSystemServices);

        //Init PDCP
        pdcpConfig.vehicleParams = &vehicle_Params;
        pdcpConfig.pdcpParams = &fc_PDCP_Params;
        pdcp::PDCP_Interface::getInstance().init(pdcpConfig);

        //Init DrvWarnSM
        drvWarnSMConfig.drvWarnSMParams = &fc_DrvWarnSM_Params;
        drvWarnSMConfig.pdcpParams = &fc_PDCP_Params;
        mf_drvwarnsm::DrvWarnSM_Interface::getInstance().init(drvWarnSMConfig);

        //Init DrvWarnSMCore
        drvWarnSMCoreConfig.drvWarnSMCoreParams = &fc_DrvWarnSMCore_Params;
        mf_drvwarnsm_core::DrvWarnSMCore_Interface::getInstance().init(drvWarnSMCoreConfig);

        //Init LSCA
        if (carMakerInterface.lscaDisabled_nu) {
            lsca_Params.general.LscaActive_nu = false;
        }
        // Configure function plug-ins
        if (carMakerInterface.lscaBrakeDisabled_nu) {
            lsca_Params.general.brakeStaticActive_nu = false;
            lsca_Params.general.brakeDynamicActive_nu = false;
        }
        if (carMakerInterface.lscaSteeringProposalDisabled_nu) {
            lsca_Params.general.proposalActive_nu = false;
        }
        if (carMakerInterface.lscaVirtualWallDisabled_nu) {
            lsca_Params.general.resistanceActive_nu = false;
        }
        lscaConfig.lscaParams    = &lsca_Params;
        lscaConfig.vehicleParams = &vehicle_Params;
        mf_lsca::LSCA_Interface::getInstance().init(lscaConfig);

#ifndef VARIANT_CUS_ONLY
        //Init LVMD
        if (carMakerInterface.lvmdDisabled_nu) {
            lvmd_Params.lvmdAct_nu = false;
        }
        lvmdConfig.lvmdParams = &lvmd_Params;
        lvmdConfig.vehicleparams = &vehicle_Params;
        mf_lvmd::LVMD_Interface::getInstance().init(lvmdConfig);
#endif

        //Init ToneHandler
        tonhConfig.toneHandlerParams = &fc_toneHandler_Params;
        tonhConfig.drvWarnSMParams = &fc_DrvWarnSM_Params;
        tonhConfig.pdcpParams = &fc_PDCP_Params;
        mf_tonh::MF_ToneHandler_Interface::getInstance().init(tonhConfig, &carMakerSystemServices);

        //Init WhlProtectProc
        whpConfig.whlProtectProcParams = &fc_WhlProtectProc_Params;
        whpConfig.vehicleParams = &vehicle_Params;
        mf_whlprotectproc::MFWhlProtectProc_Interface::getInstance().init(whpConfig);

        //Init Automated Vehicle Guidance Arbiter
        avgaConfig.avgaParamsPort = &avga_Params;
        avga_swc::AVGA_Interface::getInstance().init(avgaConfig, &carMakerSystemServices);

        // Init PARKSM HMI visualization
#ifdef USE_HMI_VIS
        userHMIAction = true;
#endif

        // Init TRJPLA visualization tool
#ifdef AP_DEBUG_DRAW_ENABLE
        if (!gVisuClient.isConnected()) {
            gVisuClient.connectToServer();
            VisuClientInterface::setInterface(&gVisuClient);

            VISU_CLEAR_DRAW();
        }
#endif
#ifdef USE_ENV_PLOTTER
        // Init mf_plot replanning data
        gPreviousNumOfReplanCalls = 0;
        gPreviousTrajPlanVisuPort = {};
        gPreviousTargetPosesPort = {};
        gReplanningData.isValid = false;
        gReplanningData.replanPositions.clear();
        gReplanningData.mPPreviousTargetPose = {};
        gReplanningData.mPPreviousTrjplaVisuPort = {};
#endif

        //Update CarMakerInterface
        carMakerInterface.vehicle_Params = vehicle_Params;
        carMakerInterface.variantSemiAPActive_nu = sys_FUNC_PARA_Params.AP_G_VARIANT_SEMI_AP_ACTIVE_NU;
        carMakerInterface.trajplaSysFuncParams = fc_trjpla_params.sysFuncParams;

        // Initialize odometry position transformation variables
        gOdometryWasReseted = false;
        gOdoPositionWasRecalibrated = false;
        gOdometryOffset = { 0.0F, 0.0F, 0.0F };
    }

    mf_lsca::structs::plot_t stepLSCA(
        const si::CollEnvModelPort&                                 inCollEnvModelPort,
        const ap_psm::SlotCtrlPort&                                 inSlotCtrlPort,
        const ap_psm_app::OverrideLSCAPort&                         inOverrideLSCAPort,
        const si::EgoMotionPort&                                    inEgoMotionPort,
        const si::PerceptionAvailabilityPort&                       inPerceptionAvailabilityPort,
        const ap_vehstatesigprovider::TrailerStatusPort&            inTrailerStatusPort,
        const ap_vehstatesigprovider::DoorStatusPort&               inDoorStatusPort,
        const ap_vehstatesigprovider::VehicleOccupancyStatusPort&   inVehicleOccupancyStatusPort,
        const ap_vehstatesigprovider::TrunkLidStatusPort &          inTrunkLidStatusPort,
        const ap_commonvehsigprovider::EngineCtrlStatusPort&        inEngineCtrlStatusPort,
        const ap_commonvehsigprovider::GearboxCtrlStatusPort&       inGearboxCtrlStatusPort,
        const ap_lodmc::LoDMCStatusPort&                            inLoDMCStatusPort,
        const ap_trjctl::MFControlStatusPort&                       inMfCtrlStatusPort,
        mf_lsca::LscaStatusPort&                                    outStatusPort,
        mf_lsca::LscaBrakePort&                                     outBrakePort,
        mf_lsca::LscaSteerPort&                                     outSteerPort,
        mf_lsca::LscaHMIPort&                                       outHmiPort)
    {
        //InputDummy: driver is always active
        static ap_ladmc::LaDMCStatusPort inFakeLaDMCStatusPort{};
        inFakeLaDMCStatusPort.handSteeringTorque_Nm = 1.0F;
        inFakeLaDMCStatusPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;

        //InData
        mf_lsca::LSCA_Input  inData{};
        inData.emData                     = &inCollEnvModelPort;
        inData.overrideLSCAPort           = &inOverrideLSCAPort;
        inData.slotCtrlPort               = &inSlotCtrlPort;
        inData.egoMotionPort              = &inEgoMotionPort;
        inData.laDMCStatusPort            = &inFakeLaDMCStatusPort;
        inData.engineCtrlStatusPort       = &inEngineCtrlStatusPort;
        //new signals
        inData.doorStatusPort             = &inDoorStatusPort;
        inData.emSensorStatusPort         = &inPerceptionAvailabilityPort;
        inData.trailerStatusPort          = &inTrailerStatusPort;
        inData.vehicleOccupancyStatusPort = &inVehicleOccupancyStatusPort;
        inData.trunkLidStatusPort         = &inTrunkLidStatusPort;
        inData.gearboxCtrlStatusPort      = &inGearboxCtrlStatusPort;
        inData.loDMCStatusPort            = &inLoDMCStatusPort;
        inData.mfControlStatusPort        = &inMfCtrlStatusPort;
        hpsd::HealthVectorPort dummyHealthVectorPort{ eco::create_default<hpsd::HealthVectorPort>() };
        dummyHealthVectorPort.sSigHeader = inDoorStatusPort.sSigHeader;
        inData.healthVectorPort = &dummyHealthVectorPort;

        //OutData
        mf_lsca::LSCA_Output outData{};
        outData.statusPort                = &outStatusPort;
        outData.brakePort                 = &outBrakePort;
        outData.steerPort                 = &outSteerPort;
        outData.hmiPort                   = &outHmiPort;
        static mf_lsca::LscaDebugDataPort tempLscaDebugDataPort;
        outData.debugDataPort             = &tempLscaDebugDataPort;

        static uint8_t lscaTerminated_u8{ 0U };//this error-catching is from 10ms thread -> more similar code means better testing (this would have caught last demo's error)
        if (10U > lscaTerminated_u8) //function runs normal
        {
            com::ComResult result = mf_lsca::LSCA_Interface::getInstance().run(inData, outData);
            //Wrapper for PMP functionality: if PMP wants to request braking -> map it to brake port for MF_Manager checks
            if ((outData.statusPort->PmpModuleState_nu == mf_lsca::LSCA_STATE::LSCA_STATE_INTERVENTION) && (outData.statusPort->brakingModuleState_nu != mf_lsca::LSCA_STATE::LSCA_STATE_INTERVENTION))
            {
                outData.statusPort->brakingModuleState_nu = mf_lsca::LSCA_STATE::LSCA_STATE_INTERVENTION;
            }

            if (result != com::ComResult::COMRES_OK) {
                lscaTerminated_u8++;
            }
            else {
                lscaTerminated_u8 = 0U;
            }
            return mf_lsca::LSCA_Interface::getInstance().getPlotStruct();
        }
        else //function is forced off!
        {
            mf_lsca::structs::plot_t plotOutput = mf_lsca::LSCA_Interface::getInstance().getPlotStruct();
            plotOutput.staticBraking.on_nu = false;
            plotOutput.staticSteering.resist.isOn_nu = false;
            plotOutput.staticSteering.propose.isOn_nu = false;
            return plotOutput;
        }
    }

#ifndef VARIANT_CUS_ONLY
    void stepLVMD(
        const si::EgoMotionPort&		inEgoMotionPort,
        const si::CollEnvModelPort&		inCollEnvModelPort,
        mf_lvmd::LvmdStatusPort&	inLVMDStatusport,
        const ap_commonvehsigprovider::EngineCtrlStatusPort& inEngineCtrlStatusPort,
        const ap_commonvehsigprovider::GearboxCtrlStatusPort& inGearboxCtrlStatusPort)

    {
        mf_lvmd::LVMD_Input inData{};
        inData.egoMotionPort = &inEgoMotionPort;
        inData.envModelPort = &inCollEnvModelPort;
        inData.engineCtrlStatusPort = &inEngineCtrlStatusPort;
        inData.gearboxCtrlStatusPort = &inGearboxCtrlStatusPort;
        mf_lvmd::LVMD_Output outData{};
        outData.lvmdStatusPort = &inLVMDStatusport;
        mf_lvmd::LVMD_Interface::getInstance().run(inData, outData);
    }
#endif

    void stepTCE(
        const ap_commonvehsigprovider::WheelPulsePort& wheelPulsePort,
        const ap_commonvehsigprovider::WheelDrivingDirectionsPort& wheelDrivingDirectionsPort,
        const ap_commonvehsigprovider::WheelSpeedPort& wheelSpeedPort,
        const ap_commonvehsigprovider::VehDynamicsPort& vehDynamicsPort,
        const ap_commonvehsigprovider::SteerCtrlStatusPort& steerCtrlStatusPort,
        const ap_commonvehsigprovider::SystemTimePort& systemTimePort,
        const ap_commonvehsigprovider::OdoGpsPort& odoGpsPort,
        const ap_commonvehsigprovider::BrakeCtrlStatusPort& brakeCtrlStatusPort,
        const ap_commonvehsigprovider::EngineCtrlStatusPort& engineCtrlStatusPort,
        tce::TceEstimationPort& tceEstimationPort,
        tce::TcePersDataPort& tcePersData,
        tce::TceDebugPort& tceDebugPort)
    {
        //InData
        tce::TceInput inData{};
        inData.wheelPulsePort = &wheelPulsePort;
        inData.wheelDrivingDirectionsPort = &wheelDrivingDirectionsPort;
        inData.wheelSpeedPort = &wheelSpeedPort;
        inData.vehDynamicsPort = &vehDynamicsPort;
        inData.steerCtrlStatusPort = &steerCtrlStatusPort;
        inData.systemTimePort = &systemTimePort;
        inData.odoGpsPort = &odoGpsPort;
        inData.brakeCtrlStatusPort = &brakeCtrlStatusPort;
        inData.engineCtrlStatusPort = &engineCtrlStatusPort;
        ap_commonvehsigprovider::WheelTireEscPort dummyWheelTireEscPort{ eco::create_default<ap_commonvehsigprovider::WheelTireEscPort>() };
        dummyWheelTireEscPort.sSigHeader = wheelPulsePort.sSigHeader;
        const float32_t defaultTireCircumference{ 2.053F };
        dummyWheelTireEscPort.tireCircumFL_m = defaultTireCircumference;
        dummyWheelTireEscPort.tireCircumFR_m = defaultTireCircumference;
        dummyWheelTireEscPort.tireCircumRL_m = defaultTireCircumference;
        dummyWheelTireEscPort.tireCircumRR_m = defaultTireCircumference;
        inData.wheelTireEscPort = &dummyWheelTireEscPort;

        //OutData
        tce::TceOutput outData{};
        outData.tceEstimationPort = &tceEstimationPort;
        outData.tcePersData = &tcePersData;
        outData.tceDebugPort = &tceDebugPort;

        //Run
        tce::Tce_Interface::getInstance().run(inData, outData);
    }

    void transformToCarMakerCoordinates(const LSM_GEOML::Pose& odoEstOffset, float32_t &xPos, float32_t &yPos, float32_t &yaw) {
        const float32_t oldOdoX_m{ xPos };
        const float32_t oldOdoY_m{ yPos };
        yaw += odoEstOffset.Yaw_rad();
        xPos = oldOdoX_m * cos(odoEstOffset.Yaw_rad()) + oldOdoY_m * -sin(odoEstOffset.Yaw_rad()) + odoEstOffset.Pos().x();
        yPos = oldOdoX_m * sin(odoEstOffset.Yaw_rad()) + oldOdoY_m * cos(odoEstOffset.Yaw_rad()) + odoEstOffset.Pos().y();
    }

    void stepVEDODO(
        const CarMakerInterface& carMakerInterface,
        const lsm_vedodo::OdoEstimation& odoEstimationPortCM,
        const ap_commonvehsigprovider::WheelPulsePort& wheelPulsePort,
        const ap_commonvehsigprovider::WheelDrivingDirectionsPort& wheelDrivingDirectionsPort,
        const ap_commonvehsigprovider::WheelSpeedPort& wheelSpeedPort,
        const tce::TceEstimationPort& tcePort,
        const ap_commonvehsigprovider::VehDynamicsPort& vehDynamicsPort,
        const ap_commonvehsigprovider::SteerCtrlStatusPort& steerCtrlStatusPort,
        const ap_commonvehsigprovider::OdoExtCtrlPort& odoExtCtrlPort,
        const ap_commonvehsigprovider::SystemTimePort& systemTimePort,
        const ap_commonvehsigprovider::GearboxCtrlStatusPort& gearboxCtrlStatusPort,
        const ap_commonvehsigprovider::SuspensionPort& suspensionPort,
        const ap_psm::SlotCtrlPort& slotCtrlPort,
        lsm_vedodo::OdoEstimationOutputPort& odoEstimationOutputPort,
        lsm_vedodo::OdoNVMData& odoPersDataPort,
        lsm_vedodo::OdoDebugPort& odoDebugPort,
        eco::CarMakerSystemServices& carMakerSystemServices)
    {

        lsm_vedodo::VEDODO_Input inData{};
        lsm_vedodo::VEDODO_Output outData{};
        if (carMakerInterface.vedodoActive_nu) {
            //InData
            inData.odoExtCtrlPort = &odoExtCtrlPort;
            inData.wheelPulsePort = &wheelPulsePort;
            inData.wheelDrivingDirectionsPort = &wheelDrivingDirectionsPort;
            inData.wheelSpeedPort = &wheelSpeedPort;
            inData.tcePort = &tcePort;
            inData.vehDynamicsPort = &vehDynamicsPort;
            inData.steerCtrlStatusPort = &steerCtrlStatusPort;

            inData.odoExtCtrlPort = &odoExtCtrlPort;
            inData.systemTimePort = &systemTimePort;
            inData.gearboxCtrlStatusPort = &gearboxCtrlStatusPort;
            inData.suspensionPort = &suspensionPort;
            //Workaround within VEDODO to deactivate side slip compensation in case of active parking or garage parking maneuver
            // ODO decided to remove this feature => disance to curb will increase in real world parking
            //if ((slotCtrlPort.planningCtrlCommands.apState == ap_psm::APState::AP_AVG_ACTIVE_IN) ||
            //    (slotCtrlPort.planningCtrlCommands.apState == ap_psm::APState::AP_AVG_ACTIVE_OUT) ||
            //    (slotCtrlPort.planningCtrlCommands.gpState == ap_psm::GPState::GP_AVG_ACTIVE_IN) ||
            //    (slotCtrlPort.planningCtrlCommands.gpState == ap_psm::GPState::GP_AVG_ACTIVE_OUT))
            //{
            //    inData.isParkingManeuver = true; // side slip compensation deactivated
            //}
            //else
            //{
            //    inData.isParkingManeuver = false; // side slip compensation not deactivated (HINT: The COR param within VEDODO might be set to zero (LSM_O_REAR_AXLE_TO_COR_1_M = 0) for SIL until side slip effect modeled in SIL. This means no effect of side slip compensation in VEDODO.)
            //}

            //OutData
            outData.odoEstimationOutputPort = &odoEstimationOutputPort;
            outData.odoPersDataPort = &odoPersDataPort;
            outData.odoDebugPort = &odoDebugPort;

            //Run
            com::ComResult comResult = lsm_vedodo::VEDODO_Interface::getInstance().run(inData, outData, &carMakerSystemServices);

            // Transform VEDODO output in CarMaker origin
            if (odoExtCtrlPort.resetPoseEstimation_nu == b_TRUE) {
                if (systemTimePort.sSigHeader.uiTimeStamp <= 2000U * VEDODO_SAMPLE_TIME_MS)
                {
                    //If reset is triggered in first two vedodo cycles, set odometry offset
                    gOdometryOffset.Pos().x() = odoEstimationPortCM.xPosition_m - odoEstimationOutputPort.odoEstimation.xPosition_m;
                    gOdometryOffset.Pos().y() = odoEstimationPortCM.yPosition_m - odoEstimationOutputPort.odoEstimation.yPosition_m;
                    gOdometryOffset.Yaw_rad() = odoEstimationPortCM.yawAngle_rad - odoEstimationOutputPort.odoEstimation.yawAngle_rad;
                }
                else
                {
                    //If reset is triggered later (e.g. for ignition cylce park out), vedodo pose has to start at zero again.
                    gOdometryOffset = LSM_GEOML::Pose{};
                }
                gOdometryWasReseted = true;
            }
            // Recalibrate the VEDODO position output to the ground-truth position from CarMaker (only possible once).
            // This is necessary if the ego vehicle drives more than 1km to compensate a systematic drift in the position output from VEDODO.
            if (carMakerInterface.odoRecalibratePosition_nu && !gOdoPositionWasRecalibrated)
            {
                //Set odometry position offset
                const float32_t xPostionCM_compensatedOdoOverflow_m{ odoEstimationPortCM.xPosition_m - std::floor(odoEstimationPortCM.xPosition_m / 1000) * 1000 };
                gOdometryOffset.Pos().x() = xPostionCM_compensatedOdoOverflow_m - odoEstimationOutputPort.odoEstimation.xPosition_m;
                gOdometryOffset.Pos().y() = odoEstimationPortCM.yPosition_m - odoEstimationOutputPort.odoEstimation.yPosition_m;
                gOdoPositionWasRecalibrated = true;
            }
            //if ((odoEstimationOutputPort.odoEstimation.sSigHeader.eSigStatus == com::ComSignalState_t::COMSIGSTATE_VALID) && (comResult == com::ComResult::COMRES_OK))
            if ((odoEstimationOutputPort.odoEstimation.sSigHeader.eSigStatus == eco::AlgoSignalState::AL_SIG_STATE_OK) && (comResult == com::ComResult::COMRES_OK))
            {
                //Transform VEDODO output
                transformToCarMakerCoordinates(gOdometryOffset, odoEstimationOutputPort.odoEstimation.xPosition_m, odoEstimationOutputPort.odoEstimation.yPosition_m, odoEstimationOutputPort.odoEstimation.yawAngle_rad);
                for (auto &odoEstBuf : odoEstimationOutputPort.odoEstimationBuffer) {
                    transformToCarMakerCoordinates(gOdometryOffset, odoEstBuf.xPosition_m, odoEstBuf.yPosition_m, odoEstBuf.yawAngle_rad);
                }
                for (auto &odoEstPreBuf : odoEstimationOutputPort.odoPredictionBuffer) {
                    transformToCarMakerCoordinates(gOdometryOffset, odoEstPreBuf.xPosition_m, odoEstPreBuf.yPosition_m, odoEstPreBuf.yawAngle_rad);
                }
            }
        }
        else {
            //VEDODO inactive:
            gOdometryWasReseted = true;
            odoEstimationOutputPort.odoEstimation = odoEstimationPortCM;
        }
    }

    // PARKSM HMI visualization
#ifdef USE_HMI_VIS
        void plotHMI() {
            gAPHMIPlotter.plot();
        }

        void setGlutData() {
            gAPHMIPlotter.setData(gHmiInputPort, gHeadUnitVisualizationPort, gRemoteVisualizationPort);
            glutPostRedisplay();
        }

        void startPlotThread() {
            gAPHMIPlotter.init(1000.0f, 600.0f, 900.0f, 50.0f);
            glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
            glutDisplayFunc(plotHMI);
            glutIdleFunc(setGlutData);
            glutMainLoop();
        }
#endif

        void stepPARKSMCore(
            const CarMakerInterface& carMakerInterface,
            const ap_tp::TargetPosesPort& targetPosesPort,
            const ap_trjctl::MFControlStatusPort& mfControlStatusPort,
            const si::EgoMotionPort& egoMotionPort,
            const si::ApEnvModelPort& apEnvModelPort,
            const ap_psm_app::CtrlCommandPort& ctrlCommandPort,
            ap_psm::TrajCtrlRequestPort& trajCtrlRequestPort,
            ap_psm::SlotCtrlPort& slotCtrlPort,
            ap_psm::PARKSMCoreStatusPort& parksmCoreStatusPort,
            ap_psm::PARKSMCoreDebugPort& parksmoreDebugPort)
        {
        // Extern overwrite of VEDODO velocity to allow parking in without scanning Testruns
        si::EgoMotionPort egoMotionPortLocal = egoMotionPort;
        if (carMakerInterface.odoOverwriteVehVelocity_nu) {
            egoMotionPortLocal.vel_mps = carMakerInterface.odoOverwriteVehVelocity_mps;
        }

        // InData
        mf_psm::PARKSMCore_Input inData{};
        inData.ctrlCommandPort = &ctrlCommandPort;
        inData.egoMotionPort = &egoMotionPortLocal;
        inData.apEnvModelPort = &apEnvModelPort;
        inData.mfControlStatusPort = &mfControlStatusPort;
        inData.targetPosesPort = &targetPosesPort;
        // Determine parksmCoreSampleTimePort
        const uint64_t parksmSampleTime_us{ PARKSM_SAMPLE_TIME_MS * 1000u };

        // OutData
        mf_psm::PARKSMCore_Output outData{};
        outData.parksmCoreStatusPort = &parksmCoreStatusPort;
        outData.parksmCoreDebugPort = &parksmoreDebugPort;
        outData.trajCtrlRequestPort = &trajCtrlRequestPort;
        outData.slotCtrlPort = &slotCtrlPort;

        //Run
        mf_psm::PARKSM_Core_Interface::getInstance().run(inData, outData, parksmSampleTime_us);
    }

    void stepPARKSM(
        const CarMakerInterface& carMakerInterface,
        const ap_trjctl::LoDMCCtrlRequestPort& loDMCCtrlRequestPort,
        const ap_trjctl::GearboxCtrlRequestPort& gearBoxCtrlRequestPort,
        const ap_trjctl::MFControlStatusPort& mfControlStatusPort,
        const ap_tp::TargetPosesPort& targetPosesPort,
        const mf_mempark::MemoryParkingStatusPort &memoryParkingStatusPort,
        const ap_commonvehsigprovider::GearboxCtrlStatusPort& gearboxCtrlStatusPort,
        const ap_commonvehsigprovider::EngineCtrlStatusPort& engineCtrlStatusPort,
        const ap_vehstatesigprovider::StarterStatusPort &starterStatusPort,
        const ap_vehstatesigprovider::TrunkLidStatusPort &trunkLidStatusPort,
        const ap_vehstatesigprovider::ConvertibleTopStatusPort &convertibleTopStatusPort,
        const ap_vehstatesigprovider::SteeringColSwitchesStatusPort &steeringColSwitchesStatusPort,
        const ap_vehstatesigprovider::TrailerStatusPort &trailerStatusPort,
        const ap_vehstatesigprovider::DoorStatusPort &doorStatusPort,
        const ap_vehstatesigprovider::VehicleOccupancyStatusPort &vehicleOccupancyStatusPort,
        const ap_vehstatesigprovider::AdditionalBCMStatusPort &additionalBCMStatusPort,
        const ap_vehstatesigprovider::ACCInformationPort &accInformationPort,
        const ap_vehstatesigprovider::ESCInformationPort &escInformationPort,
        const ap_vehstatesigprovider::ExternalFunctionStatusPort &externalFunctionStatusPort,
        const ap_vehstatesigprovider::KeylessStatusPort &keylessStatusPort,
        const ap_vehstatesigprovider::AuthenticationStatusPort &authenticationStatusPort,
        const ap_ladmc::LaDMCStatusPort& laDMCStatusPort,
        const ap_lodmc::LoDMCStatusPort& loDMCStatusPort,
        const mf_drvwarnsm_core::DrvWarnCoreStatusPort& drvWarnCoreStatusPort,
        const mf_hmih::APUserInteractionPort& apUserInteractionPort,
        const mf_lsca::LscaStatusPort& lscaStatusPort,
        const si::EgoMotionPort& egoMotionPort,
        const si::PerceptionAvailabilityPort& perceptionAvailabilityPort,
        const ap_psm::SlotCtrlPort& slotCtrlPort,
        const ap_psm::PARKSMCoreStatusPort& parksmCoreStatusPort,
        ap_psm_app::PSMToSSPPort& psmToSSPPort,
        ap_psm_app::CtrlCommandPort& ctrlCommandPort,
        ap_psm_app::APUserInformationPort& apUserInformationPort,
        ap_psm_app::OverrideLSCAPort& overrideLSCAPort,
        ap_psm_app::PSMDebugPort& psmDebugPort,
        eco::CarMakerSystemServices& carMakerSystemServices)
    {
        //InData
        ap_psm_app::PARKSM_Input inData{};

        // Extern overwrite of VEDODO velocity to allow parking in without scanning Testruns
        si::EgoMotionPort egoMotionPortLocal{ egoMotionPort };
        if (carMakerInterface.odoOverwriteVehVelocity_nu) {
            egoMotionPortLocal.vel_mps = carMakerInterface.odoOverwriteVehVelocity_mps;
        }

        // Fake drvWarnCoreStatusPort in order to run AP simulation if PDC disabled/failure
        mf_drvwarnsm_core::DrvWarnCoreStatusPort dummyDrvWarnCoreStatusPort;
        dummyDrvWarnCoreStatusPort.pdwActiveState_nu = true;
        dummyDrvWarnCoreStatusPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;

        static bool pdwFailureTestRun{ false };
        if (carMakerInterface.pdwFailure_nu) {
            pdwFailureTestRun = true;
        }
        if (carMakerInterface.pdwDisabled_nu || pdwFailureTestRun) {
            inData.drvWarnCoreStatusPort = &dummyDrvWarnCoreStatusPort;
        }
        else {
            inData.drvWarnCoreStatusPort = &drvWarnCoreStatusPort;
        }

        inData.gearboxCtrlStatusPort = &gearboxCtrlStatusPort;
        inData.engineCtrlStatusPort = &engineCtrlStatusPort;
        inData.starterStatusPort = &starterStatusPort;
        inData.trunkLidStatusPort = &trunkLidStatusPort;
        inData.convertibleTopStatusPort = &convertibleTopStatusPort;
        inData.steeringColSwitchesStatusPort = &steeringColSwitchesStatusPort;
        inData.trailerStatusPort = &trailerStatusPort;
        inData.doorStatusPort = &doorStatusPort;
        inData.vehicleOccupancyStatusPort = &vehicleOccupancyStatusPort;
        inData.perceptionAvailabilityPort = &perceptionAvailabilityPort;
        inData.additionalBCMStatusPort = &additionalBCMStatusPort;
        inData.accInformationPort = &accInformationPort;
        inData.escInformationPort = &escInformationPort;
        inData.externalFunctionStatusPort = &externalFunctionStatusPort;
        inData.keylessStatusPort = &keylessStatusPort;
        inData.authenticationStatusPort = &authenticationStatusPort;
        inData.laDMCStatusPort = &laDMCStatusPort;
        inData.egoMotionPort = &egoMotionPortLocal;
        inData.lscaStatusPort = &lscaStatusPort;
        inData.apUserInteractionPort = &apUserInteractionPort;
        inData.loDMCStatusPort = &loDMCStatusPort;
        inData.loDMCCtrlRequestPort = &loDMCCtrlRequestPort;
        inData.mfControlStatusPort = &mfControlStatusPort;
        inData.targetPosesPort = &targetPosesPort;
        inData.memoryParkingStatusPort = &memoryParkingStatusPort;
        inData.gearboxCtrlRequestPort = &gearBoxCtrlRequestPort;
        // Determine psmSampleTimePort
        ap_commonvehsigprovider::PSMSampleTimePort psmSampleTimePortLocal{};
        psmSampleTimePortLocal.psmSampleTime_us = PARKSM_SAMPLE_TIME_MS * 1000u;
        psmSampleTimePortLocal.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        inData.psmSampleTimePort = &psmSampleTimePortLocal;
        inData.slotCtrlPort = &slotCtrlPort;
        inData.parksmCoreStatusPort = &parksmCoreStatusPort;
        hpsd::HealthVectorPort dummyHealthVectorPort{ eco::create_default<hpsd::HealthVectorPort>() };
        dummyHealthVectorPort.sSigHeader = gearboxCtrlStatusPort.sSigHeader;
        inData.healthVectorPort = &dummyHealthVectorPort;

        //OutData
        ap_psm_app::PARKSM_Output outData{};
        outData.apUserInformationPort = &apUserInformationPort;
        outData.psmToSSPPort = &psmToSSPPort;
        outData.psmDebugPort = &psmDebugPort;
        outData.overrideLSCAPort = &overrideLSCAPort;
        outData.ctrlCommandPort = &ctrlCommandPort;

        //Run
        ap_psm_app::PARKSM_Interface::getInstance().run(inData, outData, &carMakerSystemServices);

        // PARKSM HMI visualization
#ifdef USE_HMI_VIS
        // compare memory of HMI matrix memcmp
        int erg = abs(memcmp(&gHmiInputPort.parkingSpaces.right, &gOldHmiInputPort.parkingSpaces.right, sizeof(gHmiInputPort.parkingSpaces.right)));
        erg = erg + abs(memcmp(&gHmiInputPort.parkingSpaces.left, &gOldHmiInputPort.parkingSpaces.left, sizeof(gHmiInputPort.parkingSpaces.left)));
        erg = erg + abs(memcmp(&gHeadUnitVisualizationPort.screen_nu, &gOldHeadUnitVisualizationPort.screen_nu, sizeof(gHeadUnitVisualizationPort.screen_nu)));
        erg = erg + abs(memcmp(&gHeadUnitVisualizationPort.message_nu, &gOldHeadUnitVisualizationPort.message_nu, sizeof(gHeadUnitVisualizationPort.message_nu)));
        erg = erg + abs(memcmp(&gRemoteVisualizationPort.screen_nu, &gOldRemoteVisualizationPort.screen_nu, sizeof(gRemoteVisualizationPort.screen_nu)));
        erg = erg + abs(memcmp(&gRemoteVisualizationPort.message_nu, &gOldRemoteVisualizationPort.message_nu, sizeof(gRemoteVisualizationPort.message_nu)));
        // start glut loop
        static bool threadStarted = false;
        if (!threadStarted) {
            try
            {
                std::thread t1(startPlotThread);
                t1.detach();
            }
            catch (const std::exception &ex)
            {
                std::cout << "Thread exited with exception: " << ex.what() << "\n";
            }
            threadStarted = true;
        }
        if (erg != 0) {
            while (!userHMIAction) {
                if ((GetKeyState(VK_LBUTTON) & 0x80) != 0) {
                    userHMIAction = true;
                }
            }
            userHMIAction = false;

            gOldHmiInputPort = gHmiInputPort;
            gOldHeadUnitVisualizationPort = gHeadUnitVisualizationPort;
            gOldRemoteVisualizationPort = gRemoteVisualizationPort;
        }
#endif
    }

    void stepTRJPLA(
        const si::ApEnvModelPort& apEnvModelPort,
        const si::EgoMotionPort& egoMotionPort,
        const si::ApParkingBoxPort& apParkingBoxPort,
        const ap_trjctl::MFControlStatusPort& mfControlStatusPort,
        const ap_psm::SlotCtrlPort& slotCtrlPort,
        ap_tp::TargetPosesPort& targetPosesPort,
        ap_tp::PlannedTrajPort& plannedTrajPort,
        ap_tp::TrajPlanVisuPort& trajPlanVisuPort,
        ap_tp::TrajPlanDebugPort& trajPlanDebugPort,
        ap_tp::TAPOSDDebugPort& taposdDebugPort,
        ap_tp::ReverseAssistAvailabilityPort reverseAssistAvailabilityPort
        )
    {
        //InData
        ap_tp::TRJPLA_Input trjplaInData{};
        trjplaInData.emData = &apEnvModelPort;
        trjplaInData.egoMotionData = &egoMotionPort;
        trjplaInData.parkingBoxPort = &apParkingBoxPort;
        trjplaInData.stateMachineIn = &slotCtrlPort;
        trjplaInData.mfControlStatusPort = &mfControlStatusPort;

        //OutData
        ap_tp::TRJPLA_Output trjplaOutData{};
        trjplaOutData.targetPoses = &targetPosesPort;
        trjplaOutData.plannedTrajectory = &plannedTrajPort;
        trjplaOutData.trjplaVisuPort = &trajPlanVisuPort;
        trjplaOutData.trjplaDebugPort = &trajPlanDebugPort;
        trjplaOutData.taposdDebugPort = &taposdDebugPort;
        trjplaOutData.reverseAssistAvailabilityPort = &reverseAssistAvailabilityPort;
        ap_tp::DrivenPathDataPort dummyDrivenPathDataPort{};
        trjplaOutData.drivenPathDataPort = &dummyDrivenPathDataPort;

        //Run
        ap_tp::TRJPLA_Interface::getInstance().run(trjplaInData, trjplaOutData);
    }

    void stepMFManager(
        const ap_psm::TrajCtrlRequestPort& trajCtrlRequestPort,
        const ap_tp::PlannedTrajPort& plannedTrajPort,
        const mf_lsca::LscaStatusPort& lscaStatusPort,
        const mf_lsca::LscaBrakePort& lscaBrakePort,
        const mf_lsca::LscaSteerPort& lscaSteerPort,
        mf_manager::ActiveManeuveringFunctionPort& activeManeuveringFunctionPort,
        mf_manager::LaCtrlRequestPort& laCtrlRequestPort,
        mf_manager::LoCtrlRequestPort& loCtrlRequestPort,
        mf_manager::TrajRequestPort& trajRequestPort)
    {
        //InData
        mf_manager::MFManager_Input mfmanagerInData{};
        mfmanagerInData.apPlannedTrajPort = &plannedTrajPort;
        mfmanagerInData.apTrajCtrlRequestPort = &trajCtrlRequestPort;
        mfmanagerInData.lscaBrakePort = &lscaBrakePort;
        mfmanagerInData.lscaSteerPort = &lscaSteerPort;
        mfmanagerInData.lscaStatusPort = &lscaStatusPort;

        //OutData
        mf_manager::MFManager_Output mfmanagerOutData{};
        mfmanagerOutData.activeManeuveringFunctionPort = &activeManeuveringFunctionPort;
        mfmanagerOutData.laCtrlRequestPort = &laCtrlRequestPort;
        mfmanagerOutData.loCtrlRequestPort = &loCtrlRequestPort;
        mfmanagerOutData.trajRequestPort = &trajRequestPort;

        //Run
        mf_manager::MF_Manager_Interface::getInstance().run(mfmanagerInData, mfmanagerOutData);
    }

    void stepMFControl(
        const ap_commonvehsigprovider::GearboxCtrlStatusPort& gearboxCtrlStatusPort,
        ap_commonvehsigprovider::TRJCTLGeneralInputPort& trjctrlGeneralInputPort,
        const ap_ladmc::LaDMCStatusPort& laDMCStatusPort,
#ifdef MOCO_REPLACES_LODMC
        const TRATCO_t_CpldTratcoStatus& tratcoStatusPort,
        const VECONA_t_LongVeconaStatus& longVeconaStatusPort,
#else
        const ap_lodmc::LoDMCStatusPort& loDMCStatusPort,
#endif
        const lsm_vedodo::OdoEstimationOutputPort& odoEstimationOutputPort,
        const mf_manager::LaCtrlRequestPort& laCtrlRequestPort,
        const mf_manager::LoCtrlRequestPort& loCtrlRequestPort,
        const mf_manager::TrajRequestPort& trajRequestPort,
        ap_trjctl::GearboxCtrlRequestPort& gearboxCtrlRequestPort,
        ap_trjctl::LaDMCCtrlRequestPort& laDMCCtrlRequestPort,
        ap_trjctl::LoDMCCtrlRequestPort& loDMCCtrlRequestPort,
#ifdef MOCO_REPLACES_LODMC
        ap_trjctl::MF_CONTROL_t_LongManeuverRequestPort& longManeuverRequestPort,
#endif
        ap_trjctl::MFControlStatusPort& mfControlStatusPort,
        ap_trjctl::DrivingResistancePort& drivingResistancePort,
        ap_trjctl::TrajCtrlDebugPort& trajCtrlDebugPort,
        eco::CarMakerSystemServices& carMakerSystemServices)
    {
        //InData
        ap_trjctl::MF_Control_Input trjctlInData{};
        trjctlInData.gearboxCtrlStatusPort = &gearboxCtrlStatusPort;

#ifdef MOCO_REPLACES_LODMC
        trjctlInData.tratcoStatusPort = &tratcoStatusPort;
        trjctlInData.longVeconaStatusPort = &longVeconaStatusPort;
#else
        trjctlInData.loDMCStatusPort = &loDMCStatusPort;
#endif
        trjctlInData.laDMCStatusPort = &laDMCStatusPort;
        trjctlInData.odoEstimationOutputPort = &odoEstimationOutputPort;
        trjctlInData.trajRequestPort = &trajRequestPort;
        trjctlInData.laCtrlRequestPort = &laCtrlRequestPort;
        trjctlInData.loCtrlRequestPort = &loCtrlRequestPort;
        // Determine trjctlGeneralInputPort
        trjctrlGeneralInputPort.trjctlSampleTime_s = (float)TRJCTL_SAMPLE_TIME_MS / 1000.0f;
        trjctrlGeneralInputPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        trjctrlGeneralInputPort.sSigHeader.uiTimeStamp = odoEstimationOutputPort.sSigHeader.uiTimeStamp;
        trjctlInData.trjctlGeneralInputPort = &trjctrlGeneralInputPort;

        //OutData
        ap_trjctl::MF_Control_Output trjctlOutData{};
        trjctlOutData.gearboxCtrlRequestPort = &gearboxCtrlRequestPort;
        trjctlOutData.loDMCCtrlRequestPort = &loDMCCtrlRequestPort;
#ifdef MOCO_REPLACES_LODMC
        trjctlOutData.longManeuverRequestPort = &longManeuverRequestPort;
#endif
        trjctlOutData.laDMCCtrlRequestPort = &laDMCCtrlRequestPort;
        trjctlOutData.trajCtrlDebugPort = &trajCtrlDebugPort;
        trjctlOutData.mfControlStatusPort = &mfControlStatusPort;
        trjctlOutData.drivingResistancePort = &drivingResistancePort;

        //Run
        ap_trjctl::MF_Control_Interface::getInstance().run(trjctlInData, trjctlOutData, &carMakerSystemServices);
    }

    void stepMFHMIHandler(
        const si::EgoMotionPort& egoMotionPort,
        const si::ApEnvModelPort& apEnvModelPort,
        const si::ApParkingBoxPort& apParkingBoxPort,
        const ap_hmitoap::HMIOutputPort& hmiOutputPort,
        const ap_hmitoap::RemoteHMIOutputPort& remoteHMIOutputPort,
        const ap_hmitoap::VisuInputData& visuInputData,
        const ap_psm::SlotCtrlPort& slotCtrlPort,
        const ap_psm_app::APUserInformationPort& apUserInformationPort,
        const mf_lsca::LscaHMIPort& lscaHmiPort,
        const mf_drvwarnsm::DrvWarnStatusPort& drvWarnStatusPort,
        const mf_drvwarnsm_core::DrvWarnCoreStatusPort& drvWarnCoreStatusPort,
        const pdcp::PDCPSectorsPort& pdcpSectorsPort,
        const pdcp::PDCPDrivingTubePort& pdcpDrivingTubePort,
        const mf_whlprotectproc::WHPProcOutputPort& whpProcOutputPort,
        const ap_psm::PARKSMCoreStatusPort& parkSMCoreStatusPort,
        const ap_trjctl::LoDMCCtrlRequestPort& loDMCCtrlRequestPort,
        const ap_tp::TargetPosesPort& targetPosesPort,
        const ap_tp::TrajPlanVisuPort& trajPlanVisuPort,
        const vc::ScreenSwitchDataPort& visuOutputData,
        const lsm_vedodo::OdoEstimationOutputPort& odoEstimationOutputPort,
        CarMakerInterface& carMakerInterface,
        mf_hmih::HMIGeneralInputPort &hmiInputPort,
        mf_hmih::HeadUnitVisualizationPort& headUnitVisualizationPort,
        mf_hmih::RemoteVisualizationPort& remoteVisualizationPort,
        mf_hmih::APUserInteractionPort& apUserInteractionPort,
        mf_hmih::PDCUserInteractionPort& pdcUserInteractionPort,
        mf_hmih::SurroundViewRequestPort& surroundViewRequestPort,
        mf_hmih::MFHmiHDebugPort &mfHmiHDebugPort,
        mf_hmih::UserDefinedSlotPort& userDefinedSlotPort,
        mf_hmih::VisuInputPort& visuInputPort,
        mf_hmih::LVMDUserInteractionPort& lvmdUserInteractionPort,
        eco::CarMakerSystemServices& carMakerSystemServices
        )
    {
        //InData
        mf_hmih::MFHMIH_Input inData{};
        mf_lvmd::LvmdStatusPort dummyLVMDStatusPort;
        vc::OverlayRequestDataPort dummyOverlayRequestDataPort;
        dummyOverlayRequestDataPort.selectedParkingPose.selected_pose_id = 255U;//port containing visu slot selection, dummy in mf_sil integration
        dummyOverlayRequestDataPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        dummyOverlayRequestDataPort.sSigHeader.uiCycleCounter = targetPosesPort.sSigHeader.uiCycleCounter;
        dummyOverlayRequestDataPort.sSigHeader.uiMeasurementCounter = targetPosesPort.sSigHeader.uiMeasurementCounter;
        dummyOverlayRequestDataPort.sSigHeader.uiTimeStamp = targetPosesPort.sSigHeader.uiTimeStamp;

        inData.apUserInformationPort = &apUserInformationPort;
        inData.egoMotionPort = &egoMotionPort;
        inData.apEnvModelPort = &apEnvModelPort;
        inData.apParkingBoxPort = &apParkingBoxPort;
        inData.targetPosesPort = &targetPosesPort;
        inData.hmiOutputPort = &hmiOutputPort;
        inData.slotCtrlPort = &slotCtrlPort;
        inData.remoteHMIOutputPort = &remoteHMIOutputPort;
        inData.screenSwitchDataPort = &visuOutputData;
        inData.drvWarnStatusPort = &drvWarnStatusPort;
        inData.drvWarnCoreStatusPort = &drvWarnCoreStatusPort;
        inData.pdcpDrivingTubePort = &pdcpDrivingTubePort;
        inData.pdcpSectorsPort = &pdcpSectorsPort;
        inData.whpProcOutputPort = &whpProcOutputPort;
        inData.lscaHmiPort = &lscaHmiPort;
        inData.parksmCoreStatusPort = &parkSMCoreStatusPort;
        inData.visuInputData = &visuInputData;
        inData.loDMCCtrlRequestPort = &loDMCCtrlRequestPort;
        inData.lvmdStatusPort = &dummyLVMDStatusPort;
        inData.trajPlanVisuPort = &trajPlanVisuPort;
        inData.overlayRequestDataPort = &dummyOverlayRequestDataPort;
        inData.odoEstimationOutputPort = &odoEstimationOutputPort;

        // Determine hmiHandlerSampleTimePort
        ap_commonvehsigprovider::HMIHandlerSampleTimePort hmiHandlerSampleTimePortLocal{};
        hmiHandlerSampleTimePortLocal.hmiHandlerSampleTime_us = HMIH_SAMPLE_TIMES_MS * 1000u;
        hmiHandlerSampleTimePortLocal.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        inData.hmiHandlerSampleTimePort = &hmiHandlerSampleTimePortLocal;

        //OutData
        mf_hmih::MFHMIH_Output outData{};
        outData.headUnitVisualizationPort = &headUnitVisualizationPort;
        outData.hmiInputPort = &hmiInputPort;
        outData.remoteVisualizationPort = &remoteVisualizationPort;
        outData.apUserInteractionPort = &apUserInteractionPort;
        outData.surroundViewRequestPort = &surroundViewRequestPort;
        outData.mfHmiHDebugPort = &mfHmiHDebugPort;
        outData.pdcUserInteractionPort = &pdcUserInteractionPort;
        outData.userDefinedSlotPort = &userDefinedSlotPort;
        outData.visuInputPort = &visuInputPort;
        outData.lvmdUserInteractionPort = &lvmdUserInteractionPort;

        //Run
        mf_hmih::MFHMIH_Interface::getInstance().run(inData, outData, &carMakerSystemServices);

        //Update global variables
        gHmiInputPort = *outData.hmiInputPort;
        gHeadUnitVisualizationPort = *outData.headUnitVisualizationPort;
        gRemoteVisualizationPort = *outData.remoteVisualizationPort;
        gUserDefinedSlotPort = *outData.userDefinedSlotPort;
        gAPUserInteractionPort = *outData.apUserInteractionPort;
        gLvmdUserInteractionPort = *outData.lvmdUserInteractionPort;
        gVisuInputPort = *outData.visuInputPort;

        //Copy screen/message to carMakerInterface
        carMakerInterface.headUnitScreen_nu = static_cast<uint8_t>(outData.headUnitVisualizationPort->screen_nu);
        carMakerInterface.headUnitMessage_nu = static_cast<uint8_t>(outData.headUnitVisualizationPort->message_nu);
        carMakerInterface.remoteScreen_nu = static_cast<uint8_t>(outData.remoteVisualizationPort->screen_nu);
        carMakerInterface.remoteMessage_nu = static_cast<uint8_t>(outData.remoteVisualizationPort->message_nu);
    }

    void stepDrvWarnSMCore(
        const si::EgoMotionPort& egoMotionPort,
        const mf_drvwarnsm::AppToCoreSMPort& appToCoreSMPort,
        mf_drvwarnsm_core::DrvWarnCoreStatusPort& drvWarnCoreStatusPort,
        mf_drvwarnsm_core::DrvWarnCoreDebugPort& drvWarnCoreDebugPort)
    {
        //InData
        mf_drvwarnsm_core::DrvWarnSMCore_Input inData{};
        inData.egoMotionPort = &egoMotionPort;
        inData.appToCoreSMPort = &appToCoreSMPort;
        // Determine drvWarnSMCoreSampleTimePort
        ap_commonvehsigprovider::DrvWarnSMCoreSampleTimePort drvWarnSMCoreSampleTimePortLocal{};
        drvWarnSMCoreSampleTimePortLocal.drvWarnSMCoreSampleTime_us = DRVWARNSM_SAMPLE_TIME_MS * 1000u;
        drvWarnSMCoreSampleTimePortLocal.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        inData.drvWarnSMCoreSampleTimePort = &drvWarnSMCoreSampleTimePortLocal;

        //OutData
        mf_drvwarnsm_core::DrvWarnSMCore_Output outData{};
        outData.drvWarnCoreStatusPort = &drvWarnCoreStatusPort;
        outData.drvWarnCoreDebugPort = &drvWarnCoreDebugPort;

        //Run
        mf_drvwarnsm_core::DrvWarnSMCore_Interface::getInstance().run(inData, outData);
    }

    void stepDrvWarnSM(const ap_commonvehsigprovider::GearboxCtrlStatusPort& gearboxCtrlStatusPort,
        const ap_vehstatesigprovider::ESCInformationPort &escInformationPort,
        const ap_vehstatesigprovider::DoorStatusPort &doorStatusPort,
        const ap_vehstatesigprovider::TrunkLidStatusPort &trunkLidStatusPort,
        const ap_vehstatesigprovider::AdditionalBCMStatusPort &additionalBCMStatusPort,
        const ap_vehstatesigprovider::TrailerStatusPort& trailerStatusPort,
        const ap_lodmc::LoDMCStatusPort& loDMCStatusPort,
        const ap_psm::SlotCtrlPort& slotCtrlPort,
        const pdcp::PDCPSectorsPort& pdcpSectorsPort,
        const pdcp::ProcToLogicPort& procToLogicPort,
        const mf_hmih::PDCUserInteractionPort& pdcUserInteractionPort,
        const mf_whlprotectproc::WHPProcOutputPort& whpProcOutputPort,
        const si::EgoMotionPort& egoMotionPort,
        const si::PerceptionAvailabilityPort& perceptionAvailabilityPort,
        mf_drvwarnsm::DrvWarnStatusPort& drvWarnStatusPort,
        mf_drvwarnsm::DrvWarnDebugPort& drvWarnDebugPort,
        mf_drvwarnsm::AppToCoreSMPort& appToCoreSMPort,
        mf_drvwarnsm::LogicToProcPort& logicToProcPort)
    {
        //InData
        mf_drvwarnsm::DrvWarnSM_Input inData{};
        inData.gearboxCtrlStatusPort = &gearboxCtrlStatusPort;
        inData.escInformationPort = &escInformationPort;
        inData.doorStatusPort = &doorStatusPort;
        inData.trunkLidStatusPort = &trunkLidStatusPort;
        inData.additionalBCMStatusPort = &additionalBCMStatusPort;
        inData.trailerStatusPort = &trailerStatusPort;
        inData.loDMCStatusPort = &loDMCStatusPort;
        inData.slotCtrlPort = &slotCtrlPort;
        inData.pdcUserInteractionPort = &pdcUserInteractionPort;
        inData.egoMotionPort = &egoMotionPort;
        inData.perceptionAvailabilityPort = &perceptionAvailabilityPort;
        inData.pdcpSectorsPort = &pdcpSectorsPort;
        inData.procToLogicPort = &procToLogicPort;
        inData.whpProcOutputPort = &whpProcOutputPort;
        // Determine drvWarnSMSampleTimePort
        ap_commonvehsigprovider::DrvWarnSMSampleTimePort drvWarnSMSampleTimePortLocal{};
        drvWarnSMSampleTimePortLocal.drvWarnSMSampleTime_us = DRVWARNSM_SAMPLE_TIME_MS * 1000u;
        drvWarnSMSampleTimePortLocal.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        inData.drvWarnSMSampleTimePort = &drvWarnSMSampleTimePortLocal;

        //OutData
        mf_drvwarnsm::DrvWarnSM_Output outData{};
        outData.drvWarnStatusPort = &drvWarnStatusPort;
        outData.drvWarnDebugPort = &drvWarnDebugPort;
        outData.appToCoreSMPort = &appToCoreSMPort;
        outData.logicToProcPort = &logicToProcPort;

        //Run
        mf_drvwarnsm::DrvWarnSM_Interface::getInstance().run(inData, outData);
    }

    void stepPDWProc(
        const si::EgoMotionPort& egoMotionPort,
        const si::CollEnvModelPort& collEnvModelPort,
        const mf_drvwarnsm::LogicToProcPort& logicToProcPort,
        const CarMakerInterface& carMakerInterface,
        pdcp::PDCPSectorsPort& pdcpSectorsPort,
        pdcp::PDCPDrivingTubePort& pdcpDrivingTubePort,
        pdcp::ProcToLogicPort& pdwProcToLogicPort,
        pdcp::PDCPDebugPort& pdcpDebugPort)
    {
        //InData
        pdcp::PDCP_Input inData{};
        inData.egoMotionPort = &egoMotionPort;
        inData.envModelPort = &collEnvModelPort;
        inData.logicToProcPort = &logicToProcPort;

        //OutData
        pdcp::PDCP_Output outData{};
        outData.pdcpSectorsPort = &pdcpSectorsPort;
        outData.pdcpDrivingTubePort = &pdcpDrivingTubePort;
        outData.procToLogicPort = &pdwProcToLogicPort;
        outData.pdcpDebugPort = &pdcpDebugPort;

        //Run
        pdcp::PDCP_Interface::getInstance().run(inData, outData);

        //Simulate PDW failure via CarMaker testrun
        if (carMakerInterface.pdwFailure_nu) {
            pdwProcToLogicPort.processingError_nu = true;
        }
    }

    void stepToneHandler(
        const mf_drvwarnsm_core::DrvWarnCoreStatusPort& drvWarnCoreStatusPort,
        const mf_drvwarnsm::DrvWarnStatusPort& drvWarnStatusPort,
        const pdcp::PDCPSectorsPort& pdcpSectorsPort,
        const mf_whlprotectproc::WHPProcOutputPort& whpProcOutputPort,
        const mf_lsca::LscaHMIPort& lscaHMIPort,
        mf_tonh::ToneOutputPort& toneOutputPort,
        mf_tonh::ToneHandlerDebugPort& toneHandlerDebugPort,
        eco::CarMakerSystemServices& carMakerSystemServices)
    {
        //InData
        mf_tonh::MFToneHandler_Input inData{};
        inData.drvWarnCoreStatusPort = &drvWarnCoreStatusPort;
        inData.drvWarnStatusPort = &drvWarnStatusPort;
        inData.pdcpSectorsPort = &pdcpSectorsPort;
        inData.whpProcPort = &whpProcOutputPort;
        // Determine thSampleTimePort
        ap_commonvehsigprovider::THSampleTimePort thSampleTimePortLocal{};
        thSampleTimePortLocal.thSampleTime_us = TONH_SAMPLE_TIME_MS * 1000u;
        thSampleTimePortLocal.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        inData.thSampleTimePort = &thSampleTimePortLocal;
        inData.lscaHmiPort = &lscaHMIPort;

        //OutData
        mf_tonh::MFToneHandler_Output outData{};
        outData.toneOutputPort = &toneOutputPort;
        outData.toneHandlerDebugPort = &toneHandlerDebugPort;

        //Run
        mf_tonh::MF_ToneHandler_Interface::getInstance().run(inData, outData, &carMakerSystemServices);
    }

    void stepWhlProtecProc(
        const si::EgoMotionPort& egoMotionPort,
        const si::CollEnvModelPort& collEnvModelPort,
        const ap_vehstatesigprovider::DoorStatusPort &doorStatusPort,
        const mf_drvwarnsm::LogicToProcPort& logicToProcPort,
        mf_whlprotectproc::WHPProcOutputPort& whpProcOutputPort,
        mf_whlprotectproc::WHPProcDebugPort& whpProcDebugPort)
    {
        //InData
        mf_whlprotectproc::MFWhlProtectProc_Input inData{};
        inData.egoMotionPort = &egoMotionPort;
        inData.envModelPort = &collEnvModelPort;
        inData.logicToProcPort = &logicToProcPort;
        inData.doorStatusPort = &doorStatusPort;

        //OutData
        mf_whlprotectproc::MFWhlProtectProc_Output outData{};
        outData.whpProcOutputPort = &whpProcOutputPort;
        outData.whpProcDebugPort = &whpProcDebugPort;

        //Run
        mf_whlprotectproc::MFWhlProtectProc_Interface::getInstance().run(inData, outData);
    }

    void stepAVGA(
        const mf_drvwarnsm::DrvWarnStatusPort& drvWarnStatusPort,
        const mf_lsca::LscaStatusPort& lscaStatusPort,
        avga_swc::AVGA_SupervisionRequest& supervisionRequestLSCA,
        avga_swc::AVGA_SupervisionRequest& supervisionRequestPDW,
        avga_swc::AVGA_AutomatedVehicleGuidanceState& automatedVehicleGuidanceStateAUP,
        avga_swc::AvgaStopRequestPort& stopRequestMCRA,
        eco::CarMakerSystemServices& carMakerSystemServices)
    {
        //InData
        avga_swc::AvgaInput inData{};
        inData.drvWarnStatusPort = &drvWarnStatusPort;
        inData.lscaStatusPort = &lscaStatusPort;

        //OutData
        avga_swc::AvgaOutput outData{};
        outData.supervisionRequestLSCA = &supervisionRequestLSCA;
        outData.supervisionRequestPDW = &supervisionRequestPDW;
        outData.automatedVehicleGuidanceStateAUP = &automatedVehicleGuidanceStateAUP;
        outData.stopRequestMCRA = &stopRequestMCRA;

        //Run
        avga_swc::AVGA_Interface::getInstance().run(inData, outData, &carMakerSystemServices);
    }

    DllExport void step(
        uint64_t timeStamp_ms,
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
        const ap_commonvehsigprovider::SystemTimePort& SystemTimePort,
        const ap_commonvehsigprovider::WheelPulsePort& wheelPulsePort,
        const ap_commonvehsigprovider::WheelDrivingDirectionsPort& wheelDrivingDirectionsPort,
        const ap_commonvehsigprovider::WheelSpeedPort& wheelSpeedPort,
        const ap_commonvehsigprovider::VehDynamicsPort& vehDynamicsPort,
        const ap_commonvehsigprovider::SteerCtrlStatusPort& steerCtrlStatusPort,
        ap_commonvehsigprovider::TRJCTLGeneralInputPort& trjctrlGeneralInputPort,
        const ap_hmitoap::HMIOutputPort& hmiOutputPort,
        const ap_hmitoap::RemoteHMIOutputPort& remoteHMIOutputPort,
        const ap_hmitoap::VisuInputData& visuInputData,
        const si::ApParkingBoxPort& apParkingBoxPort,
        const lsm_vedodo::OdoEstimation& odoEstimationPortCM,
        const si::ApEnvModelPort& apEnvModelPort,
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
        lsm_vedodo::OdoNVMData& odoPersDataPort,
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
        ap_trjctl::MF_CONTROL_t_LongManeuverRequestPort& longManeuverRequestPort,
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
        mf_lsca::structs::plot_t& lscaPlotDataPort,
        ap_tp::TargetPosesPort &targetPosesPort,
        ap_tp::PlannedTrajPort &plannedTrajPort,
        ap_tp::TrajPlanDebugPort &trajPlanDebugPort,
        ap_tp::TrajPlanVisuPort &trajPlanVisuPort,
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
        mf_whlprotectproc::WHPProcOutputPort& whpOutputPort,
        mf_lvmd::LvmdStatusPort& lvmdStatusPort,
        avga_swc::AVGA_SupervisionRequest& avgaSupervisionRequestLSCA,
        avga_swc::AVGA_SupervisionRequest& avgaSupervisionRequestPDW,
        avga_swc::AVGA_AutomatedVehicleGuidanceState& automatedVehicleGuidanceStateAUP,
        avga_swc::AvgaStopRequestPort& avgaStopRequestMCRA,
        eco::CarMakerSystemServices& carMakerSystemServices
       )
    {
        //Copy collEnvModelPort for mf_plot (PDC and LSCA)
        gCollEnvModelPort = collEnvModelPort;

        // Tire Circumference Estimation
        if ((timeStamp_ms + 1u) % TCE_SAMPLE_TIME_MS == 0) {
            stepTCE(
                wheelPulsePort,
                wheelDrivingDirectionsPort,
                wheelSpeedPort,
                vehDynamicsPort,
                steerCtrlStatusPort,
                SystemTimePort,
                odoGpsPort,
                brakeCtrlStatusPort,
                engineCtrlStatusPort,
                tceEstimationPort,
                tcePersDataPort,
                tceDebugPort
            );
        }

        // Vehicle Dynamics Odometry
        if((timeStamp_ms + 1u) % VEDODO_SAMPLE_TIME_MS == 0){
            stepVEDODO(
                carMakerInterface,
                odoEstimationPortCM,
                wheelPulsePort,
                wheelDrivingDirectionsPort,
                wheelSpeedPort,
                tceEstimationPort,
                vehDynamicsPort,
                steerCtrlStatusPort,
                odoExtCtrlPort,
                SystemTimePort,
                gearboxCtrlStatusPort,
                suspensionPort,
                slotCtrlPort,
                odoEstimationOutputPort,
                odoPersDataPort,
                odoDebugPort,
                carMakerSystemServices
            );
        }

        // start set signal states manually as temporary workaround
        // gAPUserInformationPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // gOverrideLSCAPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // end set signal states manually as temporary workaround

        // Parking State Machine
        if (timeStamp_ms % PARKSM_SAMPLE_TIME_MS == 0) {
            // Parking State Machine
            stepPARKSM(
                carMakerInterface,
                loDMCCtrlRequestPort,
                gearBoxCtrlRequestPort,
                mfControlStatusPort,
                targetPosesPort,
                memoryParkingStatusPort,
                gearboxCtrlStatusPort,
                engineCtrlStatusPort,
                starterStatusPort,
                trunkLidStatusPort,
                convertibleTopStatusPort,
                steeringColSwitchesStatusPort,
                trailerStatusPort,
                doorStatusPort,
                vehicleOccupancyStatusPort,
                additionalBCMStatusPort,
                accInformationPort,
                escInformationPort,
                externalFunctionStatusPort,
                keylessStatusPort,
                authenticationStatusPort,
                laDMCStatusPort,
                loDMCStatusPort,
                drvWarnCoreStatusPort,
                apUserInteractionPort,
                lscaStatusPort,
                egoMotionPort,
                perceptionAvailabilityPort,
                slotCtrlPort,
                parksmCoreStatusPort,
                psmToSSPPort,
                ctrlCommandPort,
                gAPUserInformationPort,
                gOverrideLSCAPort,
                psmDebugPort,
                carMakerSystemServices
            );

            // Parking state machine (Core)
            stepPARKSMCore(
                carMakerInterface,
                targetPosesPort,
                mfControlStatusPort,
                egoMotionPort,
                apEnvModelPort,
                ctrlCommandPort,
                trajCtrlRequestPort,
                slotCtrlPort,
                parksmCoreStatusPort,
                parksmoreDebugPort
            );

            //Copy slotCtrlPort for mf_plot
            gSlotCtrlPort = slotCtrlPort;
        }

        // start set signal states manually as temporary workaround
        // gPdcUserInteractionPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // end set signal states manually as temporary workaround

        if (!carMakerInterface.pdwDisabled_nu) {
            // Driver Warning State Machine
            if (timeStamp_ms % DRVWARNSM_SAMPLE_TIME_MS == 0) {
                 stepDrvWarnSM(
                     gearboxCtrlStatusPort,
                     escInformationPort,
                     doorStatusPort,
                     trunkLidStatusPort,
                     additionalBCMStatusPort,
                     trailerStatusPort,
                     loDMCStatusPort,
                     slotCtrlPort,
                     pdcpSectorsPort,
                     pdwProcToLogicPort,
                     gPdcUserInteractionPort,
                     whpOutputPort,
                     egoMotionPort,
                     perceptionAvailabilityPort,
                     drvWarnStatusPort,
                     drvWarnDebugPort,
                     appToCoreSMPort,
                     logicToProcPort
                 );

                 stepDrvWarnSMCore(
                     egoMotionPort,
                     appToCoreSMPort,
                     drvWarnCoreStatusPort,
                     gDrvWarnCoreDebugPort
                 );
            }
            // Parking Distance Warning Processing
            if (timeStamp_ms % PDWPROC_SAMPLE_TIME_MS == 0) {
                stepPDWProc(
                    egoMotionPort,
                    collEnvModelPort,
                    logicToProcPort,
                    carMakerInterface,
                    pdcpSectorsPort,
                    pdcpDrivingTubePort,
                    pdwProcToLogicPort,
                    pdcpDebugPort
                );
            }
            // Wheel Protection Processing
            if (!carMakerInterface.whpDisabled_nu) {
                if (timeStamp_ms % WHP_SAMPLE_TIME_MS == 0) {
                    stepWhlProtecProc(
                        egoMotionPort,
                        collEnvModelPort,
                        doorStatusPort,
                        logicToProcPort,
                        whpOutputPort,
                        gWhpProcDebugPort
                    );
                }
            }
            // Tone Handler
            if (timeStamp_ms % TONH_SAMPLE_TIME_MS == 0) {
                stepToneHandler(
                    drvWarnCoreStatusPort,
                    drvWarnStatusPort,
                    pdcpSectorsPort,
                    whpOutputPort,
                    gLscaHmiPort,
                    toneOutputPort,
                    gToneHandlerDebugPort,
                    carMakerSystemServices
                );
            }

        }

#ifndef NO_TAPOSD_TRJPLA
        // Trajectory Planning (incl. Target Pose Definition)
        if (timeStamp_ms % TRJPLA_SAMPLE_TIME_MS == 0) {

            // TRJPLA visualization tool
#if AP_DEBUG_DRAW_ENABLE
            VISU_UPDATE_EM(envModelPort);
            VISU_UPDATE_EGO_PORT(envModelPort, egoMotionPort);
            VISU_UPDATE_TARGET_POSES(trjplaTPPort);
#endif
            //Save the previous data for replanning visualization
            gPreviousTargetPosesPort = targetPosesPort;
            gPreviousTrajPlanVisuPort = trajPlanVisuPort;
            gPreviousNumOfReplanCalls = trajPlanDebugPort.mNumOfReplanCalls;

            stepTRJPLA(
                apEnvModelPort,
                egoMotionPort,
                apParkingBoxPort,
                mfControlStatusPort,
                slotCtrlPort,
                targetPosesPort,
                plannedTrajPort,
                trajPlanVisuPort,
                trajPlanDebugPort,
                taposdDebugPort,
                reverseAssistAvailabilityPort
            );

            //Update global variables for mf_plot
            gTargetPosesPort = targetPosesPort;
            gTaposdDebugPort = taposdDebugPort;
            gTrajPlanVisuPort = trajPlanVisuPort;
        }

        // Plot data in mf_plot
#ifdef USE_ENV_PLOTTER
        if (timeStamp_ms % TRJPLA_SAMPLE_TIME_MS == 0) {

            // Update DynamicReplanningVisuData
            if ((trajPlanDebugPort.mNumOfReplanCalls == 0U) && (gPreviousNumOfReplanCalls > 0U))
            {
                // invalidate replanning data of previous simulation run
                gReplanningData.isValid = false;
                gReplanningData.replanPositions.clear();
            }
            else
            {
                if ((trajPlanDebugPort.mNumOfReplanCalls != gPreviousNumOfReplanCalls) && (trajPlanDebugPort.mNumOfReplanCalls != 0U))
                {
                    gReplanningData.mPPreviousTargetPose = gPreviousTargetPosesPort;
                    gReplanningData.mPPreviousTrjplaVisuPort = gPreviousTrajPlanVisuPort;
                    gReplanningData.replanPositions.append({ trajPlanVisuPort.plannedPathXPos_m[trajPlanVisuPort.currentPoseIdx_nu], trajPlanVisuPort.plannedPathYPos_m[trajPlanVisuPort.currentPoseIdx_nu] });
                    gReplanningData.isValid = true;
                }
            }
            //Copy trajectory output and transform trajectory for plotter in SI origin
            gPlannedTrajPort_AP_Transformed = plannedTrajPort;
            LSM_GEOML::CoordinateTransformer2D transformToOdometryCSRotation({ 0.0F, 0.0F, apEnvModelPort.transformationToOdometry.yaw_rad});
            LSM_GEOML::CoordinateTransformer2D transformToOdometryCSTranslation({ apEnvModelPort.transformationToOdometry.x_dir, apEnvModelPort.transformationToOdometry.y_dir, 0.0F });
            for (auto& trajPoint : gPlannedTrajPort_AP_Transformed.plannedTraj) {
                const LSM_GEOML::Pose transformedPose = transformToOdometryCSTranslation.inverseTransform(transformToOdometryCSRotation.inverseTransform({ trajPoint.xTrajRAReq_m, trajPoint.yTrajRAReq_m, trajPoint.yawReq_rad }));
                trajPoint.xTrajRAReq_m = transformedPose.Pos().x();
                trajPoint.yTrajRAReq_m = transformedPose.Pos().y();
                trajPoint.yawReq_rad = transformedPose.Yaw_rad();
            }

            gEnvModelPort = apEnvModelPort;
            gParkingBoxPort = apParkingBoxPort;
            gEgoMotionPort = egoMotionPort;
            gLscaPlotDataPort = lscaPlotDataPort;
            gTrailerStatusPort = trailerStatusPort;
            gDoorStatusPort = doorStatusPort;
            gVehicleOccupancyStatusPort = vehicleOccupancyStatusPort;
            gTrunkLidStatusPort = trunkLidStatusPort;
            gEngineCtrlStatusPort = engineCtrlStatusPort;
            gGearboxCtrlStatusPort = gearboxCtrlStatusPort;
            gCemObjectsForPlotter = cemObjectsForPlotter;
            gCemDynObjectsForPlotter = cemDynObjectsForPlotter;
            gCemODSlotsForPlotter = odSlotsForPlotter;
            gCusReflectionData = cusReflectionData;
            gCemLinesForPlotter = cemLinesForPlotter;
            gSiPlotData = siPlotData;
            gMemParkPlotData = memParkPlotData;
            gOptimalTargetPose = optimalTargetPose;
            gSvcPlotData = svcPlotData;
            // start glut loop

            if (!guiThread.joinable()) {
                try
                {
                    guiThread = std::thread([]() {
                        // Init visualization
                        AP_Utils::gVisu.plotter.init(1000.0f, 600.0f, 900.0f, 50.0f);
                        // Activate AP view
                        AP_Utils::gVisu.plotter.mButtonManager.mApButton.mActivated_nu = true;
                        // Display visualization
                        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
                        glutDisplayFunc(AP_Utils::plotEnv);
                        glutIdleFunc(AP_Utils::checkVisuData);
                        glutPassiveMotionFunc(AP_Utils::mousePosHandler);
                        glutMotionFunc(AP_Utils::mouseShiftHandler);
                        glutMouseFunc(AP_Utils::mouseClickHandler);
                        glutSpecialFunc(AP_Utils::keyboardHandler);
                        glutMainLoop();
                        //return;
                    });
                }
                catch (const std::exception &ex)
                {
                    std::cout << "Thread exited with exception: " << ex.what() << "\n";
                }
            }

        }
#endif //USE_ENV_PLOTTER
#endif //NO_TAPOSD_TRJPLA

        // LSCA
        if (carMakerInterface.lscaFakeStandstillRequest_nu)
        {
            slotCtrlPort.planningCtrlCommands.apState = ap_psm::APState::AP_AVG_PAUSE;
        }

        if (timeStamp_ms % LSCA_SAMPLE_TIME_MS == 0)
        {
            //update during track
            lsca_Params.general.proposalActive_nu       = !carMakerInterface.lscaSteeringProposalDisabled_nu;
            lsca_Params.general.resistanceActive_nu     = !carMakerInterface.lscaVirtualWallDisabled_nu;
            lsca_Params.general.LscaActive_nu       = !carMakerInterface.lscaDisabled_nu;
            lsca_Params.general.brakeStaticActive_nu    = !carMakerInterface.lscaBrakeDisabled_nu;
            lsca_Params.general.brakeDynamicActive_nu   = !carMakerInterface.lscaBrakeDisabled_nu;
            lscaPlotDataPort = stepLSCA(    collEnvModelPort,
                                            slotCtrlPort,
                                            gOverrideLSCAPort,
                                            egoMotionPort,
                                            perceptionAvailabilityPort,
                                            trailerStatusPort,
                                            doorStatusPort,
                                            vehicleOccupancyStatusPort,
                                            trunkLidStatusPort,
                                            engineCtrlStatusPort,
                                            gearboxCtrlStatusPort,
                                            loDMCStatusPort,
                                            mfControlStatusPort,
                                            lscaStatusPort,
                                            gLscaBrakePort,
                                            gLscaSteerPort,
                                            gLscaHmiPort);
        }

        // start set signal states manually as temporary workaround
        // gLscaBrakePort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // gLscaSteerPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // gLscaHmiPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // end set signal states manually as temporary workaround

        // Automated Vehicle Guidance Arbiter
        if (timeStamp_ms % AVGA_SAMPLE_TIME_MS == 0) {
            stepAVGA(drvWarnStatusPort,
                lscaStatusPort,
                avgaSupervisionRequestLSCA,
                avgaSupervisionRequestPDW,
                automatedVehicleGuidanceStateAUP,
                avgaStopRequestMCRA,
                carMakerSystemServices
            );
        }

#ifndef VARIANT_CUS_ONLY
        // MF_LVMD
        if (timeStamp_ms % LVMD_SAMPLE_TIME_MS == 0) {
            stepLVMD(egoMotionPort,
                collEnvModelPort,
                lvmdStatusPort,
                engineCtrlStatusPort,
                gearboxCtrlStatusPort
            );
        }
#endif

        // MF Manager
        if (timeStamp_ms % MF_MANAGER_SAMPLE_TIME_MS == 0)
        {
            stepMFManager(
                trajCtrlRequestPort,
                plannedTrajPort,
                lscaStatusPort,
                gLscaBrakePort,
                gLscaSteerPort,
                gActiveManeuveringFunctionPort,
                laCtrlRequestPort,
                loCtrlRequestPort,
                trajRequestPort
            );
        }

        // HACK sSigHeader as not set by MFManager correctly
        // start set signal states manually as temporary workaround
        // gActiveManeuveringFunctionPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // laCtrlRequestPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // loCtrlRequestPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // trajRequestPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // end set signal states manually as temporary workaround

        // MF Control
        if(timeStamp_ms % TRJCTL_SAMPLE_TIME_MS == 0){
            stepMFControl(
                gearboxCtrlStatusPort,
                trjctrlGeneralInputPort,
                laDMCStatusPort,
#ifdef MOCO_REPLACES_LODMC
                tratcoStatusPort,
                longVeconaStatusPort,
#else
                loDMCStatusPort,
#endif
                odoEstimationOutputPort,
                laCtrlRequestPort,
                loCtrlRequestPort,
                trajRequestPort,
                gearBoxCtrlRequestPort,
                laDMCCtrlRequestPort,
                loDMCCtrlRequestPort,
#ifdef MOCO_REPLACES_LODMC
                longManeuverRequestPort,
#endif
                mfControlStatusPort,
                drivingResistancePort,
                trajCtrlDebugPort,
                carMakerSystemServices
            );

            ////Validate trjctl signals. Car not steering if this is removed
            //loDMCCtrlRequestPort.signalState_nu = com::ComSignalState_t::COMSIGSTATE_VALID;
            loDMCCtrlRequestPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
            //laDMCCtrlRequestPort.signalState_nu = com::ComSignalState_t::COMSIGSTATE_VALID;
            laDMCCtrlRequestPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        }

        // start set signal states manually as temporary workaround
        // gScreenSwitchDataPort.sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
        // end set signal states manually as temporary workaround

        if (timeStamp_ms % HMIH_SAMPLE_TIMES_MS == 0) {
            stepMFHMIHandler(
                egoMotionPort,
                apEnvModelPort,
                apParkingBoxPort,
                hmiOutputPort,
                remoteHMIOutputPort,
                visuInputData,
                slotCtrlPort,
                gAPUserInformationPort,
                gLscaHmiPort,
                drvWarnStatusPort,
                drvWarnCoreStatusPort,
                pdcpSectorsPort,
                pdcpDrivingTubePort,
                whpOutputPort,
                parksmCoreStatusPort,
                loDMCCtrlRequestPort,
                targetPosesPort,
                trajPlanVisuPort,
                gScreenSwitchDataPort,
                odoEstimationOutputPort,
                carMakerInterface,
                hmiInputPort,
                headUnitVisualizationPort,
                remoteVisualizationPort,
                apUserInteractionPort,
                gPdcUserInteractionPort,
                gSurroundViewRequestPort,
                mfHmiHDebugPort,
                userDefinedSlotPort,
                gVisuInputPort,
                lvmdUserInteractionPort,
                carMakerSystemServices
            );
        }
    }

    DllExport void terminate(void)
    {
#ifdef USE_ENV_PLOTTER
        if (guiThread.joinable()) {
            glutLeaveMainLoop();
            guiThread.join();
        }
#endif
    }
 }