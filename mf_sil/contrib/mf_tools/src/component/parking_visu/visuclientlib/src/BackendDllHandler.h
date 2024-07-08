#ifndef BACKENDDLLHANDLER_H
#define BACKENDDLLHANDLER_H

#include <QLibrary>

#include <TRJPLA_Interface.h>
#include <mf_taposd/TAPOSD_Interface.h>

#include <mf_trjpla_types/FC_TRJPLA_Params.h>
#include <ap_common/vehicle_params.h>
#include <ap_common/sys_func_params.h>

#include <si/ap_env_model_port.h>
#include <si/ap_parking_box_port.h>
#include <si/ego_motion_port.h>
#include <ap_psm/ap_psm_generated_types.h>
#include <ap_trjctl/ap_trjctl_generated_types.h>

#include <MF_TRJPLA_TestDataHandler_wMetaData.h>

#include <TrajectoryPlanner.h>

#include <functional>

class VisuClientInterface;

class BackendDllHandler
{
public:
    BackendDllHandler();

    void loadLibrary(QString filename);

    struct TrjplaData {
        const si::ApEnvModelPort*     emData;
		const si::ApParkingBoxPort*   parkBoxPort;
        const si::EgoMotionPort*    egoPort;
		const ap_tp::TargetPosesPort*			targetPosesPort;
		const int*								selectedTargetPoseIndex;
        const std::array<float, 2U>*            selectedTpDeviations;
        const ap_tp::TrjplaMetaData*            trjplaMetaData;
    };

    struct TaposdData {
        const si::ApEnvModelPort*     emData;
        const si::ApParkingBoxPort*   parkBoxPort;
        const si::EgoMotionPort*    egoPort;
    };


    /**
     * @brief Intiate scan mode for TRJPLA
     *
     * Calls init() and multiple run() of Trajectory Planning internally.
     *
     * @param Trajpla config
     * @param Planner Data
     * @params scanModeOn
     * @return PoseReachableStatus::TP_FULLY_REACHABLE on success, otherwise PoseReachableStatus::TP_NOT_REACHABLE
     */
    bool initToScanTransitionTrjpla(const ap_tp::TRJPLA_Config &cfg, const TrjplaData &data, bool scanModeOn = false);

    bool scanToPlanTransitionTrjpla(const ap_tp::TRJPLA_Config &cfg, const TrjplaData &data);

    /**
     * @brief Intiate scan mode for parking in TRJPLA
     *
     * Calls init() and multiple run() TRJPLA-internally.
     *
     * @param Trajpla config
     * @param Planner Data
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool startScanModeTrjpla(ap_tp::TRJPLA_Config const& cfg, const TrjplaData& data, QString *errorMsg = nullptr);

    /**
     * @brief Try to plan paths from current pose using the current parking scene model
     * simulating scanning mode
     *
     * Calls run() internally.
     *
     * @param Trajpla config
     * @param Planner Data
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool runForScanTrjpla(ap_tp::TRJPLA_Config const& cfg, const TrjplaData& data, QString *errorMsg = nullptr);


    /**
     * @brief Intiate and runs scan mode for parking in TAPOSD
     *
     * Calls several interface functions of TAPOSD simulating the scanning phase
     *
     * @param Taposd config
     * @param TaPoser Data
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool startAndRunScanModeTaposd(const TaposdData data, QString *errorMsg = nullptr);

    /**
     * @brief Plan a new path from scratch using the current parking scene model
     *
     * Calls init() and multiple run() internally.
     *
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool planNewPath(ap_tp::TRJPLA_Config const& cfg, const TrjplaData& data, QString *errorMsg = nullptr, bool scanModeOn = false);

    /**
     * @brief Trigger replanning based on current parking scene model
     *
     * Assumes AP_FOLLOW_PATH state. Replanning is triggered by setting strokeFailed
     * and strokeCompleted to true in the mfControlStatusPort.
     *
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool replanExistingPath(const TrjplaData& data, QString *errorMsg = nullptr);

    /**
     * @brief Move ego pose by one step on the path and run the tajectory planner
     * state machine with current parking scene model
     *
     * Assumes loaded EM and  AP_FOLLOW_PATH state.
     *
     * @param errorMsg error message, untouched if successful
     * @return true on success, otherwise false
     */
    bool makeStepOnPath(const TrjplaData& data, QString *errorMsg = nullptr);

    /**
     * @brief Move ego pose along the path and run the tajectory planner
     * state machine with current parking scene model until ReqDistToStop
     * is reached
     *
     * Assumes loaded EM and  AP_FOLLOW_PATH state.
     *
     * @param:
     * @param errorMsg error message, untouched if successful
     * @return true on success, otherwise false
     */
    bool jumpOnPath(const TrjplaData& data, const double& reqDistToStop, QString *errorMsg = nullptr);

    const ap_tp::TrajPlanDebugPort& getDebugPortTrjpla() const {
        return trjplaDebugPort;
    }

    const ap_tp::TAPOSDDebugPort& getDebugPortTaposd() const {
        return taposdDebugPort;
    }

    const ap_tp::TargetPosesPort& getTargetPosesPort() const {
        return targetPosesPort;
    }

    const ap_tp::PlannedTrajPort& getPlannedTrajectory() const {
        return plannedTrajectory;
    }

    const ap_tp::TrajPlanVisuPort& getTrajplaVisuPort() const {
        return trjplaVisuPort;
    }

    /**
     * @brief return last ego pose of vehicle to send it to server
     * @return
     */
    const LSM_GEOML::Pose getLastEgoPose() const {
        return emData.egoVehiclePoseForAP;
    }

    float32_t getLastEgoCurvature() const {
        return egoPort.frontWheelAngle_rad;
    }

    void setVisuInterface(VisuClientInterface* iface);

    bool isLibraryLoaded() const {
        return mLibrary.isLoaded();
    }

    bool planGeometricPath(ap_tp::TRJPLA_Config const& cfg, const TrjplaData& data,
        ap_tp::GeomPathRepresentation* path, ap_tp::ParkingPath& trajectory, bool isReplanning, QString *errorMsg = nullptr,bool* isInCollAtStart= nullptr);

    bool planGaragePath(const ap_tp::TRJPLA_Config &cfg, const BackendDllHandler::TrjplaData &data,
        ap_tp::GarageParkingPath *path, ap_tp::ParkingPath* trajectory, bool isReplanning, QString *errorMsg = nullptr, bool* isInCollAtStart = nullptr);

private:
    QLibrary mLibrary;

    typedef void (__cdecl *Planner_Init_Func)(const ap_tp::TRJPLA_Config& cfg);
    typedef com::ComResult (__cdecl *Planner_Run_Func)(const ap_tp::TRJPLA_Input& inDataTrjpla, ap_tp::TRJPLA_Output& outDataTrjpla);
    typedef void (__cdecl *Poser_Init_Func)();
    typedef void (__cdecl *Poser_Run_Func)(const AP_TPD::TAPOSD_Input& inDataTaposd, AP_TPD::TAPOSD_Output<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU>& outDataTaposd);
    typedef void (__cdecl* Set_Visu_Interface_Fun)(VisuClientInterface*);

    typedef void (__cdecl* Geometric_Plan_Init)(ap_tp::TRJPLA_Config const&);
    typedef void (__cdecl* Geometric_Plan_Exec)(const si::ApEnvModelPort&,
                                                const si::ApParkingBoxPort&,
                                                const si::EgoMotionPort&,
                                                const ap_tp::TargetPose&,
                                                const std::array<float, 2U>&,
                                                const ap_tp::TrjplaMetaData&,
                                                ap_tp::GeomPathRepresentation*,
                                                ap_tp::ParkingPath&,
                                                bool,
                                                bool*);

    typedef void(__cdecl* Garage_Plan_Init)(ap_tp::TRJPLA_Config const&);
    typedef void(__cdecl* Garage_Plan_Exec)(const si::ApEnvModelPort&,
                                            const si::ApParkingBoxPort&,
                                            const si::EgoMotionPort&,
                                            const ap_tp::TargetPose&,
                                            ap_tp::GarageParkingPath*,
                                            ap_tp::ParkingPath*,
                                            bool,
                                            bool*);

    typedef  ap_tp::Maneuver* (__cdecl* Geometric_Plan_GetManeuver)();

    Planner_Init_Func initFuncTrjpla = nullptr;
    Planner_Run_Func runFuncTrjpla = nullptr;
    Poser_Init_Func initFuncTaposd = nullptr;
    Poser_Run_Func runFuncTaposd = nullptr;
    Set_Visu_Interface_Fun setVisuIfaceFunc = nullptr;

    Geometric_Plan_Init geomInitFunc = nullptr;
    Geometric_Plan_Exec geomPlanFunc = nullptr;
    Garage_Plan_Init garageInitFunc = nullptr;
    Garage_Plan_Exec garagePlanFunc = nullptr;
    Geometric_Plan_GetManeuver geomPlanGetManeuver = nullptr;

    // TRJPLA Input
    si::ApEnvModelPort     emData;
    si::ApParkingBoxPort   parkBoxPort;
    si::EgoMotionPort    egoPort;
    ap_psm::SlotCtrlPort         stateMachineIn;
    ap_trjctl::MFControlStatusPort   mfControlStatusPort;
    ap_tp::TRJPLA_Input inDataTrjpla {&emData, &parkBoxPort, &egoPort, &stateMachineIn, &mfControlStatusPort };
	int selectedTargetPoseIndex;

    // PathPlanner Interface input
    std::array<float, 2U> selectedTpDeviations;
    ap_tp::TrjplaMetaData trjplaMetaData;

    // TRJPLA Output
    ap_tp::PlannedTrajPort             plannedTrajectory;
    ap_tp::TargetPosesPort			   targetPosesPort;
    ap_tp::TrajPlanDebugPort           trjplaDebugPort;
    ap_tp::TrajPlanVisuPort            trjplaVisuPort;
    ap_tp::TAPOSDDebugPort             taposdDebugPort;
    ap_tp::TRJPLA_Output outDataTrjpla {&plannedTrajectory, &targetPosesPort, &trjplaDebugPort, &trjplaVisuPort, &taposdDebugPort};

    // TAPOSD Input
    AP_TPD::TAPOSD_Input inDataTaposd{ {{false, 0U, 0U}, stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu}, &emData, &parkBoxPort, &egoPort};
    AP_TPD::TAPOSD_Output<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU> outDataTaposd{ { {}, &targetPosesPort} , &taposdDebugPort };

    /**
     * @brief Check if everything is loaded for path planning (i.e. parameters, EM, plugin DLL)
     * @param errorMsg
     * @return true if loaded, otherwise false
     */
    bool checkPrerequisitesLoaded(QString *errorMsg);

    /**
     * @brief print msg, and also set *errorMsg = msg if errorMsg not null
     * @param errorMsg
     * @return always false
     */
    static bool failWithMessage(QString *errorMsg, QString msg);

    /**
     * @brief get the next trajectory control point
     *
     * Assumes plannedTrajectory port is valid
     * @return
     */
    ap_tp::PlannedTraj getNextControlPoint() const;
};

#endif // BACKENDDLLHANDLER_H
