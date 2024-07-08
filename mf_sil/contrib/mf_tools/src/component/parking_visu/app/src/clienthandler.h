#ifndef VISUALIZATIONSERVER_H
#define VISUALIZATIONSERVER_H

#include <mf_trjpla/TRJPLA_Interface.h>
#include <mf_taposd/TAPOSD_Interface.h>

#ifndef ULTRASONIC_ONLY
    #include <mf_trjpla_types/FC_TRJPLA_Params.h>
    #include <ap_common/vehicle_params.h>
    #include <ap_common/sys_func_params.h>
#else
    #include <mf_trjpla/FC_TRJPLA_Params_default_set.h>
    #include <mf_common/Sys_Func_Params_default_set.h>
    #include <mf_common/Vehicle_Params_default_set.h>
#endif

#include <ap_tp/ap_tp_generated_types.h>
#include <ap_trjctl/ap_trjctl_generated_types.h>

#include <QObject>
#include <QLocalServer>
#include <QLocalSocket>
#include <QProcess>
#include <QSharedMemory>

#include <memory>
#include <functional>

#include <TrajectoryPlanner.h>

#include "parkingvisuclient.h"
#include "models/enumproperty.h"
#include "models/parkingscenemodel.h"
#include "ui/debugdrawer.h"

class ClientHandler : public QObject
{
    Q_OBJECT

    Q_PROPERTY(EnumProperty trajPlanState READ getTrajPlanStateProp NOTIFY trajPlanStateChanged)
    Q_PROPERTY(bool computationInProgress READ isComputationInProgress NOTIFY computationInProgressChanged)

public:
    explicit ClientHandler(ParkingSceneModel* model, QObject *parent = nullptr);
    ~ClientHandler();

    bool startServer();

    void startClientProcessIfNotRunning();

    void loadComponentsDLL(QString filename);

    void loadTRJPLAParameters(QString filename);
    void loadVehicleParameters(QString filename);
    void loadSysFuncParameters(QString filename);
    void loadEM(QString filename);
    void loadREACH(QString filename);

    /**
     * @brief Check for
     *
     * Calls init() and multiple run() internally.
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool findReachableArea(QString *errorMsg, bool is3D);

    /**
     * @brief Intiate scan mode for parking (TRJPLA)
     *
     * Calls init() and multiple run() internally.
     *
     * @param enable scan mode
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool startScanModeTrjpla(bool scanModeOn, QString *errorMsg = nullptr);

    /**
     * @brief Try to plan paths from current pose using the current parking scene model
     * simulating scanning mode
     *
     * Calls run() internally.
     *
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool runForScanTrjpla(QString *errorMsg = nullptr);

    /**
     * @brief Intiate scan mode for parking (TAPOSD)
     *
     * Simulates a scanning phase internally.
     *
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool startAndRunScanModeTaposd(QString *errorMsg = nullptr);

    /**
     * @brief Plan a new path from scratch using the current parking scene model
     *
     * Simulates a scanning and planning phase internally.
     *
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool planNewPath(QString *errorMsg = nullptr);

    /**
     * @brief Trigger replanning based on current parking scene model
     *
     * Assumes AP_FOLLOW_PATH state. Replanning is triggered by setting trajCtrlStatus->strokeFailed_nu
     * and trajCtrlStatus->strokeCompleted_nu to true.
     *
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool replanExistingPath(QString *errorMsg = nullptr);

    /**
     * @brief Move ego pose by one step on the path and run the tajectory planner
     * state machine with current parking scene model
     *
     * Assumes loaded EM and AP_FOLLOW_PATH state.
     *
     * @param errorMsg error message, untouched if successful
     * @return true on success, otherwise false
     */
    bool makeStepOnPath(QString *errorMsg = nullptr);

    /**
     * @brief Move ego pose until distance to Stop on the path and run the tajectory planner
     * state machine with current parking scene model
     *
     * Assumes loaded EM and AP_FOLLOW_PATH state.
     *
     * @param the required distance to stop
     * @param errorMsg error message, untouched if successful
     * @return true on success, otherwise false
     */
    bool jumpOnPath(double reqDistToStop, QString *errorMsg = nullptr);

    /**
     * @brief Plan a new geometric path from scratch using the current parking scene model
     *
     * In opposite to planNewPath(), no scanning is performed and the GeometricPlanner is
     * called directly.
     *
     * @param errorMsg error message, untouched if planning successful
     * @return true on success, otherwise false
     */
    bool planNewGeomPath(QString *errorMsg = nullptr);

    /**
    * @brief Plan a new path for garage parking from scratch using the current parking scene model
    *
    * @param errorMsg error message, untouched if planning successful
    * @return true on success, otherwise false
    */
    bool planNewGaragePark(QString *errorMsg = nullptr);

    /**
     * @brief Get current state of trajectory planner
     * @return
     */
    ap_tp::TrajPlanState getTrajPlanState() const;

    EnumProperty getTrajPlanStateProp() const {
        return getTrajPlanState();
    }

    /**
     * @brief Set the item for debug draws
     * @param item DebugDrawItem
     */
    void setDebugDrawer(DebugDrawer* item);

    inline bool isComputationInProgress() const {
        return mComputationInProgress;
    }

    inline bool isTargetPoseReachable(uint8_t selectedPoseIdx_nu) const {
        return mTargetPoseReachable[selectedPoseIdx_nu];
    }

    /**
     * @return true iff DLL is loaded
     */
    bool isLibraryLoaded() const;

    const ap_tp::FC_TRJPLA_Params*   getTrjplaParams() const {return trjplaParams.get(); }
    const ap_common::Vehicle_Params*   getVehicleParams() const {return vehicleParams.get(); }
    const ap_common::Sys_Func_Params*   getSysFuncParams() const {return sysFuncParams.get(); }

    static Trajectory trajectoryFromParkingPath(const ap_tp::ParkingPath& parkingPath);
    static Trajectory trajectoryFromSimpleParkingPath(const SimpleParkPath &parkingPath);

signals:
    void trajPlanStateChanged(ap_tp::TrajPlanState state);

    void planningFailed(QString msg);

    void computationInProgressChanged(bool inProgress);

    void clientConnected();

    void distanceToStopChanged(float distance);

public slots:
    void enableDebugDraw(bool enabled);

private slots:
    void onNewConnection();

    void onClientDisconnected();

    void onClientBytesReady();

    void onClientProcessOutputReady();

private:
    ParkingSceneModel* mParkingModel = nullptr;

    QLocalServer mServerSocket;

    QLocalSocket* mConnectedSocket = nullptr;

    QProcess* mClientProcess = nullptr;

    mutable QSharedMemory mServerMemory;

    std::shared_ptr<ap_tp::FC_TRJPLA_Params>    trjplaParams;
    std::shared_ptr<ap_common::Vehicle_Params>  vehicleParams;
    std::shared_ptr<ap_common::Sys_Func_Params> sysFuncParams;

    std::shared_ptr<StreamCommunicator> mComm;

    ap_tp::TrajPlanVisuPort           trjplaVisuPort;
    ap_tp::TrajPlanDebugPort          trjplaDebugPort;
    ap_tp::TAPOSDDebugPort			  taposdDebugPort;
    ap_tp::PlannedTrajPort            plannedTrajectory;

    DebugDrawer* mDebugDrawer = nullptr; /**< responsible for drawing received SERVER_DRAW_* commands */

    bool mComputationInProgress = false; /** Indicates whether a path computation is in progress right now */

    bool mTargetPoseReachable[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU]; /** Indicates whether target pose is reachable or not during scan mode */

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

    void handleClientCommand(ParkingSocketCommand cmd, QDataStream& in);

    /**
     * @brief set current vehicle path from debug output and EM egomotion
     */
    void setPathFromDebugOut();

    void setPathFromTrajPort();

    void setComputationInProgress(bool inProgress);

    void setTargetPoseReachableAreaReady2D(bool areaReady);
    void setTargetPoseReachableAreaReady3D();
};

#endif // VISUALIZATIONSERVER_H
