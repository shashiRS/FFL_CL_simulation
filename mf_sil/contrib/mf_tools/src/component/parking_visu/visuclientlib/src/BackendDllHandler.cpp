#include "BackendDllHandler.h"

#include <QDebug>

BackendDllHandler::BackendDllHandler()
{
    memset(&egoPort, 0, sizeof(egoPort));
}

void BackendDllHandler::loadLibrary(QString filename)
{
    if(mLibrary.isLoaded()) {
        mLibrary.unload();
    }

    mLibrary.setFileName(filename);
    if(!mLibrary.load()) {
        qDebug() << "BackendDllHandler: cannot open library" << filename;
        return;
    }

    initFuncTrjpla = (Planner_Init_Func) mLibrary.resolve("Planner_Init");
    if(!initFuncTrjpla) {
        qDebug() << "BackendDllHandler: cannot load 'Planner_Init'";
        mLibrary.unload();
        return;
    }

    runFuncTrjpla = (Planner_Run_Func) mLibrary.resolve("Planner_Run");
    if(!runFuncTrjpla) {
        qDebug() << "BackendDllHandler: cannot load 'Planner_Run'";
        mLibrary.unload();
        return;
    }

	initFuncTaposd = (Poser_Init_Func)mLibrary.resolve("Poser_Init");
	if (!initFuncTrjpla) {
		qDebug() << "BackendDllHandler: cannot load 'Poser_Init'";
		mLibrary.unload();
		return;
	}

	runFuncTaposd = (Poser_Run_Func)mLibrary.resolve("Poser_Run");
	if (!runFuncTrjpla) {
		qDebug() << "BackendDllHandler: cannot load 'Poser_Run'";
		mLibrary.unload();
		return;
	}

    setVisuIfaceFunc = (Set_Visu_Interface_Fun) mLibrary.resolve("Set_Visu_Interface_Fun");
    if(!setVisuIfaceFunc) {
        qDebug() << "BackendDllHandler: cannot load 'Set_Visu_Interface_Fun'";
    }

    geomInitFunc = (Geometric_Plan_Init) mLibrary.resolve("Geometric_Plan_Init");
    if(!geomInitFunc) {
        qDebug() << "BackendDllHandler: cannot load 'Geometric_Plan_Init'";
    }

    geomPlanFunc = (Geometric_Plan_Exec) mLibrary.resolve("Geometric_Plan_Exec");
    if(!geomInitFunc) {
        qDebug() << "BackendDllHandler: cannot load 'Geometric_Plan_Exec'";
    }

    garageInitFunc = (Garage_Plan_Init) mLibrary.resolve("Garage_Plan_Init");
    if (!garageInitFunc) {
        qDebug() << "BackendDllHandler: cannot load 'Garage_Plan_Init'";
    }

    garagePlanFunc = (Garage_Plan_Exec) mLibrary.resolve("Garage_Plan_Exec");
    if (!garageInitFunc) {
        qDebug() << "BackendDllHandler: cannot load 'Garage_Plan_Exec'";
    }

    geomPlanGetManeuver = (Geometric_Plan_GetManeuver) mLibrary.resolve("Geometric_Plan_GetManeuver");
    if(!geomInitFunc) {
        qDebug() << "BackendDllHandler: cannot load 'Geometric_Plan_GetManeuver'";
    }
}

bool BackendDllHandler::initToScanTransitionTrjpla(const ap_tp::TRJPLA_Config &cfg, const TrjplaData &data, bool scanModeOn){

    initFuncTrjpla(cfg);

    emData = *data.emData;
	parkBoxPort = *data.parkBoxPort;
    egoPort = *data.egoPort;

    /******* init -> idle **********/

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
    stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_PAUSE;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = 255;

    runFuncTrjpla(inDataTrjpla, outDataTrjpla);

    /******* idle -> scanning **********/

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
    stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_SCAN_IN;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = 255;

    bool reachable = false;

    for(int i = 0; i < 15 && !reachable; i++) { // TODO remove hardcoded nr. of attempts
        runFuncTrjpla(inDataTrjpla, outDataTrjpla);
        ap_tp::PoseReachableStatus reachStat = outDataTrjpla.targetPoses->targetPoses[*data.selectedTargetPoseIndex].reachableStatus;
        reachable = (reachStat != ap_tp::PoseReachableStatus::TP_NOT_VALID)
            && (reachStat != ap_tp::PoseReachableStatus::TP_NOT_REACHABLE)
            && (reachStat != ap_tp::PoseReachableStatus::MAX_NUM_POSE_REACHABLE_STATUS);
        if(scanModeOn){
            break;
        }
        // qDebug() << "pose is reachable" << reachable;
    }

    // TODO emit trajPlanStateChanged(getTrajPlanState());

    return reachable;
}

bool BackendDllHandler::scanToPlanTransitionTrjpla(const ap_tp::TRJPLA_Config &cfg, const TrjplaData &data){

    /******* scanning in -> replan path in **********/

	targetPosesPort = *data.targetPosesPort;

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
    stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_ACTIVE_IN;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = *data.selectedTargetPoseIndex;

    runFuncTrjpla(inDataTrjpla, outDataTrjpla);

    /******* replan path -> follow path **********/

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
    stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_ACTIVE_IN;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = *data.selectedTargetPoseIndex;

    memset((void*)&mfControlStatusPort, 0, sizeof(mfControlStatusPort));

    runFuncTrjpla(inDataTrjpla, outDataTrjpla);
    runFuncTrjpla(inDataTrjpla, outDataTrjpla); // distance to stop is broken on replan->follow transition, so execute one more step
	bool reachable = true;

    if(!(outDataTrjpla.plannedTrajectory->trajValid_nu)){
        reachable = false;
    }

    return reachable;

    // TODO setPathFromDebugOut();
}

bool BackendDllHandler::startScanModeTrjpla(const ap_tp::TRJPLA_Config &cfg, const TrjplaData &data, QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

	// if no valid parking boxes present, TAPOSD cannot calculate target poses
	if (data.parkBoxPort->numValidParkingBoxes_nu == 0) {
		qDebug() << "No valid parking boxes";

		if (errorMsg) {
			*errorMsg = "No valid parking boxes";
		}
		return false;
	}

    return initToScanTransitionTrjpla(cfg,  data, true);
}

bool BackendDllHandler::runForScanTrjpla(const ap_tp::TRJPLA_Config &cfg, const TrjplaData &data, QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    emData = *data.emData;
	parkBoxPort = *data.parkBoxPort;
    egoPort = *data.egoPort;

    runFuncTrjpla(inDataTrjpla, outDataTrjpla);

    ap_tp::PoseReachableStatus reachStat = outDataTrjpla.targetPoses->targetPoses[*data.selectedTargetPoseIndex].reachableStatus;
    bool reachable = reachStat != ap_tp::PoseReachableStatus::TP_NOT_VALID && reachStat != ap_tp::PoseReachableStatus::TP_NOT_REACHABLE && reachStat != ap_tp::PoseReachableStatus::MAX_NUM_POSE_REACHABLE_STATUS;
    return reachable;
}

bool BackendDllHandler::startAndRunScanModeTaposd(const TaposdData data, QString *errorMsg)
{

    if(!initFuncTaposd) {
        return failWithMessage(errorMsg, "initFuncTrjpla not resolved");
    }

    if(!runFuncTaposd) {
        return failWithMessage(errorMsg, "runFuncTrjpla not resolved");
    }

    initFuncTaposd();

    emData = *data.emData;
    parkBoxPort = *data.parkBoxPort;
    egoPort = *data.egoPort;
    inDataTaposd.inputMetaData.apChosenTargetPose_nu = std::numeric_limits<uint8_t>::max();

    runFuncTaposd(inDataTaposd, outDataTaposd);

    // check whether a pose was found
	bool foundPose = outDataTaposd.targetPoses.targetPosesPort->numValidPoses != 0;

    return foundPose;
}

bool BackendDllHandler::planNewPath(const ap_tp::TRJPLA_Config &cfg, const TrjplaData &data, QString *errorMsg, bool scanModeOn){
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

	// if no valid parking boxes present, TAPOSD cannot calculate target poses
	if (data.parkBoxPort->numValidParkingBoxes_nu == 0) {
		qDebug() << "No valid parking boxes";

		if (errorMsg) {
			*errorMsg = "No valid parking boxes";
		}
		return false;
	}

    bool reachable = true;

    if(!scanModeOn){
        reachable = initToScanTransitionTrjpla(cfg,  data);
    }


    if(reachable){
        reachable = scanToPlanTransitionTrjpla(cfg, data);
    }

    if(!reachable) {
        qDebug() << "target pose is not reachable";

        if(errorMsg) {
            *errorMsg = "target pose is not reachable";
        }

        // TODO mParkingModel->setVehiclePath();

        // leave path empty
    }

    return reachable;

}

bool BackendDllHandler::replanExistingPath(const TrjplaData &data, QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    if(trjplaDebugPort.mTrajPlanState != (uint8_t)ap_tp::TrajPlanState::AP_FOLLOW_PATH) {
        return failWithMessage(errorMsg, "Cannot replan since not in follow path state");
    }

    /******* follow path -> replan path **********/

    emData = *data.emData;
	parkBoxPort = *data.parkBoxPort;
    egoPort = *data.egoPort;

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
	stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_ACTIVE_IN;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = *data.selectedTargetPoseIndex;

    memset((void*)&mfControlStatusPort, 0, sizeof(mfControlStatusPort));
    // set both stroke completed and failed to true to trigger replanning
    mfControlStatusPort.longitudinalControlFinished_nu = true;
    mfControlStatusPort.lateralControlFailed_nu = true;
    mfControlStatusPort.longitudinalControlFailed_nu = true;

    runFuncTrjpla(inDataTrjpla, outDataTrjpla);

    if(outDataTrjpla.trjplaDebugPort->mReplanSuccessful_nu == false) {
        return failWithMessage(errorMsg, "Replanning failed");
    }

    /******* replan path -> follow path **********/

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
	stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_ACTIVE_IN;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = *data.selectedTargetPoseIndex;

    memset((void*)&mfControlStatusPort, 0, sizeof(mfControlStatusPort));

    runFuncTrjpla(inDataTrjpla, outDataTrjpla);

    qDebug() << "replanning successful";

    return true;
}

bool BackendDllHandler::makeStepOnPath(const TrjplaData& data, QString *errorMsg)
{
    static constexpr float AP_C_MANEUV_FINISHED_LIMIT_M = 0.05f;

    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    auto lastState = (ap_tp::TrajPlanState)trjplaDebugPort.mTrajPlanState;

    if(lastState != ap_tp::TrajPlanState::AP_FOLLOW_PATH
		&&  trjplaDebugPort.mTrajPlanState != (uint8_t)ap_tp::TrajPlanState::AP_FOLLOW_PATH) {
        return failWithMessage(errorMsg, "Cannot make step since not in follow path state");
    } else if(plannedTrajectory.numValidCtrlPoints_nu <= 0) {
        return failWithMessage(errorMsg, "Cannot make step since there are no more control points available");
    }

    const float distToStop = plannedTrajectory.plannedTraj[
            plannedTrajectory.numValidCtrlPoints_nu >= ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS?
                ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS :  plannedTrajectory.numValidCtrlPoints_nu].distanceToStopReq_m;

    auto const& nextPos = getNextControlPoint();

    emData = *data.emData;
	parkBoxPort = *data.parkBoxPort;
    egoPort = *data.egoPort;
    emData.egoVehiclePoseForAP.x_dir = nextPos.xTrajRAReq_m;
    emData.egoVehiclePoseForAP.y_dir = nextPos.yTrajRAReq_m;
    emData.egoVehiclePoseForAP.yaw_rad = nextPos.yawReq_rad;
    egoPort.frontWheelAngle_rad = nextPos.crvRAReq_1pm;

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
    stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_ACTIVE_IN;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = *data.selectedTargetPoseIndex;

    memset((void*)&mfControlStatusPort, 0, sizeof(mfControlStatusPort));

    // start new stroke if required
    if(distToStop < AP_C_MANEUV_FINISHED_LIMIT_M) {
        qDebug() << "stroke completed";

        mfControlStatusPort.lateralControlFinished_nu = true;
        mfControlStatusPort.longitudinalControlFinished_nu = true;

        runFuncTrjpla(inDataTrjpla, outDataTrjpla);
        mfControlStatusPort.lateralControlFinished_nu = false;
        mfControlStatusPort.longitudinalControlFinished_nu = false;
		if (outDataTrjpla.targetPoses->selectedPoseData.reachedStatus == ap_tp::PoseReachedStatus::TP_REACHED) {
			stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_FINISHED;
		}
    }

    runFuncTrjpla(inDataTrjpla, outDataTrjpla);

    //if dynamic replanning required then do one more component call to go to follow path again
    if (trjplaDebugPort.mTrajPlanState == (uint8_t)ap_tp::TrajPlanState::AP_DYNAMIC_REPLAN_PATH) {
        runFuncTrjpla(inDataTrjpla, outDataTrjpla);
    }

    if(lastState == ap_tp::TrajPlanState::AP_FOLLOW_PATH
            &&  trjplaDebugPort.mTrajPlanState == (uint8_t)ap_tp::TrajPlanState::AP_FOLLOW_PATH) {
        qDebug() << "Path in collision";
        if(false /* TODO followCollisionFunc()*/ ) {
            qDebug() << "Follow until collision";

            while(true) {
                int nextIndex = plannedTrajectory.numValidCtrlPoints_nu >= ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS+1?
                    ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS+1 : plannedTrajectory.numValidCtrlPoints_nu-1;
                auto const& nextPos = plannedTrajectory.plannedTraj[nextIndex];
                if(nextPos.distanceToStopReq_m < AP_C_MANEUV_FINISHED_LIMIT_M) {
                    break;
                }

                emData.egoVehiclePoseForAP.x_dir = nextPos.xTrajRAReq_m;
                emData.egoVehiclePoseForAP.y_dir = nextPos.yTrajRAReq_m;
                emData.egoVehiclePoseForAP.yaw_rad = nextPos.yawReq_rad;
                egoPort.frontWheelAngle_rad = nextPos.crvRAReq_1pm;
                runFuncTrjpla(inDataTrjpla, outDataTrjpla);
            }

            return replanExistingPath(data, errorMsg);
        } else {
            qDebug() << "Continue manual stepping";
        }
    }
    return true;
}

bool BackendDllHandler::jumpOnPath(const TrjplaData& data, const double& reqDistToStop, QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    auto lastState = (ap_tp::TrajPlanState)trjplaDebugPort.mTrajPlanState;

    if(lastState != ap_tp::TrajPlanState::AP_FOLLOW_PATH
            && lastState != ap_tp::TrajPlanState::AP_FOLLOW_PATH) {
        return failWithMessage(errorMsg, "Cannot make step since not in follow path state");
    } else if(plannedTrajectory.numValidCtrlPoints_nu <= 0) {
        return failWithMessage(errorMsg, "Cannot make step since there are no more control points available");
    }

    float distToStop = plannedTrajectory.plannedTraj[
            plannedTrajectory.numValidCtrlPoints_nu >= ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS?
                ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS :  plannedTrajectory.numValidCtrlPoints_nu].distanceToStopReq_m;

    if(reqDistToStop < 0 || reqDistToStop > distToStop)
        return failWithMessage(errorMsg, "Desired distance to stop is not within current stroke");

    memset((void*)&stateMachineIn, 0, sizeof(stateMachineIn));
    stateMachineIn.planningCtrlPort.apState = ap_psm::APState::AP_AVG_ACTIVE_IN;
    stateMachineIn.planningCtrlPort.apChosenTargetPoseId_nu = *data.selectedTargetPoseIndex;

    memset((void*)&mfControlStatusPort, 0, sizeof(mfControlStatusPort));

    while(distToStop > reqDistToStop)
    {
        distToStop = plannedTrajectory.plannedTraj[
            plannedTrajectory.numValidCtrlPoints_nu >= ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS ?
                ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS : plannedTrajectory.numValidCtrlPoints_nu].distanceToStopReq_m;
        auto const& nextPos = getNextControlPoint();

        emData = *data.emData;
        parkBoxPort = *data.parkBoxPort;
        egoPort = *data.egoPort;
        emData.egoVehiclePoseForAP.x_dir = nextPos.xTrajRAReq_m;
        emData.egoVehiclePoseForAP.y_dir = nextPos.yTrajRAReq_m;
        emData.egoVehiclePoseForAP.yaw_rad = nextPos.yawReq_rad;
        egoPort.frontWheelAngle_rad = nextPos.crvRAReq_1pm;

        runFuncTrjpla(inDataTrjpla, outDataTrjpla);
    }
    return true;
}

void BackendDllHandler::setVisuInterface(VisuClientInterface *iface)
{
    if(setVisuIfaceFunc) {
        setVisuIfaceFunc(iface);
    }
}

bool BackendDllHandler::planGeometricPath(const ap_tp::TRJPLA_Config &cfg, const BackendDllHandler::TrjplaData &data,
    ap_tp::GeomPathRepresentation *path, ap_tp::ParkingPath& trajectory, bool isReplanning, QString *errorMsg, bool* isInCollAtStart)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    /*if(data.targetPoseIn->selectedPoseIdx_nu >= AP_Common::AP_G_MAX_NUM_TARGET_POSES_NU) {
        return failWithMessage(errorMsg, "No valid target pose selected");
    }*/

    geomInitFunc(cfg);

    emData = *data.emData;
	parkBoxPort = *data.parkBoxPort;
    egoPort = *data.egoPort;
	targetPosesPort = *data.targetPosesPort;
	selectedTargetPoseIndex = *data.selectedTargetPoseIndex;
    selectedTpDeviations = *data.selectedTpDeviations;
    trjplaMetaData = *data.trjplaMetaData;

    geomPlanFunc(emData, parkBoxPort, egoPort, targetPosesPort.targetPoses[selectedTargetPoseIndex], selectedTpDeviations, trjplaMetaData, path, trajectory, isReplanning, isInCollAtStart);

    return path->isOk() || path->getResult() == ap_tp::ManeuverResult::FALLBACK_OK;
}

bool BackendDllHandler::planGaragePath(const ap_tp::TRJPLA_Config &cfg, const BackendDllHandler::TrjplaData &data,
    ap_tp::GarageParkingPath *path, ap_tp::ParkingPath* trajectory, bool isReplanning, QString *errorMsg, bool* isInCollAtStart)
{
    if (!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    garageInitFunc(cfg);

    emData = *data.emData;
    parkBoxPort = *data.parkBoxPort;
    egoPort = *data.egoPort;
    targetPosesPort = *data.targetPosesPort;
    selectedTargetPoseIndex = *data.selectedTargetPoseIndex;

    garagePlanFunc(emData, parkBoxPort, egoPort, targetPosesPort.targetPoses[selectedTargetPoseIndex], path, trajectory, isReplanning, isInCollAtStart);

    return path->isOK();
}

/*
QPainter *BackendDllHandler::beginPaint()
{
    return nullptr;
}

void BackendDllHandler::endPaint()
{

}
*/

bool BackendDllHandler::checkPrerequisitesLoaded(QString *errorMsg)
{
    if(!initFuncTrjpla) {
        return failWithMessage(errorMsg, "initFuncTrjpla not resolved");
    }

    if(!runFuncTrjpla) {
        return failWithMessage(errorMsg, "runFuncTrjpla not resolved");
    }

    return true;
}

bool BackendDllHandler::failWithMessage(QString *errorMsg, QString msg)
{
    qDebug() << "BackendDllHandler:" << msg;
    if(errorMsg) {
        *errorMsg = msg;
    }
    return false;
}

ap_tp::PlannedTraj BackendDllHandler::getNextControlPoint() const
{
    int nextIndex = plannedTrajectory.numValidCtrlPoints_nu >= ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS+1?
        ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS+1 : plannedTrajectory.numValidCtrlPoints_nu-1;
    return plannedTrajectory.plannedTraj[nextIndex];
}
