#define _USE_MATH_DEFINES
#include <cmath>

#include "clienthandler.h"

#include <QDebug>
#include <QCoreApplication>
#include <QDataStream>
#include <QTimer>

#include <iostream>

#include "AP_ReadJsonParams.h"
#include <mf_trjpla/ReadJsonTRJPLAParams.h>

#include <si_types_JsonReader/SI_OUTPUT_TestDataHandler.h>


static inline float normalizeAngle(float rad) {
    float val = std::fmod(rad + M_PI, 2*M_PI);
    if(val < 0)
        val += 2*(float)M_PI;
    return val - (float)M_PI;
}

ClientHandler::ClientHandler(ParkingSceneModel *model, QObject *parent)
    : QObject(parent)
    , mParkingModel(model)
    , mServerMemory("parking_visu_memory")
{
    connect(&mServerSocket, &QLocalServer::newConnection, this, &ClientHandler::onNewConnection);

    if(!mServerMemory.create(sizeof(ServerMemory))) {
        qDebug("VisualizationServer: failed to create shared memory");
    }

    memset(&trjplaDebugPort, 0, sizeof(trjplaDebugPort));

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this] {
        onClientBytesReady();
    });
    timer->start(100);
}

ClientHandler::~ClientHandler()
{
    if(mClientProcess) {
        mClientProcess->kill();
    }
}

bool ClientHandler::startServer()
{
    return mServerSocket.listen("parking_visu");
}

void ClientHandler::startClientProcessIfNotRunning()
{
    if(!mClientProcess) {
        mClientProcess = new QProcess(this);
        connect(mClientProcess, &QProcess::readyReadStandardOutput, this, &ClientHandler::onClientProcessOutputReady);
        connect(mClientProcess, &QProcess::readyReadStandardError, this, &ClientHandler::onClientProcessOutputReady);

        mClientProcess->start(QCoreApplication::applicationFilePath(), {"-c"}, QIODevice::ReadWrite | QIODevice::Unbuffered);
    }
}

void ClientHandler::loadComponentsDLL(QString filename)
{
    if(mConnectedSocket) {
        mComm->sendCommand(ParkingSocketCommand::CLIENT_LOAD_DLL, filename);
    }
}

void ClientHandler::loadTRJPLAParameters(QString filename)
{
    trjplaParams = std::make_shared<ap_tp::FC_TRJPLA_Params>();
#ifndef ULTRASONIC_ONLY
    auto& reader = ap_read_params::ReadJsonTRJPLAParams::getInstance();
    reader.readParams(filename.toStdString(), *trjplaParams);
#else
    initFC_TRJPLA_Params(&(*trjplaParams));
#endif
    trjplaParams->taposdParams.AP_T_ACTUAL_LENGTH_POSE_HISTORY = 1;
    mParkingModel->setTrajplaParams(*trjplaParams);
}

void ClientHandler::loadVehicleParameters(QString filename)
{
    vehicleParams = std::make_shared<ap_common::Vehicle_Params>();
#ifndef ULTRASONIC_ONLY
    auto& reader = ap_read_params::AP_ParamReader::getInstance();
    reader.readVehicleParams(filename.toStdString(), *vehicleParams);
#else
    initVehicle_Params(&(*vehicleParams));
#endif
    mParkingModel->setVehicleParameters(*vehicleParams);
}

void ClientHandler::loadSysFuncParameters(QString filename)
{
    sysFuncParams = std::make_shared<ap_common::Sys_Func_Params>();
#ifndef ULTRASONIC_ONLY
    auto& reader = ap_read_params::AP_ParamReader::getInstance();
    reader.readSysFuncParams(filename.toStdString(), *sysFuncParams);
#else
    initSys_Func_Params(&(*sysFuncParams));
#endif
}

void ClientHandler::loadEM(QString filename)
{
    mParkingModel->loadEM(filename);
}

void ClientHandler::loadREACH(QString filename)
{
    mParkingModel->getTargetPoseReachableAreaModel()->loadREACH(filename);
}

bool ClientHandler::findReachableArea(QString *errorMsg, bool is3D)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }
    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->trjplaParams = *trjplaParams;
    mem->vehicleParams = *vehicleParams;
    mem->sysFuncParams = *sysFuncParams;

    mem->emData = *mParkingModel->getEnvModelPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mem->selectedTpDeviations = *mParkingModel->getSelectedTargetPoseModel()->getMaxAllowedDeviations();
    mServerMemory.unlock();

    TargetPoseModel* selectedTargetPoseModel = mParkingModel->getSelectedTargetPoseModel();

    if (selectedTargetPoseModel == nullptr || !selectedTargetPoseModel->isPoseValid()) {
         return failWithMessage(errorMsg, QString("The selected target pose #%1 is not valid.").arg(0) );
    }

    setComputationInProgress(true);
    mComm->sendCommand(ParkingSocketCommand::CLIENT_FIND_TARGET_POSE_REACHABLE_AREA,mParkingModel->getTargetPoseReachableAreaModel()->getXstart(),
                       mParkingModel->getTargetPoseReachableAreaModel()->getXend(), mParkingModel->getTargetPoseReachableAreaModel()->getXstep(),
                       mParkingModel->getTargetPoseReachableAreaModel()->getYstart(), mParkingModel->getTargetPoseReachableAreaModel()->getYend(),
                       mParkingModel->getTargetPoseReachableAreaModel()->getYstep(),
                       mParkingModel->getTargetPoseReachableAreaModel()->getYawAngleDegStart(), mParkingModel->getTargetPoseReachableAreaModel()->getYawAngleDegEnd(),
                       mParkingModel->getTargetPoseReachableAreaModel()->getYawAngleDegStep(),
                       mParkingModel->getTargetPoseReachableAreaModel()->isReplanning(), is3D);

    return true;
}

bool ClientHandler::startScanModeTrjpla(bool scanModeOn, QString *errorMsg){
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->trjplaParams = *trjplaParams;
    mem->vehicleParams = *vehicleParams;
    mem->sysFuncParams = *sysFuncParams;

    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->parkingBoxPort = *mParkingModel->getParkingBoxPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mServerMemory.unlock();

    mem->scanModeEnabled = scanModeOn;

    setComputationInProgress(true);

    mComm->sendCommand(ParkingSocketCommand::CLIENT_START_SCAN_MODE_TRJPLA);

    return true;
}

bool ClientHandler::runForScanTrjpla(QString *errorMsg){

    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->trjplaParams = *trjplaParams;
    mem->vehicleParams = *vehicleParams;
    mem->sysFuncParams = *sysFuncParams;

    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->parkingBoxPort = *mParkingModel->getParkingBoxPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mServerMemory.unlock();

    mem->scanModeEnabled = true;

    setComputationInProgress(true);

    mComm->sendCommand(ParkingSocketCommand::CLIENT_RUN_FOR_SCAN_TRJPLA);

    return true;
}

bool ClientHandler::startAndRunScanModeTaposd(QString *errorMsg){

    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->vehicleParams = *vehicleParams;
    mem->sysFuncParams = *sysFuncParams;

    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->parkingBoxPort = *mParkingModel->getParkingBoxPort();
    mServerMemory.unlock();

    setComputationInProgress(true);

    mComm->sendCommand(ParkingSocketCommand::CLIENT_START_SCAN_MODE_TAPOSD);

    return true;
}

bool ClientHandler::planNewPath(QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }
    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->trjplaParams = *trjplaParams;
    mem->vehicleParams = *vehicleParams;
    mem->sysFuncParams = *sysFuncParams;

    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->parkingBoxPort = *mParkingModel->getParkingBoxPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mServerMemory.unlock();

    TargetPoseModel* selectedTargetPoseModel = mParkingModel->getSelectedTargetPoseModel();

    setComputationInProgress(true);
    mComm->sendCommand(ParkingSocketCommand::CLIENT_PLAN_NEW_PATH);

    return true;
}

bool ClientHandler::replanExistingPath(QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    setComputationInProgress(true);

    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->parkingBoxPort = *mParkingModel->getParkingBoxPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mServerMemory.unlock();

    mComm->sendCommand(ParkingSocketCommand::CLIENT_REPLAN_PATH);

    return true;
}

bool ClientHandler::makeStepOnPath(QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    setComputationInProgress(true);

    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->parkingBoxPort = *mParkingModel->getParkingBoxPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mServerMemory.unlock();

    mComm->sendCommand(ParkingSocketCommand::CLIENT_MAKE_STEP);
    return true;
}

bool ClientHandler::jumpOnPath(double reqDistToStop, QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }

    setComputationInProgress(true);

    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->reqDistToStop = reqDistToStop;
    mServerMemory.unlock();

    mComm->sendCommand(ParkingSocketCommand::CLIENT_JUMP);
    return true;

}

bool ClientHandler::planNewGeomPath(QString *errorMsg)
{
    if(!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }
    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->trjplaParams = *trjplaParams;
    mem->vehicleParams = *vehicleParams;
    mem->sysFuncParams = *sysFuncParams;

    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->parkingBoxPort = *mParkingModel->getParkingBoxPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mem->selectedTpDeviations = *mParkingModel->getSelectedTargetPoseModel()->getMaxAllowedDeviations();
    mem->trjplaMetaData = *mParkingModel->getTrjplaMetaData();
    mServerMemory.unlock();

    TargetPoseModel* selectedTargetPoseModel = mParkingModel->getSelectedTargetPoseModel();

    if(selectedTargetPoseModel == nullptr || !selectedTargetPoseModel->isPoseValid()) {
         return failWithMessage(errorMsg, QString("The selected target pose #%1 is not valid.").arg(0) );
    }

    setComputationInProgress(true);
    mComm->sendCommand(ParkingSocketCommand::CLIENT_PLAN_GEOM_PATH, mParkingModel->isReplanning());

    return true;
}

bool ClientHandler::planNewGaragePark(QString *errorMsg)
{
    if (!checkPrerequisitesLoaded(errorMsg)) {
        return false;
    }
    ServerMemory* mem = (ServerMemory*)mServerMemory.data();
    mServerMemory.lock();
    mem->trjplaParams = *trjplaParams;
    mem->vehicleParams = *vehicleParams;
    mem->sysFuncParams = *sysFuncParams;

    mem->emData = *mParkingModel->getEnvModelPort();
    mem->egoPort = *mParkingModel->getEgoMotionPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->TargetPoses = *mParkingModel->getTargetPosesPort();
    mem->selectedTargetPoseIndex = *mParkingModel->getSelectedTargetPoseIndex();
    mServerMemory.unlock();

    TargetPoseModel* selectedTargetPoseModel = mParkingModel->getSelectedTargetPoseModel();
    if (selectedTargetPoseModel == nullptr || !selectedTargetPoseModel->isPoseValid()) {
        return failWithMessage(errorMsg, QString("The selected target pose #%1 is not valid.").arg(0));
    }

    setComputationInProgress(true);
    mComm->sendCommand(ParkingSocketCommand::CLIENT_PLAN_GARAGE_PARK);

    return true;
}

ap_tp::TrajPlanState ClientHandler::getTrajPlanState() const
{
    if(!mServerMemory.isAttached()) {
        return ap_tp::TrajPlanState::INIT;
    } else {
        mServerMemory.lock();
        auto state = ((const ServerMemory*)mServerMemory.data())->trjplaDebugPort.mTrajPlanState;
        mServerMemory.unlock();
        qDebug() << "state" << state;
        return (ap_tp::TrajPlanState)state;
    }
}

void ClientHandler::setDebugDrawer(DebugDrawer *item)
{
    mDebugDrawer = item;
}

bool ClientHandler::isLibraryLoaded() const
{
    mServerMemory.lock();
    auto loaded = ((const ServerMemory*)mServerMemory.data())->dllLoaded;
    mServerMemory.unlock();

    return loaded;
}

void ClientHandler::onNewConnection()
{
    qDebug() << "VisualizationServer: new client connection";

    QLocalSocket *clientConnection = mServerSocket.nextPendingConnection();

    // for now just replace the current connection
    if(mConnectedSocket) {
        qDebug() << "VisualizationServer: preplacing previous client";

        disconnect(mConnectedSocket, 0, 0, 0);
        connect(mConnectedSocket, &QLocalSocket::disconnected, mConnectedSocket, &QLocalSocket::deleteLater);
        mConnectedSocket->disconnectFromServer();
    }

    mConnectedSocket = clientConnection;
    connect(mConnectedSocket, &QLocalSocket::disconnected, this, &ClientHandler::onClientDisconnected);
    connect(mConnectedSocket, &QLocalSocket::readyRead, this, &ClientHandler::onClientBytesReady);

    mComm = std::make_shared<StreamCommunicator>(mConnectedSocket, [this] (ParkingSocketCommand cmd, QDataStream& in) {
            this->handleClientCommand(cmd, in);
    });

    emit clientConnected();
}

void ClientHandler::onClientDisconnected()
{
    qDebug() << "VisualizationServer: client disconnected from server";

    mConnectedSocket->deleteLater();
    mComm.reset();
    mConnectedSocket = nullptr;
}

void ClientHandler::onClientBytesReady()
{
    if(mComm) {
        mComm->checkReceived();
    }
}

void ClientHandler::onClientProcessOutputReady()
{
    if(mClientProcess) {
        auto out = mClientProcess->readAllStandardOutput();
        std::cout.write(out.data(), out.size());

        auto err = mClientProcess->readAllStandardError();
        std::cerr.write(err.data(), err.size());
    }
}

void ClientHandler::enableDebugDraw(bool enabled)
{
    if(mServerMemory.isAttached()) {
        ((ServerMemory*) mServerMemory.data())->debugDrawEnabled = enabled;
    }

    if(!enabled) {
        mDebugDrawer->clearAll();
    }
}

bool ClientHandler::checkPrerequisitesLoaded(QString *errorMsg)
{
    if(!trjplaParams) {
        return failWithMessage(errorMsg, "trjplaParams not loaded.");
    }

    if(!vehicleParams) {
        return failWithMessage(errorMsg, "vehicleParams not loaded.");
    }

    if(!sysFuncParams) {
        return failWithMessage(errorMsg, "sysFuncParams not loaded.");
    }

    if(!mParkingModel->getEnvModelPort()) {
        return failWithMessage(errorMsg, "No EM loaded.");
    }

    if(!mServerMemory.isAttached()) {
        return failWithMessage(errorMsg, "Server shared memory not attached.");
    }

    if(!mConnectedSocket) {
        return failWithMessage(errorMsg, "Planner client not connected.");
    }

    if(!mComm) {
        return failWithMessage(errorMsg, "Stream communicator not connected.");
    }

    if(mComputationInProgress) {
        return failWithMessage(errorMsg, "A computation is already in progress.");
    }

    return true;
}

bool ClientHandler::failWithMessage(QString *errorMsg, QString msg)
{
    qDebug() << "VisualizationServer:" << msg;
    if(errorMsg) {
        *errorMsg = msg;
    }
    return false;
}

void ClientHandler::handleClientCommand(ParkingSocketCommand cmd, QDataStream &in)
{
    //static int counter = 0;
    // qDebug() << counter++ << "commend received" << (int)cmd;

    switch(cmd) {
    case ParkingSocketCommand::SERVER_UPDATE_PATH:
    {
        setComputationInProgress(false);

        mServerMemory.lock();
        const ServerMemory* serverMem = (const ServerMemory*) mServerMemory.data();
        trjplaDebugPort = serverMem->trjplaDebugPort;
        plannedTrajectory = serverMem->plannedTrajectory;
        trjplaVisuPort = serverMem->trjplaVisuPort;
        mServerMemory.unlock();

        setPathFromDebugOut();
        // setPathFromTrajPort();
        emit trajPlanStateChanged(getTrajPlanState());

        float distanceToStop = std::numeric_limits<float>::min();
        if(plannedTrajectory.numValidCtrlPoints_nu > ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS) {
            distanceToStop = plannedTrajectory.plannedTraj[ap_tp::AP_P_NUM_PASSED_TRAJ_CTRL_PTS].distanceToStopReq_m;
        }
        emit distanceToStopChanged(distanceToStop);
    }
        break;
    case ParkingSocketCommand::SERVER_PLANNING_FAILED:
    {
        setComputationInProgress(false);

        mParkingModel->getTrajectorySetModel()->setTrajectories();

        QString msg;
        in >> msg;
        emit planningFailed(msg);
        emit trajPlanStateChanged(getTrajPlanState());
    }
        break;

    case ParkingSocketCommand::SERVER_CONT_SCAN: {
        mTargetPoseReachable[mParkingModel->getSelectedTargetPose()]= false;
        setComputationInProgress(false);
    }
        break;

    case ParkingSocketCommand::SERVER_OFFER_PARK_AND_CONT_SCAN: {
        mTargetPoseReachable[mParkingModel->getSelectedTargetPose()]= true;
        setComputationInProgress(false);
    }
        break;

    case ParkingSocketCommand::SERVER_UPDATE_EGOPOSE:
    {

        float x, y, yaw, curvature;
        in >> x >> y >> yaw >> curvature;
        mParkingModel->getStartPoseModel()->setPos(x, y, yaw);
        float radius = std::abs(curvature) < 0.01F? 0.0F : 1.0F / curvature;
        mParkingModel->getStartPoseModel()->setRotationRadius(radius);
    }
        break;

    case ParkingSocketCommand::SERVER_UPDATE_EM:{
        si::ApEnvModelPort em;
        in >> em;
        mParkingModel->setEnvModelPort(em);
    }
        break;

    case ParkingSocketCommand::SERVER_UPDATE_PARKBOX:{
        si::ApParkingBoxPort pbBox;
        in >> pbBox;
        mParkingModel->setParkingBoxPort(pbBox);
    }
        break;

    case ParkingSocketCommand::SERVER_UPDATE_TARGET_POSES:{
        ap_tp::TargetPosesPort targetPoses;
        in >> targetPoses;
        mParkingModel->setTargetPosesPort(targetPoses);
    }
        break;

    case ParkingSocketCommand::SERVER_UPDATE_TAPOSD:{
        setComputationInProgress(false);

        mServerMemory.lock();
        const ServerMemory* serverMem = (const ServerMemory*) mServerMemory.data();
        mParkingModel->setTargetPosesPort(serverMem->TargetPoses);
        mParkingModel->getTaposdDebugModel()->setData(serverMem->taposdDebugPort);
        mServerMemory.unlock();
    }
        break;

    case ParkingSocketCommand::SERVER_UPDATE_GEOM_TRAJECTORY:
    case ParkingSocketCommand::SERVER_UPDATE_POLYNOM_TRAJECTORY:
    {
        setComputationInProgress(false);

        mServerMemory.lock();
        const ServerMemory* serverMem = (const ServerMemory*)mServerMemory.data();
        SimpleParkPath parkingPath = serverMem->parkPath;
        mServerMemory.unlock();

        mParkingModel->getTrajectorySetModel()->setMainTrajectory(trajectoryFromSimpleParkingPath(parkingPath));
    }
        break;

    case ParkingSocketCommand::SERVER_DRAW_POINT: {
        float x,y;
        QString name;
        in >> x >> y >> name;
        //qDebug() << "SERVER_DRAW_POINT" << x << y << name;

        if(mDebugDrawer) {
            mDebugDrawer->drawPoint(name, x, y);
        }
    }
        break;
    case ParkingSocketCommand::SERVER_DRAW_LINE: {
        float x1, y1, x2, y2;
        QString name;
        in >> x1 >> y1 >> x2 >> y2 >> name;
        //qDebug() << "SERVER_DRAW_LINE" << x << y << name;

        if(mDebugDrawer) {
            mDebugDrawer->drawLine(name, x1, y1, x2, y2);
        }
    }
        break;
    case ParkingSocketCommand::SERVER_DRAW_CIRCLE: {
        float x, y, r;
        QString name;
        in >> x >> y >> r >> name;
        // qDebug() << "SERVER_DRAW_CIRCLE" << x << y << name;

        if(mDebugDrawer) {
            mDebugDrawer->drawCircle(name, x, y, r);
        }
    }
        break;
    case ParkingSocketCommand::SERVER_DRAW_VEHICLE: {
        float x, y, yaw;
        QString name;
        in >> x >> y >> yaw >> name;
        // qDebug() << "SERVER_DRAW_VEHICLE" << x << y;

        if(mDebugDrawer) {
            mDebugDrawer->getParkingModel()->getDebugVehicleModel()->setDebugPos(x,y, yaw);
            mDebugDrawer->getParkingModel()->getDebugVehicleModel()->setVisible(true);
        }
    }
        break;
    case ParkingSocketCommand::SERVER_DRAW_VEHICLE_RADIUS: {
        float r;
        QString name;
        in >> r >> name;
        // qDebug() << "SERVER_DRAW_VEHICLE_RADIUS" << x << y << name;

        if(mDebugDrawer) {
            mDebugDrawer->getParkingModel()->getDebugVehicleModel()->setDebugRotationRadius(r);
            mDebugDrawer->getParkingModel()->getDebugVehicleModel()->setVisible(true);
        }
    }
        break;
    case ParkingSocketCommand::SERVER_DRAW_POLYGON: {
        QVector<QPointF> poly;
        QString name;
        in >> poly >> name;
        // qDebug() << "SERVER_DRAW_POLYGON" << x << y << name;

        if(mDebugDrawer) {
            mDebugDrawer->drawPolygon(name, poly);
        }
    }
        break;

    case ParkingSocketCommand::SERVER_DRAW_PATH: {
        QString name;
        ap_tp::ParkingPath parkingPath;
        in >> parkingPath >> name;

        mParkingModel->getTrajectorySetModel()->addTrajectory(name, trajectoryFromParkingPath(parkingPath));
    }
        break;

    case ParkingSocketCommand::SERVER_CLEAR_DRAW: {
        if(mDebugDrawer) {
            mDebugDrawer->clearAll();
        }

        mParkingModel->getTrajectorySetModel()->setTrajectories({});
    }
        break;

    case ParkingSocketCommand::SERVER_DRAW_GEOM_PATH: {
        ap_tp::GeomPathRepresentation geomPath;
        QString name;
        in >> geomPath >> name;

        ap_tp::PathSampler sampler;
        ap_tp::TRJPLA_Config config { trjplaParams.get(), vehicleParams.get(), sysFuncParams.get() };
        ap_tp::PlanningEgoVehicle egoVeh;
        egoVeh.init(config, ap_common::EgoVehicleShapeType::EGO_VEH_SHAPE_BOUNDING, nullptr);
        sampler.init(*config.trjplaParams, &egoVeh);
        ap_tp::ParkingPath sampledPath;
        sampler.samplePath(geomPath, sampledPath);
        mParkingModel->getTrajectorySetModel()->addTrajectory(name, trajectoryFromParkingPath(sampledPath));
    }
        break;


    case ParkingSocketCommand::SERVER_FORM_REACHABLE_TARGET_POSE_AREA:{
        float32_t x,y,yawRad;
        bool reachable;
        int nr_strokes;
        in >> x >> y >> yawRad >> reachable >> nr_strokes;
        QVector3D position{x,y,yawRad};

        ReachablePoseEntry entry {position, reachable, nr_strokes};

        mParkingModel->getTargetPoseReachableAreaModel()->setPoseInTargetReachableArea(entry);
    }
        break;
    case ParkingSocketCommand::SERVER_GET_REACHABLE_TARGET_POSE_AREA:{
        bool in3D;
        in>> in3D;
        if(in3D){
          setTargetPoseReachableAreaReady3D();
          setTargetPoseReachableAreaReady2D(true);
        }else{
          setTargetPoseReachableAreaReady2D(true);
        }

        setComputationInProgress(false);
    }
        break;
    case ParkingSocketCommand::SERVER_UPDATE_VEHICLE_INFLATION:
    {
        float r;
        in >> r;
        // first reset the veh inflation to always have an update
        mParkingModel->setVehInflRadius_m(-1.0F);
        mParkingModel->setVehInflRadius_m(r);
    }
    break;
    case ParkingSocketCommand::SERVER_UPDATE_VEHICLE_PARAMS:{
        ap_common::Vehicle_Params params;
        in >> params;
        mParkingModel->setVehicleParameters(params);
    }
        break;
    }
}

void ClientHandler::setPathFromDebugOut()
{
    auto start = mParkingModel->getStartPoseModel();
    Trajectory path;
    {
        // Prepend ego pose at path start
        QTransform trans;
        trans.translate(start->getPosX_m(), start->getPosY_m());
        trans.rotateRadians(start->getPosYawAngle_Rad());
        path.append(trans);
    }

    float previousYaw = start->getPosYawAngle_Rad();
    bool flipped = false;
    bool flippedChanged = false; // if flipped changed last time

    for(int i = 0; i < trjplaVisuPort.numValidPoses_nu; i++) {
        QTransform trans;
        trans.translate(trjplaVisuPort.plannedPathXPos_m[i], trjplaVisuPort.plannedPathYPos_m[i]);

        float currentYaw = previousYaw;

        if (i > 0) {
            float dx = trjplaVisuPort.plannedPathXPos_m[i-1] - trjplaVisuPort.plannedPathXPos_m[i];
            float dy = trjplaVisuPort.plannedPathYPos_m[i-1] - trjplaVisuPort.plannedPathYPos_m[i];

            currentYaw = std::atan2(dy, dx);
        }

        if((std::abs(normalizeAngle(currentYaw - previousYaw)) > M_PI_4/2) && (!flippedChanged)) {
            flipped = !flipped;
            flippedChanged = true;
        } else {
            flippedChanged = false;
        }

        trans.rotateRadians(currentYaw);
        if(flipped) {
            trans.rotateRadians(M_PI);
        }

        previousYaw = currentYaw;
        path.append(TrajectoryPose(trans));
    }

    mParkingModel->getTrajectorySetModel()->setMainTrajectory(path);
}

void ClientHandler::setPathFromTrajPort()
{
    Trajectory path;
    for(int i = 0; i < plannedTrajectory.numValidCtrlPoints_nu; i++) {
        TrajectoryPose pose;
        pose.trans.translate(plannedTrajectory.plannedTraj[i].xTrajRAReq_m, plannedTrajectory.plannedTraj[i].yTrajRAReq_m);
        pose.trans.rotateRadians(plannedTrajectory.plannedTraj[i].yawReq_rad);
        pose.velocity_mps = plannedTrajectory.plannedTraj[i].velocityLimitReq_mps;
        path.append(pose);
    }
    mParkingModel->getTrajectorySetModel()->setMainTrajectory(path);
}

Trajectory ClientHandler::trajectoryFromParkingPath(const ap_tp::ParkingPath &parkingPath)
{
    Trajectory traj;
    for(lsm_geoml::size_type i = 0; i < parkingPath.getNumPoses(); i++) {
        TrajectoryPose pose;
        pose.trans.translate(parkingPath[i].pose.Pos().x(), parkingPath[i].pose.Pos().y());
        pose.trans.rotateRadians(parkingPath[i].pose.Yaw_rad());
        pose.velocity_mps = parkingPath[i].velocity_mps;
        pose.curvature_1pm = parkingPath[i].curvature_1pm;
        traj.append(pose);
    }

    return traj;
}

Trajectory ClientHandler::trajectoryFromSimpleParkingPath(const SimpleParkPath &parkingPath)
{
    Trajectory traj;
    for (lsm_geoml::size_type i = 0; i < (lsm_geoml::size_type)parkingPath.numPosesInParkPath; i++) {
        TrajectoryPose pose;
        pose.trans.translate(parkingPath.sampledPoses[i].pose.Pos().x(), parkingPath.sampledPoses[i].pose.Pos().y());
        pose.trans.rotateRadians(parkingPath.sampledPoses[i].pose.Yaw_rad());
        pose.velocity_mps = parkingPath.sampledPoses[i].velocity_mps;
        pose.curvature_1pm = parkingPath.sampledPoses[i].curvature_1pm;
        traj.append(pose);
    }

    return traj;
}

void ClientHandler::setComputationInProgress(bool inProgress)
{
    if(mComputationInProgress != inProgress) {
        mComputationInProgress = inProgress;
        emit computationInProgressChanged(mComputationInProgress);
    }
}

void ClientHandler::setTargetPoseReachableAreaReady2D(bool areaReady){
    mParkingModel->setTargetPoseReachableAreaReady2D(areaReady);
}

void ClientHandler::setTargetPoseReachableAreaReady3D(){
    mParkingModel->setTargetPoseReachableAreaReady3D();
}

