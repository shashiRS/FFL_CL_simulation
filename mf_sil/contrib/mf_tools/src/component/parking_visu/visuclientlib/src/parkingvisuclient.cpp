#include "parkingvisuclient.h"

#include <QCoreApplication>
#include <QDebug>
#include <QPointF>
#include <QtMath>

VisuClient::VisuClient(QObject *parent)
    : QObject(parent)
    , mComm(&mSocket, [this](ParkingSocketCommand cmd, QDataStream& in) {
        this->handleCommand(cmd, in);
    })
    , mServerMemory("parking_visu_memory")
{
    connect(&mSocket, &QLocalSocket::connected, this, &VisuClient::onConnected);
    connect(&mSocket, &QLocalSocket::disconnected, this, &VisuClient::onDisconnected);
    connect(&mSocket, &QLocalSocket::readyRead, this, &VisuClient::onReadyRead);
    connect(&mSocket, QOverload<QLocalSocket::LocalSocketError>::of(&QLocalSocket::error),
            this, &VisuClient::onSocketError);
}

void VisuClient::connectToServer()
{
    if(!mSocket.isOpen()) {
        mSocket.connectToServer("parking_visu");
        if(!mServerMemory.isAttached()) {
            if(!mServerMemory.attach()) {
                qDebug() << "ParkingVisuClient: failed to attach shared memory";
            }
        }
    }
}

bool VisuClient::isConnected()
{
    return mSocket.isOpen();
}

void VisuClient::clearDraw()
{
    // don't use sendDrawCommand because it is ignored if debug draw is off
    mComm.sendCommand(ParkingSocketCommand::SERVER_CLEAR_DRAW);
}

void VisuClient::drawPoint(float x, float y, const std::string &name)
{
    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_POINT, x,y, QString::fromStdString(name));
}

void VisuClient::drawLine(float x1, float y1, float x2, float y2, const std::string &name)
{
    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_LINE, x1, y1, x2, y2, QString::fromStdString(name));
}

void VisuClient::drawCircle(float x, float y, float r, const std::string &name)
{
    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_CIRCLE, x, y, r, QString::fromStdString(name));
}

void VisuClient::drawPolygon(const cml::Vec2Df *begin, const cml::Vec2Df *end, const std::string &name)
{
    QVector<QPointF> poly;
    while(begin != end) {
        poly.append(QPointF(begin->x(), begin->y()));
        begin++;
    }

    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_POLYGON, poly, QString::fromStdString(name));
}

void VisuClient::drawDebugVehicle(float x, float y, float yaw)
{
    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_VEHICLE, x, y, yaw, QString::fromStdString("egoVehicle"));
}

void VisuClient::drawDebugVehicleRadius(float radius)
{
    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_VEHICLE_RADIUS, radius, QString::fromStdString("egoVehicle"));
}

void VisuClient::drawPath(const ap_tp::ParkingPath &path, const std::string &name)
{
    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_PATH, path, QString::fromStdString(name));
}

void VisuClient::drawPathSet(const std::list<ap_tp::ParkingPath> &pathSet, const std::string &name)
{
    lsm_geoml::size_type counter = 0;
    for(auto const& path: pathSet) {
        drawPath(path, name + std::to_string(counter));
        counter++;
    }
}

void VisuClient::drawGeomPath(const ap_tp::GeometricPath &geomPath, const std::string &name)
{
    sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_GEOM_PATH, geomPath, QString::fromStdString(name));
}

void VisuClient::drawManeuverSet(const ap_tp::ManeuverSet &meneuverSet, const std::string &name)
{
    for(lsm_geoml::size_type i = 0; i < meneuverSet.getSize(); i++) {
        sendDrawCommand(ParkingSocketCommand::SERVER_DRAW_GEOM_PATH, *meneuverSet.allManeuvers()[i],
                        QString::fromStdString(name + "[" + std::to_string(i) + "]"));
    }
}

void VisuClient::updateVehicleInflation(float radius)
{
    mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_VEHICLE_INFLATION, radius);
}

void VisuClient::updateVehicleParams(const ap_common::Vehicle_Params& params)
{
    mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_VEHICLE_PARAMS, params);
}

void VisuClient::updateEM(const si::ApEnvModelPort &em)
{
    mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_EM, em);
}

void VisuClient::updateParkBox(const si::ApParkingBoxPort &pbPort)
{
    mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_PARKBOX, pbPort);
}

void VisuClient::updateEgoPort(const si::ApEnvModelPort &emData, const si::EgoMotionPort &egoPort)
{
    mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_EGOPOSE,
                      (float)emData.egoVehiclePoseForAP.x_dir, (float)emData.egoVehiclePoseForAP.y_dir,
                      (float)emData.egoVehiclePoseForAP.yaw_rad, (float)egoPort.frontWheelAngle_rad);
}

void VisuClient::updateTargetPosesPort(const ap_tp::TargetPosesPort &targetPoses)
{
    mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_TARGET_POSES, targetPoses);
}

void VisuClient::onConnected()
{
    qDebug() << "ParkingVisuClient: connected to server";
}

void VisuClient::onDisconnected()
{
    qDebug() << "ParkingVisuClient: disconnected from server";
    QCoreApplication::quit();
}

void VisuClient::onReadyRead()
{
    mComm.checkReceived();
}

void VisuClient::onSocketError(QLocalSocket::LocalSocketError socketError)
{
    qDebug() << "ParkingVisuClient: socket error" << socketError;
}

void VisuClient::handleCommand(ParkingSocketCommand cmd, QDataStream &in)
{

}

void ParkingVisuClient::handleCommand(ParkingSocketCommand cmd, QDataStream &in)
{
    switch (cmd) {
    case ParkingSocketCommand::CLIENT_LOAD_DLL:
    {
        QString filename;
        in >> filename;
        qDebug() << "ParkingVisuClient::handleCommand(CMD_LOAD_DLL)" << filename;
        mDllHandler.loadLibrary(filename);
        mDllHandler.setVisuInterface(this);

        if(mDllHandler.isLibraryLoaded() && mServerMemory.isAttached()) {
            mServerMemory.lock();
            ((ServerMemory*) mServerMemory.data())->dllLoaded = true;
            mServerMemory.unlock();
        }
    }
        break;
    case ParkingSocketCommand::CLIENT_FIND_TARGET_POSE_REACHABLE_AREA:
        float32_t xStart, xEnd, xStep, yStart, yEnd, yStep;
        float32_t yawAngleDegStart, yawAngleDegEnd, yawAngleDegStep;

        bool isReplanning, is3D;

        in >> xStart >> xEnd >> xStep >> yStart >> yEnd >> yStep >> yawAngleDegStart >> yawAngleDegEnd >> yawAngleDegStep >> isReplanning >> is3D;

        float32_t yawAngleRadStart, yawAngleRadEnd, yawAngleRadStep;
        /* Angles are sent in degrees and should be converted */
        yawAngleRadStart= qDegreesToRadians(yawAngleDegStart);
        yawAngleRadEnd= qDegreesToRadians(yawAngleDegEnd);
        yawAngleRadStep= qDegreesToRadians(yawAngleDegStep);

        findTargetPoseReachableArea(xStart, xEnd, xStep, yStart, yEnd, yStep, yawAngleRadStart, yawAngleRadEnd, yawAngleRadStep, isReplanning, is3D);
        break;
    case ParkingSocketCommand::CLIENT_START_SCAN_MODE_TRJPLA:
        startScanModeTrjpla();
        break;
    case ParkingSocketCommand::CLIENT_RUN_FOR_SCAN_TRJPLA:
        runForScanTrjpla();
        break;
    case ParkingSocketCommand::CLIENT_START_SCAN_MODE_TAPOSD:
        startAndRunScanModeTaposd();
        break;
    case ParkingSocketCommand::CLIENT_PLAN_NEW_PATH:
        startPlanNewPath();
        break;
    case ParkingSocketCommand::CLIENT_REPLAN_PATH:
        startReplan();
        break;
    case ParkingSocketCommand::CLIENT_MAKE_STEP:
        startStep();
        break;
    case ParkingSocketCommand::CLIENT_JUMP:
        startJump();
        break;
    case ParkingSocketCommand::CLIENT_PLAN_GEOM_PATH:
        in >> isReplanning;
        planPath(isReplanning);
        break;
    case ParkingSocketCommand::CLIENT_PLAN_GARAGE_PARK:
        planGaragePath(true);
        break;
    default:
        break;
    }
}

void ParkingVisuClient::findTargetPoseReachableArea(const float32_t xStart,const float32_t xEnd,const float32_t xStep,const float32_t yStart,const float32_t yEnd, const float32_t yStep,
                                                    const float32_t yawAngleRadStart, const float32_t yawAngleRadEnd, const float32_t yawAngleRadStep,
                                                    bool isReplanning, bool is3D) {
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, "failed to attach shared memory.");
            return;
        }
    }

    mServerMemory.lock();
    ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg {&serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams};

    const BackendDllHandler::TrjplaData data{ &serverMemCopy.emData, &serverMemCopy.parkingBoxPort ,&serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex, &serverMemCopy.selectedTpDeviations, &serverMemCopy.trjplaMetaData };

    qDebug("ParkingVisuClient: Draw Target Pose Reachable Area");

    QString msg;
    ap_tp::ParkingPath                parkingPath;
    ap_tp::GeomPathRepresentation     geomPath;

    /*
     * Use reachability area defined by the user to form the vehicle pose used in the iterative method used to form such area
     * */
    for(float32_t theta=yawAngleRadStart;(yawAngleRadStart < yawAngleRadEnd ? theta <= yawAngleRadEnd : theta >= yawAngleRadEnd); (yawAngleRadStart < yawAngleRadEnd ? theta+=yawAngleRadStep : theta-=yawAngleRadStep)  ){
        serverMemCopy.emData.egoVehiclePoseForAP.yaw_rad=theta;
        for( float32_t i= xStart ; (xStart < xEnd ? i <= xEnd : i >= xEnd) ; (xStart < xEnd ? i+=xStep : i-=xStep)){
            serverMemCopy.emData.egoVehiclePoseForAP.x_dir=i;
            for( float32_t j= yStart ; (yStart < yEnd ? j <= yEnd : j >= yEnd) ; (yStart < yEnd ? j+=yStep : j-=yStep)){
                serverMemCopy.emData.egoVehiclePoseForAP.y_dir=j;
                bool isInColl=false;
                bool reachable =  mDllHandler.planGeometricPath(cfg, data, &geomPath, parkingPath, isReplanning, &msg, &isInColl);
                if(!isInColl){
                    for(const auto& seg: geomPath) {
                        if(seg.planPhase_nu == ap_tp::GeomPlanPhase::GENERIC_PLANNING_PHASE) {
                            break;
                        }
                    };

                    mComm.sendCommand(ParkingSocketCommand::SERVER_FORM_REACHABLE_TARGET_POSE_AREA,
                                      serverMemCopy.emData.egoVehiclePoseForAP.x_dir,
                                      serverMemCopy.emData.egoVehiclePoseForAP.y_dir,
                                      serverMemCopy.emData.egoVehiclePoseForAP.yaw_rad,
                                      reachable,
                                      geomPath.getNumStrokes());
                }

            }
        }

        if(!is3D){
            break;
        }
    }


    mComm.sendCommand(ParkingSocketCommand::SERVER_GET_REACHABLE_TARGET_POSE_AREA,is3D);

}

void ParkingVisuClient::startScanModeTrjpla()
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, "failed to attach shared memory.");
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg {&serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams};
    const BackendDllHandler::TrjplaData data {&serverMemCopy.emData, &serverMemCopy.parkingBoxPort ,&serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex };

    qDebug("ParkingVisuClient: Scanning TRJPLA");

    QString msg;
    if(mDllHandler.startScanModeTrjpla(cfg, data, &msg)) {
        setTrjplaServerData();

        //mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_PATH);
    }
    mComm.sendCommand(ParkingSocketCommand::SERVER_CONT_SCAN);

}

void ParkingVisuClient::runForScanTrjpla()
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, "failed to attach shared memory.");
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg {&serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams};
    const BackendDllHandler::TrjplaData data{ &serverMemCopy.emData, &serverMemCopy.parkingBoxPort, &serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex };

    qDebug("ParkingVisuClient: Scanning TRJPLA");

    QString msg;
    if(mDllHandler.runForScanTrjpla(cfg, data, &msg)) {
        setTrjplaServerData();

        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_TAPOSD);
        mComm.sendCommand(ParkingSocketCommand::SERVER_OFFER_PARK_AND_CONT_SCAN);
    }else{
        mComm.sendCommand(ParkingSocketCommand::SERVER_CONT_SCAN);
    }

}

void ParkingVisuClient::startAndRunScanModeTaposd()
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, "failed to attach shared memory.");
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const BackendDllHandler::TaposdData data {&serverMemCopy.emData, &serverMemCopy.parkingBoxPort, &serverMemCopy.egoPort};

    qDebug("ParkingVisuClient: Scanning TAPOSD");

    QString msg;

    if(mDllHandler.startAndRunScanModeTaposd(data, &msg)) {
        mServerMemory.lock();
        ((ServerMemory*) mServerMemory.data())->TargetPoses = mDllHandler.getTargetPosesPort();
        ((ServerMemory*) mServerMemory.data())->taposdDebugPort = mDllHandler.getDebugPortTaposd();
        mServerMemory.unlock();
    }
    mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_TAPOSD);
}

void ParkingVisuClient::planPath(bool isReplanning)
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, "failed to attach shared memory.");
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg {&serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams};
    const BackendDllHandler::TrjplaData data { &serverMemCopy.emData, &serverMemCopy.parkingBoxPort ,&serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex, &serverMemCopy.selectedTpDeviations, &serverMemCopy.trjplaMetaData };

    qDebug("ParkingVisuClient: planning new path");

    QString msg;

    ap_tp::ParkingPath                parkingPath;
    ap_tp::GeomPathRepresentation     geomPath;

    if(mDllHandler.planGeometricPath(cfg, data, &geomPath, parkingPath, isReplanning, &msg)) {
        mServerMemory.lock();
        ((ServerMemory*)mServerMemory.data())->parkPath.numPosesInParkPath = parkingPath.getNumPoses();
        for (lsm_geoml::size_type i = 0; i < parkingPath.getNumPoses(); i++) {
            ((ServerMemory*)mServerMemory.data())->parkPath.sampledPoses[i] = parkingPath[i];
        } 
        mServerMemory.unlock();

        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_GEOM_TRAJECTORY);
    } else {
        mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, msg);
    }
}

void ParkingVisuClient::planGaragePath(bool isReplanning)
{
    if (!mServerMemory.isAttached()) {
        if (!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, "failed to attach shared memory.");
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*)mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg{ &serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams };
    const BackendDllHandler::TrjplaData data{ &serverMemCopy.emData, &serverMemCopy.parkingBoxPort, &serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex };

    qDebug("ParkingVisuClient: planning new path");

    QString msg;

    ap_tp::ParkingPath                parkingPath;
    ap_tp::GarageParkingPath          garagePath;

    if (mDllHandler.planGaragePath(cfg, data, &garagePath, &parkingPath, isReplanning, &msg)) {
        mServerMemory.lock();
        ((ServerMemory*)mServerMemory.data())->parkPath.numPosesInParkPath = parkingPath.getNumPoses();
        for (lsm_geoml::size_type i = 0; i < parkingPath.getNumPoses(); i++) {
            ((ServerMemory*)mServerMemory.data())->parkPath.sampledPoses[i] = parkingPath[i];
        }
        mServerMemory.unlock();

        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_POLYNOM_TRAJECTORY);
    }
    else {
        mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, msg);
    }
}

void ParkingVisuClient::startPlanNewPath()
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, "failed to attach shared memory.");
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg {&serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams};
    const BackendDllHandler::TrjplaData data{ &serverMemCopy.emData, &serverMemCopy.parkingBoxPort ,&serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex };

    qDebug("ParkingVisuClient: planning new path");

    QString msg;

    if(mDllHandler.planNewPath(cfg, data, &msg,((ServerMemory*) mServerMemory.data())->scanModeEnabled)) {
        setTrjplaServerData();

        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_PATH);
        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_TAPOSD);
    } else {
        mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, msg);
    }
}

void ParkingVisuClient::startReplan()
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const BackendDllHandler::TrjplaData data{ &serverMemCopy.emData, &serverMemCopy.parkingBoxPort ,&serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex };

    qDebug("ParkingVisuClient: replanning path");

    QString msg;
    if(mDllHandler.replanExistingPath(data, &msg)) {
        setTrjplaServerData();

        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_PATH);
    } else {
        mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, msg);
    }
}

void ParkingVisuClient::startStep()
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg {&serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams};
    const BackendDllHandler::TrjplaData data{ &serverMemCopy.emData, &serverMemCopy.parkingBoxPort ,&serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex };

    qDebug("ParkingVisuClient: making step");

    QString msg;
    if(mDllHandler.makeStepOnPath(data, &msg)) {
        setTrjplaServerData();

        auto ego = mDllHandler.getLastEgoPose();
        auto curvature = mDllHandler.getLastEgoCurvature();
        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_EGOPOSE, float(ego.Pos().x()), float(ego.Pos().y()), float(ego.Yaw_rad()), curvature);

        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_PATH);
    } else {
        mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, msg);
    }
}

void ParkingVisuClient::startJump()
{
    if(!mServerMemory.isAttached()) {
        if(!mServerMemory.attach()) {
            qDebug() << "ParkingVisuClient: failed to attach shared memory";
            return;
        }
    }

    mServerMemory.lock();
    const ServerMemory serverMemCopy = *(const ServerMemory*) mServerMemory.data();
    mServerMemory.unlock();

    const ap_tp::TRJPLA_Config cfg {&serverMemCopy.trjplaParams,
                &serverMemCopy.vehicleParams, &serverMemCopy.sysFuncParams};
    const BackendDllHandler::TrjplaData data{ &serverMemCopy.emData, &serverMemCopy.parkingBoxPort ,&serverMemCopy.egoPort, &serverMemCopy.TargetPoses, &serverMemCopy.selectedTargetPoseIndex };

    qDebug("ParkingVisuClient: jumping");

    QString msg;
    if(mDllHandler.jumpOnPath(data, serverMemCopy.reqDistToStop, &msg)) {
        setTrjplaServerData();

        auto ego = mDllHandler.getLastEgoPose();
        auto curvature = mDllHandler.getLastEgoCurvature();
        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_EGOPOSE, float(ego.Pos().x()), float(ego.Pos().y()), float(ego.Yaw_rad()), curvature);

        mComm.sendCommand(ParkingSocketCommand::SERVER_UPDATE_PATH);
    } else {
        mComm.sendCommand(ParkingSocketCommand::SERVER_PLANNING_FAILED, msg);
    }
}

void ParkingVisuClient::setTrjplaServerData()
{
    mServerMemory.lock();
    ((ServerMemory*) mServerMemory.data())->trjplaDebugPort = mDllHandler.getDebugPortTrjpla();
    ((ServerMemory*) mServerMemory.data())->plannedTrajectory = mDllHandler.getPlannedTrajectory();
    ((ServerMemory*) mServerMemory.data())->trjplaVisuPort = mDllHandler.getTrajplaVisuPort();
    ((ServerMemory*) mServerMemory.data())->TargetPoses = mDllHandler.getTargetPosesPort();
    ((ServerMemory*) mServerMemory.data())->taposdDebugPort = mDllHandler.getDebugPortTaposd();
    mServerMemory.unlock();
}

StreamCommunicator::StreamCommunicator(QLocalSocket *device, const StreamCommunicator::MessageHandler &handler)
    : mDevice(device)
    , mMessageHandler(handler)
{
}

StreamCommunicator::~StreamCommunicator()
{
    // write out remaining data if any before closing
    if(mDevice->isOpen()) {
        // mDevice->waitForBytesWritten(-1);
    }
}

void StreamCommunicator::checkReceived()
{
    while(true) {
        if(mCurrentBlockSize == 0) { // new message
            if(mDevice->bytesAvailable() < (int)sizeof(quint32)) {
                return;
            }

            mDevice->read((char*)&mCurrentBlockSize, sizeof(mCurrentBlockSize));
        }

        if (mDevice->bytesAvailable() < mCurrentBlockSize)
            return;

        QByteArray data(mCurrentBlockSize, 0);
        mCurrentBlockSize = 0; // to receive next block

        Q_ASSERT(mDevice->read(data.data(), data.size()) == data.size());
        QDataStream in(data);
        in.setVersion(QDataStream::Qt_5_12);

        uint32_t cmd_raw;
        in >> cmd_raw;
        ParkingSocketCommand cmd = (ParkingSocketCommand) cmd_raw;

        mMessageHandler(cmd, in);
    }
}

template<class T>
QDataStream& writeBlob(QDataStream &out, const T &rhs) {
    QByteArray ba;
    ba.resize(sizeof(rhs));
    ba.setRawData(reinterpret_cast<const char*>(&rhs), sizeof(rhs));
    return out << ba;
}

template<class T>
QDataStream& readBlob(QDataStream &in, T &rhs)
{
    QByteArray ba;
    in >> ba;
    assert(ba.size() == sizeof(rhs));

    memcpy(reinterpret_cast<char*>(&rhs), ba.constData(), sizeof(rhs));
    return in;
}

QDataStream &operator<<(QDataStream &out, const ap_tp::ParkingPath &rhs)
{
    return writeBlob(out, rhs);
}

QDataStream &operator>>(QDataStream &in, ap_tp::ParkingPath &rhs)
{
    return readBlob(in, rhs);
}

QDataStream &operator<<(QDataStream &out, const ap_tp::GeometricPath &rhs)
{
    out << rhs.size();
    for(const ap_tp::GeomPathSegment & segment : rhs) {
        writeBlob(out, segment);
    }

    return out;
}

QDataStream &operator>>(QDataStream &in, ap_tp::GeometricPath &rhs)
{
    lsm_geoml::size_type size;
    in >> size;
    rhs.clear();
    for(lsm_geoml::size_type i = 0; i < size; i++) {
        ap_tp::GeomPathSegment segment;
        readBlob(in, segment);
        rhs.append(segment);
    }

    return in;
}

QDataStream &operator<<(QDataStream &out, const si::ApEnvModelPort &rhs)
{
    return writeBlob(out, rhs);
}

QDataStream &operator>>(QDataStream &in, si::ApEnvModelPort &rhs)
{
    return readBlob(in, rhs);
}

QDataStream &operator<<(QDataStream &out, const si::ApParkingBoxPort &rhs)
{
    return writeBlob(out, rhs);
}

QDataStream &operator>>(QDataStream &in, si::ApParkingBoxPort &rhs)
{
    return readBlob(in, rhs);
}

QDataStream &operator<<(QDataStream &out, const ap_tp::TargetPosesPort &rhs)
{
    return writeBlob(out, rhs);
}

QDataStream &operator>>(QDataStream &in, ap_tp::TargetPosesPort &rhs)
{
    return readBlob(in, rhs);
}

QDataStream& operator<<(QDataStream& out, const ap_common::Vehicle_Params &rhs)
{
    return writeBlob(out,rhs);
}

QDataStream& operator>>(QDataStream& in, ap_common::Vehicle_Params &rhs)
{
    return readBlob(in,rhs);
}
