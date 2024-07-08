#ifndef PARKINGVISUCLIENT_H
#define PARKINGVISUCLIENT_H

#include <QObject>
#include <QLocalSocket>
#include <QDataStream>
#include <QSharedMemory>

#include <functional>

#include <BackendDllHandler.h>
#include <visuclientinterface.h>

#include <ParkingPath.h>

/**
 * Commands to send over pipe
 *
 * CLIENT_... are commands from frontend to backend
 * SERVER_... are commands from backend to the frontend
 */
enum class ParkingSocketCommand {
    CLIENT_LOAD_DLL = 1,
    CLIENT_START_SCAN_MODE_TRJPLA,
    CLIENT_RUN_FOR_SCAN_TRJPLA,
    CLIENT_PLAN_NEW_PATH,
    CLIENT_REPLAN_PATH,
    CLIENT_MAKE_STEP,
    CLIENT_JUMP,
    CLIENT_PLAN_GEOM_PATH,
    CLIENT_FIND_TARGET_POSE_REACHABLE_AREA,
    CLIENT_START_SCAN_MODE_TAPOSD,
    CLIENT_PLAN_GARAGE_PARK,

    SERVER_UPDATE_PATH = 1001,
    SERVER_PLANNING_FAILED,
    SERVER_UPDATE_EGOPOSE,
    SERVER_UPDATE_GEOM_TRAJECTORY,
    SERVER_UPDATE_POLYNOM_TRAJECTORY,
    SERVER_CONT_SCAN,
    SERVER_OFFER_PARK_AND_CONT_SCAN,
    SERVER_FORM_REACHABLE_TARGET_POSE_AREA,
    SERVER_GET_REACHABLE_TARGET_POSE_AREA,
    SERVER_UPDATE_EM,
    SERVER_UPDATE_PARKBOX,
    SERVER_UPDATE_TARGET_POSES,
    SERVER_UPDATE_TAPOSD,
    SERVER_UPDATE_VEHICLE_INFLATION,
    SERVER_UPDATE_VEHICLE_PARAMS,

    SERVER_DRAW_POINT = 2001,
    SERVER_DRAW_LINE,
    SERVER_DRAW_CIRCLE,
    SERVER_DRAW_VEHICLE,
    SERVER_DRAW_VEHICLE_RADIUS,
    SERVER_DRAW_POLYGON,
    SERVER_DRAW_PATH,
    SERVER_CLEAR_DRAW,
    SERVER_DRAW_GEOM_PATH,
};

class StreamCommunicator {
public:
    using MessageHandler = std::function<void(ParkingSocketCommand, QDataStream&)>;

    StreamCommunicator(QLocalSocket* device, const MessageHandler& handler);

    ~StreamCommunicator();

    void checkReceived();

    template<class ...Args>
    void sendCommand(ParkingSocketCommand cmd, Args&& ...args) {
        if (!mDevice->isOpen()) {
            return;
        }

        QByteArray block;
        QDataStream out(&block, QIODevice::WriteOnly);
        out.setVersion(QDataStream::Qt_5_12);
        out << quint32(cmd);

        // some crazy hack to call out << args[i] for every arg
        // see https://stackoverflow.com/questions/27375089/what-is-the-easiest-way-to-print-a-variadic-parameter-pack-using-stdostream
        using expander = int[];
        (void)expander{0, (void(out << std::forward<Args>(args)), 0)...};

        uint32_t size = block.size();
        static_assert(sizeof(size) == 4, "sizeof(uint32 size) is not 4");
        mDevice->write((char*)&size, 4);
        mDevice->write(block);
        mDevice->flush();
        mDevice->waitForBytesWritten(-1);
    }

private:
    QLocalSocket* mDevice;
    uint32_t mCurrentBlockSize = 0;

    MessageHandler mMessageHandler;
};

struct SimpleParkPath {
    std::array<ap_tp::SampledPose, ap_tp::AP_TP_Const::AP_P_MAX_NUM_POSES_IN_PATH_NU> sampledPoses;
    int numPosesInParkPath = 0;
};

struct ServerMemory {
    ap_tp::FC_TRJPLA_Params         trjplaParams;
    ap_tp::FC_TAPOSD_Params         taposdParams;
    ap_common::Vehicle_Params       vehicleParams;
    ap_common::Sys_Func_Params      sysFuncParams;

    si::ApEnvModelPort              emData;
    si::EgoMotionPort               egoPort;
    si::ApParkingBoxPort            parkingBoxPort;

    ap_tp::PlannedTrajPort          plannedTrajectory;
    ap_tp::TargetPosesPort          TargetPoses;
    ap_tp::TrjplaMetaData           trjplaMetaData;
    ap_tp::TrajPlanDebugPort        trjplaDebugPort;
    ap_tp::TrajPlanVisuPort         trjplaVisuPort;
    ap_tp::TAPOSDDebugPort          taposdDebugPort;

    SimpleParkPath                  parkPath;
    //ap_tp::ParkingPath              parkingPath;
    //ap_tp::GeomPathRepresentation   geomPath;
    //ap_tp::GarageParkingPath        garagePath;

    bool dllLoaded = false;             /// flag set by client process to indicate if planner DLL was loaded
    bool debugDrawEnabled = false;      /// flag set by server if false, draw...() commands are ignored and not sent.
    bool scanModeEnabled = false;       /// flag enables scan mode
    double reqDistToStop = 0.0;         /// double for jumpOnPath feature
    int selectedTargetPoseIndex = 0;	/// Index of in GUI selected target pose
    std::array<float, 2U> selectedTpDeviations{ 0.0f, 0.0f };   /// allowed deviations for selected target pose
};

QDataStream& operator<<(QDataStream& out, const ap_tp::ParkingPath& rhs);
QDataStream& operator>>(QDataStream& in, ap_tp::ParkingPath& rhs);

QDataStream& operator<<(QDataStream& out, const ap_tp::GeometricPath& rhs);
QDataStream& operator>>(QDataStream& in, ap_tp::GeometricPath& rhs);

QDataStream& operator<<(QDataStream& out, const si::ApEnvModelPort& rhs);
QDataStream& operator>>(QDataStream& in, si::ApEnvModelPort& rhs);

QDataStream& operator<<(QDataStream& out, const si::ApParkingBoxPort& rhs);
QDataStream& operator>>(QDataStream& in, si::ApParkingBoxPort& rhs);

QDataStream& operator<<(QDataStream& out, const ap_tp::TargetPosesPort& rhs);
QDataStream& operator>>(QDataStream& in, ap_tp::TargetPosesPort& rhs);

QDataStream& operator<<(QDataStream& out, const ap_common::Vehicle_Params& rhs);
QDataStream& operator>>(QDataStream& in, ap_common::Vehicle_Params& rhs);

/**
 * Represents a backend for connecting to the GUI application.
 *
 * Can be used by external components (such as gtest executables) to
 * actively make VISU_DRAW_... calls from within internal code.
 *
 * Usage:
 *
 * VisuClient client;
 * client.connectToServer();
 * VisuClientInterface::setInterface(&client);
 *
 * ...
 *
 * VISU_DRAW_CIRCLE(1, 2, 5, "circle");
 */
class VisuClient: public QObject, public VisuClientInterface
{
    Q_OBJECT
public:
    explicit VisuClient(QObject *parent = nullptr);

    void connectToServer();

    bool isConnected();

    virtual void clearDraw() override;

    virtual void drawPoint(float x, float y, const std::string &name) override;
    virtual void drawLine(float x1, float y1, float x2, float y2, const std::string &name) override;
    virtual void drawCircle(float x, float y, float r, const std::string &name) override;
    virtual void drawPolygon(const cml::Vec2Df* begin, const cml::Vec2Df* end, const std::string &name) override;

    virtual void drawDebugVehicle(float x, float y, float yaw) override;
    virtual void drawDebugVehicleRadius(float radius) override;

    virtual void drawPath(const ap_tp::ParkingPath &path, const std::string &name) override;
    virtual void drawPathSet(const std::list<ap_tp::ParkingPath> &pathSet, const std::string &name) override;

    virtual void drawGeomPath(const ap_tp::GeometricPath &geomPath, const std::string &name) override;
    virtual void drawManeuverSet(const ap_tp::ManeuverSet& meneuverSet, const std::string& name) override;

    virtual void updateEM(const si::ApEnvModelPort& em) override;
    virtual void updateParkBox(const si::ApParkingBoxPort &pbPort) override;
    virtual void updateEgoPort(const si::ApEnvModelPort& emData, const si::EgoMotionPort& egoPort) override;
    virtual void updateTargetPosesPort(const ap_tp::TargetPosesPort& targetPoses) override;

    virtual void updateVehicleInflation(float radius) override;
    virtual void updateVehicleParams(const ap_common::Vehicle_Params& params) override;

signals:

public slots:

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onSocketError(QLocalSocket::LocalSocketError socketError);

protected:
    StreamCommunicator mComm;
    QLocalSocket mSocket;

    QSharedMemory mServerMemory;

    virtual void handleCommand(ParkingSocketCommand cmd, QDataStream& in);

    template<class ...Args>
    inline void sendDrawCommand(ParkingSocketCommand cmd, Args&& ...args) {
        const auto memoryAttached{ mServerMemory.isAttached() };
        if(memoryAttached) {
            const auto memoryData { (const ServerMemory*)mServerMemory.data() };
            if (memoryData->debugDrawEnabled) {
                mComm.sendCommand(cmd, std::forward<Args>(args)...);
            }
        }
    }
};

class ParkingVisuClient : public VisuClient
{
    Q_OBJECT
public:
    using VisuClient::VisuClient;

private:

    BackendDllHandler mDllHandler;

    virtual void handleCommand(ParkingSocketCommand cmd, QDataStream& in) override;

    void findTargetPoseReachableArea(const float32_t xStart, const float32_t xEnd,const float32_t xStep, const float32_t yStart, const float32_t yEnd, const float32_t yStep,
                                     const float32_t yawAngleRadStart, const float32_t yawAngleRadEnd, const float32_t yawAngleRadStep,
                                     bool isReplanning, bool is3D);

    void startScanModeTrjpla();

    void runForScanTrjpla();

    void startAndRunScanModeTaposd();

    void startPlanNewPath();

    void planPath(bool rePlanning);

    void planGaragePath(bool isReplanning);

    void startReplan();

    void startStep();

    void startJump();

    void setTrjplaServerData();
};

#endif // PARKINGVISUCLIENT_H
