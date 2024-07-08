#include <QApplication>
#include <QStyleFactory>
#include <QThread>

#include <memory>
#include <thread>

#include "overpaintedparkingscenewidget.h"

#include <parking_visu_user_lib.h>

namespace parking_visu {

class SimpleParkingVisu {
public:
    SimpleParkingVisu() {
        memset(&TargetPosesPort, 0, sizeof(TargetPosesPort));
        memset(&mCurrentEM, 0, sizeof(mCurrentEM));
        memset(&parkingBoxPort, 0, sizeof(parkingBoxPort));
        memset(&mVehParams, 0, sizeof(mVehParams));
    }

    void run() {
        int argc = 1;
        char* argv[] = { _strdup("app") };

        QApplication a(argc, argv);

        OverpaintedParkingSceneWidget widget;
        widget.setOverpaintHandler(mPaintHandler);

        ParkingSceneModel model;
        widget.setModel(&model);

        model.setVehicleParameters(mVehParams);
        model.setEnvModelPort(mCurrentEM);
        model.setTargetPosesPort(TargetPosesPort);
        model.setParkingBoxPort(parkingBoxPort);

        widget.resize(800, 800);
        widget.show();
        widget.showAll(2.0f);
        a.exec();
    }

    ap_tp::TargetPosesPort TargetPosesPort;
    si::ApEnvModelPort mCurrentEM;
    si::ApParkingBoxPort parkingBoxPort;
    ap_common::Vehicle_Params mVehParams;

    std::function<void (QPainter *)> mPaintHandler;
};

static SimpleParkingVisu _simpleParkingVisu;

void setPossibleTargetPoses(const ap_tp::TargetPosesPort &TargetPosesPort)
{
    _simpleParkingVisu.TargetPosesPort = TargetPosesPort;
}

void setEnvModelData(const si::ApEnvModelPort &envModelPort)
{
    _simpleParkingVisu.mCurrentEM = envModelPort;
}

void setParkingBoxData(const si::ApParkingBoxPort &parkingBoxPort)
{
    _simpleParkingVisu.parkingBoxPort = parkingBoxPort;
}

void setVehicleParams(const ap_common::Vehicle_Params &vehicleParams)
{
    _simpleParkingVisu.mVehParams = vehicleParams;
}

void showData()
{
    static std::thread thread;

    if(thread.get_id() == std::thread::id()) {
        thread.swap(std::thread([]() {_simpleParkingVisu.run(); }));
    }
}

void setPaintingHandler(const std::function<void (QPainter *)> &handler)
{
    _simpleParkingVisu.mPaintHandler = handler;
}

} // namespace parking_visu
