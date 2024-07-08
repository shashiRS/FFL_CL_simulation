#define _USE_MATH_DEFINES

#include "vehiclemodel.h"
#include "parkingscenemodel.h"
#include <geoml/LSM_Math.h>
#include <cmath>

VehicleModel::VehicleModel(ParkingSceneModel *parkingSceneModel, QObject *parent)
    : QObject(parent)
    , mParkingSceneModel(parkingSceneModel)
    , mPose{0, 0, 0}
{
}

void VehicleModel::setPosX_m(float val)
{
    mPose.Pos().x() = val;
    emit posXChanged();
    emit positionChanged();
}

float VehicleModel::getPosX_m() const
{
    return mPose.Pos().x();
}

void VehicleModel::setPosY_m(float val)
{
    mPose.Pos().y() = val;
    emit posYChanged();
    emit positionChanged();
}

float VehicleModel::getPosY_m() const
{
    return mPose.Pos().y();
}

void VehicleModel::setPosXY_m(float x, float y)
{

    mPose.Pos() = cml::Vec2Df(x, y);

    emit posXChanged();
    emit posYChanged();
    emit positionChanged();
}

QPointF VehicleModel::getPosXY_m() const
{
    return QPointF(mPose.Pos().x(), mPose.Pos().y());
}

bool VehicleModel::setPos(float x, float y, float yaw)
{
    if(mPose.Pos().x() != x || mPose.Pos().y() != y || mPose.Yaw_rad() != yaw) {
        mPose = { {x, y}, yaw };

        emit posXChanged();
        emit posYChanged();
        emit posYawChanged();
        emit positionChanged();
        return true;
    }
    return false;
}

bool VehicleModel::setPos(const QTransform &trans)
{
    return setPos(trans.dx(), trans.dy(), std::atan2(trans.m12(), trans.m11()));
}

void VehicleModel::setPosYawAngle_Rad(float val)
{
    val = LSM_GEOML::radMod(val);

    if(val != mPose.Yaw_rad()) {
        mPose.Yaw_rad() = val;

        emit posYawChanged();
        emit positionChanged();
    }
}

float VehicleModel::getPosYawAngle_Rad() const
{
    return mPose.Yaw_rad();
}

float VehicleModel::getWheelbase() const {
    return mParkingSceneModel->getVehicleParams().AP_V_WHEELBASE_M;
}

float VehicleModel::getFrontTrack() const {
    return mParkingSceneModel->getVehicleParams().AP_V_TRACK_FRONT_M;
}

/*
void VehiclePositionModel::setFrontWheelAngle_Rad(float val)
{
    m_frontWheelAngle_rad = val;
    emit positionChanged();
}

float VehiclePositionModel::getFrontWheelAngle_Rad() const
{
    return m_frontWheelAngle_rad;
}

*/
