#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include "debugvehiclemodel.h"
#include "parkingscenemodel.h"

#include <QTransform>
#include <QDebug>

#if 0
DebugVehicleModel::DebugVehicleModel(QObject *parent)
    : VehicleModel(parent)
{
    memset(&mVehicleParameters, 0, sizeof(mVehicleParameters));

    mVehicleParameters.AP_V_WHEELBASE_M = 2.786f; // some default value
    mVehicleParameters.AP_V_MAX_STEER_ANG_RAD = 0.595f; // some default value
    mVehicleParameters.AP_V_TRACK_FRONT_M = 1.586f; // some default value
}

#endif

void DebugVehicleModel::setDebugPos(float x, float y, float yaw)
{
    mLastDebugPose = {{x, y}, yaw};

    setPos(x, y, yaw);
}

void DebugVehicleModel::resetToDebugPose()
{
    setPos(mLastDebugPose.Pos().x(), mLastDebugPose.Pos().y(), mLastDebugPose.Yaw_rad());
    setRotationRadius(mLastDebugRadius);
}

void DebugVehicleModel::moveVehicle(float distance)
{
    QTransform trans;
    trans.translate(getPosX_m(), getPosY_m());
    trans.rotateRadians(getPosYawAngle_Rad());

    // drive forward
    if(std::fabs(mSteeringAngle) < 0.005) { // TODO
        setPos(trans.translate(distance, 0));
    }
    // rotate around rotation point
    else {
        float rotationAngle = distance / mRotationRadius;
        // qDebug() << "mWheelAngle" << mWheelAngle << "radius" << radius << "rotationAngle" << rotationAngle;
        setPos(trans.translate(0, mRotationRadius).rotateRadians(rotationAngle).translate(0, -mRotationRadius));
    }

    emit vehicleMovedByKey();
}

void DebugVehicleModel::setSteeringAngle(float angle)
{
    if(mSteeringAngle != angle) {
        mSteeringAngle = angle;
        if(isDrivingStraight()) // TODO
            mRotationRadius = 0;
        else
            mRotationRadius = angleToRadius(mSteeringAngle);

        mStepAngle = isDrivingStraight()? mStepDistance : mStepDistance / std::abs(getRotationRadius());

        emit steeringAngleChanged();
        emit rotationRadiusChanged();
        emit stepAngleChanged();
    }
}

void DebugVehicleModel::setRotationRadius(float radius)
{
    if(radius != mRotationRadius) {
        mRotationRadius = radius;
        mSteeringAngle = radiusToAngle(radius);
        mStepAngle = isDrivingStraight()? mStepDistance : mStepDistance / std::abs(getRotationRadius());

        emit steeringAngleChanged();
        emit rotationRadiusChanged();
        emit stepAngleChanged();
    }
}

void DebugVehicleModel::setDebugRotationRadius(float radius)
{
    mLastDebugRadius = radius;
    setRotationRadius(radius);
}


#if 0
void DebugVehicleModel::setVehicleParameters(const ap_common::Vehicle_Params &params)
{
    mVehicleParameters = params;
    setSteeringAngle(mSteeringAngle);
}
#endif

float DebugVehicleModel::angleToRadius(float angle_rad) const {
    return std::tan(M_PI_2-angle_rad) * mParkingSceneModel->getVehicleParams().AP_V_WHEELBASE_M; // positive to left
}

float DebugVehicleModel::radiusToAngle(float radius_m) const {
    if(std::abs(radius_m) < 0.005)
        return 0.0F;
    else
        return -std::atan(radius_m/mParkingSceneModel->getVehicleParams().AP_V_WHEELBASE_M) + (radius_m > 0? M_PI_2 : -M_PI_2);
}

void DebugVehicleModel::setStepDistance(float distance)
{
    if(mStepDistance != distance) {
        mStepDistance = distance;
        mStepAngle = isDrivingStraight()? mStepDistance : mStepDistance / std::abs(getRotationRadius());

        emit stepDistanceChanged();
        emit stepAngleChanged();
    }
}

void DebugVehicleModel::setStepAngle(float angle)
{
    if(mStepAngle != angle) {
        mStepAngle = angle;
        mStepDistance = isDrivingStraight()? mStepAngle : mStepAngle * std::abs(getRotationRadius());

        emit stepDistanceChanged();
        emit stepAngleChanged();
    }
}

bool DebugVehicleModel::isVisible() const
{
    return mIsVisible;
}

void DebugVehicleModel::setVisible(bool visible)
{
    if(mIsVisible != visible) {
        mIsVisible = visible;
        emit visibilityChanged(mIsVisible);
    }
}
