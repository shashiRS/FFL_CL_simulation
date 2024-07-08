#include "targetposemodel.h"

#include <QDebug>

TargetPoseModel::TargetPoseModel(ParkingSceneModel *parkingSceneModel, QObject *parent):
    VehicleModel(parkingSceneModel, parent)
{
    memset(&mTargetPoseModel, 0, sizeof(mTargetPoseModel));
    mPoseValid = false;
    mMaxAllowedDeviations = { 0.0f, 0.0f };
}

void TargetPoseModel::setPoseData(ap_tp::TargetPose const& data)
{
    mTargetPoseModel = data;
    setPos(data.pose.x_dir, data.pose.y_dir, data.pose.yaw_rad);
    emit visibilityChanged(mPoseValid);
    emit poseTypeChanged(mTargetPoseModel.type);
}

const ap_tp::TargetPose &TargetPoseModel::getPoseData()
{
    mTargetPoseModel.pose.x_dir = getPosX_m();
    mTargetPoseModel.pose.y_dir = getPosY_m();
    mTargetPoseModel.pose.yaw_rad = getPosYawAngle_Rad();
    return mTargetPoseModel;
}

void TargetPoseModel::setPoseReachableStatusProp(const EnumProperty& prop) {
    if(mTargetPoseModel.reachableStatus != (ap_tp::PoseReachableStatus)prop.asInt()) {
		mTargetPoseModel.reachableStatus = (ap_tp::PoseReachableStatus)prop.asInt();
    }
}

void TargetPoseModel::setPoseValid(bool valid) {
    if (mPoseValid != valid) {
        mPoseValid = valid;
        emit visibilityChanged(mPoseValid);
    }
}
