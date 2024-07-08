#ifndef TARGETPOSEMODEL_H
#define TARGETPOSEMODEL_H

#include <ap_tp/ap_tp_generated_types.h>

#include "vehiclemodel.h"

#include "models/enumproperty.h"

class TargetPoseModel : public VehicleModel
{
    Q_OBJECT

    Q_PROPERTY(EnumProperty poseType READ poseType WRITE setPoseTypeProp NOTIFY poseTypeChanged)
    Q_PROPERTY(EnumProperty poseFailReason READ poseFailReason WRITE setPoseFailReasonProp)
    Q_PROPERTY(EnumProperty targetSide READ targetSide WRITE setTargetSideProp)
    Q_PROPERTY(bool poseValid READ isPoseValid WRITE setPoseValid)
	Q_PROPERTY(EnumProperty reachableStatus READ reachableStatus WRITE setPoseReachableStatusProp)
    Q_PROPERTY(float rightDeviationMax READ getRightDeviation WRITE setRightDeviation)
    Q_PROPERTY(float leftDeviationMax READ getLeftDeviation WRITE setLeftDeviation)

public:
    TargetPoseModel(ParkingSceneModel* parkingSceneModel, QObject* parent = nullptr);

    virtual bool isVisible() const override {
         return mPoseValid;
    }

    const float getRightDeviation() const {
        return mMaxAllowedDeviations[0];
    }

    void setRightDeviation(float32_t rightDeviation) {
        mMaxAllowedDeviations[0] = rightDeviation;
    }

    const float getLeftDeviation() const {
        return mMaxAllowedDeviations[1];
    }

    void setLeftDeviation(float32_t leftDeviation) {
        mMaxAllowedDeviations[1] = leftDeviation;
    }

    const std::array<float, 2U>* getMaxAllowedDeviations() {
        return &mMaxAllowedDeviations;
    }

    void setPoseData(const ap_tp::TargetPose &data);

	const ap_tp::TargetPose& getPoseData();

    ap_tp::PoseType getPoseType() const {
        return mTargetPoseModel.type;
    }

    EnumProperty poseType() const {
        return mTargetPoseModel.type;
    }

    void setPoseType(ap_tp::PoseType tp) {
        mTargetPoseModel.type = tp;
        emit poseTypeChanged(tp);
    }

    void setPoseTypeProp(const EnumProperty& prop) {
        mTargetPoseModel.type = (ap_tp::PoseType)prop.asInt(); // TODO
        emit poseTypeChanged(prop);
    }

    ap_tp::PoseFailReason getPoseFailReason() const {
        return mTargetPoseModel.poseFailReason;
    }

    EnumProperty poseFailReason() const {
        return mTargetPoseModel.poseFailReason;
    }


    void setPoseFailReason(ap_tp::PoseFailReason pfr) {
        mTargetPoseModel.poseFailReason = pfr;
    }

    void setPoseFailReasonProp(const EnumProperty& prop) {
        mTargetPoseModel.poseFailReason = (ap_tp::PoseFailReason)prop.asInt(); // TODO
    }

    ap_tp::TargetSide getTargetSide() const {
        return mTargetPoseModel.targetSide;
    }

    EnumProperty targetSide() const {
        return mTargetPoseModel.targetSide;
    }

    void setTargetSide(ap_tp::TargetSide tp) {
        mTargetPoseModel.targetSide = tp;
    }

    void setTargetSideProp(const EnumProperty& prop) {
        mTargetPoseModel.targetSide = (ap_tp::TargetSide)prop.asInt(); // TODO
    }

    bool isPoseValid() const {
        return mPoseValid;
    }

    void setPoseValid(bool valid);

    ap_tp::PoseReachableStatus getPoseReachableStatus() const {
		return mTargetPoseModel.reachableStatus;
	}

	EnumProperty reachableStatus() const {
		return mTargetPoseModel.reachableStatus;
	}

	void setPoseReachableStatus(ap_tp::PoseReachableStatus tp) {
		mTargetPoseModel.reachableStatus = tp;
	}

	void setPoseReachableStatusProp(const EnumProperty& prop);

signals:
    void poseTypeChanged(EnumProperty newPoseType);

private:
    ap_tp::TargetPose mTargetPoseModel; // position and angle are stored in parent class attributes
    bool mPoseValid;
    std::array<float, 2U> mMaxAllowedDeviations; // max allowed target pose deviations
};
#endif // TARGETPOSEMODEL_H
