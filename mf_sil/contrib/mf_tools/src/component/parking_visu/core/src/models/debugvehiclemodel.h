#ifndef DRIVINGVEHICLEMODEL_H
#define DRIVINGVEHICLEMODEL_H

#include <QObject>

#include <ap_common/vehicle_params.h>
#include "vehiclemodel.h"
#include "enumproperty.h"

#include <ap_common/ap_common_generated_types.h>
#include <ap_common_consts/ap_common_consts.h>

class DebugVehicleModel : public VehicleModel
{
    Q_OBJECT
    Q_PROPERTY(float steeringAngle READ getSteeringAngle WRITE setSteeringAngle NOTIFY steeringAngleChanged)
    Q_PROPERTY(float stepDistance READ getStepDistance WRITE setStepDistance NOTIFY stepDistanceChanged)
    Q_PROPERTY(float stepAngle READ getStepAngle WRITE setStepAngle NOTIFY stepAngleChanged)
    Q_PROPERTY(bool visible READ isVisible WRITE setVisible NOTIFY visibilityChanged)
    Q_PROPERTY(float rotationRadius READ getRotationRadius WRITE setRotationRadius NOTIFY rotationRadiusChanged)
    Q_PROPERTY(float previewTrackSpan READ previewTrackSpan WRITE setPreviewTrackSpan NOTIFY previewTrackSpanChanged)
    // read only
    Q_PROPERTY(float maxSteeringAngle READ getMaxSteeringAngle)
    Q_PROPERTY(EnumProperty DrivingDirection READ getDrivingDirection WRITE setDrivingDirectionProp)
    Q_PROPERTY(EnumProperty SteeringDirection READ getSteeringDirection WRITE setSteeringDirectionProp)

public:
    using VehicleModel::VehicleModel;

    /**
     * @brief Same as setPos(), but additionally stores pose for later reset
     * @param x
     * @param y
     * @param yaw
     */
    void setDebugPos(float x, float y, float yaw);

    /**
     * @brief Reset pose and steering angle to last stored debug pose
     */
    void resetToDebugPose();

    void moveVehicle(float distance);

    void setSteeringAngle(float angle);

    float getSteeringAngle() const {
        return mSteeringAngle;
    }

    void setStepDistance(float distance);
    float getStepDistance() const {
        return mStepDistance;
    }

    void setStepAngle(float angle);
    float getStepAngle() const {
        return mStepAngle;
    }

    virtual bool isVisible() const override;
    void setVisible(bool visible);

    void setRotationRadius(float radius);

    /**
     * @brief same as setRotationRadius(),  but additionally stores radius for later reset
     * @param radius
     */
    void setDebugRotationRadius(float radius);

    float getRotationRadius() const {
        return mRotationRadius;
    }

    inline float getMaxSteeringAngle() const {
        return radiusToAngle(ap_common_consts::AP_COMMON_Consts::AP_V_MIN_TURN_RADIUS_M + 1.1F);
    }

    EnumProperty getSteeringDirection() {
        return mSteeringDir;
    }

    void setSteeringDirection(ap_common::SteeringDirection strDir) {
        mSteeringDir = strDir;
    }

    void setSteeringDirectionProp(EnumProperty strDir) {
        mSteeringDir = static_cast<ap_common::SteeringDirection>(strDir.asInt());
    }

    EnumProperty getDrivingDirection() {
        return mDrivingDir;
    }

    void setDrivingDirection(ap_common::DrivingDirection drvDir) {
        mDrivingDir = drvDir;
    }

    void setDrivingDirectionProp(EnumProperty drvDir) {
        mDrivingDir = static_cast<ap_common::DrivingDirection>(drvDir.asInt());
    }

    float previewTrackSpan() const {
        return mPreviewTrackSpan;
    }

    void setPreviewTrackSpan(float span) {
        if(mPreviewTrackSpan != span) {
            mPreviewTrackSpan = span;
            emit previewTrackSpanChanged(mPreviewTrackSpan);
        }
    }

signals:
    void steeringAngleChanged();
    void rotationRadiusChanged();
    void stepDistanceChanged();
    void stepAngleChanged();
    void vehicleMovedByKey();
    void visibilityChanged(bool visible); /**< need to be declared in this class, see https://bugreports.qt.io/browse/QTBUG-7684 */
    void previewTrackSpanChanged(float span);

public slots:

private:
    float mSteeringAngle = 0; /**< current wheel angle **/
    float mRotationRadius = 0; /**< positive to the left, 0 if driving straight */
    float mStepDistance = 0.1f; /**< distance driven per step, e.g. when pressing UP key **/
    float mStepAngle = 0.1f; /**< angle driven per step, depends on mStepDistance and current radius. Same as mStepDistance if radius = inf. **/

    float mPreviewTrackSpan = 15.0f; /**< span of track preview in meters **/

    ap_common::SteeringDirection mSteeringDir{ ap_common::SteeringDirection::STRAIGHT };
    ap_common::DrivingDirection mDrivingDir{ ap_common::DrivingDirection::DIRECTION_UNKNOWN };

    bool mIsVisible = false;

    VehiclePose mLastDebugPose;
    float mLastDebugRadius = 0.0;

    /**
     * @brief angleToRadius
     * @param angle_rad must not be 0
     * @return
     */
    float angleToRadius(float angle_rad) const;

    /**
     * @brief radiusToAngle
     * @param radius_m must not be 0
     * @return
     */
    float radiusToAngle(float radius_m) const;

    inline bool isDrivingStraight() const {
        return std::fabs(mSteeringAngle) < 0.005;
    }
};

#endif // DRIVINGVEHICLEMODEL_H
