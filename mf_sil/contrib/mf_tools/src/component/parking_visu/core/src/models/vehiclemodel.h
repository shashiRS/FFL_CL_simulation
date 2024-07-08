#ifndef VEHICLEPOSITIONMODEL_H
#define VEHICLEPOSITIONMODEL_H

#include <QObject>
#include <QPolygonF>
#include <QTransform>
#include <geoml/CoordinateTransformer2D.h>

class ParkingSceneModel;

/**
 * @brief Contains the high-res vehicle shape and the
 * simplified bounding box.
 */
#if 0
struct VehicleShape
{
    QPolygonF boundingBox; /**< simplified bounding box from ap_common::Vehicle_Params::AP_V_BOUNDINGBOX_X_M /...Y_M **/
    QPolygonF standardShape; /**< full shape from ap_common::Vehicle_Params::AP_V_STANDARD_SHAPE_X_M / ...Y_M **/
};
#endif

class VehicleModel: public QObject
{
    Q_OBJECT

    Q_PROPERTY(float posX READ getPosX_m WRITE setPosX_m NOTIFY posXChanged)
    Q_PROPERTY(float posY READ getPosY_m WRITE setPosY_m NOTIFY posYChanged)
    Q_PROPERTY(float yawAngle READ getPosYawAngle_Rad WRITE setPosYawAngle_Rad NOTIFY posYawChanged)

public:
    using VehiclePose = LSM_GEOML::Pose;

    VehicleModel(ParkingSceneModel* parkingSceneModel,  QObject* parent = nullptr);

    virtual bool isVisible() const {return true;}

    void setPosX_m(float val);
    float getPosX_m() const;

    void setPosY_m(float val);
    float getPosY_m() const;

    void setPosXY_m(float x, float y);
    QPointF getPosXY_m() const;

    bool setPos(float x, float y, float yaw);

    /**
     * @brief Set vehicle position.
     * @param trans transformation matrix, only translation and rotation allowed, no scaling or shearing
     */
    bool setPos(const QTransform& trans);

    void setPosYawAngle_Rad(float val);
    float getPosYawAngle_Rad() const;

    //void setFrontWheelAngle_Rad(float val);
    //float getFrontWheelAngle_Rad() const;

    VehiclePose getPose() const {
        return mPose;
    }


    float getWheelbase() const;

    float getFrontTrack() const;

signals:
    void posXChanged();
    void posYChanged();
    void posYawChanged();

    /**
     * @brief Any position property (x,y,yaw) changed
     */
    void positionChanged();
    void shapeChanged();
    void visibilityChanged(bool visible);

protected:
    ParkingSceneModel* mParkingSceneModel = nullptr;
    
    LSM_GEOML::Pose mPose;
};

#endif // VEHICLEPOSITIONMODEL_H
