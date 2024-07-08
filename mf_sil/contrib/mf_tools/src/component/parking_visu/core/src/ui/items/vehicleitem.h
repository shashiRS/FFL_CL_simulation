#ifndef VEHICLEITEM_H
#define VEHICLEITEM_H

#include <QGraphicsPolygonItem>
#include <QPolygon>

#include "models/vehiclemodel.h"
#include "models/parkingscenemodel.h"
#include <mf_common/EgoVehicle.h>
#include <ap_common/vehicle_params.h>

class VehicleItem : public QObject, public QGraphicsPolygonItem
{
    Q_OBJECT
public:
    VehicleItem(VehicleModel* model, ParkingSceneModel* parkingModel, QGraphicsItem* parent = nullptr);

    void mousePressEvent(QGraphicsSceneMouseEvent * evt);
    void mouseMoveEvent(QGraphicsSceneMouseEvent * evt);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent * evt);

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * evt);

    virtual void keyPressEvent(QKeyEvent* evt) override;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;

    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

    /**
     * @brief Sets the brush color of this polygon item and the child bounding box item
     * @param color
     */
    void setVehicleColor(QColor color);

    QPolygonF getBoundingShape() const {
        return mBoundingBoxItem->polygon();
    }

    QPolygonF getStandardShape() const {
        return this->polygon();
    }

    static QPolygonF calcShapeFromArray(const float *xValues, const float *yValues, lsm_geoml::size_type count);

    static QPolygonF calcBoundingShapeFromVehicleParams(const ap_common::Vehicle_Params& params);

    static QPolygonF calcStandardShapeFromVehicleParams(const ap_common::Vehicle_Params& params);

public slots:
    void onVehiclePositionChanged();
    void onVehicleParamsChanged(const ap_common::Vehicle_Params& params);
    void onVisibilityChanged(bool visible);
    void onVehInflRadiusChanged(float radius);

    void onSelectionChanged(QObject *selected);

    void reloadVehicleShape();

signals:

protected:
    /*
    * Updates the wheel inflations of vehPart whereas only wheel vehicle parts are allowed as input.
    */
    void updateWheelInflation(ap_common::EgoVehicle& egoVeh, ap_common::VehiclePart vehPart);

    VehicleModel* mModel;
    ParkingSceneModel* mParkingModel;

    QGraphicsPolygonItem* mBoundingBoxItem = nullptr;

    bool mVehicleMovedByMouse = false; /**< indicates whether vehicle position was changed by last mouse button pressed period */

    QGraphicsPolygonItem* mFrontLeftWheelItem = nullptr;
    QGraphicsPolygonItem* mFrontRightWheelItem = nullptr;

    QGraphicsPolygonItem* mRearLeftWheelItem = nullptr;
    QGraphicsPolygonItem* mRearRightWheelItem = nullptr;

    QGraphicsPolygonItem* mFrontLeftWheelBoundingBoxItem = nullptr;
    QGraphicsPolygonItem* mFrontRightWheelBoundingBoxItem = nullptr;
    QGraphicsPolygonItem* mRearLeftWheelBoundingBoxItem = nullptr;
    QGraphicsPolygonItem* mRearRightWheelBoundingBoxItem = nullptr;

    QGraphicsPolygonItem* mTrailerHitch = nullptr;
    QGraphicsPolygonItem* mLeftMirror = nullptr;
    QGraphicsPolygonItem* mRightMirror = nullptr;

    void createWheelItems();
};

#endif // VEHICLEITEM_H
