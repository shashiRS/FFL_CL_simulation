#ifndef VEHICLEGHOSTITEM_H
#define VEHICLEGHOSTITEM_H

#include <QObject>

#include <QGraphicsPolygonItem>
#include <QPolygon>
#include "models/parkingscenemodel.h"
#include "models/trajectorysetmodel.h"

class VehicleGhostItem : public QObject, public QGraphicsPolygonItem
{
    Q_OBJECT
public:
    explicit VehicleGhostItem(ParkingSceneModel* parkingModel, QObject *parent = nullptr);

signals:

public slots:
    void onPathChanged();
    void onVehicleParamsChanged(ap_common::Vehicle_Params const& params);
    void onVehInflRadiusChanged(float radius);
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:
    ParkingSceneModel* mParkingModel = nullptr;

    QGraphicsPolygonItem* mBoundingBoxItem = nullptr;

    int mPathIndex = 0;
};

#endif // VEHICLEGHOSTITEM_H
