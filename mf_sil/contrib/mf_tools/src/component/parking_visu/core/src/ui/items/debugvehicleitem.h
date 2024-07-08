#ifndef DRIVINGVEHICLEITEM_H
#define DRIVINGVEHICLEITEM_H

#include <QObject>
#include <QGraphicsEllipseItem>

#include "vehicleitem.h"
#include "models/debugvehiclemodel.h"

class ArcEllipseItem;

class DebugVehicleItem : public VehicleItem
{
    Q_OBJECT
public:
    DebugVehicleItem(DebugVehicleModel* model, ParkingSceneModel* parkingModel, QGraphicsItem* parent = nullptr);

    void setCreateHistoryPointOnMouseMove(bool flag); ///< if set to true, every time this item is moved a new history point will be created

private slots:
    void onWheelAngleChanged();


private:
    virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent * evt) override;

    virtual void keyPressEvent(QKeyEvent* evt) override;
    virtual void keyReleaseEvent(QKeyEvent* evt) override;

    inline DebugVehicleModel* getModel() {
        return dynamic_cast<DebugVehicleModel*>(mModel);
    }

    ArcEllipseItem* mRotationCenterItem = nullptr;
    QGraphicsLineItem* mLineRearItem = nullptr, *mLineFrontItem = nullptr;

    ArcEllipseItem* mInnerRotationCircleItem = nullptr;
    ArcEllipseItem* mOuterRotationCircleItem = nullptr;

    bool mCreateHistoryPointOnMouseMove = false;
};

#endif // DRIVINGVEHICLEITEM_H
