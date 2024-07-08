#ifndef TARGETPOSEITEM_H
#define TARGETPOSEITEM_H

#include "vehicleitem.h"

#include "models/targetposemodel.h"

class TargetPoseItem : public VehicleItem
{
    Q_OBJECT
public:
    TargetPoseItem(TargetPoseModel *tpModel, ParkingSceneModel* parkingModel);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    void mouseReleaseEvent(QGraphicsSceneMouseEvent * evt) override;

public slots:
    void onSelectedTargetPoseChanged(int prev, int current);

private:
    bool mIsSelectedTargetPose = false;

    inline TargetPoseModel* getModel() {
        return dynamic_cast<TargetPoseModel*>(mModel);
    }
};

#endif // TARGETPOSEITEM_H
