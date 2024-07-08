#ifndef PARKINGBOXCOLLECTIONITEM_H
#define PARKINGBOXCOLLECTIONITEM_H

#include <QGraphicsObject>

#include "parkingboxitem.h"
#include "models/parkingscenemodel.h"

class ParkingBoxCollectionItem : public QGraphicsObject
{
    Q_OBJECT
public:
    explicit ParkingBoxCollectionItem(ParkingSceneModel* model);

    void paint(QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget = 0) override;
    QRectF boundingRect() const override;

signals:

public slots:
    void onBoxAdded(ParkingBoxModel* model);
    void onBoxRemoved(ParkingBoxModel* model);

private:
    ParkingSceneModel* mModel = nullptr;

    QMap<ParkingBoxModel*, ParkingBoxItem*> mBoxItems;
};

#endif // PARKINGBOXCOLLECTIONITEM_H
