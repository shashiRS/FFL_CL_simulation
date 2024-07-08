#include "parkingboxcollectionitem.h"

ParkingBoxCollectionItem::ParkingBoxCollectionItem(ParkingSceneModel *model)
    : mModel(model)
{
    setFlags(QGraphicsItem::ItemHasNoContents);

    connect(model->getParkingBoxesModel(), &ParkingBoxCollectionModel::parkingBoxAdded, this, &ParkingBoxCollectionItem::onBoxAdded);
    connect(model->getParkingBoxesModel(), &ParkingBoxCollectionModel::parkingBoxRemoved, this, &ParkingBoxCollectionItem::onBoxRemoved);
}

void ParkingBoxCollectionItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(painter)
    Q_UNUSED(option)
    Q_UNUSED(widget)
    // nothing to paint here, just a metaobject
}

QRectF ParkingBoxCollectionItem::boundingRect() const
{
    return QRectF();
}

void ParkingBoxCollectionItem::onBoxAdded(ParkingBoxModel *model)
{
    mBoxItems.insert(model, (new ParkingBoxItem(mModel, model, this)));
}

void ParkingBoxCollectionItem::onBoxRemoved(ParkingBoxModel *model)
{
    delete mBoxItems.take(model);
}
