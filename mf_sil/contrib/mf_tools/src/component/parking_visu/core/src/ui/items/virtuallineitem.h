#ifndef VIRTUALLINEITEM_H
#define VIRTUALLINEITEM_H

#include <QObject>
#include <QGraphicsLineItem>
#include <QPainterPath>

#include "models/parkingscenemodel.h"
#include "models/parkingboxmodel.h"

class VirtualLineItem : public QObject, public QGraphicsLineItem
{
    Q_OBJECT
public:
    explicit VirtualLineItem(
            ParkingBoxModel* model,
            ParkingSceneModel* parkingModel,
            const uint8_t virtualLineIndex,
            QGraphicsItem *parent = nullptr);

    VirtualLineItem(const VirtualLineItem &vlItem);

    void mousePressEvent(QGraphicsSceneMouseEvent * evt);
    void mouseMoveEvent(QGraphicsSceneMouseEvent * evt);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent * evt);

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * evt);

    QPainterPath shape() const;
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

signals:

public slots:
    void onVirtualLineChanged();

private:
    ParkingBoxModel* mModel;
    ParkingSceneModel* mParkingModel;

    uint8_t mThisLineIndex;
    QPainterPath mShape;

    bool mShapeChangedByMouse = false; /**< Indicates whether the shape was changed during last mouse button pressed period*/
};

#endif // VIRTUALLINEITEM_H
