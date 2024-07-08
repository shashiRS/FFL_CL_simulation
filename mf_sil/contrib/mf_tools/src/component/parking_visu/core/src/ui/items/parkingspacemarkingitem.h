#ifndef PARKINGSPACEMARKINGITEM_H
#define PARKINGSPACEMARKINGITEM_H

#ifndef ULTRASONIC_ONLY

#include <QObject>
#include <QGraphicsLineItem>
#include <QPainterPath>

#include "models/parkingscenemodel.h"
#include "models/parkingspacemarkingmodel.h"

class ParkingSpaceMarkingItem : public QObject, public QGraphicsLineItem
{
    Q_OBJECT
public:
    explicit ParkingSpaceMarkingItem(ParkingSpaceMarkingModel* model, ParkingSceneModel* parkingModel, QGraphicsItem *parent = nullptr);

    void mousePressEvent(QGraphicsSceneMouseEvent * evt);
    void mouseMoveEvent(QGraphicsSceneMouseEvent * evt);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent * evt);

    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * evt);

    QPainterPath shape() const;
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    QVariant itemChange(GraphicsItemChange change, const QVariant &value);

signals:

public slots:
    void onMarkingChanged();
    void onSelectedObjectChanged(QObject* selected);

private:
    ParkingSpaceMarkingModel* mModel;
    ParkingSceneModel* mParkingModel;

    QPainterPath mShape;

    bool mShapeChangedByMouse = false; /**< Indicates whether the shape was changed during last mouse button pressed period*/
};
#endif // ULTRASONIC_ONLY
#endif // PARKINGSPACEMARKINGITEM_H
