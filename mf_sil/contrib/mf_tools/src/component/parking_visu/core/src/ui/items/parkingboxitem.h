#ifndef PARKINGBOXITEM_H
#define PARKINGBOXITEM_H

#include <QObject>
#include <QGraphicsObject>
#include <QPen>

#include "models/parkingboxmodel.h"
#include "models/parkingscenemodel.h"
#include "virtuallineitem.h"

class ParkingBoxItem : public QGraphicsObject
{
    Q_OBJECT
public:
    explicit ParkingBoxItem(ParkingSceneModel* parkingModel, ParkingBoxModel* model, QGraphicsItem *parent = nullptr);

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;

    QPainterPath shape() const override;

    void mousePressEvent(QGraphicsSceneMouseEvent * evt);
    void mouseMoveEvent(QGraphicsSceneMouseEvent * evt);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt);

    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

    virtual bool contains(const QPointF &point) const;

signals:

public slots:
    void onShapeChanged();
    void selectedObjectChanged(QObject *selected);

private:
    ParkingBoxModel* mModel = nullptr;
    ParkingSceneModel* mParkingModel = nullptr;

    QPolygonF mPolygon;
    std::array<VirtualLineItem, ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU> mVirtualLines;
    QPen mMainPen;

    bool mShapeChangedByMouse = false; /**< Indicates whether the shape was changed during last mouse button pressed period*/

    static QPolygonF polygonFromData(const si::ParkingBoxSerializable &shape);

    int findEditedPolygonIndex(QPointF const& pos) const;

    void updateDataFromPolygon();

    void drawPolygonCornerNumbers(QPainter *painter, const QPolygonF &poly);

    void drawParkingBoxEdgeType(QPainter *painter, const QPolygonF &poly);
};

#endif // PARKINGBOXITEM_H
