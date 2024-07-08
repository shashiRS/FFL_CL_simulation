#ifndef STATICOBSTACLEITEM_H
#define STATICOBSTACLEITEM_H

#include <QObject>
#include <QGraphicsObject>

#include "models/parkingscenemodel.h"
#include "models/staticobstaclemodel.h"

class StaticObstacleItem: public QGraphicsObject
{
    Q_OBJECT
public:
    explicit StaticObstacleItem(ParkingSceneModel * parkingModel, StaticObstacleModel* model, QGraphicsItem *parent = nullptr);

    void keyPressEvent(QKeyEvent* evt);
    void keyReleaseEvent(QKeyEvent* evt);

    void mousePressEvent(QGraphicsSceneMouseEvent * evt);
    void mouseMoveEvent(QGraphicsSceneMouseEvent * evt);
    void mouseReleaseEvent(QGraphicsSceneMouseEvent *evt);
    void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt);
    void hoverMoveEvent(QGraphicsSceneHoverEvent * evt);

    QVariant itemChange(GraphicsItemChange change, const QVariant &value) override;

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    bool contains(const QPointF &point) const override;

    QPainterPath shape() const override;

public slots:
    void onShapeChanged();
    void onSelectionChanged(QObject *selected);

private:
    static constexpr float POLYGON_CIRCLES_RADIUS = 0.2f;
    static constexpr float SELECTION_BOUNDING_MARGIN = POLYGON_CIRCLES_RADIUS + 1.0f;

    StaticObstacleModel* mModel = nullptr;
    ParkingSceneModel* mParkingModel = nullptr;

    QPolygonF mPolygon;

    si::StaticObjectSerializable mStructureDataCopy;

    QPointF lastMousePos;

    bool mRotating = false; /**< Object is currently being rotated by right mouse button */
    QPointF mRotationCenterMousePos; /**< Center of current rotation, only valid if mRotating == true */

    bool mShapeChangedByMouse = false; /**< Indicates whether the shape was changed during last mouse button pressed period*/

    void addPolygonPoint(QPointF const& p);

    int findPolygonIndexForAdding(QPointF const& p) const;

    static qreal linePointDistance(const QLineF &line, QPointF const& p);

    int findEditedPolygonIndex(QPointF const& pos) const;

    static QPolygonF polygonFromData(const si::StaticObjectSerializable &object);

    void updateDataFromPolygon(bool saveHistory = true);

    static void drawPolygonCornerNumbers(QPainter* painter, const QPolygonF& poly);
};

#endif // STATICOBSTACLEITEM_H
