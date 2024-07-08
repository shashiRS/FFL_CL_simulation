#include <QGraphicsItemGroup>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QDebug>

#include <QPen>
#include <QBrush>
#include <QFont>

#include "debugdrawer.h"

#include "ui/items/debugvehicleitem.h"

DebugDrawer::DebugDrawer(QGraphicsScene* scene, QObject* parent, ParkingSceneModel *model)
    : QObject(parent)
    , mParkingModel(model)
{
    mRootItem = new QGraphicsItemGroup();
    mRootItem->setFlag(QGraphicsItem::ItemHasNoContents);

    auto debugVehicle = new DebugVehicleItem(model->getDebugVehicleModel(), model);
    scene->addItem(debugVehicle);
    debugVehicle->setZValue(4);

    mCaptionRootItem = new QGraphicsItemGroup();
    mCaptionRootItem->setFlag(QGraphicsItem::ItemHasNoContents);

    scene->addItem(mRootItem);
    scene->addItem(mCaptionRootItem);

    mRootItem->setZValue(3);
    mCaptionRootItem->setZValue(3);
}

template<class T>
std::pair<T*, QGraphicsSimpleTextItem*> DebugDrawer::getCreateItem(QString const& name) {
    auto iter = mItemMap.find(name);
    if(iter == mItemMap.end()) {
        iter = mItemMap.emplace(std::piecewise_construct,
                                std::forward_as_tuple(name),
                                std::forward_as_tuple(new T(mRootItem), new QGraphicsSimpleTextItem(name, mCaptionRootItem))
                                ).first;
    } else if(iter->second.first->type() != T::Type) {
        iter->second.first.reset(new T(mRootItem));
        iter->second.second.reset(new QGraphicsSimpleTextItem(name, mCaptionRootItem));
    }

    iter->second.second->setTransform(QTransform(0.01, 0, 0,
                                                 0, -0.01, 0,
                                                 0, 0, 1));

    return std::make_pair(dynamic_cast<T*>(iter->second.first.get()),
                          iter->second.second.get());
}

void DebugDrawer::drawPoint(const QString &name, float x, float y)
{
    auto ellipse = getCreateItem<QGraphicsEllipseItem>(name);
    ellipse.first->setPen(Qt::NoPen);
    ellipse.first->setBrush(QBrush(QColor(0,0,0)));
    constexpr float d = 0.05F;
    ellipse.first->setRect(x-d/2, y-d/2.0F, d, d);
    ellipse.second->setPos(x, y);
}

void DebugDrawer::drawLine(const QString &name, float x1, float y1, float x2, float y2)
{
    auto line = getCreateItem<QGraphicsLineItem>(name);
    QPen pen;
    pen.setWidthF(0.0);
    line.first->setPen(pen);
    line.first->setLine(x1, y1, x2, y2);
    line.second->setPos(x1, y1);
}

void DebugDrawer::drawCircle(const QString &name, float x, float y, float r)
{
    auto ellipse = getCreateItem<QGraphicsEllipseItem>(name);
    QPen pen;
    pen.setWidthF(0.0);
    ellipse.first->setPen(pen);
    ellipse.first->setBrush(Qt::NoBrush);
    ellipse.first->setRect(x-r, y-r, r*2, r*2);
    ellipse.second->setPos(x, y);
}

void DebugDrawer::drawPolygon(const QString &name, const QPolygonF &poly)
{
    auto item = getCreateItem<QGraphicsPolygonItem>(name);
    QPen pen;
    pen.setWidthF(0.0);
    item.first->setPen(pen);
    item.first->setBrush(Qt::NoBrush);
    item.first->setPolygon(poly);
    const auto bounds = poly.boundingRect();
    item.second->setPos(bounds.bottomRight());
}

void DebugDrawer::clearAll()
{
    mItemMap.clear();
}

void DebugDrawer::setCaptionsVisible(bool visible)
{
    mCaptionRootItem->setVisible(visible);
}
