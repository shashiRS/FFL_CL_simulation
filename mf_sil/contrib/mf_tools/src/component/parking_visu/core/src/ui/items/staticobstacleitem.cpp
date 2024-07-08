#include "staticobstacleitem.h"

#include <QPen>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QVector2D>
#include <QKeyEvent>
#include <QPainter>
#include <QGuiApplication>
#include <QTime>
#include <QStyleOptionGraphicsItem>
#include <QMessageBox>
#include <QGraphicsScene>
#include <qmath.h>

#include <PlanningObject.h>

static constexpr float ROTATION_DRAG_RADIUS = 0.5f;

StaticObstacleItem::StaticObstacleItem(ParkingSceneModel *parkingModel, StaticObstacleModel *model, QGraphicsItem *parent)
    : QGraphicsObject(parent)
    , mModel(model)
    , mParkingModel(parkingModel)
{
    setFlag(QGraphicsItem::ItemIsSelectable);
    setFlag(QGraphicsItem::ItemIsFocusable);
    setAcceptHoverEvents(true);

    memset(&mStructureDataCopy, 0, sizeof(mStructureDataCopy));

    onShapeChanged();

    connect(mModel, &StaticObstacleModel::shapeChanged, this, &StaticObstacleItem::onShapeChanged);
    connect(mModel, &StaticObstacleModel::visibilityChanged, this, [this](bool visible) {
        setVisible(visible);
        if (mParkingModel->getSelectedObject() == mModel) {
            setSelected(true);
        }
    });

    connect(mParkingModel, &ParkingSceneModel::selectedObjectChanged, this, &StaticObstacleItem::onSelectionChanged);
}

void StaticObstacleItem::keyPressEvent(QKeyEvent *evt)
{
    if(evt->key() == Qt::Key_Shift) {
        prepareGeometryChange();
    }
}

void StaticObstacleItem::keyReleaseEvent(QKeyEvent *evt)
{
    if(evt->key() == Qt::Key_Shift) {
        prepareGeometryChange();
    }
}

void StaticObstacleItem::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    mShapeChangedByMouse = false;

    if(isSelected() && mPolygon.size() > 0 && evt->button() == Qt::LeftButton && (evt->modifiers() & Qt::ShiftModifier)) {
        // qDebug() << "adding new point to poly";

        if(mPolygon.size() >= ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU) {
            QMessageBox::critical(nullptr, "Cannot add point!", "Cannot add point to static obstacle, since maximum number of points "
                                                              "already reached!");
            return;
        }

        addPolygonPoint(evt->pos());
    } else if(isSelected() && mPolygon.size() > 0 && evt->button() == Qt::RightButton && !(evt->modifiers() & Qt::ShiftModifier)) {
        // qDebug() << "remove point from poly";

        if(mPolygon.size() <= 3) {
            QMessageBox::critical(nullptr, "Cannot remove point!", "Cannot remove point from static obstacle, since obstacle needs to have "
                                                          "at least 3 points!");
            return;
        }

        int index = findEditedPolygonIndex(evt->pos());
        if(index >= 0) {
            prepareGeometryChange();
            mPolygon.remove(index);
            updateDataFromPolygon();
            evt->accept();
        }
    } else if(!isSelected() && evt->button() == Qt::RightButton) {
        mRotating = true;
        mRotationCenterMousePos = evt->pos();
        update();
    }
}

void StaticObstacleItem::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
    lastMousePos = evt->pos();

    const int index = (isSelected() && evt->buttons() & Qt::LeftButton)?
                findEditedPolygonIndex(evt->lastScenePos())
              : -1;

    if(index >= 0) {
        prepareGeometryChange();
        mPolygon[index] += evt->scenePos() - evt->lastScenePos();
        updateDataFromPolygon(false);
        mShapeChangedByMouse = true;
        evt->accept();
    } else {
        // no polygon endpoint was selected, check if we need to move/rotate whole polygon

        // move whole polygon by mouse
        if(evt->buttons() & Qt::LeftButton) {
            if(mPolygon.size() > 0) {
                prepareGeometryChange();
                mPolygon.translate(evt->scenePos() - evt->lastScenePos());
                updateDataFromPolygon(false);
                mShapeChangedByMouse = true;
                evt->accept();
            }
        }
        // rotate whole polygon around mRotationCenterMousePos
        else if(evt->buttons() & Qt::RightButton) {
            if(mPolygon.size() > 0) {
                auto center = evt->buttonDownPos(Qt::RightButton);
                auto dCurrent = evt->scenePos() - center;

                if(QVector2D(dCurrent).length() > ROTATION_DRAG_RADIUS) {
                    auto dPrev = evt->lastScenePos() - center;

                    auto angleCurrent = qAtan2(dCurrent.y(), dCurrent.x());
                    auto anglePrev = qAtan2(dPrev.y(), dPrev.x());

                    QTransform trans;
                    trans.translate(center.x(), center.y());
                    trans.rotateRadians(angleCurrent - anglePrev);
                    trans.translate(-center.x(), -center.y());

                    prepareGeometryChange();
                    mPolygon = trans.map(mPolygon);
                    updateDataFromPolygon(false);
                    mShapeChangedByMouse = true;
                    evt->accept();
                }
            }
        }
    }

    if(isSelected() && evt->modifiers() & Qt::ShiftModifier) {
        update();
    }
}

void StaticObstacleItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    update();
    mRotating = false;

    if(mShapeChangedByMouse) {
        mParkingModel->createHistoryPoint();
    }
}

void StaticObstacleItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt)
{
    if(evt->button() == Qt::LeftButton) {
        mParkingModel->setSelectedObject(isSelected()? nullptr: mModel);
    }

    // qDebug() << "double click";

    /*
    if(mPolygon.size() > 0 && evt->button() == Qt::LeftButton && (evt->modifiers() & Qt::ShiftModifier)) {
        qDebug() << "adding new point to poly";

        addPolygonPoint(evt->pos());
    } */
}

void StaticObstacleItem::hoverMoveEvent(QGraphicsSceneHoverEvent *evt)
{
    lastMousePos = evt->pos();
    if(isSelected()) {
        update();
    }
}

QVariant StaticObstacleItem::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemSelectedChange) {
		if (value.toBool()) {
			mParkingModel->setSelectedObject(mModel);
		}
		else if (mParkingModel->getSelectedObject() == mModel) {
			mParkingModel->setSelectedObject(nullptr);
		}
    }
    return QGraphicsItem::itemChange(change, value);
}

QRectF StaticObstacleItem::boundingRect() const
{
    auto rect = mPolygon.boundingRect();

    float margin = SELECTION_BOUNDING_MARGIN;

    //if(QGuiApplication::queryKeyboardModifiers() & Qt::ShiftModifier) {
    rect += QMarginsF(margin, margin, margin, margin);
    //}

    return rect;
}

void StaticObstacleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget)
    // qDebug() << QTime::currentTime() << "paint";

    if(mPolygon.empty()) {
        return;
    }

    painter->setPen(Qt::NoPen);
    painter->setBrush(QBrush(QColor(255,200,200, 100)));
    painter->drawPolygon(mPolygon);


#if 0
    QPointF centroid(0, 0);

    float A = 0.0;
    for(int i = 0; i < mInflatedPolygon.size(); i++) {
        const auto p1 = mInflatedPolygon[i];
        const auto p2 = mInflatedPolygon[(i+1)%mInflatedPolygon.size()];
        const auto f = p1.x()*p2.y() - p2.x()*p1.y();

        A += f;
        centroid += (p1+p2)*f;
    }

    centroid /= (3*A);


    float radius = 0;

    for(const auto& point: mInflatedPolygon) {
        radius = std::max(radius, QVector2D(point - centroid).length());
    }

    {
        QPen pen;
        pen.setWidthF(0);
        pen.setStyle(Qt::DotLine);
        painter->setPen(pen);
        painter->setBrush(Qt::NoBrush);
        painter->drawEllipse(centroid, radius, radius);
    }

#endif

    if(isSelected()) {
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(255,100, 100)));

        for(auto& p: mPolygon) {
            painter->drawEllipse(p, POLYGON_CIRCLES_RADIUS, POLYGON_CIRCLES_RADIUS);
        }

        if(QGuiApplication::queryKeyboardModifiers() & Qt::ShiftModifier && !(QGuiApplication::mouseButtons() & Qt::LeftButton)) {
            int minIndex = findPolygonIndexForAdding(lastMousePos);
            if(minIndex >= 0) {
                painter->drawEllipse(lastMousePos, POLYGON_CIRCLES_RADIUS, POLYGON_CIRCLES_RADIUS);

                painter->setBrush(Qt::NoBrush);
                QPen pen;
                pen.setWidthF(0);
                pen.setStyle(Qt::DotLine);
                painter->setPen(pen);
                painter->drawLine(mPolygon.at(minIndex), lastMousePos);
                painter->drawLine(lastMousePos, mPolygon.at((minIndex+1) % mPolygon.size()));
            }
        }



    } else {
        if(mRotating) {
            QPen pen;
            pen.setWidthF(0);
            pen.setStyle(Qt::DotLine);
            painter->setPen(pen);
            painter->drawPoint(mRotationCenterMousePos);
            painter->drawEllipse(mRotationCenterMousePos, ROTATION_DRAG_RADIUS, ROTATION_DRAG_RADIUS);
        }
    }

    QPen pen;
    pen.setWidthF(0);
    painter->setPen(pen);
    painter->setBrush(QBrush(QColor(255,200,200, 127)));
    painter->drawPolygon(mPolygon, Qt::WindingFill);



    if (option->state & QStyle::State_Selected) {
        float itemPenWidth = 0.05f;
        const qreal pad = itemPenWidth / 2;
        painter->setPen(QPen(option->palette.windowText(), 0, Qt::DotLine));
        painter->setBrush(Qt::NoBrush);
        painter->drawRect(boundingRect().adjusted(pad, pad, -pad, -pad));
    }


    if(isSelected()) {
        drawPolygonCornerNumbers(painter, mPolygon);
    }
}

bool StaticObstacleItem::contains(const QPointF &point) const
{
    //qDebug() << "contains";

    if(isSelected()) {
        return boundingRect().contains(point);
    } else {
        return mPolygon.containsPoint(point, Qt::WindingFill);
    }
}

QPainterPath StaticObstacleItem::shape() const
{
    // qDebug() << "shape";

    QPainterPath path;
    if(isSelected()) {
        path.addRect(boundingRect());
    } else {

        path.addPolygon(mPolygon);
    }

    return path;
}

void StaticObstacleItem::onShapeChanged()
{
    QPolygonF poly = polygonFromData(mModel->getStructureData());

    if(mPolygon != poly) {
        prepareGeometryChange();
        mStructureDataCopy = mModel->getStructureData();
        mPolygon = poly;
        // setVisible(mPolygon.size() > 0);
    }
}

void StaticObstacleItem::onSelectionChanged(QObject *selected)
{
    setSelected(selected == mModel);
}

void StaticObstacleItem::addPolygonPoint(const QPointF &p)
{
    int minIndex = findPolygonIndexForAdding(p);
    if(minIndex < 0)
        return;

    prepareGeometryChange();
    mPolygon.insert(minIndex+1, p);
    updateDataFromPolygon();
}

int StaticObstacleItem::findPolygonIndexForAdding(const QPointF &p) const
{
    if(mPolygon.isEmpty())
        return -1;

    if(findEditedPolygonIndex(p) >= 0) {
        return -1;
    }

    qreal minDistance = 0;
    int minIndex = -1;

    // find closest polygon line
    for(int i = 0; i < mPolygon.size(); i++) {
        qreal d = linePointDistance(QLineF(mPolygon.at(i), mPolygon.at(i+1 < mPolygon.size()? i+1 : 0)), p);
        if(minIndex < 0 || d < minDistance) {
            minDistance = d;
            minIndex = i;
        }
    }

    return minIndex;
}

qreal StaticObstacleItem::linePointDistance(const QLineF &line, const QPointF &p)
{
    QTransform trans;
    trans.scale(1.0 / line.length(), 1);
    trans.rotate(line.angle());
    trans.translate(-line.p1().x(), -line.p1().y());

    QPointF mapped = trans.map(p);
    if(mapped.x() < 0) {
        return QVector2D(line.p1() - p).length();
    } else if(mapped.x() > 1) {
        return QVector2D(line.p2() - p).length();
    } else {
        return qAbs(mapped.y());
    }
}

int StaticObstacleItem::findEditedPolygonIndex(const QPointF &pos) const
{
    for(int i = 0; i < mPolygon.size(); i++) {
        if(QVector2D(mPolygon.at(i) - pos).length() < POLYGON_CIRCLES_RADIUS) {
            return i;
        }
    }

    return -1;
}

QPolygonF StaticObstacleItem::polygonFromData(const si::StaticObjectSerializable &object)
{
    QPolygonF poly;
    poly.resize(object.objShape_m.actualSize);
    for(int i = 0; i < poly.size(); i++) {
        poly[i] = QPointF(object.objShape_m.array[i].x_dir, object.objShape_m.array[i].y_dir);
    }

    return poly;
}

void StaticObstacleItem::updateDataFromPolygon(bool saveHistory)
{
    mStructureDataCopy.objShape_m.actualSize = 0U;
    for (int i = 0; i < mPolygon.size(); i++) {
        mStructureDataCopy.objShape_m.actualSize++;
        mStructureDataCopy.objShape_m.array[i] = cml::Vec2Df_POD{ float32_t(mPolygon[i].x()), float32_t(mPolygon[i].y()) };
    }

    mModel->updateShapePolygon(mStructureDataCopy);
    if(saveHistory) {
        mParkingModel->createHistoryPoint();
    }
}

void StaticObstacleItem::drawPolygonCornerNumbers(QPainter *painter, const QPolygonF &poly)
{
    QTransform origTrans = painter->transform();

    for(int i = 0; i < poly.size(); i++) {
        const QVector2D pt(poly[i]);
        const QVector2D prev(poly[(i+poly.size()-1) % poly.size()]);
        const QVector2D next(poly[(i+1) % poly.size()]);

        // vector pointing away from polygon point
        const auto away = (((pt - next).normalized() + (pt-prev).normalized()).normalized()) * 0.1F;

        // 0.05 is approximate text height, TODO: remove hardcoded value
        painter->translate(poly[i] + away.toPointF() - QPointF(0.05F, 0.05F));
        painter->scale(0.01, -0.01); // scale to proper text size

        painter->drawText(QPointF(0, 0), QString::number(i));
        painter->setTransform(origTrans);
    }
}
