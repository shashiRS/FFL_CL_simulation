#include "parkingboxitem.h"

#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QGraphicsSceneMouseEvent>
#include <QVector2D>

static constexpr float POLYGON_CIRCLES_RADIUS = 0.2f;
static constexpr float SELECTION_BOUNDING_MARGIN = POLYGON_CIRCLES_RADIUS + 1.0f;


/// copied from original qt graphicsitem code
static QPainterPath shapeFromPath(const QPainterPath &path, const QPen &pen)
{
    // We unfortunately need this hack as QPainterPathStroker will set a width of 1.0
    // if we pass a value of 0.0 to QPainterPathStroker::setWidth()
    const qreal penWidthZero = qreal(0.00000001);
    if (path == QPainterPath() || pen == Qt::NoPen)
        return path;
    QPainterPathStroker ps;
    ps.setCapStyle(pen.capStyle());
    if (pen.widthF() <= 0.0)
        ps.setWidth(penWidthZero);
    else
        ps.setWidth(pen.widthF());
    ps.setJoinStyle(pen.joinStyle());
    ps.setMiterLimit(pen.miterLimit());
    QPainterPath p = ps.createStroke(path);
    p.addPath(path);
    return p;
}

ParkingBoxItem::ParkingBoxItem(ParkingSceneModel *parkingModel, ParkingBoxModel *model, QGraphicsItem *parent)
    : QGraphicsObject(parent)
    , mModel(model)
    , mParkingModel(parkingModel)
    , mVirtualLines{  VirtualLineItem{ model, parkingModel, 0U, this }
                     ,VirtualLineItem{ model, parkingModel, 1U, this }
                     ,VirtualLineItem{ model, parkingModel, 2U, this }
                     ,VirtualLineItem{ model, parkingModel, 3U, this }
#ifndef ULTRASONIC_ONLY
                     ,VirtualLineItem{ model, parkingModel, 4U, this }
                     ,VirtualLineItem{ model, parkingModel, 5U, this }
                     ,VirtualLineItem{ model, parkingModel, 6U, this }
                     ,VirtualLineItem{ model, parkingModel, 7U, this }
#endif
                   } // TODO: ugly non-static member initialization since VirtualLineItem inherits from QObject that is not copy-constructible
{
    setFlag(QGraphicsItem::ItemIsSelectable);

    mMainPen.setStyle(Qt::NoPen);

    connect(mModel, &ParkingBoxModel::shapeChanged, this, &ParkingBoxItem::onShapeChanged);
    connect(mParkingModel, &ParkingSceneModel::selectedObjectChanged, this, &ParkingBoxItem::selectedObjectChanged);

    onShapeChanged();
}

QRectF ParkingBoxItem::boundingRect() const
{
    auto rect = mPolygon.boundingRect();
    float margin = SELECTION_BOUNDING_MARGIN;
    rect += QMarginsF(margin, margin, margin, margin);

    return rect;
}

void ParkingBoxItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    if(isSelected()) {
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(255,100, 100)));

        for(auto& p: mPolygon) {
            painter->drawEllipse(p, POLYGON_CIRCLES_RADIUS, POLYGON_CIRCLES_RADIUS);
        }
    }

    if (option->state & QStyle::State_Selected) {
        float itemPenWidth = 0.05f;
        const qreal pad = itemPenWidth / 2;
        painter->setPen(QPen(option->palette.windowText(), 0, Qt::DotLine));
        painter->setBrush(Qt::NoBrush);
        painter->drawRect(boundingRect().adjusted(pad, pad, -pad, -pad));
    }

    if (isSelected()) {
        drawPolygonCornerNumbers(painter, mPolygon);
        drawParkingBoxEdgeType(painter, mPolygon);
    }

    painter->setPen(mMainPen);
    painter->setBrush(QBrush(QColor(255,200,0, 127)));

    painter->drawPolygon(mPolygon);

}

QPainterPath ParkingBoxItem::shape() const
{
    QPainterPath path;
    if(isSelected()) {
        path.addRect(boundingRect());
    } else {
        path.addPolygon(mPolygon);
        path = shapeFromPath(path, mMainPen);
    }

    return path;
}

void ParkingBoxItem::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    mShapeChangedByMouse = false;
}

void ParkingBoxItem::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
    if(!isSelected() && evt->buttons() & Qt::LeftButton) {
        if(mPolygon.size() > 0) {
            prepareGeometryChange();
            mPolygon.translate(evt->scenePos() - evt->lastScenePos());
            updateDataFromPolygon();
            mShapeChangedByMouse = true;
            evt->accept();
        }
    } else if(isSelected() && evt->buttons() & Qt::LeftButton) {
        int index = findEditedPolygonIndex(evt->lastScenePos());
        if(index >= 0) {
            prepareGeometryChange();
            mPolygon[index] += evt->scenePos() - evt->lastScenePos();
            updateDataFromPolygon();
            mShapeChangedByMouse = true;
            evt->accept();
        }
    }
}

void ParkingBoxItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    if(mShapeChangedByMouse) {
        mParkingModel->createHistoryPoint();
    }
}

void ParkingBoxItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt)
{
    if(evt->button() == Qt::LeftButton) {
        mParkingModel->setSelectedObject(isSelected()? nullptr: mModel);
    }
}

QVariant ParkingBoxItem::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
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

bool ParkingBoxItem::contains(const QPointF &point) const
{
    return boundingRect().contains(point);
}

void ParkingBoxItem::onShapeChanged()
{
    QPolygonF poly = polygonFromData(mModel->getBoxData());

    if(mPolygon != poly) {
        prepareGeometryChange();
        mPolygon = poly;
    }
}

void ParkingBoxItem::selectedObjectChanged(QObject *selected)
{
    setSelected(selected == mModel);
}

QPolygonF ParkingBoxItem::polygonFromData(const si::ParkingBoxSerializable &shape)
{
    QPolygonF poly;
    poly.resize(shape.slotCoordinates_m.actualSize);
    for(int i = 0; i < poly.size(); i++) {
        poly[i] = QPointF(shape.slotCoordinates_m.array[i].x_dir, shape.slotCoordinates_m.array[i].y_dir);
    }

    return poly;
}

int ParkingBoxItem::findEditedPolygonIndex(const QPointF &pos) const
{
    for(int i = 0; i < mPolygon.size(); i++) {
        if(QVector2D(mPolygon.at(i) - pos).length() < POLYGON_CIRCLES_RADIUS) {
            return i;
        }
    }

    return -1;
}

void ParkingBoxItem::updateDataFromPolygon()
{
    auto shape = mModel->getBoxData();
    Q_ASSERT(mPolygon.size() == shape.slotCoordinates_m.actualSize);
    for (int i = 0; i < mPolygon.size(); i++) {
        shape.slotCoordinates_m.array[i] = cml::Vec2Df_POD{ float32_t(mPolygon[i].x()), float32_t(mPolygon[i].y()) };
    }
    mModel->setBoxData(shape);
}

void ParkingBoxItem::drawPolygonCornerNumbers(QPainter *painter, const QPolygonF &poly)
{
    QTransform origTrans = painter->transform();

    for (int i = 0; i < poly.size(); i++) {
        const QVector2D pt(poly[i]);
        const QVector2D prev(poly[(i + poly.size() - 1) % poly.size()]);
        const QVector2D next(poly[(i + 1) % poly.size()]);

        // vector pointing away from polygon point
        const auto away = (((pt - next).normalized() + (pt - prev).normalized()).normalized()) * 0.1F;

        // 0.05 is approximate text height, TODO: remove hardcoded value
        painter->translate(poly[i] + away.toPointF() - QPointF(0.075F, 0.075F));
        painter->scale(0.015, -0.015); // scale to proper text size

        painter->drawText(QPointF(0, 0), QString::number(i));
        painter->setTransform(origTrans);
    }
}

void ParkingBoxItem::drawParkingBoxEdgeType(QPainter *painter, const QPolygonF &poly)
{
    QTransform origTrans = painter->transform();

    std::array<std::string, 4U> pBoxEdgeType{ "road side edge", "right edge", "curb side edge", "left edge"};

    for (int i = 0; i < poly.size(); i++) {
        const QVector2D pt(poly[i]);
        const QVector2D next(poly[(i + 1) % poly.size()]);

        // vector pointing away from edge
        QVector2D away;
        const QVector2D edge = next - pt;
        const QVector2D edgeCenter = pt + 0.5F*edge;
        // find orthogonal vector to the edge in proper outward direction
        away.setX((edge.normalized()).y());
        away.setY((-edge.normalized()).x());
        // final vector pointing outwards of size 0.1F
        const auto finalAway = edgeCenter + away * 0.2F;

        QString edgeType{ QString::fromStdString(pBoxEdgeType[i]) };

        // 0.05 is approximate text height, TODO: remove hardcoded value
        painter->translate(finalAway.toPointF()- QPointF(0.4F, 0.0F) - QPointF(0.075F, 0.075F));
        painter->scale(0.02F, -0.02F); // scale to proper text size

        painter->drawText(QPointF(0, 0), edgeType);
        painter->setTransform(origTrans);
    }
}