#include "virtuallineitem.h"

#include <QPen>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QVector2D>
#include <QDebug>

static constexpr float LINE_MARGIN = 0.1f;
static constexpr float LINE_CIRCLES_RADIUS = 0.15f;
static constexpr float SELECTION_MARGIN = LINE_MARGIN + LINE_CIRCLES_RADIUS;
static constexpr float LINE_WIDTH_M = 0.05f;

static constexpr uint8_t EXIST_PROP_THRES = 10;

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

VirtualLineItem::VirtualLineItem(
        ParkingBoxModel *model,
        ParkingSceneModel *parkingModel,
        const uint8_t virtualLineIndex,
        QGraphicsItem *parent)
    : QGraphicsLineItem (parent)
    , mModel(model)
    , mParkingModel(parkingModel)
    , mThisLineIndex(virtualLineIndex)
{
    setFlag(QGraphicsItem::ItemIsSelectable);

    connect(mModel, &ParkingBoxModel::virtualLinesChanged, this, &VirtualLineItem::onVirtualLineChanged);

    onVirtualLineChanged();
}

VirtualLineItem::VirtualLineItem(const VirtualLineItem &vlItem)
    : mModel(vlItem.mModel),
    mParkingModel(vlItem.mParkingModel),
    mThisLineIndex(vlItem.mThisLineIndex)
{}

void VirtualLineItem::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    mShapeChangedByMouse = false;
}

void VirtualLineItem::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
    // qDebug() << "moved" << evt->scenePos();

    const auto delta = evt->scenePos() - evt->lastScenePos();
    if(!isSelected() && evt->buttons() & Qt::LeftButton) {
        auto l = line();
        mModel->setVirtualLinePoints(mThisLineIndex, l.p1() + delta, l.p2() + delta);
        mShapeChangedByMouse = true;
    } else if(isSelected() && evt->buttons() & Qt::LeftButton) {
        QPointF p1 = line().p1();
        QPointF p2 = line().p2();

        QPointF* p = nullptr;

        if(QVector2D(evt->lastScenePos() - p1).length() <= LINE_CIRCLES_RADIUS) {
            p = &p1;
        } else if(QVector2D(evt->lastScenePos() - p2).length() <= LINE_CIRCLES_RADIUS) {
            p = &p2;
        } else {
            mModel->setVirtualLinePoints(mThisLineIndex, p1 + delta, p2 + delta);
            return;
        }

        *p += delta;
        mModel->setVirtualLinePoints(mThisLineIndex, p1, p2);
        mShapeChangedByMouse = true;
    }
}

void VirtualLineItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    if(mShapeChangedByMouse) {
        mParkingModel->createHistoryPoint();
    }
}

void VirtualLineItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt)
{
    setSelected(true);
    prepareGeometryChange();
}

QPainterPath VirtualLineItem::shape() const
{
    if(isSelected()) {
        QPainterPath path;
        path.addRect(mShape.controlPointRect()
                     + QMarginsF(SELECTION_MARGIN, SELECTION_MARGIN, SELECTION_MARGIN, SELECTION_MARGIN));
        return path;
    } else {
        return mShape;
    }
}

QRectF VirtualLineItem::boundingRect() const
{
    float margin = isSelected()? SELECTION_MARGIN : LINE_MARGIN;
    return QGraphicsLineItem::boundingRect() + QMarginsF(margin, margin, margin, margin);
}

void VirtualLineItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QGraphicsLineItem::paint(painter, option, widget);

    if(isSelected()) {
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(QColor(100,150, 255, 150)));

        auto l = line();

        painter->drawEllipse(l.p1(), LINE_CIRCLES_RADIUS, LINE_CIRCLES_RADIUS);
        painter->drawEllipse(l.p2(), LINE_CIRCLES_RADIUS, LINE_CIRCLES_RADIUS);
    }
}

void VirtualLineItem::onVirtualLineChanged()
{
    auto& virtualLine = mModel->getVirtualLine(mThisLineIndex);
    if (virtualLine.virtLineVertices_m.actualSize != 0){
    setLine(static_cast<qreal>(virtualLine.virtLineVertices_m.array[0].x_dir),
            static_cast<qreal>(virtualLine.virtLineVertices_m.array[0].y_dir),
            static_cast<qreal>(virtualLine.virtLineVertices_m.array[1].x_dir),
            static_cast<qreal>(virtualLine.virtLineVertices_m.array[1].y_dir));
    } else {
        setLine(0.0, 0.0, 0.0, 0.0);
    }
    QPen pen;
    pen.setWidthF(LINE_WIDTH_M); // Senseful value?
    pen.setColor(QColor(0, 200, 0, 150));
    setPen(pen);

    QPainterPath path;
    path.moveTo(line().p1());
    path.lineTo(line().p2());

    pen.setWidthF(pen.widthF() + static_cast<double>(LINE_MARGIN) * 2.0);
    mShape = shapeFromPath(path, pen);

    setVisible(virtualLine.virtLineVertices_m.actualSize!=0);
}
