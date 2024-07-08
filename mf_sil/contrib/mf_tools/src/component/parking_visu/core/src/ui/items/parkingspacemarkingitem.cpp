#ifndef ULTRASONIC_ONLY
#include "parkingspacemarkingitem.h"

#include <QPen>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QVector2D>
#include <QDebug>

static constexpr float LINE_MARGIN = 0.1f;
static constexpr float LINE_CIRCLES_RADIUS = 0.15f; // needed for drawing the bubbles at the end of the markings for altering positions
static constexpr float SELECTION_MARGIN = LINE_MARGIN + LINE_CIRCLES_RADIUS; // needed for the bounding rectangle around the marking with bubbles

static constexpr uint8_t EXIST_PROP_THRES = 1;

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

ParkingSpaceMarkingItem::ParkingSpaceMarkingItem(ParkingSpaceMarkingModel *model, ParkingSceneModel *parkingModel, QGraphicsItem *parent)
    : QGraphicsLineItem (parent)
    , mModel(model)
    , mParkingModel(parkingModel)
{
    setFlag(QGraphicsItem::ItemIsSelectable);

    connect(mParkingModel, &ParkingSceneModel::selectedObjectChanged, this, &ParkingSpaceMarkingItem::onSelectedObjectChanged);
    connect(mModel, &ParkingSpaceMarkingModel::markingChanged, this, &ParkingSpaceMarkingItem::onMarkingChanged);

    onMarkingChanged();
}

void ParkingSpaceMarkingItem::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    mShapeChangedByMouse = false;
}

void ParkingSpaceMarkingItem::mouseMoveEvent(QGraphicsSceneMouseEvent *evt)
{
    // qDebug() << "moved" << evt->scenePos();

    const auto delta = evt->scenePos() - evt->lastScenePos();
    if(!isSelected() && evt->buttons() & Qt::LeftButton) {
        auto l = line();
        mModel->setLinePosition(l.p1() + delta, l.p2() + delta);
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
            mModel->setLinePosition(p1 + delta, p2 + delta);
            return;
        }

        *p += delta;
        mModel->setLinePosition(p1, p2);
        mShapeChangedByMouse = true;
    }
}

void ParkingSpaceMarkingItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    if(mShapeChangedByMouse) {
        mParkingModel->createHistoryPoint();
    }
}

void ParkingSpaceMarkingItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt)
{
    if(evt->buttons() & Qt::LeftButton) {
        if(evt->button() == Qt::LeftButton) {
            mParkingModel->setSelectedObject(isSelected()? nullptr: mModel);
        }
    }
}

QPainterPath ParkingSpaceMarkingItem::shape() const
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

QRectF ParkingSpaceMarkingItem::boundingRect() const
{
    float margin = isSelected()? SELECTION_MARGIN : LINE_MARGIN;
    return QGraphicsLineItem::boundingRect() + QMarginsF(margin, margin, margin, margin);
}

void ParkingSpaceMarkingItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QGraphicsLineItem::paint(painter, option, widget);

    if(isSelected()) {
        auto& marking = mModel->getMarking();
        auto color = QColor(0, 0, 0);
        if (marking.type_nu == si::ParkingLineType::PLT_WHITE) {
            color = QColor(220, 220, 220);
        } else if (marking.type_nu == si::ParkingLineType::PLT_BLUE) {
            color = QColor(0, 0, 200);
        } else if (marking.type_nu == si::ParkingLineType::PLT_RED) {
            color = QColor(200, 0, 0);
        } else if (marking.type_nu == si::ParkingLineType::PLT_YELLOW) {
            color = QColor(200, 200, 0);
        } else if (marking.type_nu == si::ParkingLineType::PLT_GREEN) {
            color = QColor(0, 200, 0);
        } else {
            // take black
        }
        painter->setPen(Qt::NoPen);
        painter->setBrush(QBrush(color));

        auto l = line();

        float radius = std::max(LINE_CIRCLES_RADIUS, 1.1F * marking.width_m);
        painter->drawEllipse(l.p1(), radius, radius);
        painter->drawEllipse(l.p2(), radius, radius);
    }
}

QVariant ParkingSpaceMarkingItem::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemSelectedChange) {
        if (value.toBool()) {
            mParkingModel->setSelectedObject(mModel);
        }
        else if (mParkingModel->getSelectedObject() == mModel) {
            mParkingModel->setSelectedObject(nullptr);
        }
    }
    return QGraphicsLineItem::itemChange(change, value);
}

void ParkingSpaceMarkingItem::onMarkingChanged()
{
    auto& marking = mModel->getMarking();
    if (marking.pos_m.actualSize >= 2) {
        setLine(marking.pos_m.array[0].x_dir, marking.pos_m.array[0].y_dir, marking.pos_m.array[1].x_dir, marking.pos_m.array[1].y_dir);
        QPen pen;
        pen.setWidthF(marking.width_m);
        if (marking.type_nu == si::ParkingLineType::PLT_BLUE) {
            pen.setColor(QColor(0, 0, 200));
        } else if (marking.type_nu == si::ParkingLineType::PLT_WHITE) {
            pen.setColor(QColor(220, 220, 220));
        } else if(marking.type_nu == si::ParkingLineType::PLT_RED){
            pen.setColor(QColor(200, 0, 0));
        } else if (marking.type_nu == si::ParkingLineType::PLT_YELLOW) {
            pen.setColor(QColor(200, 200, 0));
        } else if (marking.type_nu == si::ParkingLineType::PLT_GREEN) {
            pen.setColor(QColor(0, 200, 0));
        } else {
            pen.setColor(QColor(0, 0, 0));
        }
        setPen(pen);

        QPainterPath path;
        path.moveTo(line().p1());
        path.lineTo(line().p2());

        pen.setWidthF(pen.widthF() + static_cast<double>(LINE_MARGIN) * 2.0);
        mShape = shapeFromPath(path, pen);

        setVisible(marking.existenceProb_perc >= EXIST_PROP_THRES);
    }
}

void ParkingSpaceMarkingItem::onSelectedObjectChanged(QObject *selected)
{
    setSelected(selected == mModel);
    prepareGeometryChange();
}
#endif
