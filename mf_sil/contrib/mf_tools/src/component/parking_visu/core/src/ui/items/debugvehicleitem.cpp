#define _USE_MATH_DEFINES // for C++
#include <cmath>

#include "debugvehicleitem.h"

#include <QDebug>
#include <QKeyEvent>
#include <QVector2D>
#include <QPainter>
#include <QtMath>
#include <QGraphicsSceneContextMenuEvent>
#include <QMenu>
#include <QAction>
#include <QStyleOptionGraphicsItem>

/**
 * Draws an arc instead of a pie when span angle is less than 360 degrees
 *
 * See https://stackoverflow.com/questions/14279162/qt-qgraphicsscene-drawing-arc
 */
class ArcEllipseItem : public QGraphicsEllipseItem {
public:
    ArcEllipseItem ( QGraphicsItem * parent = 0 ) :
        QGraphicsEllipseItem(parent) {
    }

protected:
    void paint ( QPainter * painter, const QStyleOptionGraphicsItem * option, QWidget * widget) {
        painter->setPen(pen());
        painter->setBrush(brush());
        painter->drawArc(rect(), startAngle(), spanAngle());

        // TODO: qt_graphicsItem_highlightSelected() is private static function inside qt
//        if (option->state & QStyle::State_Selected)
//            qt_graphicsItem_highlightSelected(this, painter, option);
    }
};



DebugVehicleItem::DebugVehicleItem(DebugVehicleModel *model, ParkingSceneModel *parkingModel, QGraphicsItem *parent)
    : VehicleItem(model, parkingModel, parent)
{
    setFlag(QGraphicsItem::ItemIsFocusable);

    setVehicleColor(QColor(255,0,255, 64));

    mRotationCenterItem = new ArcEllipseItem(this);
    mRotationCenterItem->setPen(Qt::NoPen);
    mRotationCenterItem->setBrush(QColor(0,0,0));
    mRotationCenterItem->setRect(-0.05, -0.05, 0.1, 0.1);

    QPen pen;
    pen.setWidthF(0);
    pen.setStyle(Qt::DashLine);

    mLineRearItem = new QGraphicsLineItem(this);
    mLineRearItem->setPen(pen);

    mLineFrontItem = new QGraphicsLineItem(this);
    mLineFrontItem->setPen(pen);

    mInnerRotationCircleItem = new ArcEllipseItem(this);
    mInnerRotationCircleItem->setPen(pen);
    mInnerRotationCircleItem->setVisible(false);

    mOuterRotationCircleItem = new ArcEllipseItem(this);
    mOuterRotationCircleItem->setPen(pen);
    mOuterRotationCircleItem->setVisible(false);

    onWheelAngleChanged();

    connect(model, &DebugVehicleModel::steeringAngleChanged, this, &DebugVehicleItem::onWheelAngleChanged);
    connect(model, &DebugVehicleModel::previewTrackSpanChanged, this, &DebugVehicleItem::onWheelAngleChanged);

    connect(model, &DebugVehicleModel::visibilityChanged, this, [this] (bool visible) {
       setVisible(visible);
    });
}

void DebugVehicleItem::setCreateHistoryPointOnMouseMove(bool flag)
{
    mCreateHistoryPointOnMouseMove = flag;
}

void DebugVehicleItem::onWheelAngleChanged()
{
    auto model = getModel();
    float steeringDeg = qRadiansToDegrees(model->getSteeringAngle());

    mFrontLeftWheelItem->setRotation(steeringDeg);
    mFrontRightWheelItem->setRotation(steeringDeg);

    const float radius = model->getRotationRadius();
    mRotationCenterItem->setPos(0, radius);
    mLineRearItem->setLine(0, 0, 0, radius);
    mLineFrontItem->setLine(getModel()->getWheelbase(), 0, 0, radius);

    const auto boundingShape{ getBoundingShape() };
    if(std::fabs(model->getRotationRadius()) < 0.005 || boundingShape.empty()) {
        mInnerRotationCircleItem->setVisible(false);
        mOuterRotationCircleItem->setVisible(false);
    } else {
        float minDist = std::numeric_limits<float>::max();
        float maxDist = 0;
        QVector2D rotPoint(0, radius);

        // find outer vertex
        for(const auto& corner: boundingShape) {
            float dist = (QVector2D(corner) - rotPoint).length();
            minDist = std::min(minDist, dist);
            maxDist = std::max(maxDist, dist);
        }

        mInnerRotationCircleItem->setRect(-minDist, radius-minDist, 2*minDist, 2*minDist);
        mInnerRotationCircleItem->setVisible(true);

        mOuterRotationCircleItem->setRect(-maxDist, radius-maxDist, 2*maxDist, 2*maxDist);
        mOuterRotationCircleItem->setVisible(true);

        float span = model->previewTrackSpan() / std::abs(radius);
        int spanInt = qRadiansToDegrees(span)*16;

        mOuterRotationCircleItem->setSpanAngle(spanInt);
        mInnerRotationCircleItem->setSpanAngle(spanInt);

        float angleOffset = radius > 0? M_PI_2 : -M_PI_2;
        int yawInt = qRadiansToDegrees(angleOffset - span/2.0)*16;
        mOuterRotationCircleItem->setStartAngle(yawInt);
        mInnerRotationCircleItem->setStartAngle(yawInt);
    }
}


void DebugVehicleItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{

    mModel->setPosXY_m(mModel->getPosX_m(), mModel->getPosY_m());
    // original VehicleItem parent implementation creates a history point in parking model

    // right click without movement
    if(evt->button() == Qt::RightButton && evt->buttonDownScreenPos(Qt::RightButton) == evt->screenPos()) {
        QMenu menu;
        auto fromStart = menu.addAction("Apply start pose");
        auto fromTarget = menu.addAction("Apply target pose");
        auto fromDebug = menu.addAction("Reset to debug pose");
        QAction *a = menu.exec(evt->screenPos());
        if(a == fromStart) {
            auto start = mParkingModel->getStartPoseModel();
            mModel->setPos(start->getPosX_m(), start->getPosY_m(), start->getPosYawAngle_Rad());
        } else if(a == fromTarget) {
            auto target = mParkingModel->getSelectedTargetPoseModel();
            if(target) {
                mModel->setPos(target->getPosX_m(), target->getPosY_m(), target->getPosYawAngle_Rad());
            }
        } else if(a == fromDebug) {
            getModel()->resetToDebugPose();
        }
    }

    if(mCreateHistoryPointOnMouseMove && mVehicleMovedByMouse) {
        mParkingModel->createHistoryPoint();
    }
}

void DebugVehicleItem::keyPressEvent(QKeyEvent *evt)
{
    auto model = getModel();
    if(evt->key() == Qt::Key_Left) {
        model->setSteeringAngle(std::min(model->getSteeringAngle()+0.05f, model->getMaxSteeringAngle()));
    } else if(evt->key() == Qt::Key_Right) {
        model->setSteeringAngle(std::max(model->getSteeringAngle()-0.05f, -model->getMaxSteeringAngle()));
    } else if(evt->key() == Qt::Key_Up) {
        model->moveVehicle(model->getStepDistance());
    } else if(evt->key() == Qt::Key_Down) {
        model->moveVehicle(-model->getStepDistance());
    } else if(evt->key() == Qt::Key_Plus) {
        model->setStepDistance(model->getStepDistance() + 0.05);
    } else if(evt->key() == Qt::Key_Minus) {
        model->setStepDistance(std::max(model->getStepDistance() - 0.05f, 0.0f));
    } else {
        VehicleItem::keyPressEvent(evt);
    }
}

void DebugVehicleItem::keyReleaseEvent(QKeyEvent *evt)
{
    /*
     * TODO: discuss if it is needed to create history on every movement by key??
    if(mCreateHistoryPointOnMouseMove) {
        switch (evt->key()) {
        case Qt::Key_Up:
        case Qt::Key_Down:
            mParkingModel->createHistoryPoint();
            break;
        default:
            break;
        }
    }
    */
}
