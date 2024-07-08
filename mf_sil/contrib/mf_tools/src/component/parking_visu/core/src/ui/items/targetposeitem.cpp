#include "targetposeitem.h"

#include <QPainter>
#include <QtMath>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QVector2D>
#include <QMenu>
#include <QAction>

#include <algorithm>

TargetPoseItem::TargetPoseItem(TargetPoseModel* tpModel, ParkingSceneModel *parkingModel)
    : VehicleItem(tpModel, parkingModel)
{
    setFlag(QGraphicsItem::ItemIsFocusable);

    connect(parkingModel, &ParkingSceneModel::selectedTargetPoseChanged, this, &TargetPoseItem::onSelectedTargetPoseChanged);

    onSelectedTargetPoseChanged(mParkingModel->getSelectedTargetPose(), mParkingModel->getSelectedTargetPose());
}

void TargetPoseItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    if(mIsSelectedTargetPose) {
        setVehicleColor(QColor(0,255,0, 127));
    } else {
        setVehicleColor(QColor(180,200,180, 127));
    }

    VehicleItem::paint(painter, option, widget);
    QPen pen;
    pen.setWidthF(0.01);
    pen.setColor(QColor(255,0,0, 127));
    painter->setPen(pen);
}

void TargetPoseItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
    // click without movement
    if(evt->button() == Qt::RightButton && evt->buttonDownScreenPos(Qt::RightButton) == evt->screenPos()) {
        QMenu menu;
        auto swapStart = menu.addAction("Swap with start pose");
        QAction *a = menu.exec(evt->screenPos());
        if(a == swapStart) {
            // swap position
            auto start = mParkingModel->getStartPoseModel();
            auto prevPos = mModel->getPosXY_m();
            auto prevYaw = mModel->getPosYawAngle_Rad();
            mModel->setPos(start->getPosX_m(), start->getPosY_m(), start->getPosYawAngle_Rad());
            start->setPos(prevPos.x(), prevPos.y(), prevYaw);

            // correct target pose type
            ap_tp::PoseType newType {getModel()->getPoseType()};
            switch (newType) {
            case ap_tp::PoseType::T_PARALLEL_PARKING:
                newType = ap_tp::PoseType::T_PAR_PARKING_OUT;
                break;
            case ap_tp::PoseType::T_PAR_PARKING_OUT:
                newType = ap_tp::PoseType::T_PARALLEL_PARKING;
                break;
            case ap_tp::PoseType::T_PERP_PARKING_BWD:
                newType = ap_tp::PoseType::T_PERP_PARKING_OUT_FWD;
                break;
            case ap_tp::PoseType::T_PERP_PARKING_OUT_FWD:
                newType = ap_tp::PoseType::T_PERP_PARKING_BWD;
                break;
            case ap_tp::PoseType::T_PERP_PARKING_FWD:
                newType = ap_tp::PoseType::T_PERP_PARKING_OUT_BWD;
                break;
            case ap_tp::PoseType::T_PERP_PARKING_OUT_BWD:
                newType = ap_tp::PoseType::T_PERP_PARKING_FWD;
                break;
            case ap_tp::PoseType::T_GP_FWD:
                newType = ap_tp::PoseType::T_GP_OUT_BWD;
                break;
            case ap_tp::PoseType::T_GP_OUT_BWD:
                newType = ap_tp::PoseType::T_GP_FWD;
                break;
            case ap_tp::PoseType::T_GP_BWD:
                newType = ap_tp::PoseType::T_GP_OUT_FWD;
                break;
            case ap_tp::PoseType::T_GP_OUT_FWD:
                newType = ap_tp::PoseType::T_GP_BWD;
                break;
            case ap_tp::PoseType::T_GP_FWD_AXIS:
                newType = ap_tp::PoseType::T_GP_OUT_BWD_AXIS;
                break;
            case ap_tp::PoseType::T_GP_OUT_BWD_AXIS:
                newType = ap_tp::PoseType::T_GP_FWD_AXIS;
                break;
            case ap_tp::PoseType::T_GP_BWD_AXIS:
                newType = ap_tp::PoseType::T_GP_OUT_FWD_AXIS;
                break;
            case ap_tp::PoseType::T_GP_OUT_FWD_AXIS:
                newType = ap_tp::PoseType::T_GP_BWD_AXIS;
                break;
            default:
                break;
            }

            getModel()->setPoseType(newType);
        }
    } else {
        // forward everything else to vehicle item
        VehicleItem::mouseReleaseEvent(evt);
    }
}

void TargetPoseItem::onSelectedTargetPoseChanged(int prev, int current)
{
    mIsSelectedTargetPose = (current >= 0)
            && ((size_t)current < mParkingModel->numTargetPoseModels())
            && (mParkingModel->getTargetPoseModel(current) == mModel);
    update();
}

