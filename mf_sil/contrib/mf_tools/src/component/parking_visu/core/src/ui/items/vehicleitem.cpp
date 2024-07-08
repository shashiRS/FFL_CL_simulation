#include "vehicleitem.h"

#include <QPainter>
#include <QtMath>
#include <QGraphicsSceneMouseEvent>
#include <QDebug>
#include <QVector2D>
#include <QKeyEvent>
#include <QClipboard>
#include <QApplication>

#include <algorithm>


VehicleItem::VehicleItem(VehicleModel *model, ParkingSceneModel *parkingModel, QGraphicsItem *parent)
    : QGraphicsPolygonItem(parent)
    , mModel(model)
    , mParkingModel(parkingModel)
{
    // bounding box used for visualizing the ego vehicle inflation
    mBoundingBoxItem = new QGraphicsPolygonItem(this);
    QPen pen;
    pen.setWidthF(0);
    this->setPen(pen);
    mBoundingBoxItem->setPen(Qt::NoPen);

    // set color for vehicle and vehicle inflation
    setVehicleColor(QColor(0,255,0, 127));

    onVehicleParamsChanged(parkingModel->getVehicleParams());
    setVisible(mModel->isVisible());

    setPos(mModel->getPosX_m(), mModel->getPosY_m());
    setRotation(mModel->getPosYawAngle_Rad());

    createWheelItems();

    connect(mModel, &VehicleModel::positionChanged, this, &VehicleItem::onVehiclePositionChanged);
    connect(parkingModel, &ParkingSceneModel::vehicleParamsChanged, this, &VehicleItem::onVehicleParamsChanged);
    connect(mModel, &VehicleModel::visibilityChanged, this, &VehicleItem::onVisibilityChanged);

    connect(mParkingModel, &ParkingSceneModel::vehInflRadiusChanged, this, &VehicleItem::onVehInflRadiusChanged);

    connect(mParkingModel, &ParkingSceneModel::selectedObjectChanged, this, &VehicleItem::onSelectionChanged);

    connect(mParkingModel, &ParkingSceneModel::vehicleParamsChanged, this, &VehicleItem::reloadVehicleShape);

    reloadVehicleShape();

    setFlag(QGraphicsItem::ItemIsSelectable);
}

void VehicleItem::mousePressEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)
    mVehicleMovedByMouse = false;
    //qDebug() << "pressed" << evt->scenePos();

    // mPositionModel->setPosX_m(30);
}

void VehicleItem::mouseReleaseEvent(QGraphicsSceneMouseEvent *evt)
{
    Q_UNUSED(evt)

    //qDebug() << "released" << evt->scenePos();

    if(mVehicleMovedByMouse) {
        mParkingModel->createHistoryPoint();
    }
}

void VehicleItem::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *evt)
{
    if(evt->button() == Qt::LeftButton) {
        mParkingModel->setSelectedObject(isSelected()? nullptr: mModel);
    }
}

void VehicleItem::keyPressEvent(QKeyEvent *evt)
{
    // qDebug() << evt->key();
    if(evt->key() == Qt::Key_C && (evt->modifiers() & Qt::ControlModifier)) {
        QString poseText = QString("%1, %2, %3").arg(mModel->getPosX_m()).arg(mModel->getPosY_m()).arg(mModel->getPosYawAngle_Rad());
        QClipboard* clibboard = QApplication::clipboard();
        clibboard->setText(poseText);

        qDebug() << "vehicle pose copied";
    } else if(evt->key() == Qt::Key_Backspace) {
        mModel->setPos(0, 0, 0);
    } else {
        QGraphicsPolygonItem::keyPressEvent(evt);
    }
}

void VehicleItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    QPen pen;
    pen.setWidthF(0.05);
    painter->setPen(pen);
    painter->drawLine(QPointF(0, 0), QPointF(0.3, 0));
    painter->drawLine(QPointF(0, -0.3), QPointF(0, 0.3));

    QGraphicsPolygonItem::paint(painter, option, widget);
}

QRectF VehicleItem::boundingRect() const
{
    if(isVisible()) {
        return QGraphicsPolygonItem::boundingRect();
    } else {
        return QRectF();
    }
}

QVariant VehicleItem::itemChange(QGraphicsItem::GraphicsItemChange change, const QVariant &value)
{
    if(change == ItemSelectedChange) {
        // qDebug() << "ItemSelectedChange vehicle";

        if (value.toBool()) {
            mParkingModel->setSelectedObject(mModel);
        }
        else if (mParkingModel->getSelectedObject() == mModel) {
            mParkingModel->setSelectedObject(nullptr);
        }
    }
    return QGraphicsItem::itemChange(change, value);
}

void VehicleItem::onVehiclePositionChanged()
{
    prepareGeometryChange();

    setPos(mModel->getPosX_m(), mModel->getPosY_m());
    setRotation(qRadiansToDegrees(mModel->getPosYawAngle_Rad()));

    // qDebug() << "onVehiclePositionChanged";
}

void VehicleItem::onVehicleParamsChanged(const ap_common::Vehicle_Params& params)
{
    prepareGeometryChange();

    setPolygon(calcBoundingShapeFromVehicleParams(params));
    onVehInflRadiusChanged(mParkingModel->getVehInflRadius_m());

}

void VehicleItem::onVisibilityChanged(bool visible)
{
    // when item becomes invisible, it is automatically unselected
    // select it again if so
    bool wasSelected = isSelected();

    setVisible(visible);
    // setFlag(QGraphicsItem::ItemHasNoContents, !visible);

    if(wasSelected) {
        mParkingModel->setSelectedObject(mModel);
    }
}

void VehicleItem::onVehInflRadiusChanged(float radius) {

    // get an egoVeh and inflate it
    ap_common::EgoVehicle egoVeh;
    egoVeh.init(&mParkingModel->getVehicleParams(), ap_common::EgoVehicleShapeType::EGO_VEH_SHAPE_BOUNDING);
    egoVeh.inflateVehicleShape(radius);

    // get the inflated shape and draw the bounding box around the original shape
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> egoShape = egoVeh.getVehicleShape();

    float xValVeh[ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU];
    float yValVeh[ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU];

    for (uint8_t i = 0; i < egoShape.getSize(); ++i) {
        xValVeh[i] = egoShape[i][0];
        yValVeh[i] = egoShape[i][1];
    }
    // define bounding box based on inflation of vehicle
    mBoundingBoxItem->setPolygon(calcShapeFromArray(xValVeh, yValVeh, egoShape.getSize()));

    updateWheelInflation(egoVeh, ap_common::VehiclePart::WHEEL_FRONT_LEFT);
    updateWheelInflation(egoVeh, ap_common::VehiclePart::WHEEL_FRONT_RIGHT);
    updateWheelInflation(egoVeh, ap_common::VehiclePart::WHEEL_REAR_LEFT);
    updateWheelInflation(egoVeh, ap_common::VehiclePart::WHEEL_REAR_RIGHT);

    update();
}

void VehicleItem::updateWheelInflation(ap_common::EgoVehicle& egoVeh, ap_common::VehiclePart vehPart) {

    auto wheelShape{ egoVeh.getWheel(vehPart) };

    float xValues[ap_common::AP_COMMON_TYPES_Consts::AP_V_WHEEL_SHAPE_MAX_SIZE_NU];
    float yValues[ap_common::AP_COMMON_TYPES_Consts::AP_V_WHEEL_SHAPE_MAX_SIZE_NU];
    for (uint8_t i{ 0 }; i < wheelShape.getSize(); ++i) {
        xValues[i] = wheelShape[i][0];
        yValues[i] = wheelShape[i][1];
    }

    if (wheelShape.getSize() != 0) {
        switch (vehPart) {
            case ap_common::VehiclePart::WHEEL_FRONT_LEFT:
                mFrontLeftWheelBoundingBoxItem->setPolygon(calcShapeFromArray(xValues, yValues, wheelShape.getSize()));
            case ap_common::VehiclePart::WHEEL_FRONT_RIGHT:
                mFrontRightWheelBoundingBoxItem->setPolygon(calcShapeFromArray(xValues, yValues, wheelShape.getSize()));
            case ap_common::VehiclePart::WHEEL_REAR_LEFT:
                mRearLeftWheelBoundingBoxItem->setPolygon(calcShapeFromArray(xValues, yValues, wheelShape.getSize()));
            case ap_common::VehiclePart::WHEEL_REAR_RIGHT:
                mRearRightWheelBoundingBoxItem->setPolygon(calcShapeFromArray(xValues, yValues, wheelShape.getSize()));

        }
    }
}

void VehicleItem::onSelectionChanged(QObject *selected)
{
    setSelected(selected == mModel);
}

void VehicleItem::reloadVehicleShape()
{
    auto const& params = mParkingModel->getVehicleParams();
    mFrontLeftWheelItem->setPolygon(calcShapeFromArray(params.AP_V_FL_WHEEL_SHAPE_X_M, params.AP_V_FL_WHEEL_SHAPE_Y_M, params.AP_V_WHEEL_SHAPE_SIZE_NU));
    mFrontRightWheelItem->setPolygon(calcShapeFromArray(params.AP_V_FR_WHEEL_SHAPE_X_M, params.AP_V_FR_WHEEL_SHAPE_Y_M, params.AP_V_WHEEL_SHAPE_SIZE_NU));
    mRearLeftWheelItem->setPolygon(calcShapeFromArray(params.AP_V_RL_WHEEL_SHAPE_X_M, params.AP_V_RL_WHEEL_SHAPE_Y_M, params.AP_V_WHEEL_SHAPE_SIZE_NU));
    mRearRightWheelItem->setPolygon(calcShapeFromArray(params.AP_V_RR_WHEEL_SHAPE_X_M, params.AP_V_RR_WHEEL_SHAPE_Y_M, params.AP_V_WHEEL_SHAPE_SIZE_NU));

    mTrailerHitch->setPolygon(calcShapeFromArray(params.AP_V_HITCH_SHAPE_X_M, params.AP_V_HITCH_SHAPE_Y_M, params.AP_V_HITCH_SHAPE_SIZE_NU));
    mLeftMirror->setPolygon(calcShapeFromArray(params.AP_V_LEFT_MIRROR_SHAPE_X_M, params.AP_V_LEFT_MIRROR_SHAPE_Y_M, params.AP_V_MIRROR_SHAPE_SIZE_NU));
    mRightMirror->setPolygon(calcShapeFromArray(params.AP_V_RIGHT_MIRROR_SHAPE_X_M, params.AP_V_RIGHT_MIRROR_SHAPE_Y_M, params.AP_V_MIRROR_SHAPE_SIZE_NU));
}

void VehicleItem::createWheelItems()
{
    auto color = QColor(0, 0, 0);
    mFrontLeftWheelItem = new QGraphicsPolygonItem(this);
    mFrontLeftWheelItem->setPen(Qt::NoPen);
    mFrontLeftWheelItem->setBrush(color);
    // set rotation center to center of the front wheel item for steering visualization
    mFrontLeftWheelItem->setTransformOriginPoint(mModel->getWheelbase(), mModel->getFrontTrack() / 2);
    mFrontRightWheelItem = new QGraphicsPolygonItem(this);
    mFrontRightWheelItem->setPen(Qt::NoPen);
    mFrontRightWheelItem->setBrush(color);
    // set rotation center to center of the front wheel item for steering visualization
    mFrontRightWheelItem->setTransformOriginPoint(mModel->getWheelbase(), -mModel->getFrontTrack() / 2);

    mRearLeftWheelItem = new QGraphicsPolygonItem(this);
    mRearLeftWheelItem->setPen(Qt::NoPen);
    mRearLeftWheelItem->setBrush(color);
    mRearRightWheelItem = new QGraphicsPolygonItem(this);
    mRearRightWheelItem->setPen(Qt::NoPen);
    mRearRightWheelItem->setBrush(color);

    color.setAlpha(color.alpha() / 3);
    mFrontLeftWheelBoundingBoxItem = new QGraphicsPolygonItem(mFrontLeftWheelItem);
    mFrontLeftWheelBoundingBoxItem->setPen(Qt::NoPen);
    mFrontLeftWheelBoundingBoxItem->setBrush(color);
    mFrontRightWheelBoundingBoxItem = new QGraphicsPolygonItem(mFrontRightWheelItem);
    mFrontRightWheelBoundingBoxItem->setPen(Qt::NoPen);
    mFrontRightWheelBoundingBoxItem->setBrush(color);

    mRearLeftWheelBoundingBoxItem = new QGraphicsPolygonItem(mRearLeftWheelItem);
    mRearLeftWheelBoundingBoxItem->setPen(Qt::NoPen);
    mRearLeftWheelBoundingBoxItem->setBrush(color);
    mRearRightWheelBoundingBoxItem = new QGraphicsPolygonItem(mRearRightWheelItem);
    mRearRightWheelBoundingBoxItem->setPen(Qt::NoPen);
    mRearRightWheelBoundingBoxItem->setBrush(color);

    QPen pen;
    pen.setWidthF(0);

    mTrailerHitch = new QGraphicsPolygonItem(this);
    mTrailerHitch->setPen(pen);
    mTrailerHitch->setBrush(Qt::NoBrush);

    mLeftMirror = new QGraphicsPolygonItem(this);
    mLeftMirror->setPen(pen);
    mLeftMirror->setBrush(Qt::NoBrush);

    mRightMirror = new QGraphicsPolygonItem(this);
    mRightMirror->setPen(pen);
    mRightMirror->setBrush(Qt::NoBrush);
}

void VehicleItem::setVehicleColor(QColor color)
{
    this->setBrush(color);
    color.setAlpha(color.alpha() / 2);
    mBoundingBoxItem->setBrush(color);
}

QPolygonF VehicleItem::calcShapeFromArray(const float *xValues, const float *yValues, lsm_geoml::size_type count)
{
    QPolygonF shape;
    for(lsm_geoml::size_type i = 0; i < count; i++) {
        shape.append(QPointF(xValues[i], yValues[i]));
    }

    return shape;
}

QPolygonF VehicleItem::calcBoundingShapeFromVehicleParams(const ap_common::Vehicle_Params &params)
{
    static_assert(sizeof(params.AP_V_BOUNDINGBOX_X_M) == sizeof(params.AP_V_BOUNDINGBOX_Y_M),
                  "size of Vehicle_Params::AP_V_BOUNDINGBOX_X/Y does not match");

    return calcShapeFromArray(params.AP_V_BOUNDINGBOX_X_M, params.AP_V_BOUNDINGBOX_Y_M, params.AP_V_NUM_BOUNDING_PTS);
}

QPolygonF VehicleItem::calcStandardShapeFromVehicleParams(const ap_common::Vehicle_Params &params)
{
    static_assert(sizeof(params.AP_V_STANDARD_SHAPE_X_M) == sizeof(params.AP_V_STANDARD_SHAPE_Y_M),
                  "size of Vehicle_Params::AP_V_STANDARD_SHAPE_X/Y does not match");

   return calcShapeFromArray(params.AP_V_STANDARD_SHAPE_X_M, params.AP_V_STANDARD_SHAPE_Y_M, params.AP_V_NUM_STANDARD_SHAPE_PTS);
}

void VehicleItem::mouseMoveEvent(QGraphicsSceneMouseEvent  *evt)
{
    // qDebug() << "moved" << evt->scenePos();

    if(evt->buttons() & Qt::LeftButton) {
        prepareGeometryChange();
        auto delta = evt->scenePos() - evt->lastScenePos();
        mModel->setPosXY_m(mModel->getPosX_m() + delta.x(), mModel->getPosY_m() + delta.y());
        mVehicleMovedByMouse = true;
    } else if(evt->buttons() & Qt::RightButton) {
        prepareGeometryChange();
        auto angleStart = qAtan2(evt->lastPos().y(), evt->lastPos().x());
        auto angleCurrent = qAtan2(evt->pos().y(), evt->pos().x());
        mModel->setPosYawAngle_Rad(mModel->getPosYawAngle_Rad() + angleCurrent - angleStart);
        mVehicleMovedByMouse = true;
    }
}
