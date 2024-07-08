#include "vehicleghostitem.h"

#include <ui/items/vehicleitem.h>

#include <QPen>
#include <QPainter>
#include <QVector2D>

VehicleGhostItem::VehicleGhostItem(ParkingSceneModel* parkingModel, QObject *parent)
    : QObject(parent)
    , mParkingModel(parkingModel)
{
    // setBrush(QBrush(QColor(0,255,0, 127)));
    QPen pen(Qt::DashLine);
    pen.setWidthF(0);
    setPen(pen);

    mBoundingBoxItem = new QGraphicsPolygonItem(this);
    QPen penBox(Qt::NoPen);
    penBox.setWidthF(0);
    mBoundingBoxItem->setPen(penBox);
    QColor color = QColor(100, 150, 255, 127);
    color.setAlpha(color.alpha() / 2);
    mBoundingBoxItem->setBrush(color);

    onVehicleParamsChanged(parkingModel->getVehicleParams());
    onPathChanged();
    connect(mParkingModel, &ParkingSceneModel::vehicleParamsChanged, this, &VehicleGhostItem::onVehicleParamsChanged);
    connect(mParkingModel->getTrajectorySetModel(), &TrajectorySetModel::currentPoseIndexChanged, this, &VehicleGhostItem::onPathChanged);
    connect(mParkingModel, &ParkingSceneModel::vehInflRadiusChanged, this, &VehicleGhostItem::onVehInflRadiusChanged);
}

void VehicleGhostItem::onPathChanged()
{
    mPathIndex = mParkingModel->getTrajectorySetModel()->getCurrentPoseIndex();

    const Trajectory* selectedTraj = mParkingModel->getTrajectorySetModel()->getSelectedTrajectory();

    if(selectedTraj && mPathIndex >= 0 && mPathIndex < selectedTraj->size()) {
        setTransform(selectedTraj->at(mPathIndex).trans);
        setVisible(true);
        return;
    } else {
        setTransform(QTransform());
        setVisible(false);
    }
}

void VehicleGhostItem::onVehicleParamsChanged(ap_common::Vehicle_Params const& params)
{
    setPolygon(VehicleItem::calcBoundingShapeFromVehicleParams(params));
    onVehInflRadiusChanged(mParkingModel->getVehInflRadius_m());
}

void VehicleGhostItem::onVehInflRadiusChanged(float radius) {

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
    mBoundingBoxItem->setPolygon(VehicleItem::calcShapeFromArray(xValVeh, yValVeh, egoShape.getSize()));
}

void VehicleGhostItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{

    QPen pen;
    pen.setWidthF(0.05);
    painter->setPen(pen);
    painter->drawLine(QPointF(0, 0), QPointF(0.3, 0));
    painter->drawLine(QPointF(0, -0.3), QPointF(0, 0.3));

    QGraphicsPolygonItem::paint(painter, option, widget);
}
