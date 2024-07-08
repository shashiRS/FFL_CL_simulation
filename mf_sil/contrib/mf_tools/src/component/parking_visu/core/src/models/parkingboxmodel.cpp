#include "parkingboxmodel.h"

ParkingBoxModel::ParkingBoxModel(QObject *parent) : QObject(parent)
{
    memset(&mBoxData, 0, sizeof(mBoxData));
    mBoxData.slotCoordinates_m.actualSize = ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU;
}

void ParkingBoxModel::setBoxData(const si::ParkingBoxSerializable &data)
{
    mBoxData = data;
    emit shapeChanged();
    emit virtualLinesChanged();
}

int ParkingBoxModel::getParkingBoxID() const
{
    return static_cast<int>(mBoxData.parkingBoxID_nu);
}

void ParkingBoxModel::setParkingBoxID(int parkingBoxID)
{
    mBoxData.parkingBoxID_nu = static_cast<uint16_t>(parkingBoxID);
}

int ParkingBoxModel::getExistenceProb() const
{
    return static_cast<int>(mBoxData.existenceProb_perc);
}

void ParkingBoxModel::setExistenceProb(int existenceProb_perc)
{
    mBoxData.existenceProb_perc = static_cast<uint8_t>(existenceProb_perc);
}

EnumProperty ParkingBoxModel::getParkingScenario() const
{
    return mBoxData.parkingScenario_nu;
}

void ParkingBoxModel::setParkingScenario(EnumProperty parkingScenario_nu)
{
    mBoxData.parkingScenario_nu = static_cast<si::ParkingScenarioTypes>(parkingScenario_nu.asInt());
}

si::VirtualLineSerializable& ParkingBoxModel::getVirtualLine(uint8_t index)
{
    return mBoxData.virtualLines[index];
}

void ParkingBoxModel::setVirtualLinePoints(uint8_t index, QPointF point1, QPointF point2)
{
    setVirtualLinePoint(index, 0U, point1);
    setVirtualLinePoint(index, 1U, point2);

    mBoxData.numVirtualLines_nu = 0U;
    for (uint8_t i{ 0U }; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU; i++) {
        if (mBoxData.virtualLines[i].virtLineVertices_m.actualSize != 0U && isVirtualLineVisible(i)) {
            mBoxData.numVirtualLines_nu++;
        }
    }
}

void ParkingBoxModel::setVirtualLinePoint(uint8_t vlIndex, uint8_t pIndex, QPointF point)
{
    if (vlIndex < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU && pIndex < 2U &&
            (mBoxData.virtualLines[vlIndex].virtLineVertices_m.actualSize == 2U)) {
        mBoxData.virtualLines[vlIndex].virtLineVertices_m.array[pIndex].x_dir = static_cast<float32_t>(point.x());
        mBoxData.virtualLines[vlIndex].virtLineVertices_m.array[pIndex].y_dir = static_cast<float32_t>(point.y());
        emit shapeChanged();
        emit virtualLinesChanged();
    } else {
        // Do nothing. Throw exception?
    }
}

bool ParkingBoxModel::isVirtualLineVisible(uint8_t vlIndex) const {
    si::VirtualLineSerializable vl{ mBoxData.virtualLines[vlIndex] };
    return vl.virtLineVertices_m.array[0U].x_dir != 0.0F || vl.virtLineVertices_m.array[0U].y_dir != 0.0F ||
        vl.virtLineVertices_m.array[1U].x_dir != 0.0F || vl.virtLineVertices_m.array[1U].y_dir != 0.0F;
}


