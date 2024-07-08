#ifndef ULTRASONIC_ONLY
#include "parkingspacemarkingmodel.h"

ParkingSpaceMarkingModel::ParkingSpaceMarkingModel(QObject *parent) : QObject(parent)
{
    memset(&mData, 0, sizeof(mData));
    mData.pos_m.array[0] = cml::Vec2Df_POD{ 0.0F, 0.0F };
    mData.pos_m.array[1] = cml::Vec2Df_POD{ 0.0F, 1.0F };
    mData.pos_m.actualSize = 2U;
}

void ParkingSpaceMarkingModel::setMarking(const si::ParkingSpaceMarkingsSerializable &marking)
{
    mData = marking;
    emit markingChanged();
}

void ParkingSpaceMarkingModel::setLinePosition(const QPointF &p1, const QPointF &p2)
{
    mData.pos_m.array[0] = cml::Vec2Df_POD{ float32_t(p1.x()), float32_t(p1.y()) };
    mData.pos_m.array[1] = cml::Vec2Df_POD{ float32_t(p2.x()), float32_t(p2.y()) };
    emit markingChanged();
}

EnumProperty ParkingSpaceMarkingModel::getMarkingType() const
{
    return mData.type_nu;
}

void ParkingSpaceMarkingModel::setMarkingType(const EnumProperty markingType)
{
    if(markingType.asInt() < static_cast<int>(si::ParkingLineType::PLT_MAX_NUM_COLORS)) {
        mData.type_nu = static_cast<si::ParkingLineType>(markingType.asInt());
        emit markingChanged();
    }
}

bool ParkingSpaceMarkingModel::isVisible() const
{
    return mData.existenceProb_perc > 0;
}

void ParkingSpaceMarkingModel::setVisible(bool visible)
{
    mData.existenceProb_perc = visible? 100 : 0;
    emit markingChanged();
}

uint8_t ParkingSpaceMarkingModel::getExistanceProbability() const
{
    return mData.existenceProb_perc;
}

void ParkingSpaceMarkingModel::setExistanceProbability(uint8_t existProb)
{
    mData.existenceProb_perc = existProb;
    emit markingChanged();
}

float ParkingSpaceMarkingModel::getWidth() const
{
    return mData.width_m;
}

void ParkingSpaceMarkingModel::setWidth(float width)
{
    mData.width_m = static_cast<float32_t>(width);
    emit markingChanged();
}
#endif