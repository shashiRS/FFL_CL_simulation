#include "staticobstaclemodel.h"

StaticObstacleModel::StaticObstacleModel(QObject *parent) : QObject(parent)
{
    memset(&mStructureData, 0, sizeof(mStructureData));
}

void StaticObstacleModel::setStructureData(const si::StaticObjectSerializable &structure)
{
    int prevCount = mStructureData.objShape_m.actualSize;
    mStructureData = structure;
    emit shapeChanged(prevCount);
    emit visibilityChanged(isVisible());
    emit dataChanged();

    /*
    if(mShape != structure) {
        mShape = structure;
        emit shapeChanged();
    } */
}

void StaticObstacleModel::updateShapePolygon(const si::StaticObjectSerializable& object)
{
    int prevCount = mStructureData.objShape_m.actualSize;
    memcpy(&mStructureData.objShape_m, &object.objShape_m, sizeof(mStructureData.objShape_m));
    emit shapeChanged(prevCount);
    emit dataChanged();
}

void StaticObstacleModel::setHeightType(si::StaticObjHeigthType height) {
    mStructureData.objHeightClass_nu = height;
    emit dataChanged();
}

void StaticObstacleModel::setHeightTypeProp(const EnumProperty &prop) {
    if(prop.asInt() < (int)si::StaticObjHeigthType::MAX_NUM_HEIGHT_TYPES) {
        mStructureData.objHeightClass_nu = (si::StaticObjHeigthType)prop.asInt();
        emit dataChanged();
    }
}

#ifndef ULTRASONIC_ONLY
EnumProperty StaticObstacleModel::getObjClass() const {
    return mStructureData.refObjClass_nu;
}

void StaticObstacleModel::setObjClass(const EnumProperty &prop) {
    if(prop.asInt() < static_cast<int>(si::StaticObjectClass::STAT_OBJ_MAX_NUM_TYPES)) {
        mStructureData.refObjClass_nu = static_cast<si::StaticObjectClass>(prop.asInt());
        emit dataChanged();
    }
}
#endif

void StaticObstacleModel::setVisible(bool visible)
{
    auto newProb = visible? 100 : 0;
    if(mStructureData.existenceProb_perc != newProb) {
        mStructureData.existenceProb_perc = newProb;
        emit visibilityChanged(isVisible());
        emit dataChanged();
    }
}
