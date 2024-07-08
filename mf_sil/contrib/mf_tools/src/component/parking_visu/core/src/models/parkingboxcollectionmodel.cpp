#include "parkingboxcollectionmodel.h"
#include "parkingscenemodel.h"

ParkingBoxCollectionModel::ParkingBoxCollectionModel(ParkingSceneModel *sceneModel)
    : QObject(sceneModel)
    , mSceneModel(sceneModel)
{
}

void ParkingBoxCollectionModel::setParkingBoxes(int num, const si::ParkingBoxSerializable *boxes)
{
    bool changed = (num > 0) ? true : false;
    for(int i = 0; i < num; i++) {
        if(childCount() <= i) {
            auto model = new ParkingBoxModel(this);
            connect(model, &ParkingBoxModel::shapeChanged, this, &ParkingBoxCollectionModel::onDataChanged);
            connect(model, &ParkingBoxModel::virtualLinesChanged, this, &ParkingBoxCollectionModel::onDataChanged);
            emit parkingBoxAdded(model);
        }

        getChildModel(i)->setBoxData(boxes[i]);
    }

    while(childCount() > num) {
        auto model = getChildModel(childCount()-1);
        delete model;
        emit parkingBoxRemoved(model);
        changed = true;
    }

    if (changed) {
        emit dataChanged();
    }
}

ParkingBoxModel *ParkingBoxCollectionModel::getChildModel(int index)
{
    return dynamic_cast<ParkingBoxModel*>(children()[index]);
}

void ParkingBoxCollectionModel::addParkingBox()
{
    if (childCount() < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU) {
        si::ParkingBoxSerializable data;
        memset(&data, 0, sizeof(data));
        data.existenceProb_perc = 100;
        data.parkingBoxID_nu = childCount();
        data.slotCoordinates_m.array[0] = cml::Vec2Df_POD{ 1.0F, 1.0F };
        data.slotCoordinates_m.array[1] = cml::Vec2Df_POD{ 0.0F, 1.0F };
        data.slotCoordinates_m.array[2] = cml::Vec2Df_POD{ 0.0F, 0.0F };
        data.slotCoordinates_m.array[3] = cml::Vec2Df_POD{ 1.0F, 0.0F };

        auto model = new ParkingBoxModel(this);
        model->setBoxData(data);

        emit parkingBoxAdded(model);
        emit dataChanged();

        mSceneModel->createHistoryPoint();
    }
}

void ParkingBoxCollectionModel::removeParkingBox(int index)
{
    auto model = getChildModel(index);
    model->setParent(nullptr); // remove box from children of this
    emit parkingBoxRemoved(model);
    emit dataChanged();
    delete model;

    mSceneModel->createHistoryPoint();
}

void ParkingBoxCollectionModel::onDataChanged()
{
    emit dataChanged();
}
