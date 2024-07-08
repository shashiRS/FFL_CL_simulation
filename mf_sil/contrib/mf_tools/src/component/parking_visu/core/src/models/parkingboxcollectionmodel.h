#ifndef PARKINGBOXCOLLECTIONMODEL_H
#define PARKINGBOXCOLLECTIONMODEL_H

#include <QObject>

#include <si/parking_box_serializable.h>

#include "parkingboxmodel.h"

class ParkingSceneModel;

class ParkingBoxCollectionModel : public QObject
{
    Q_OBJECT
public:
    explicit ParkingBoxCollectionModel(ParkingSceneModel *sceneModel);

    void setParkingBoxes(int num, const si::ParkingBoxSerializable *boxes);

    ParkingBoxModel *getChildModel(int index);

    int childCount() const {
        return children().size(); // TODO
    }

    void addParkingBox();
    void removeParkingBox(int index);

signals:
    void parkingBoxAdded(ParkingBoxModel* model);
    void parkingBoxRemoved(ParkingBoxModel* model);
    void dataChanged();

public slots:
    void onDataChanged();

private:
    ParkingSceneModel* mSceneModel = nullptr;
};

#endif // PARKINGBOXCOLLECTIONMODEL_H
