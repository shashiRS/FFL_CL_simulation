#ifndef VEHICLEPATHMODEL_H
#define VEHICLEPATHMODEL_H

#include "vehiclemodel.h"
#include "parkingscenemodel.h"

class VehiclePathModel : public VehicleModel
{
    Q_OBJECT
public:
    VehiclePathModel(ParkingSceneModel* parkingModel, QObject* parent = nullptr);

public slots:
    void onPathChanged();

private:
    ParkingSceneModel* mParkingModel = nullptr;

    int mPathIndex = 0;
};

#endif // VEHICLEPATHMODEL_H
