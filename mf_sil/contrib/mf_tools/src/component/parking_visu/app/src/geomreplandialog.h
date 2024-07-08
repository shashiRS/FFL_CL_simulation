#ifndef GEOMREPLANDIALOG_H
#define GEOMREPLANDIALOG_H

#include <QDialog>
#include <QSettings>

#include <ap_common/ap_common_generated_types.h>

#include "models/enumproperty.h"
#include "models/parkingscenemodel.h"

namespace Ui {
    class GeomReplanDialog;
}


class GeomReplanDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GeomReplanDialog(ParkingSceneModel* sceneModel, QWidget *parent = 0);
    ~GeomReplanDialog();

    void loadMetaDataFromModel();
    double getInflDistFromDialog();
    int getDrvDirFromDialog();
    int getSteerDirFromDialog();
    int getReplanTriggerFromDialog();
    
private:
    Ui::GeomReplanDialog *ui;

    ParkingSceneModel* mModel;
};

#endif // GEOMREPLANDIALOG_H
