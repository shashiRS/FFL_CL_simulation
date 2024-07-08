#ifndef VEHICLEMODELWIDGET_H
#define VEHICLEMODELWIDGET_H

#include <QWidget>

#include "models/vehiclemodel.h"

namespace Ui {
class VehicleModelWidget;
}

class VehicleModelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit VehicleModelWidget(QWidget *parent = 0);
    ~VehicleModelWidget();

    void setModel(VehicleModel* model);

private slots:
    void onVehiclePosChanged();
    void onInputChanged();

private:
    Ui::VehicleModelWidget *ui;

    VehicleModel* mModel = 0;

    bool mChangingPos = false;
};

#endif // VEHICLEMODELWIDGET_H
