#include "vehiclemodelwidget.h"
#include "ui_vehiclemodelwidget.h"


VehicleModelWidget::VehicleModelWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::VehicleModelWidget)
{
    ui->setupUi(this);

    connect(ui->posXDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onInputChanged()));
    connect(ui->posYDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onInputChanged()));
    connect(ui->angleDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(onInputChanged()));
}

VehicleModelWidget::~VehicleModelWidget()
{
    delete ui;
}

void VehicleModelWidget::setModel(VehicleModel *model)
{
    if(mModel)
        disconnect(mModel, &VehicleModel::positionChanged, this, &VehicleModelWidget::onVehiclePosChanged);

    mModel = nullptr;

    ui->posXDoubleSpinBox->setValue(model->getPosX_m());
    ui->posYDoubleSpinBox->setValue(model->getPosY_m());
    ui->angleDoubleSpinBox->setValue(model->getPosYawAngle_Rad());

    mModel = model;
    connect(mModel, &VehicleModel::positionChanged, this, &VehicleModelWidget::onVehiclePosChanged);
}

void VehicleModelWidget::onVehiclePosChanged()
{
    // set a flag indicating that we currently manually modify the spinboxes
    // so that the valueChangedSignal can be ignored
    mChangingPos = true;
    ui->posXDoubleSpinBox->setValue(mModel->getPosX_m());
    ui->posYDoubleSpinBox->setValue(mModel->getPosY_m());
    ui->angleDoubleSpinBox->setValue(mModel->getPosYawAngle_Rad());
    mChangingPos = false;
}

void VehicleModelWidget::onInputChanged()
{
    // apply only if this comes from user input and not from model update
    if(mModel && !mChangingPos)
        mModel->setPos(ui->posXDoubleSpinBox->value(), ui->posYDoubleSpinBox->value(), ui->angleDoubleSpinBox->value());
}
