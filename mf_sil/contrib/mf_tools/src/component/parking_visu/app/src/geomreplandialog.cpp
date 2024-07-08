#include "geomreplandialog.h"
#include "ui_geomreplandialog.h"

GeomReplanDialog::GeomReplanDialog(ParkingSceneModel* sceneModel, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::GeomReplanDialog),
    mModel(sceneModel)
{
    ui->setupUi(this);
    this->setWindowTitle("Trjpla Meta Data");

    /* define the entries of the combo boxes according to the enum property descriptions
        this way, only these have to be changed if the enum classes of the properties are changed */
    const auto drvDirEnum = EnumPropertyDescription::getForType<ap_common::DrivingDirection>()->nameMap;
    for (auto i = 0; i < drvDirEnum.size(); i++) {
        ui->DrivingDirectionComboBox->addItem(drvDirEnum.value((int)i, "<Unknown>"));
    }

    const auto steerDirEnum = EnumPropertyDescription::getForType<ap_common::SteeringDirection>()->nameMap;
    for (auto i = 0; i < steerDirEnum.size(); i++) {
        ui->SteeringDirectionComboBox->addItem(steerDirEnum.value((int)i, "<Unknown>"));
    }

    const auto ReplanTriggerEnum = EnumPropertyDescription::getForType<ap_tp::ReplanTrigger>()->nameMap;
    for (auto i = 0; i < ReplanTriggerEnum.size(); i++) {
        ui->ReplanTriggerComboBox->addItem(ReplanTriggerEnum.value((int)i, "<Unknown>"));
    }
}

GeomReplanDialog::~GeomReplanDialog()
{
    delete ui;
}

void GeomReplanDialog::loadMetaDataFromModel()
{
    // get the data from the internal models to display them as default in the dialogue box
    double inflDist = mModel->getVehInflRadius_m();
    EnumProperty drvDir = mModel->getStartPoseModel()->getDrivingDirection();
    EnumProperty steerDir = mModel->getStartPoseModel()->getSteeringDirection();
    EnumProperty trigger = mModel->getReplanTrigger();

    ui->InflationDistanceSpinBox->setValue(inflDist);
    ui->DrivingDirectionComboBox->setCurrentIndex((drvDir.asInt()));
    ui->SteeringDirectionComboBox->setCurrentIndex((steerDir.asInt()));
    ui->ReplanTriggerComboBox->setCurrentIndex((trigger.asInt()));
}

double GeomReplanDialog::getInflDistFromDialog() {
    return ui->InflationDistanceSpinBox->value();
}

int GeomReplanDialog::getDrvDirFromDialog()
{
    return ui->DrivingDirectionComboBox->currentIndex();
}

int GeomReplanDialog::getSteerDirFromDialog()
{
    return ui->SteeringDirectionComboBox->currentIndex();
}

int GeomReplanDialog::getReplanTriggerFromDialog()
{
    return ui->ReplanTriggerComboBox->currentIndex();
}
