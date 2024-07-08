#include "settingsdialog.h"
#include "ui_settingsdialog.h"

SettingsDialog::SettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SettingsDialog)
{
    ui->setupUi(this);
}

SettingsDialog::~SettingsDialog()
{
    delete ui;
}

void SettingsDialog::loadFromSettings(const QSettings &settings)
{
    ui->loadLastParametersOnStartupCheckBox->setChecked(settings.value(PlannerVisuSettingsKeys::KEY_LOAD_PARAMETERS_ON_STARTUP, true).toBool());
    ui->loadLastParkingPluginDLLOnStartupCheckBox->setChecked(settings.value(PlannerVisuSettingsKeys::KEY_LOAD_DLL_ON_STARTUP, true).toBool());
}

bool SettingsDialog::isLoadLastParametersChecked() const
{
    return ui->loadLastParametersOnStartupCheckBox->isChecked();
}

bool SettingsDialog::isLoadLastDLLChecked() const
{
    return ui->loadLastParkingPluginDLLOnStartupCheckBox->isChecked();
}
