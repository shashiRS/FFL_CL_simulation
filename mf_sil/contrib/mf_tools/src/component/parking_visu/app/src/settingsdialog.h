#ifndef SETTINGSDIALOG_H
#define SETTINGSDIALOG_H

#include <QDialog>
#include <QSettings>

namespace Ui {
class SettingsDialog;
}

struct PlannerVisuSettingsKeys {
    static constexpr auto KEY_TRAJ_PARAM_DIR = "TRAJ_PARAM_DIR";
    static constexpr auto KEY_TAPOSD_PARAM_DIR = "TAPOSD_PARAM_DIR";
    static constexpr auto KEY_VEHICLE_PARAM_DIR = "VEHICLE_PARAM_DIR";
    static constexpr auto KEY_SYSFUNC_PARAM_DIR = "SYSFUNC_PARAM_DIR";
    static constexpr auto KEY_DLL_DIR = "DLL_DIR";
    static constexpr auto KEY_EM_DIR = "EM_DIR";
    static constexpr auto KEY_EM_DIR_HIST = "EM_DIR_HIST";
    static constexpr auto KEY_REACH_DIR = "KEY_REACH";
    static constexpr auto KEY_MEASUREMENT_DIR = "KEY_MEASUREMENT";
    static constexpr auto KEY_LOAD_PARAMETERS_ON_STARTUP = "LOAD_PARAMETERS_ON_STARTUP";
    static constexpr auto KEY_LOAD_DLL_ON_STARTUP = "LOAD_DLL_ON_STARTUP";
};

class SettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SettingsDialog(QWidget *parent = 0);
    ~SettingsDialog();

    void loadFromSettings(QSettings const& settings);

    bool isLoadLastParametersChecked() const;

    bool isLoadLastDLLChecked() const;

private:
    Ui::SettingsDialog *ui;
};

#endif // SETTINGSDIALOG_H
