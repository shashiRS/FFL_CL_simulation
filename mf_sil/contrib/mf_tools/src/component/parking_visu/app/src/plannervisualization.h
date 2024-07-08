#ifndef PLANNERVISUALIZATION_H
#define PLANNERVISUALIZATION_H

#define WIN64_LEAN_AND_MEAN
#include <QMainWindow>
#include <QSettings>
#include <QFileDialog>
#include "customtoolbutton.h"

#include "models/vehiclemodel.h"
#include "models/parkingscenemodel.h"
#include "clienthandler.h"
#include "ui/items/vehicleghostitem.h"
#include "ui/objectpropertiesmodel.h"
#include "ui/objectpropertiesitemdelegate.h"
#include "trajectorygraphview.h"
#include "ui/targetposereachableareawidget.h"
#include "ui/debugdrawer.h"

namespace Ui {
class PlannerVisualization;
}

class PlannerVisualization : public QMainWindow
{
    Q_OBJECT

public:
    explicit PlannerVisualization(bool startClientProcess = true, QWidget *parent = nullptr);
    ~PlannerVisualization();

    /**
     * @brief grabJSONScene
     * @param filename
     * @return emtry string on success, error message on error
     */
    QString grabJSONScene(QString const& jsonName, const QString &pngName, int width, int height, const QDir &parameterPathTrjpla, const QDir &parameterPathSysFunc, const QDir &parameterPathVeh);

private slots:
    void onStartScanMode(bool enable);
    void onRunForScan();
    void onToggleScanModeTaposd(bool start);
    //void onRunForScanTaposd();
    void onPlanNewPath();
    void onReplanPath();
    void onPlanGeometricPath();
    void onPlanGaragePark();

    void onDrawTargetPoseReachableArea2D();
    void onDrawTargetPoseReachableArea3D();

    void onPathIndexChanged();
    void onVehiclePathChanged();
    void onMakeStep();
    void onJump();

    void onShowSettings();

    void onShowTrajectoryPlot();
    void onDataChanged();

    void onExportFakeEM();
    void onSaveReach();
    void onContinunousReplanningToggled(bool active);
    void doContinuousReplan();
    void planGeomWithReplan();
    void planGeomWithoutReplan();

    //void onDefineReachabilityArea(bool active);

protected:
    virtual void closeEvent(QCloseEvent *event) override;

    virtual void dropEvent(QDropEvent* event) override;
    virtual void dragEnterEvent(QDragEnterEvent *event) override;

private:
    static constexpr auto MaxNumEMHistory = 5;

    QAction* actionPlanGeomWithReplan;
    QAction* actionPlanGeomWithoutReplan;
    QMenu* menu;
    QLabel* mDistanceLabel = nullptr;
    QLabel* mStatusLabel = nullptr;
    CustomToolButton* button;
    Ui::PlannerVisualization *ui;

    ParkingSceneModel mParkingSceneModel;

    VehicleGhostItem * mGhostVehicle = nullptr;

    DebugDrawer* mDebugDrawer = nullptr; /**< Debug draw for mTrajHandler */

    ClientHandler mTrajHandler;

    void tryLoadSettings(QFileDialog& dialog, void (ClientHandler::*loadFunc)(QString), QString settingKey);

    QSettings mSettings;
    QFileDialog mTrajParamsFileDialog;
    QFileDialog mVehicleParamsFileDialog;
    QFileDialog mSysFuncParamsFileDialog;
    QFileDialog mDLLFileDialog;
    QFileDialog mEMFileDialog;
    QFileDialog mREACHFileDialog;

    ObjectPropertiesModel mSelectedObjectPropertiesModel;

    TrajectoryGraphView mTrajectoryGraphView;
    
    /**
     * @brief Reload parameter files if paths are set in mSettings
     *
     * Nothing is done if at least one of the required path is not set in settings
     */
    void reloadParametersIfSet();

    /**
     * @brief Reload plugin DLL if path is set in mSettings
     *
     * Nothing is done if path is not set in settings
     */
    void reloadDLLIfSet();

    /**
     * @brief For the plan/replan/step actions adjust their enabled state
     * according to current computation and planner state
     */
    void adjustPlanActionsState();

    void updatePathIndexLabel();
    void display3DReachablityArea();

    void updateRecentEMsMenuHistory();

    std::list<std::unique_ptr<QAction>> mRecentEMsActions;
};

#endif // PLANNERVISUALIZATION_H
