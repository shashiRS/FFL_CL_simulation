#include "plannervisualization.h"
#ifndef ULTRASONIC_ONLY
    #include "ui_plannervisualization_premium_parking.h"
#else
    #include "ui_plannervisualization_ultrasonic_parking.h"
#endif

#include <QFileDialog>
#include <QFileInfo>
#include <QDebug>
#include <QMessageBox>
#include <QLabel>
#include <QMovie>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChartView>
#include <QMimeData>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <QInputDialog>

#include "settingsdialog.h"
#include "geomreplandialog.h"

using namespace QtCharts;

PlannerVisualization::PlannerVisualization(bool startClientProcess, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PlannerVisualization),
    mTrajHandler(&mParkingSceneModel),
#ifdef ULTRASONIC_ONLY
    mSettings("Continental", "AUP trajectory visu Ultrasonic Only"),
#else
    mSettings("Continental", "AUP trajectory visu Premium Parking"),
#endif
    mSelectedObjectPropertiesModel(&mParkingSceneModel),
    mTrajectoryGraphView(mParkingSceneModel.getTrajectorySetModel())
{
    ui->setupUi(this);
    // TODO: please fix inside the .ui file if you know how!
    ui->splitter->setStretchFactor(0, 1);
    ui->splitter->setStretchFactor(1, 50);
    ui->splitter->setStretchFactor(2, 0);

    ui->menu_File->addAction(ui->parkingWidget->getSaveEMAction());
    ui->mainToolBar->addAction(ui->parkingWidget->getMeasurementAction());
    ui->mainToolBar->addAction(ui->parkingWidget->getScreenshotAction());

    ui->menuTools->addAction(ui->parkingWidget->getScreenshotAction());
    ui->menuTools->addAction(ui->parkingWidget->getMeasurementAction());

    ui->parkingWidget->setModel(&mParkingSceneModel);

    updateRecentEMsMenuHistory();

    mGhostVehicle = new VehicleGhostItem(&mParkingSceneModel);
    ui->parkingWidget->scene()->addItem(mGhostVehicle);

    mDebugDrawer = new DebugDrawer(ui->parkingWidget->scene(), this, &mParkingSceneModel);
    mTrajHandler.setDebugDrawer(mDebugDrawer);

    ui->objectPropertiesView->setModel(&mSelectedObjectPropertiesModel);
    ui->objectPropertiesView->setItemDelegate(new ObjectPropertiesItemDelegate(this));
    ui->objectPropertiesView->setEditTriggers(QAbstractItemView::AllEditTriggers);
    connect(&mSelectedObjectPropertiesModel, &QAbstractItemModel::modelReset, ui->objectPropertiesView, &QTreeView::expandAll);

    ui->objectTreeWidget->setModel(&mParkingSceneModel);
    ui->objectTreeWidget->addItem(mDebugDrawer, "Debug Draw");

    onVehiclePathChanged();

    connect(mParkingSceneModel.getTrajectorySetModel(), &TrajectorySetModel::currentTrajectoryUpdated, this, &PlannerVisualization::onVehiclePathChanged);

    connect(&mParkingSceneModel, &ParkingSceneModel::loadedEMPathChanged, this, [this](QString filename) {
        setWindowTitle("Planner Visualization: " + filename);
        ui->parkingWidget->showAll(1);
        QFileInfo fileInfo(filename);
        mEMFileDialog.setDirectory(fileInfo.absoluteDir());
        mEMFileDialog.selectFile(fileInfo.absoluteFilePath());
        mSettings.setValue(PlannerVisuSettingsKeys::KEY_EM_DIR, filename);

        QStringList files = mSettings.value(PlannerVisuSettingsKeys::KEY_EM_DIR_HIST).toStringList();
        files.removeAll(filename);
        files.prepend(filename);
        while (files.size() > MaxNumEMHistory)
            files.removeLast();

        mSettings.setValue(PlannerVisuSettingsKeys::KEY_EM_DIR_HIST, files);

        updateRecentEMsMenuHistory();
    });

    {
        QFileInfo fileInfo(mSettings.value(PlannerVisuSettingsKeys::KEY_TRAJ_PARAM_DIR, "").toString());
        mTrajParamsFileDialog.setDirectory(fileInfo.absoluteDir());
        mTrajParamsFileDialog.selectFile(fileInfo.absoluteFilePath());
    }

    {
        QFileInfo fileInfo(mSettings.value(PlannerVisuSettingsKeys::KEY_VEHICLE_PARAM_DIR, "").toString());
        mVehicleParamsFileDialog.setDirectory(fileInfo.absoluteDir());
        mVehicleParamsFileDialog.selectFile(fileInfo.absoluteFilePath());
    }

    {
        QFileInfo fileInfo(mSettings.value(PlannerVisuSettingsKeys::KEY_SYSFUNC_PARAM_DIR, "").toString());
        mSysFuncParamsFileDialog.setDirectory(fileInfo.absoluteDir());
        mSysFuncParamsFileDialog.selectFile(fileInfo.absoluteFilePath());
    }

    {
        QFileInfo fileInfo(mSettings.value(PlannerVisuSettingsKeys::KEY_DLL_DIR, "").toString());
        mDLLFileDialog.setDirectory(fileInfo.absoluteDir());
        mDLLFileDialog.selectFile(fileInfo.absoluteFilePath());
    }

    {
        QFileInfo fileInfo(mSettings.value(PlannerVisuSettingsKeys::KEY_EM_DIR, "").toString());
        mEMFileDialog.setDirectory(fileInfo.absoluteDir());
        mEMFileDialog.selectFile(fileInfo.absoluteFilePath());
    }

    {
        QFileInfo fileInfo(mSettings.value(PlannerVisuSettingsKeys::KEY_REACH_DIR, "").toString());
        mREACHFileDialog.setDirectory(fileInfo.absoluteDir());
        mREACHFileDialog.selectFile(fileInfo.absoluteFilePath());
    }
    
    connect(ui->actionLoad_Trajectory_Planner_Parameters, &QAction::triggered, this, [this]() {
        tryLoadSettings(mTrajParamsFileDialog, &ClientHandler::loadTRJPLAParameters, PlannerVisuSettingsKeys::KEY_TRAJ_PARAM_DIR);});
    connect(ui->actionLoad_Vehicle_Parameters, &QAction::triggered, this, [this]() {
        tryLoadSettings(mVehicleParamsFileDialog, &ClientHandler::loadVehicleParameters, PlannerVisuSettingsKeys::KEY_VEHICLE_PARAM_DIR);});
    connect(ui->actionLoad_Sys_Func_Parameters, &QAction::triggered, this, [this]() {
        tryLoadSettings(mSysFuncParamsFileDialog, &ClientHandler::loadSysFuncParameters, PlannerVisuSettingsKeys::KEY_SYSFUNC_PARAM_DIR);});
    connect(ui->actionLoad_Trajectory_Planner_DLL, &QAction::triggered, this, [this]() {
        tryLoadSettings(mDLLFileDialog, &ClientHandler::loadComponentsDLL, PlannerVisuSettingsKeys::KEY_DLL_DIR);});
    connect(ui->actionLoad_Environment_Model, &QAction::triggered, this, [this]() {
        tryLoadSettings(mEMFileDialog, &ClientHandler::loadEM, PlannerVisuSettingsKeys::KEY_EM_DIR);
    });
    connect(ui->actionLoad_Json_Reachability , &QAction::triggered, this, [this]() {
        tryLoadSettings(mREACHFileDialog, &ClientHandler::loadREACH, PlannerVisuSettingsKeys::KEY_REACH_DIR);
    });
    
    connect(ui->actionReload_Last_Scene, &QAction::triggered, this, [this]() {
        reloadParametersIfSet();

        if(!mTrajHandler.isLibraryLoaded()) {
            reloadDLLIfSet();
        }

        mTrajHandler.loadEM(mSettings.value(PlannerVisuSettingsKeys::KEY_EM_DIR, "").toString());
    });

    connect(ui->actionStartScanMode, &QAction::toggled, this, &PlannerVisualization::onStartScanMode);
    connect(mParkingSceneModel.getStartPoseModel(), &DebugVehicleModel::vehicleMovedByKey, this, &PlannerVisualization::onRunForScan);
    connect(ui->actionPlan_Path, &QAction::triggered, this, &PlannerVisualization::onPlanNewPath);
    connect(ui->actionReplan_Path, &QAction::triggered, this, &PlannerVisualization::onReplanPath);
    connect(ui->actionMake_Step, &QAction::triggered, this, &PlannerVisualization::onMakeStep);
    connect(ui->actionJump, &QAction::triggered, this, &PlannerVisualization::onJump);
    //connect(ui->actionPlan_Geometrical_Path, &QAction::triggered, this, &PlannerVisualization::onPlanGeometricPath);

    connect(ui->actionPlan_GaragePark, &QAction::triggered, this, &PlannerVisualization::onPlanGaragePark);

    connect(ui->actionDraw_Target_Pose_Reachable_Area_2D, &QAction::toggled, this, &PlannerVisualization::onDrawTargetPoseReachableArea2D);
    connect(ui->actionDraw_Target_Pose_Reachable_Area_3D, &QAction::toggled, this, &PlannerVisualization::onDrawTargetPoseReachableArea3D);

    connect(&mParkingSceneModel,&ParkingSceneModel::targetPoseReachabilityAreaReady3D,this,&PlannerVisualization::display3DReachablityArea);

    connect(ui->btnApplyStartPose, &QPushButton::clicked, this, [this] () {
        const auto trajectory = mParkingSceneModel.getTrajectorySetModel()->getSelectedTrajectory();
        if(trajectory && !trajectory->isEmpty()) {
            if(mParkingSceneModel.getStartPoseModel()->setPos(trajectory->at(ui->pathStepSlider->value()).trans)) {
                mParkingSceneModel.createHistoryPoint();
            }
        }
    });

    connect(ui->pathStepSlider, &QSlider::valueChanged, mParkingSceneModel.getTrajectorySetModel(), &TrajectorySetModel::setCurrentPoseIndex);

    connect(mParkingSceneModel.getTrajectorySetModel(), &TrajectorySetModel::currentPoseIndexChanged, this, &PlannerVisualization::onPathIndexChanged);
    connect(&mParkingSceneModel, &ParkingSceneModel::dataChanged, this, &PlannerVisualization::onDataChanged);

    connect(ui->actionDefine_Reachability_Area, &QAction::toggled, this, [this] (bool active){
        if(active)
            ui->parkingWidget->startDefineReachabilityArea();
        else
            ui->parkingWidget->cancelDefineReachabilityArea();
    });

    connect(ui->parkingWidget,&ParkingSceneWidget::defineReachabilityAreaDone,this, [this](){
        ui->actionDefine_Reachability_Area->setChecked((false));
    });

    mStatusLabel = new QLabel("IDLE");

    QMovie* inProgressIcon = new QMovie(":/icons/IconHourglass.gif");
    inProgressIcon->setParent(this);
    inProgressIcon->setScaledSize(QSize(mStatusLabel->fontMetrics().height(), mStatusLabel->fontMetrics().height()));

    auto inProgressIconLabel = new QLabel();
    inProgressIconLabel->setMovie(inProgressIcon);
    inProgressIconLabel->setVisible(false);

    mDistanceLabel = new QLabel();

    auto inProgressLabel = new QLabel("Computation in progress ...");
    inProgressLabel->setVisible(false);

    statusBar()->addPermanentWidget(mStatusLabel);
    statusBar()->addPermanentWidget(mDistanceLabel, 1);
    statusBar()->addPermanentWidget(inProgressIconLabel);
    statusBar()->addPermanentWidget(inProgressLabel);

    connect(&mTrajHandler, &ClientHandler::trajPlanStateChanged, this, [this](ap_tp::TrajPlanState state) {
        mStatusLabel->setText(EnumPropertyDescription::getForType<ap_tp::TrajPlanState>()->nameMap.value((int)state, "<Unknown>"));
    });

    connect(&mTrajHandler, &ClientHandler::distanceToStopChanged, this, [this](float dist) {
        if(std::isfinite(dist)) {
            mDistanceLabel->setText("Distance to stop: " + QString::number(dist));
        } else {
            mDistanceLabel->clear();
        }
    });
    // inProgressIcon = inProgressIcon.scaledToHeight(statusLabel->fontMetrics().height());

    connect(&mTrajHandler, &ClientHandler::computationInProgressChanged, this, [this, inProgressIcon, inProgressIconLabel, inProgressLabel](bool inProgress) {
        qDebug() << "inProgress" << inProgress;
        inProgressIcon->start();
        inProgressIconLabel->setVisible(inProgress);
        inProgressLabel->setVisible(inProgress);

        adjustPlanActionsState();
    });

    connect(ui->actionUndo, &QAction::triggered, this, [this]() {
        mParkingSceneModel.undo();
    });

    connect(ui->actionRedo, &QAction::triggered, this, [this]() {
        mParkingSceneModel.redo();
    });

    connect(&mTrajHandler, &ClientHandler::planningFailed, this, [this] (QString msg) {
        if(!ui->actionPlan_Geometrical_Path->isChecked())
        QMessageBox::critical(this, "Planning failed", "Failed to plan new path: " + msg);
    });

    connect(ui->actionSettings, &QAction::triggered, this, &PlannerVisualization::onShowSettings);

    connect(ui->actionEnable_Debug_Draw, &QAction::toggled, &mTrajHandler, &ClientHandler::enableDebugDraw);

    connect(ui->actionShow_All, &QAction::triggered, this, [this] {
       ui->parkingWidget->showAll(1.0);
    });

    connect(ui->actionAbout, &QAction::triggered, this, [this] {
        QMessageBox::information(this, "About Parking Trajectory Visualization",
                                 "Parking Trajectory Visualization tool\n© 2020 Nicolas Stein, Continental Teves AG & Co. oHG");
    });

    connect(ui->actionGrab_to_clipboard, &QAction::triggered, ui->parkingWidget, &ParkingSceneWidget::takeScreenshotToClipboard);

    connect(ui->actionStart_TAPOSD, &QAction::toggled, this, &PlannerVisualization::onToggleScanModeTaposd);

    if(mSettings.value(PlannerVisuSettingsKeys::KEY_LOAD_PARAMETERS_ON_STARTUP, true).toBool()) {
        reloadParametersIfSet();
    }

    connect(&mTrajHandler, &ClientHandler::clientConnected, this, [this] {
        if(mSettings.value(PlannerVisuSettingsKeys::KEY_LOAD_DLL_ON_STARTUP, true).toBool()) {
            reloadDLLIfSet();
        }
    });

    connect(ui->actionShow_Trajectory_Curvature, &QAction::triggered, this, &PlannerVisualization::onShowTrajectoryPlot);

    ui->loadedScenesList->setModel(mParkingSceneModel.getLoadedScenesModel());

    connect(ui->loadedScenesList->selectionModel(), &QItemSelectionModel::currentRowChanged, this,
            [this](const QModelIndex &current, const QModelIndex &) {
        mParkingSceneModel.selectEMbyIndex(current.row());
    });

    connect(&mParkingSceneModel, &ParkingSceneModel::selectedEMIndexChanged, this, [this](int index) {
        ui->loadedScenesList->selectionModel()->select(ui->loadedScenesList->model()->index(index, 0),
                                                       QItemSelectionModel::SelectCurrent | QItemSelectionModel::Rows);
    });

    connect(ui->addEMButton, &QPushButton::clicked, &mParkingSceneModel, &ParkingSceneModel::addNewSceneEntry);
    connect(ui->removeEMButton, &QPushButton::clicked, &mParkingSceneModel, &ParkingSceneModel::removeSelectedSceneEntry);

    connect(ui->actionExport_FakeEM_for_car, &QAction::triggered, this, &PlannerVisualization::onExportFakeEM);
    connect(ui->actionSave_Json_Reachability, &QAction::triggered, this, &PlannerVisualization::onSaveReach);

    connect(mParkingSceneModel.getStartPoseModel(), &DebugVehicleModel::positionChanged, this, &PlannerVisualization::doContinuousReplan);

    connect(ui->actionContinuous_Replanning, &QAction::toggled, this, &PlannerVisualization::onContinunousReplanningToggled);

    if(startClientProcess) {
        mTrajHandler.startServer();
        mTrajHandler.startClientProcessIfNotRunning();
    }

    actionPlanGeomWithReplan = new QAction("Plan Geom (Replan On)",this);
    actionPlanGeomWithReplan->setIcon(QIcon(":/icons/IconPlanGeomWithRe.png"));
    connect(actionPlanGeomWithReplan, &QAction::triggered, this, &PlannerVisualization::planGeomWithReplan);
    actionPlanGeomWithoutReplan = new QAction("Plan Geom (Replan Off)",this);
    actionPlanGeomWithoutReplan->setIcon(QIcon(":/icons/IconGeometric.png"));
    connect(actionPlanGeomWithoutReplan, &QAction::triggered, this, &PlannerVisualization::planGeomWithoutReplan);
    menu = new QMenu;
    menu->addAction(actionPlanGeomWithReplan);
    menu->addAction(actionPlanGeomWithoutReplan);
    button = new CustomToolButton;
    button->setPopupMode(QToolButton::MenuButtonPopup);
    button->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    button->setMenu(menu);
    button->setDefaultAction(actionPlanGeomWithoutReplan);
    // insert button left of action in insertWidget(action, button)
    ui->mainToolBar->insertWidget(ui->actionContinuous_Replanning, button);
}

PlannerVisualization::~PlannerVisualization()
{
    delete ui;
}

QString PlannerVisualization::grabJSONScene(const QString &jsonName, const QString& pngName, int width, int height, const QDir& parameterPathTrjpla, const QDir& parameterPathSysFunc, const QDir& parameterPathVeh)
{
    mTrajHandler.loadTRJPLAParameters(parameterPathTrjpla.absoluteFilePath("FC_TRJPLA_Params_config.json"));
    mTrajHandler.loadVehicleParameters(parameterPathSysFunc.absoluteFilePath("Vehicle_Params_config.json"));
    mTrajHandler.loadSysFuncParameters(parameterPathVeh.absoluteFilePath("Sys_Func_Params_config.json"));

    mParkingSceneModel.loadEM(jsonName);
    ui->parkingWidget->viewport()->resize(width, height);
    ui->parkingWidget->showAll(1.0);
    ui->parkingWidget->viewport()->grab().save(pngName);
    return "";
}

void PlannerVisualization::onDrawTargetPoseReachableArea2D()
{
    QString msg;
    mParkingSceneModel.resetTargetPoseReachableArea();
    if(ui->actionDraw_Target_Pose_Reachable_Area_2D->isChecked()){
        if(!mTrajHandler.findReachableArea(&msg,false)) {
            QMessageBox::critical(this, "Planning failed", "Failed to find a traj for current pose: " + msg);
        }
    }
}

void PlannerVisualization::onDrawTargetPoseReachableArea3D()
{
    QString msg;
    mParkingSceneModel.resetTargetPoseReachableArea();
    if(ui->actionDraw_Target_Pose_Reachable_Area_3D->isChecked()){
        if(!mTrajHandler.findReachableArea(&msg,true)) {
            QMessageBox::critical(this, "Planning failed", "Failed to find a traj for current pose: " + msg);
        }
    }
}

void PlannerVisualization::onStartScanMode(bool enable)
{
    QString msg;
    if(!mTrajHandler.startScanModeTrjpla(enable, &msg)) {
        QMessageBox::critical(this, "Planning failed", "Failed to find a traj for current pose: " + msg);
    }
}

void PlannerVisualization::onRunForScan()
{
    if(ui->actionStartScanMode->isChecked()){
        QString msg;
        if(!mTrajHandler.runForScanTrjpla(&msg)) {
            QMessageBox::critical(this, "Planning failed", "Failed to find a traj for current pose: " + msg);
        }
    }
}

void PlannerVisualization::onToggleScanModeTaposd(bool start)
{
    if(start) {
        QString msg;
        if(!mTrajHandler.startAndRunScanModeTaposd(&msg)) {
            qDebug() << "Failed to reach TAPOSD scanning mode in current scenario: " + msg;
        }
    }
}

void PlannerVisualization::onDataChanged()
{
    onToggleScanModeTaposd (ui->actionStart_TAPOSD->isChecked());

    doContinuousReplan();
}

void PlannerVisualization::onExportFakeEM()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Export FakeEM"), mParkingSceneModel.getLoadedEMFilename(),
                                                    tr("JSON files (*.json);;All Files (*)"));

    if(!filename.isEmpty()) {
        mParkingSceneModel.saveEM(filename, true);
    }
}

void PlannerVisualization::onSaveReach()
{
    QString filename = QFileDialog::getSaveFileName(this,
        tr("Save Reachability Area"), mParkingSceneModel.getLoadedEMFilename(),
        tr("JSON files (*.json);;All Files (*)"));

    if (!filename.isEmpty()) {
        mParkingSceneModel.saveReach(filename);
    }

}

void PlannerVisualization::onContinunousReplanningToggled(bool active)
{
    // this is ugly
    static QMetaObject::Connection targetPoseConnection;

    if(active) {
        auto tp = mParkingSceneModel.getSelectedTargetPoseModel();

        if(tp) {
            ui->actionPlan_Geometrical_Path->setCheckable(true);
            ui->actionPlan_Geometrical_Path->setChecked(true);

            targetPoseConnection = connect(tp, &TargetPoseModel::positionChanged, this, &PlannerVisualization::doContinuousReplan);
        }
    } else {
        ui->actionPlan_Geometrical_Path->setCheckable(false);

        this->disconnect(targetPoseConnection);
    }
}

void PlannerVisualization::doContinuousReplan()
{
    if(ui->actionPlan_Geometrical_Path->isChecked()) {
        mTrajHandler.planNewGeomPath();
    }
}

void PlannerVisualization::onPlanNewPath()
{
    QString msg;
    if(!mTrajHandler.planNewPath(&msg)) {
        QMessageBox::critical(this, "Planning failed", "Failed to plan new path: " + msg);
    }
}

void PlannerVisualization::onReplanPath()
{
    QString msg;
    if(!mTrajHandler.replanExistingPath(&msg)) {
        QMessageBox::critical(this, "Planning failed", "Failed to replan path: " + msg);
    }
}

void PlannerVisualization::onPlanGeometricPath()
{
    QString msg;
    if(!mTrajHandler.planNewGeomPath(&msg)) {
        QMessageBox::critical(this, "Planning failed", "Failed to plan new path: " + msg);
    }
}

void PlannerVisualization::onPlanGaragePark()
{
    QString msg;
    if (!mTrajHandler.planNewGaragePark(&msg)) {
        QMessageBox::critical(this, "Planning failed", "Failed to plan new path: " + msg);
    }
}

void PlannerVisualization::onMakeStep()
{
    QString msg;
    if(!mTrajHandler.makeStepOnPath(/*[this]()
                                    {
                                                                    return QMessageBox::question(this, "Continue stepping", "The planned path is in collision.\n"
                                                                                                 "Press 'yes' if you want to jump directly to the collision point and perform replanning, "
                                                                                                 "'no' to continue manual stepping.",
                                                                                                 QMessageBox::Yes|QMessageBox::No) == QMessageBox::Yes;
                                    }, */ &msg)) {
        QMessageBox::critical(this, "Stepping failed", "Failed to follow path: " + msg);
    }
}

void PlannerVisualization::onJump()
{
    QString msg;
    bool ok = false;
    double ReqDistToStop = QInputDialog::getDouble(this, tr("QInputDialog::JumpTo()"),
                                         tr("Jump to Distance to stop:"), 0, 0, 1000, 2, &ok);
    if (!ok || !mTrajHandler.jumpOnPath(ReqDistToStop, &msg))
       QMessageBox::critical(this, "Jumping failed", "Failed to jump to distance to Stop: " + msg);

}

void PlannerVisualization::onShowSettings()
{
    SettingsDialog dialog(this);
    dialog.loadFromSettings(mSettings);
    if(dialog.exec() == QDialog::Accepted) {
        // qDebug() << "settings accepted";
        mSettings.setValue(PlannerVisuSettingsKeys::KEY_LOAD_PARAMETERS_ON_STARTUP, dialog.isLoadLastParametersChecked());
        mSettings.setValue(PlannerVisuSettingsKeys::KEY_LOAD_DLL_ON_STARTUP, dialog.isLoadLastDLLChecked());
    }
}

void PlannerVisualization::onShowTrajectoryPlot()
{
    mTrajectoryGraphView.show();
}

void PlannerVisualization::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
    QCoreApplication::quit(); // quit the app even if trajectory window is still open
}

void PlannerVisualization::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls()) {
        event->acceptProposedAction();
    }

    QMainWindow::dragEnterEvent(event);
}

void PlannerVisualization::dropEvent(QDropEvent *event)
{
    const QMimeData* mimeData = event->mimeData();

   // check for our needed mime type, here a file or a list of files
   if (mimeData->hasUrls())
   {
     QList<QUrl> urlList = mimeData->urls();

     if(urlList.size() > 0) {
         event->acceptProposedAction();
         qDebug() << "PlannerVisualization::dropEvent() loading file";
         mTrajHandler.loadEM(urlList.at(0).toLocalFile());
     }
   }
}

void PlannerVisualization::onPathIndexChanged()
{
    updatePathIndexLabel();
    const auto index = mParkingSceneModel.getTrajectorySetModel()->getCurrentPoseIndex();
    if(index >= 0) {
        ui->pathStepSlider->setValue(index);
    }
}

void PlannerVisualization::onVehiclePathChanged()
{
    const auto trajectory = mParkingSceneModel.getTrajectorySetModel()->getSelectedTrajectory();
    auto size = trajectory? trajectory->size() : 0;

    if(size > 0) {
        ui->pathStepSlider->setEnabled(true);
        ui->pathStepSlider->setMaximum(size-1);
        ui->btnApplyStartPose->setEnabled(true);
    } else {
        ui->pathStepSlider->setEnabled(false);
        ui->pathStepSlider->setValue(0);
        ui->btnApplyStartPose->setEnabled(false);
    }

    updatePathIndexLabel();
    adjustPlanActionsState();
}

void PlannerVisualization::tryLoadSettings(QFileDialog &dialog, void (ClientHandler::*loadFunc)(QString), QString settingKey)
{
    if(dialog.exec()) {
        QString filename = dialog.selectedFiles().at(0);
        (mTrajHandler.*loadFunc)(filename);
        mSettings.setValue(settingKey, filename);
        // set selected directory and file, otherwise dialog will not remember the last opening directory (why??)
        dialog.setDirectory(dialog.directory());
        dialog.selectFile(filename);
    }
}

void PlannerVisualization::reloadParametersIfSet()
{
    if(mSettings.contains(PlannerVisuSettingsKeys::KEY_TRAJ_PARAM_DIR)
            && mSettings.contains(PlannerVisuSettingsKeys::KEY_VEHICLE_PARAM_DIR)
            && mSettings.contains(PlannerVisuSettingsKeys::KEY_SYSFUNC_PARAM_DIR)) {
        mTrajHandler.loadTRJPLAParameters(mSettings.value(PlannerVisuSettingsKeys::KEY_TRAJ_PARAM_DIR).toString());
        mTrajHandler.loadVehicleParameters(mSettings.value(PlannerVisuSettingsKeys::KEY_VEHICLE_PARAM_DIR).toString());
        mTrajHandler.loadSysFuncParameters(mSettings.value(PlannerVisuSettingsKeys::KEY_SYSFUNC_PARAM_DIR).toString());
    }
}

void PlannerVisualization::reloadDLLIfSet()
{
    if(mSettings.contains(PlannerVisuSettingsKeys::KEY_DLL_DIR)) {
        mTrajHandler.loadComponentsDLL(mSettings.value(PlannerVisuSettingsKeys::KEY_DLL_DIR, "").toString());
    }
}

void PlannerVisualization::adjustPlanActionsState()
{
    const auto trajectory = mParkingSceneModel.getTrajectorySetModel()->getSelectedTrajectory();
    if(mTrajHandler.isComputationInProgress()) {
        // everything disabled while planning
        ui->actionPlan_Path->setEnabled(false);
        ui->actionReplan_Path->setEnabled(false);
        ui->actionMake_Step->setEnabled(false);
        ui->actionJump->setEnabled(false);
        ui->actionStartScanMode->setEnabled(false);
    } else if (!trajectory || trajectory->empty()) {
        // when not in follow path state, planning new plan or enabling scan mode are possible
        ui->actionStartScanMode->setEnabled(true);
        if(mTrajHandler.isTargetPoseReachable(mParkingSceneModel.getSelectedTargetPose())){
          ui->actionPlan_Path->setEnabled(true);
        }else{
          ui->actionPlan_Path->setEnabled(false);
        }
        ui->actionReplan_Path->setEnabled(false);
        ui->actionMake_Step->setEnabled(false);
        ui->actionJump->setEnabled(false);
    } else {
        // when in follow path, everything is possible
        ui->actionStartScanMode->setEnabled(true);
        ui->actionPlan_Path->setEnabled(true);
        ui->actionReplan_Path->setEnabled(true);
        ui->actionMake_Step->setEnabled(true);
        ui->actionJump->setEnabled(true);
    }
}

void PlannerVisualization::updatePathIndexLabel()
{
    ui->pathStepIndex->setText(ui->pathStepSlider->isEnabled()?
                (QString::number(ui->pathStepSlider->value()) + "/" + QString::number(ui->pathStepSlider->maximum())):
                QString(""));
}

void PlannerVisualization::display3DReachablityArea(void){
    auto tgtPoseReachableWidget= new TargetPoseReachableAreaWidget(mParkingSceneModel.getTargetPoseReachableAreaModel());
    tgtPoseReachableWidget->show();
}

void PlannerVisualization::updateRecentEMsMenuHistory()
{
    QStringList recentFiles = mSettings.value(PlannerVisuSettingsKeys::KEY_EM_DIR_HIST).toStringList();

    mRecentEMsActions.clear();

    const auto histSize = std::min(MaxNumEMHistory, recentFiles.size());

    for(int i = 0; i < histSize; i++) {
        auto action = new QAction(recentFiles[i], this);
        action->setData(recentFiles[i]);

        connect(action, &QAction::triggered, this, [this]() {
            QAction *action = qobject_cast<QAction *>(sender());
            if (action) {
                mTrajHandler.loadEM(action->data().toString());
            }
        });

        mRecentEMsActions.emplace_back(action);
        ui->menu_File->insertAction(ui->actionReload_Last_Scene, action);
    }
}

void PlannerVisualization::planGeomWithReplan(){
    mParkingSceneModel.setIsReplanning(true);
    mParkingSceneModel.getTargetPoseReachableAreaModel()->setIsReplanning(true);

    // pop up of a dialogue box for metaData, that is, steering, driving direction and replan Trigger atm
    GeomReplanDialog dialog(&mParkingSceneModel, this);
    dialog.loadMetaDataFromModel();
    if (dialog.exec() == QDialog::Accepted) {
        mParkingSceneModel.setVehInflRadius_m(dialog.getInflDistFromDialog());
        mParkingSceneModel.getStartPoseModel()->setDrivingDirection(static_cast<ap_common::DrivingDirection>(dialog.getDrvDirFromDialog()));
        mParkingSceneModel.getStartPoseModel()->setSteeringDirection(static_cast<ap_common::SteeringDirection>(dialog.getSteerDirFromDialog()));
        mParkingSceneModel.setReplanTrigger(static_cast<ap_tp::ReplanTrigger>(dialog.getReplanTriggerFromDialog()));
    }
    QString msg;
    if(!mTrajHandler.planNewGeomPath(&msg)) {
        QMessageBox::critical(this, "Planning failed", "Failed to plan new path: " + msg);
    }
}

void PlannerVisualization::planGeomWithoutReplan(){
    mParkingSceneModel.setIsReplanning(false);
    mParkingSceneModel.getTargetPoseReachableAreaModel()->setIsReplanning(false);
    QString msg;
    if(!mTrajHandler.planNewGeomPath(&msg)) {
        QMessageBox::critical(this, "Planning failed", "Failed to plan new path: " + msg);
    }
}
