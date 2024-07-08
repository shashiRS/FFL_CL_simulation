#include "objecttreewidget.h"
#include "ui_objecttreewidget.h"

#include <QVariant>
#include <QKeyEvent>
#include <QDebug>

ObjectTreeWidget::ObjectTreeWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ObjectTreeWidget)
{
    ui->setupUi(this);

    ui->treeWidget->installEventFilter(this);
    ui->treeWidget->viewport()->installEventFilter(this);

    connect(ui->treeWidget, &QTreeWidget::itemSelectionChanged, this, &ObjectTreeWidget::onItemSelectionChanged);
    connect(ui->treeWidget, &QTreeWidget::itemDoubleClicked, this, &ObjectTreeWidget::onItemDoubleClicked);
}

ObjectTreeWidget::~ObjectTreeWidget()
{
    delete ui;
}

void ObjectTreeWidget::setModel(ParkingSceneModel *model)
{
    Q_ASSERT(mModel == nullptr);
    mModel = model;

    connect(mModel, &ParkingSceneModel::selectedObjectChanged, this, &ObjectTreeWidget::onSelectedObjectChanged);
    // TODO connect(mModel->getObstaclesModel(), &StaticObstacleCollectionModel::obstacleAdded, this, &ObjectTreeWidget::onObstacleAdded);
    // TODO connect(mModel->getObstaclesModel(), &StaticObstacleCollectionModel::obstacleRemoved, this, &ObjectTreeWidget::onObstacleRemoved);
    connect(mModel->getParkingBoxesModel(), &ParkingBoxCollectionModel::parkingBoxAdded, this, &ObjectTreeWidget::onParkingBoxAdded);
    connect(mModel->getParkingBoxesModel(), &ParkingBoxCollectionModel::parkingBoxRemoved, this, &ObjectTreeWidget::onParkingBoxRemoved);
    connect(mModel, &ParkingSceneModel::selectedTargetPoseChanged, this, &ObjectTreeWidget::onSelectedTargetPoseChanged);
    connect(mModel->getTrajectorySetModel(), &TrajectorySetModel::trajectoriesChanged, this, &ObjectTreeWidget::onTrajectoriesChanged);

    addItem(mModel, "Parking Scene");
    addItem(mModel->getStartPoseModel(), "Start Pose")->setTextColor(0, QColor(40, 60, 255));
    addItem(mModel->getDebugVehicleModel(), "Debug Vehicle")->setTextColor(0, QColor(220,0,220));
    mTrajectorySetRootItem = addItem(mModel->getTrajectorySetModel(), "Trajectory");

    addItem(mModel->getTargetPoseReachableAreaModel(), "Reachability Area");

    auto targetPosesItem = new QTreeWidgetItem(ui->treeWidget, {"Target poses"});
    targetPosesItem->setTextColor(0, QColor(0,220,0));
    for(lsm_geoml::size_type i = 0; i < mModel->numTargetPoseModels(); i++) {
        auto tpm = mModel->getTargetPoseModel(i);
        auto item = addItem(tpm,
                            calcTargetPoseItemName(i, mModel->getSelectedTargetPose()), targetPosesItem);
        item->setData(0, Qt::UserRole+1, i);
        item->setTextColor(0, calcTextColor(tpm->isVisible()));
        mTargetPoseItemMap.insert(tpm, item);
        connect(tpm, &TargetPoseModel::visibilityChanged, this, [item] (bool visible) {
            item->setTextColor(0, calcTextColor(visible));
        });
    }

#ifndef ULTRASONIC_ONLY
    auto parkingSpaceMarkingsItem = new QTreeWidgetItem(ui->treeWidget, {"Parking Space Markings"});
    for(lsm_geoml::size_type i = 0; i < mModel->getParkingSpaceMarkingModels().size(); i++) {
        auto mm = &mModel->getParkingSpaceMarkingModels().at(i);
        auto item = addItem(mm, "Marking #" + QString::number(i), parkingSpaceMarkingsItem);
        item->setData(0, Qt::UserRole+1, i);
        item->setTextColor(0, calcTextColor(mm->getMarking().existenceProb_perc > 0));
        mParkingSpaceMarkingsItemMap.insert(mm, item);
        connect(mm, &ParkingSpaceMarkingModel::markingChanged, this, [item, mm] () {
            item->setTextColor(0, calcTextColor(mm->getMarking().existenceProb_perc > 0));
        });
    }
#endif

    mObstaclesRootItem = new QTreeWidgetItem(ui->treeWidget, {"Obstacles"});
    mObstaclesRootItem->setTextColor(0, QColor(255,150,150));

    for(lsm_geoml::size_type i = 0; i < mModel->getStaticStructureModels().size(); i++) {
        auto ssm = &mModel->getStaticStructureModels().at(i);
        auto item = addItem(ssm, "Obstacle #" + QString::number(i), mObstaclesRootItem);

        item->setData(0, Qt::UserRole+1, i);
        item->setTextColor(0, calcTextColor(ssm->isVisible()));
        connect(ssm, &StaticObstacleModel::visibilityChanged, this, [item] (bool visible) {
            item->setTextColor(0, calcTextColor(visible));
        });
    }

    mParkingBoxRootItem = new QTreeWidgetItem(ui->treeWidget, {"Parking Boxes"});
    mParkingBoxRootItem->setTextColor(0, QColor(255,190,0));
    mAddParkingBoxItem = new QTreeWidgetItem(mParkingBoxRootItem, {"+ Add..."});
    mAddParkingBoxItem->setTextColor(0, QColor(0, 128, 0));
    QFont font;
    font.setBold(true);
    mAddParkingBoxItem->setFont(0, font);

    for(int i = 0; i < mModel->getParkingBoxesModel()->childCount(); i++) {
        onParkingBoxAdded(mModel->getParkingBoxesModel()->getChildModel(i));
    }

    addItem(mModel->getTaposdDebugModel(), "TAPOSD Debug");
}

void ObjectTreeWidget::onItemSelectionChanged()
{
    QObject * selectedObject = nullptr;

    auto selected = ui->treeWidget->selectedItems();
    // qDebug() << "selected" << selected;

    if(!selected.isEmpty()) {
        QVariant itemValue = selected.at(0)->data(0, Qt::UserRole);

        if(itemValue.canConvert<QObject *>()) {
            selectedObject = itemValue.value<QObject *>();
        }
    }

    mModel->setSelectedObject(selectedObject);
}

void ObjectTreeWidget::onItemDoubleClicked(QTreeWidgetItem *item, int column)
{
    Q_UNUSED(column);

    if(mModel) {
        if(item == mAddParkingBoxItem) {
            mModel->getParkingBoxesModel()->addParkingBox();
        } else if (TargetPoseModel* tp = dynamic_cast<TargetPoseModel*>(item->data(0, Qt::UserRole).value<QObject *>())) {
            int index = item->data(0, Qt::UserRole+1).toInt();
            mModel->setSelectedTargetPose(index);
#ifndef ULTRASONIC_ONLY
        } else if (ParkingSpaceMarkingModel* mm = dynamic_cast<ParkingSpaceMarkingModel*>(item->data(0, Qt::UserRole).value<QObject *>())) {
            mm->setVisible(!mm->isVisible());
#endif
        } else if (DebugVehicleModel* mm = dynamic_cast<DebugVehicleModel*>(item->data(0, Qt::UserRole).value<QObject *>())) {
            mm->setVisible(!mm->isVisible());
        } else if(StaticObstacleModel* som = dynamic_cast<StaticObstacleModel*>(item->data(0, Qt::UserRole).value<QObject *>())) {
            // trying to make an static structure visible which has no shape yet
            // => assign it a basic squared shape
            if(!som->isVisible() && som->getStructureData().objShape_m.actualSize == 0) {
                qDebug() << "adding new obstacle";
                si::StaticObjectSerializable data;
                memset(&data, 0, sizeof(data));
                data.existenceProb_perc = 100;
                data.objHeightClass_nu = si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE;
                data.objShape_m.array[0] = cml::Vec2Df_POD{ 1.0F, 0.0F };
                data.objShape_m.array[1] = cml::Vec2Df_POD{ 1.0F, 1.0F };
                data.objShape_m.array[2] = cml::Vec2Df_POD{ 0.0F, 1.0F };
                data.objShape_m.array[3] = cml::Vec2Df_POD{ 0.0F, 0.0F };
                data.objShape_m.actualSize = 4U;
                som->setStructureData(data);
                mModel->setSelectedObject(som);
            } else {
                som->setVisible(!som->isVisible());
            }
        } else if(item->parent() == mTrajectorySetRootItem) {
            mModel->getTrajectorySetModel()->setSelectedTrajectoryIndex(mTrajectorySetRootItem->indexOfChild(item));
        }
    }
}

void ObjectTreeWidget::onSelectedObjectChanged(QObject *selected)
{
    ui->treeWidget->setCurrentItem(mObjectItemMap.value(selected, nullptr));
}

void ObjectTreeWidget::onSelectedTargetPoseChanged(int previousIndex, int currentIndex)
{
    for(lsm_geoml::size_type index = 0; index < mModel->numTargetPoseModels(); index++) {
        mTargetPoseItemMap.value(mModel->getTargetPoseModel(index))->setText(0, calcTargetPoseItemName(index, currentIndex));
    }
}

void ObjectTreeWidget::onObstacleAdded(StaticObstacleModel *obs)
{
    addItem(obs, "Static structures", mObstaclesRootItem);
}

void ObjectTreeWidget::onObstacleRemoved(StaticObstacleModel *obs)
{
    delete mObjectItemMap.take(obs);
}

void ObjectTreeWidget::onParkingBoxAdded(ParkingBoxModel *pbox)
{
    addItem(pbox, "Parking box", mParkingBoxRootItem);
}

void ObjectTreeWidget::onParkingBoxRemoved(ParkingBoxModel *pbox)
{
    delete mObjectItemMap.take(pbox);
}

void ObjectTreeWidget::onTrajectoriesChanged()
{
    for(auto item: mTrajectorySetRootItem->takeChildren()) {
        delete item;
    }

    for(const auto& traj: mModel->getTrajectorySetModel()->getTrajectorySet()) {
        // create new item, note that its automatically set as child of mTrajectorySetRootItem
        // so there is no need to manually append it somewhere
        new QTreeWidgetItem(mTrajectorySetRootItem, {traj.name});

    }
}

bool ObjectTreeWidget::eventFilter(QObject *watched, QEvent *event)
{
    if(event->type() == QEvent::MouseButtonPress) {
        ui->treeWidget->clearSelection();
    } else if(event->type() == QEvent::KeyPress) {
        QKeyEvent* key = dynamic_cast<QKeyEvent*>(event);
        if(mModel && key->key() == Qt::Key_Delete) {
            auto selected = ui->treeWidget->selectedItems();


            if(!selected.empty()) {
                auto& item = selected.at(0);
                /* TODO
                if(StaticObstacleModel* obs = dynamic_cast<StaticObstacleModel*>(selected.at(0)->data(0, Qt::UserRole).value<QObject *>())) {
                    // TODO more efficient
                    for(int i = 0; i < mModel->getObstaclesModel()->childCount(); i++) {
                        if(obs == mModel->getObstaclesModel()->getChildModel(i)) {
                            mModel->getObstaclesModel()->removeObstacle(i);
                            break;
                        }
                    }
                } else
                    */
                if(ParkingBoxModel* obs = dynamic_cast<ParkingBoxModel*>(selected.at(0)->data(0, Qt::UserRole).value<QObject *>())) {
                    // TODO more efficient
                    for(int i = 0; i < mModel->getParkingBoxesModel()->childCount(); i++) {
                        if(obs == mModel->getParkingBoxesModel()->getChildModel(i)) {
                            mModel->getParkingBoxesModel()->removeParkingBox(i);
                            break;
                        }
                    }
                } else if(selected.at(0)->parent() == mTrajectorySetRootItem) {
                    int trajIndex = mTrajectorySetRootItem->indexOfChild(selected.at(0));
                    mModel->getTrajectorySetModel()->removeTrajectory(trajIndex);
                }
            }
        }
    }

    return QObject::eventFilter(watched, event);
}

QTreeWidgetItem* ObjectTreeWidget::addItem(QObject *modelObject, QString name, QTreeWidgetItem *parentItem)
{
    auto item = new QTreeWidgetItem({name});
    item->setData(0, Qt::UserRole, QVariant::fromValue(modelObject));

    mObjectItemMap.insert(modelObject, item);

    if(parentItem) {
        parentItem->addChild(item);
    } else {
        ui->treeWidget->addTopLevelItem(item);
    }

    return item;
}

QString ObjectTreeWidget::calcTargetPoseItemName(int index, int selected)
{
    QString name = "pose #" + QString::number(index);
    if(index == selected) {
        name += " (selected)";
    }

    return name;
}

QColor ObjectTreeWidget::calcTextColor(bool visible)
{
    return visible? QColor(0,0,0) : QColor(174, 173, 172);
}
