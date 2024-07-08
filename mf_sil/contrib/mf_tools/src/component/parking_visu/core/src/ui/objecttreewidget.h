#ifndef OBJECTTREEWIDGET_H
#define OBJECTTREEWIDGET_H

#include <QWidget>
#include <QMap>
#include <QTreeWidgetItem>
#include <QSet>

#include <models/parkingscenemodel.h>

namespace Ui {
class ObjectTreeWidget;
}

class ObjectTreeWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ObjectTreeWidget(QWidget *parent = 0);
    ~ObjectTreeWidget();

    void setModel(ParkingSceneModel* model);

    QTreeWidgetItem *addItem(QObject* modelObject, QString name, QTreeWidgetItem* parentItem = nullptr);

private slots:
    void onItemSelectionChanged();
    void onItemDoubleClicked(QTreeWidgetItem *item, int column);

    void onSelectedObjectChanged(QObject* selected);
    void onSelectedTargetPoseChanged(int previousIndex, int currentIndex);

    void onObstacleAdded(StaticObstacleModel* obs);
    void onObstacleRemoved(StaticObstacleModel* obs);

    void onParkingBoxAdded(ParkingBoxModel* pbox);
    void onParkingBoxRemoved(ParkingBoxModel* pbox);

    void onTrajectoriesChanged();

private:
    Ui::ObjectTreeWidget *ui;

    ParkingSceneModel* mModel = nullptr;

    QTreeWidgetItem* mObstaclesRootItem = nullptr;

    QTreeWidgetItem* mParkingBoxRootItem = nullptr;
    QTreeWidgetItem* mAddParkingBoxItem = nullptr;

    QTreeWidgetItem* mTrajectorySetRootItem = nullptr;

    QMap<QObject*, QTreeWidgetItem*> mObjectItemMap;

    QMap<TargetPoseModel*, QTreeWidgetItem*> mTargetPoseItemMap;
#ifndef ULTRASONIC_ONLY
    QMap<ParkingSpaceMarkingModel*, QTreeWidgetItem*> mParkingSpaceMarkingsItemMap;
#endif
    bool eventFilter(QObject *watched, QEvent *event) override;

    static QString calcTargetPoseItemName(int index, int selected);

    static QColor calcTextColor(bool visible);
};

#endif // OBJECTTREEWIDGET_H
