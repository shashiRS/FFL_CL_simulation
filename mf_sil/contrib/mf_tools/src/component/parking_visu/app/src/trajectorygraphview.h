#ifndef TRAJECTORYGRAPHVIEW_H
#define TRAJECTORYGRAPHVIEW_H

#include <QWidget>
#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>

#include "models/parkingscenemodel.h"

namespace Ui {
class TrajectoryGraphView;
}


/**
 * Plot some trajectory properties, e.g. curvature
 */
class TrajectoryGraphView : public QWidget
{
    Q_OBJECT

public:
    explicit TrajectoryGraphView(TrajectorySetModel *model, QWidget *parent = 0);
    ~TrajectoryGraphView();

public slots:
    void updateGraphFromModel();

protected:
    virtual void keyPressEvent(QKeyEvent *event) override;
    virtual void wheelEvent(QWheelEvent *event) override;

private:
    Ui::TrajectoryGraphView *ui;

    TrajectorySetModel* mModel = nullptr;

    QtCharts::QXYSeries* mSeriesCurv = nullptr; ///< belongs to chart of ui->chartView
    QtCharts::QXYSeries* mSeriesVel = nullptr;
};

#endif // TRAJECTORYGRAPHVIEW_H
