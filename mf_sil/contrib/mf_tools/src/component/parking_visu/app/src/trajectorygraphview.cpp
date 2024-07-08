#include "trajectorygraphview.h"
#include "ui_trajectorygraphview.h"

#include <QDebug>

#include <QtCharts/QChart>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <qmath.h>
#include <QMouseEvent>


using namespace QtCharts;

TrajectoryGraphView::TrajectoryGraphView(TrajectorySetModel* model, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TrajectoryGraphView),
    mModel(model)
{
    ui->setupUi(this);

    QChart* chart = new QChart();
    mSeriesCurv = new QLineSeries();
    mSeriesVel = new QLineSeries();

    chart->legend()->show();
    chart->addSeries(mSeriesCurv);
    chart->addSeries(mSeriesVel);
    //chart->createDefaultAxes();

    ui->chartView->setChart(chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);

    connect(model, &TrajectorySetModel::currentTrajectoryUpdated, this, &TrajectoryGraphView::updateGraphFromModel);
}

TrajectoryGraphView::~TrajectoryGraphView()
{
    delete ui;
}

void TrajectoryGraphView::updateGraphFromModel()
{
    // for whatever reason, directly modifying the series does not change the graph
    // so we need to completely re-create the series.
    // the axes as well as the series have to be removed
    // note that removeAllSeries() will destroy all series object, so we need a new series object
    if (mSeriesCurv) {
        for (auto ax : mSeriesCurv->attachedAxes()) {
            ui->chartView->chart()->removeAxis(ax);
        }
    }
    if (mSeriesVel) {
        for (auto ax : mSeriesVel->attachedAxes()) {
            ui->chartView->chart()->removeAxis(ax);
        }
    }
    
    ui->chartView->chart()->removeAllSeries();
    mSeriesCurv = nullptr;
    mSeriesVel = nullptr;

    const auto trajectory = mModel->getSelectedTrajectory();
    if(!trajectory) {
        return;
    }

    mSeriesCurv = new QLineSeries();
    mSeriesCurv->setName("Curvature 1/m");
    mSeriesVel = new QLineSeries();
    mSeriesVel->setName("Velocity m/s");

    lsm_geoml::size_type count = 0;
    float maxcurv = 0,mincurv = 0, maxvel = 0, minvel= 0;
    for(const TrajectoryPose& pose: *trajectory) {
        mSeriesCurv->append(count++, pose.curvature_1pm);
        mSeriesVel->append(count++, pose.velocity_mps);
        if (pose.curvature_1pm >maxcurv) maxcurv = pose.curvature_1pm;
        if (pose.curvature_1pm < mincurv) mincurv = pose.curvature_1pm;
        if (pose.velocity_mps > maxvel) maxvel = pose.velocity_mps;
        if (pose.velocity_mps < minvel) minvel = pose.velocity_mps;
    }
    // define the x-axis and set it as bottom axis
    auto m_axisX = new QValueAxis;
    ui->chartView->chart()->addAxis(m_axisX, Qt::AlignBottom);

    auto m_axisYCurv = new QValueAxis;
    m_axisYCurv->setRange(mincurv, maxcurv);
    m_axisYCurv->setMin(mincurv);
    m_axisYCurv->setMax(maxcurv);

    auto m_axisYVel = new QValueAxis;
    m_axisYVel->setRange(minvel, maxvel);
    m_axisYVel->setMin(minvel);
    m_axisYVel->setMax(maxvel);

    m_axisX->setRange(0,count-1);
    m_axisX->setMin(0);
    m_axisX->setMax(count-1);

    ui->chartView->chart()->addSeries(mSeriesCurv);
    ui->chartView->chart()->addAxis(m_axisYCurv, Qt::AlignLeft);
    mSeriesCurv->attachAxis(m_axisYCurv);
    mSeriesCurv->attachAxis(m_axisX);

    ui->chartView->chart()->addSeries(mSeriesVel);
    ui->chartView->chart()->addAxis(m_axisYVel, Qt::AlignRight);
    mSeriesVel->attachAxis(m_axisYVel);
    mSeriesVel->attachAxis(m_axisX);

    //ui->chartView->chart()->createDefaultAxes();
    ui->chartView->setRubberBand(QChartView::RectangleRubberBand);

}

void TrajectoryGraphView::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_Plus:
         ui->chartView->chart()->zoomIn();
        break;
    case Qt::Key_Minus:
         ui->chartView->chart()->zoomOut();
        break;
    case Qt::Key_R:
         ui->chartView->chart()->zoomReset();
    }
}

void TrajectoryGraphView::wheelEvent(QWheelEvent *event)
{
    if (QApplication::keyboardModifiers() == Qt::NoModifier) {
        if (event->orientation() == Qt::Vertical) {
            double angle = event->angleDelta().y();
            double factor = qPow(1.0015, angle);
            ui->chartView->chart()->zoom(factor);
        }
    }
}
