#ifndef PARKINGSCENEWIDGET_H
#define PARKINGSCENEWIDGET_H

#include <QGraphicsView>
#include <QGraphicsScene>

#include "models/parkingscenemodel.h"

#include <QSettings>
#include <QFileDialog>

namespace Ui {
class ParkingSceneWidget;
}

class ParkingSceneWidget : public QGraphicsView
{
    Q_OBJECT

public:
    explicit ParkingSceneWidget(QWidget *parent = 0);
    ~ParkingSceneWidget();

    void setModel(ParkingSceneModel* model);

    /**
     * @brief Scale and translate the view such that the full scene is visible
     * @param margin additional margin in m
     */
    void showAll(float margin = 0);

    void startDistanceMeasuring();
    void cancelDistanceMeasuring();

    void startDefineReachabilityArea();
    void cancelDefineReachabilityArea();

    QAction* getMeasurementAction();


    QAction* getScreenshotAction();
    QAction* getSaveEMAction();

signals:
    void distanceMeasuringDone(float distance); // measuring canceled if negative
    void defineReachabilityAreaDone(QPointF start, QPointF end);
    void gridSpacingChanged(float spacing);

public slots:
    void takeScreenshot();
    void takeScreenshotToClipboard();
    void onSaveEM();

protected:
    virtual void drawBackground(QPainter *painter, const QRectF &rect) override;
    virtual void drawForeground(QPainter * painter, const QRectF & rect) override;

    virtual void paintEvent(QPaintEvent * event) override;

    virtual void dropEvent(QDropEvent* event) override;
    virtual void dragEnterEvent(QDragEnterEvent *event) override;

    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void mouseMoveEvent(QMouseEvent *event) override;
    virtual void mouseReleaseEvent(QMouseEvent *event) override;
    virtual void wheelEvent(QWheelEvent* event) override;

private:
    Ui::ParkingSceneWidget *ui;

    QGraphicsScene mScene;

    ParkingSceneModel* mModel = nullptr;

    enum MeasuringState {
        MEASURE_NO,
        MEASURE_INTENDED,
        MEASURE_STARTED,
        DEFINE_AREA_INTENDED,
        DEFINE_AREA_STARTED
    } mMeasuringState = MEASURE_NO;
    QPointF mMeasureStartPoint, mMeasureEndPoint;

    void gentle_zoom(double factor);

    static constexpr double _zoom_factor_base = 1.0015;
    QPointF target_scene_pos, target_viewport_pos;

    float calcGridStepSize() const;

    /**
     * @brief Given a stepsize, calculate next ceiling falue with value % step == 0
     * E.g. lower: 7.5, step size: 2 => return 8
     *      lower: 6, step size: 2 => return 6
     * @param lower
     * @param stepSize
     * @return ceil(lower / stepSize) * stepSize;
     */
    float calcCeilStep(float lower, float stepSize) const;
};

#endif // PARKINGSCENEWIDGET_H
