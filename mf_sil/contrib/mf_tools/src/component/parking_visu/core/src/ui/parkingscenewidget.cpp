#include "parkingscenewidget.h"

#include "ui_parkingscenewidget.h"

#include "ui/items/pathitem.h"
#include "ui/items/debugvehicleitem.h"
#include "ui/items/targetposeitem.h"
#ifndef ULTRASONIC_ONLY
#include "ui/items/parkingspacemarkingitem.h"
#endif
#include "ui/items/parkingboxcollectionitem.h"
#include "ui/items/staticobstacleitem.h"
#include "ui/items/targetposereachableareaitem.h"
#include <ui/items/taposddebugitem.h>


#include <QPaintEvent>
#include <QDebug>
#include <QToolTip>
#include <QFileDialog>
#include <QMouseEvent>
#include <QClipboard>
#include <QMimeData>
#include <QDropEvent>
#include <QDragEnterEvent>
#include <qmath.h>

ParkingSceneWidget::ParkingSceneWidget(QWidget *parent) :
    QGraphicsView(parent),
    ui(new Ui::ParkingSceneWidget)
{
    ui->setupUi(this);
    addActions({ui->actionMeasure_Distance, ui->actionSave_Environment_Model, ui->actionSave_Screenshot});

    setMouseTracking(true);

    setScene(&mScene);

    connect(ui->actionMeasure_Distance, &QAction::toggled, this, [this](bool active){
        if(active)
            startDistanceMeasuring();
        else
            cancelDistanceMeasuring();
    });

    connect(this, &ParkingSceneWidget::distanceMeasuringDone, this, [this] () {
        ui->actionMeasure_Distance->setChecked(false);
    });

    connect(ui->actionSave_Screenshot, &QAction::triggered, this, &ParkingSceneWidget::takeScreenshot);
    connect(ui->actionSave_Environment_Model, &QAction::triggered, this, &ParkingSceneWidget::onSaveEM);

    scale(50, -50);
    // this is a hack to allow panning around the scene at program start. Otherwise the pan area is restricted to the scene size
    setSceneRect(QRectF(-50, -50, 100, 100));
}

ParkingSceneWidget::~ParkingSceneWidget()
{
}

void ParkingSceneWidget::setModel(ParkingSceneModel *model)
{
    if(mModel)
        return;

    mModel = model;

    auto startPoseItem = new DebugVehicleItem(mModel->getStartPoseModel(), mModel);
    startPoseItem->setVehicleColor(QColor(100, 150, 255, 127));
    startPoseItem->setZValue(2);
    startPoseItem->setCreateHistoryPointOnMouseMove(true);
    mScene.addItem(startPoseItem);

    for(lsm_geoml::size_type i = 0; i < mModel->numTargetPoseModels(); i++) {
        auto tp = mModel->getTargetPoseModel(i);
        auto item = new TargetPoseItem(tp, mModel);
        item->setZValue(1);
        mScene.addItem(item);
    }

#ifndef ULTRASONIC_ONLY
    for(auto& marking: mModel->getParkingSpaceMarkingModels()) {
        mScene.addItem(new ParkingSpaceMarkingItem(&marking, mModel));
    }
#endif

    for(auto& staticStructure: mModel->getStaticStructureModels()) {
        mScene.addItem(new StaticObstacleItem(mModel, &staticStructure));
    }

    auto pathItem = new PathItem(mModel->getTrajectorySetModel());
    pathItem->setZValue(4);
    mScene.addItem(pathItem);
    mScene.addItem(new ParkingBoxCollectionItem(mModel));

    auto tgtPoseReachableAreaItem = new TargetPoseReachableAreaItem(mModel->getTargetPoseReachableAreaModel());
    mScene.addItem(tgtPoseReachableAreaItem);

    mScene.addItem(new TaposdDebugItem(mModel->getTaposdDebugModel()));
}

void ParkingSceneWidget::showAll(float margin)
{
    // for whatever reason, itemsBoundingRect() does not ignore invisible items, so
    // we need to collect the bounding rects manually

    // qDebug() << "----- showAll -----";
    QRectF rect;
    foreach(QGraphicsItem *item, mScene.items()) {
        if (item->isVisible()) {
            auto irect = item->sceneBoundingRect();
            // qDebug() << irect;
            rect = rect.united(irect);
        }
    }
    // qDebug() << "rect = " << rect;

    rect += QMarginsF(margin, margin, margin, margin);
    fitInView(rect, Qt::KeepAspectRatio);
}

void ParkingSceneWidget::startDistanceMeasuring()
{
    mMeasuringState = MEASURE_INTENDED;

    setCursor(QCursor(Qt::CrossCursor));
}

void ParkingSceneWidget::cancelDistanceMeasuring()
{
    if(mMeasuringState == MEASURE_STARTED) {
        update();
    }

    setCursor(QCursor());
    mMeasuringState = MEASURE_NO;
    emit distanceMeasuringDone(-1);
}

QAction *ParkingSceneWidget::getMeasurementAction()
{
    return ui->actionMeasure_Distance;
}

void ParkingSceneWidget::startDefineReachabilityArea()
{
    mMeasuringState = DEFINE_AREA_INTENDED;

    setCursor(QCursor(Qt::CrossCursor));
}

void ParkingSceneWidget::cancelDefineReachabilityArea()
{
    if(mMeasuringState == DEFINE_AREA_STARTED) {
        update();
    }

    setCursor(QCursor());
    mMeasuringState = MEASURE_NO;
    //QPointF invalid1, invalid2;
    //emit defineReachabilityAreaDone(invalid1, invalid2);
}

QAction *ParkingSceneWidget::getScreenshotAction()
{
    return ui->actionSave_Screenshot;
}

QAction *ParkingSceneWidget::getSaveEMAction()
{
    return ui->actionSave_Environment_Model;
}

void ParkingSceneWidget::paintEvent(QPaintEvent *event)
{
    //QPaintEvent myEvent(viewport()->rect());
    //qDebug() << myEvent.region();
    QGraphicsView::paintEvent(event /* &myEvent */);
    //event->setAccepted(myEvent.isAccepted());

    QPainter painter(viewport());
    QRect pixelRect = painter.viewport();

    auto fontMetrics = painter.fontMetrics();
    auto charWidth = fontMetrics.width('0');

    const QString info = QString("Pos: %1; %2").arg(mMeasureEndPoint.x()).arg(mMeasureEndPoint.y());
    painter.setPen(Qt::black);
    float rightInfoPadding = charWidth * 15;
    painter.drawText(rightInfoPadding, pixelRect.bottom() - 10, info);

    // currently displayed scene rect, assuming fixed aspect ratio and no shearing
    QRectF meterRect(mapToScene(pixelRect.topLeft()), mapToScene(pixelRect.bottomRight()));

    static constexpr int rulerPaddingDist = 5;
    static constexpr int rulerMarkingWidth = 6;

    static constexpr int skipTextPadding = 50;
    static constexpr int textRulerPadding = 5;

    float stepSize = calcGridStepSize();
    float stepSizePixel = transform().map(QLineF(0, 0, 1, 0)).length() * stepSize;

    // horizontal ruler
    painter.drawLine(pixelRect.left(), rulerPaddingDist, pixelRect.right(), rulerPaddingDist);

    // collect text labels and text bounding rects, calculate max text width
    int maxMeterTextWidth = 0;
    QList<std::pair<QString, QRect>> textList;
    int xTickCounter = 0;
    for(float meterX = calcCeilStep(meterRect.left(), stepSize); meterX < meterRect.right(); meterX += stepSize) {
        auto xStr = QString::number(meterX) + " m";
        auto textRect = fontMetrics.boundingRect(xStr);
        maxMeterTextWidth = std::max(maxMeterTextWidth, textRect.width() + charWidth);
        textList.append(std::make_pair(xStr, textRect));
        xTickCounter++;
    }

    // draw ruler ticks
    xTickCounter = 0;
    for(float meterX = calcCeilStep(meterRect.left(), stepSize); meterX < meterRect.right(); meterX += stepSize) {
        int pixelX = mapFromScene(QPointF(meterX, 0)).x();

        painter.drawLine(pixelX, rulerPaddingDist, pixelX, rulerPaddingDist + rulerMarkingWidth);

        if(pixelX > skipTextPadding) {
            // odd/even alternating y offset to prevent overlapping of long labels
            const float yOffset = maxMeterTextWidth > stepSizePixel && (xTickCounter&1)? fontMetrics.height() : 0;
            auto const& textEntry = textList.at(xTickCounter);
            painter.drawText(pixelX - textEntry.second.width()/2.0, rulerPaddingDist + rulerMarkingWidth + textEntry.second.height() + yOffset, textEntry.first);
        }

        xTickCounter++;
    }

    // vertical ruler
    // y needs to go from buttom to top, since positive up
    // int fontHeight = fontMetrics.ascent(); // use ascent instead of height, since there is a gap between actual letter top pixel and height()
    painter.drawLine(rulerPaddingDist, pixelRect.top(), rulerPaddingDist, pixelRect.bottom());
    for(float meterY = calcCeilStep(meterRect.bottom(), stepSize); meterY < meterRect.top(); meterY += stepSize) {
        int pixelY = mapFromScene(QPointF(0, meterY)).y();

        painter.drawLine(rulerPaddingDist, pixelY, rulerPaddingDist + rulerMarkingWidth, pixelY);

        if(pixelY > skipTextPadding) {
            auto yStr = QString::number(meterY) + " m";
            // don't vertically center, since then the - minus sign might be hard to see due to grid lines
            painter.drawText(rulerPaddingDist + rulerMarkingWidth + textRulerPadding, pixelY  /* + fontHeight/2.0 */, yStr);
        }
    }

    // draw scale
    QString scaleText;
    if(stepSize >= 1000.0) {
        scaleText = QString::number(stepSize / 1000.0) + " km";
    } else if(stepSize >= 1.0) {
        scaleText = QString::number(stepSize) + " m";
    } else if(stepSize >= 0.01) {
        scaleText = QString::number(stepSize * 100.0) + " cm";
    } else {
        scaleText = QString::number(stepSize * 1000.0) + " mm";
    }
    auto textRect = fontMetrics.boundingRect(scaleText);

    static constexpr float scalePaddingX = 20;
    const float scalePaddingY = 10 + textRect.height();

    painter.drawLine(pixelRect.right() - stepSizePixel - scalePaddingX, pixelRect.bottom() - scalePaddingY,
                     pixelRect.right() - scalePaddingX, pixelRect.bottom() - scalePaddingY);
    painter.drawLine(pixelRect.right() - stepSizePixel - scalePaddingX, pixelRect.bottom() - scalePaddingY - rulerMarkingWidth,
                     pixelRect.right() - stepSizePixel - scalePaddingX, pixelRect.bottom() - scalePaddingY);
    painter.drawLine(pixelRect.right() - scalePaddingX, pixelRect.bottom() - scalePaddingY - rulerMarkingWidth,
                     pixelRect.right() - scalePaddingX, pixelRect.bottom() - scalePaddingY);

    painter.drawText(pixelRect.right() - scalePaddingX - textRect.width(), pixelRect.bottom() - scalePaddingY + textRect.height(), scaleText);
}


void ParkingSceneWidget::dropEvent(QDropEvent *event)
{

    /*
    qDebug() << "dropEvent()";
    const QMimeData* mimeData = event->mimeData();

   // check for our needed mime type, here a file or a list of files
   if (mimeData->hasUrls())
   {
     QList<QUrl> urlList = mimeData->urls();

     if(urlList.size() > 0) {
         event->acceptProposedAction();
         mTrajHandler.loadEM(urlList.at(0).toLocalFile());
     }
   } */

    foreach (const QUrl &url, event->mimeData()->urls()) {
        QString fileName = url.toLocalFile();
        qDebug() << "Dropped file:" << fileName;
    }
}

void ParkingSceneWidget::dragEnterEvent(QDragEnterEvent *event)
{
    if (event->mimeData()->hasUrls()) {
         qDebug() << "ParkingSceneWidget::dragEnterEvent() accept";
           event->acceptProposedAction();
           event->accept();
    }
}

void ParkingSceneWidget::takeScreenshot() {
    QString fileName =  QFileDialog::getSaveFileName(this,
                                                     tr("Save Screenshot"), "",
                                                     tr("PNG files (*.png);;JPEG files (*.jpg,*.jpeg);;All Files (*)"));

    if(!fileName.isEmpty()) {
        viewport()->grab().save(fileName);
    }
}

void ParkingSceneWidget::takeScreenshotToClipboard()
{
    QApplication::clipboard()->setPixmap(viewport()->grab());
}

void ParkingSceneWidget::onSaveEM()
{
    qDebug() << "onSaveEM()";
    if(mModel) {
        QString filename = QFileDialog::getSaveFileName(this,
                                                        tr("Save EM"), mModel->getLoadedEMFilename(),
                                                        tr("JSON files (*.json);;All Files (*)"));

        if(!filename.isEmpty()) {
            mModel->saveEM(filename);
        }
    }
}

void ParkingSceneWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
    QPen pen(Qt::DotLine);
    pen.setWidthF(0);
    pen.setColor(QColor(200, 200, 200));
    painter->setPen(pen);

    float gridStepSize = calcGridStepSize();

    // vertical lines
    for(float x = calcCeilStep(rect.left(), gridStepSize); x < rect.right(); x += gridStepSize) {
        painter->drawLine(QLineF(x, rect.top(), x, rect.bottom()));
    }

    // horizontal lines
    for(float y = calcCeilStep(rect.top(), gridStepSize); y < rect.bottom(); y += gridStepSize) {
        painter->drawLine(QLineF(rect.left(), y, rect.right(), y));
    }

    if(mMeasuringState == DEFINE_AREA_STARTED) {
        QPen pen(QColor(0,0,0));
        pen.setStyle(Qt::DotLine);
        pen.setWidthF(0);
        painter->setPen(pen);
        painter->setBrush(Qt::NoBrush);
        painter->setRenderHints(QPainter::Antialiasing);
        QRectF rectangle(mMeasureStartPoint,mMeasureEndPoint);
        painter->fillRect(rectangle,Qt::yellow);
        painter->drawRect(rectangle);
    }
}

void ParkingSceneWidget::drawForeground(QPainter *painter, const QRectF &rect)
{
    QGraphicsView::drawForeground(painter, rect);

    if(mMeasuringState == MEASURE_STARTED) {
        QPen pen(QColor(0,0,0));
        pen.setStyle(Qt::DotLine);
        pen.setWidthF(0);
        painter->setPen(pen);
        painter->setBrush(Qt::NoBrush);
        painter->setRenderHints(QPainter::Antialiasing);
        painter->drawLine(mMeasureStartPoint, mMeasureEndPoint);
    }


}


void ParkingSceneWidget::mousePressEvent(QMouseEvent *event)
{
    switch (mMeasuringState) {
    case MEASURE_NO:
        QGraphicsView::mousePressEvent(event);
        break;

    case MEASURE_INTENDED:
        if(event->button() == Qt::LeftButton) {
            mMeasuringState = MEASURE_STARTED;
            mMeasureStartPoint = mapToScene(event->pos());
            mMeasureEndPoint = mMeasureStartPoint;
            event->accept();
        }
        break;

    case MEASURE_STARTED:
        break;

    case DEFINE_AREA_INTENDED:
        if(event->button() == Qt::LeftButton) {
            mMeasuringState = DEFINE_AREA_STARTED;
            mMeasureStartPoint = mapToScene(event->pos());
            mMeasureEndPoint = mMeasureStartPoint;
            event->accept();
        }
        break;

    case DEFINE_AREA_STARTED:
        break;

    default:
        break;
    }
}

void ParkingSceneWidget::mouseMoveEvent(QMouseEvent *event)
{
    QPointF scenePos = mapToScene(event->pos());
    QPointF measureDeltaScene = scenePos - mMeasureEndPoint;
    mMeasureEndPoint = scenePos;

    switch (mMeasuringState) {
    case MEASURE_NO:
        QGraphicsView::mouseMoveEvent(event);
        break;

    case MEASURE_INTENDED:
        break;

    case MEASURE_STARTED: {
        float dist = QVector2D(mMeasureEndPoint - mMeasureStartPoint).length();
        QToolTip::showText(mapToGlobal(event->pos()), QString::number(dist) + "m", this);
    }
        break;

    case DEFINE_AREA_INTENDED:
        break;

    case DEFINE_AREA_STARTED: {
        //event->setAccepted(myEvent.isAccepted());
        //QPainter painter;
    }
        break;

    default:
        break;
    }

    QPointF delta = target_viewport_pos - event->pos();
    if (qAbs(delta.x()) > 5 || qAbs(delta.y()) > 5) {
        target_viewport_pos = event->pos();
        target_scene_pos = mapToScene(event->pos());
    }

    if(event->buttons() & Qt::MidButton) {
        setTransformationAnchor(QGraphicsView::NoAnchor);
        translate(measureDeltaScene.x(), measureDeltaScene.y());
        mMeasureEndPoint = mapToScene(event->pos());
    }

    viewport()->update();
}


void ParkingSceneWidget::mouseReleaseEvent(QMouseEvent *event)
{
    switch (mMeasuringState) {
    case MEASURE_NO:
        QGraphicsView::mouseReleaseEvent(event);
        break;

    case MEASURE_INTENDED:
        break;

    case MEASURE_STARTED:
        if(event->button() == Qt::LeftButton) {
            mMeasuringState = MEASURE_NO;
            event->accept();
            viewport()->update();
            float dist = QVector2D(mMeasureEndPoint - mMeasureStartPoint).length();
            qDebug() << "distance =" << dist;
            QToolTip::showText(mapToGlobal(event->pos()), QString::number(dist) + "m" , this);
            emit distanceMeasuringDone(dist);

            setCursor(QCursor());
        }
        break;

    case DEFINE_AREA_INTENDED:
        break;

    case DEFINE_AREA_STARTED:
        if(event->button() == Qt::LeftButton) {
            mMeasuringState = MEASURE_NO;
            event->accept();
            viewport()->update();
            QToolTip::showText(mapToGlobal(event->pos()),\
                               "xStart: " + QString::number(mMeasureStartPoint.x()) + "m, yStart: " + QString::number(mMeasureStartPoint.y()) +\
                               "m, xEnd: " + QString::number(mMeasureEndPoint.x()) + "m, yEnd: " + QString::number(mMeasureEndPoint.y()) + "m", this);
            mModel->getTargetPoseReachableAreaModel()->reset();
            if(mMeasureStartPoint.x()< mMeasureEndPoint.x()){
                mModel->getTargetPoseReachableAreaModel()->setXstart(mMeasureStartPoint.x());
                mModel->getTargetPoseReachableAreaModel()->setXend(mMeasureEndPoint.x());
            }
            else{
                mModel->getTargetPoseReachableAreaModel()->setXstart(mMeasureEndPoint.x());
                mModel->getTargetPoseReachableAreaModel()->setXend(mMeasureStartPoint.x());
            }
            if(mMeasureStartPoint.y()< mMeasureEndPoint.y()){
                mModel->getTargetPoseReachableAreaModel()->setYstart(mMeasureStartPoint.y());
                mModel->getTargetPoseReachableAreaModel()->setYend(mMeasureEndPoint.y());
            }
            else{
                mModel->getTargetPoseReachableAreaModel()->setYstart(mMeasureEndPoint.y());
                mModel->getTargetPoseReachableAreaModel()->setYend(mMeasureStartPoint.y());
            }
            /*
            mModel->getTargetPoseReachableAreaModel()->setXstep(0.05);
            mModel->getTargetPoseReachableAreaModel()->setYstep(0.05);
            mModel->getTargetPoseReachableAreaModel()->setYawAngleDegStart(0);
            mModel->getTargetPoseReachableAreaModel()->setYawAngleDegEnd(0);
            mModel->getTargetPoseReachableAreaModel()->setYawAngleDegStep(5);
            */
            emit defineReachabilityAreaDone(mMeasureStartPoint, mMeasureEndPoint);
            setCursor(QCursor());
        }
        break;

    default:
        break;
    }
}

void ParkingSceneWidget::wheelEvent(QWheelEvent *event)
{
    if (QApplication::keyboardModifiers() == Qt::NoModifier) {
        if (event->orientation() == Qt::Vertical) {
            double angle = event->angleDelta().y();
            double factor = qPow(_zoom_factor_base, angle);
            gentle_zoom(factor);
            mMeasureEndPoint = mapToScene(event->pos());
            viewport()->update();
        }
    }
}

void ParkingSceneWidget::gentle_zoom(double factor)
{
    scale(factor, factor);
    centerOn(target_scene_pos);
    QPointF delta_viewport_pos = target_viewport_pos - QPointF(viewport()->width() / 2.0,
                                                               viewport()->height() / 2.0);
    QPointF viewport_center = mapFromScene(target_scene_pos) - delta_viewport_pos;
    centerOn(mapToScene(viewport_center.toPoint()));
}

float ParkingSceneWidget::calcGridStepSize() const
{
    // get current scale factor assuming fixed aspect ratio and no shearing
    float scale = transform().map(QLineF(0, 0, 1, 0)).length(); // pixels per meter

    float meterPerNPixels = 50.0 / scale;

    // round step size to the next power of 2, i.e.  ... 4m, 2m, 1m, 0.5m, 0.25m, ...
    // https://stackoverflow.com/questions/466204/rounding-up-to-next-power-of-2
    return std::pow(2.0, std::ceil(std::log(meterPerNPixels)/std::log(2.0)));
}

float ParkingSceneWidget::calcCeilStep(float lower, float stepSize) const
{
    return std::ceil(lower / stepSize) * stepSize;
}
