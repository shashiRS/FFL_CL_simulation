#include "overpaintedparkingscenewidget.h"

OverpaintedParkingSceneWidget::OverpaintedParkingSceneWidget(QWidget *parent)
    : ParkingSceneWidget(parent)
{

}

void OverpaintedParkingSceneWidget::setOverpaintHandler(const OverpaintedParkingSceneWidget::OverpaintHandler &handler)
{
    mOverpaintHandler = handler;
}

void OverpaintedParkingSceneWidget::paintEvent(QPaintEvent *event)
{
    ParkingSceneWidget::paintEvent(event);

    /// overpaint
    if(mOverpaintHandler) {
        QPainter painter(viewport());
        painter.setRenderHints(QPainter::Antialiasing);
        painter.setWorldTransform(viewportTransform());
        mOverpaintHandler(&painter);
    }
}
