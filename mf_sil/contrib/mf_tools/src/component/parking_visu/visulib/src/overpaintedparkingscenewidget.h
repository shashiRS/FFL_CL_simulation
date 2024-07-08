#ifndef OVERPAINTEDPARKINGSCENEWIDGET_H
#define OVERPAINTEDPARKINGSCENEWIDGET_H

#include <ui/parkingscenewidget.h>

#include <functional>

class OverpaintedParkingSceneWidget : public ParkingSceneWidget
{
    Q_OBJECT
public:
    typedef std::function<void(QPainter*)> OverpaintHandler;

    explicit OverpaintedParkingSceneWidget(QWidget *parent = nullptr);

    void setOverpaintHandler(const OverpaintHandler& handler);

    virtual void paintEvent(QPaintEvent *event) override;
signals:

public slots:

private:
    OverpaintHandler mOverpaintHandler;
};

#endif // OVERPAINTEDPARKINGSCENEWIDGET_H
