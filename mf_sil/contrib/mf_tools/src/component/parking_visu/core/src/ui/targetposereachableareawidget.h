#ifndef TARGETPOSEREACHABLEAREAWIDGET_H
#define TARGETPOSEREACHABLEAREAWIDGET_H

#include <QWidget>

//#include <QtDataVisualization/Q3DScatter>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtGui/QScreen>
#include <QtGui/QFontDatabase>
#include "reachabilityareahandlerin3d.h"
#include "models/targetposereachableareamodel.h"


class TargetPoseReachableAreaWidget : public QWidget
{
public:
    explicit TargetPoseReachableAreaWidget(TargetPoseReachableAreaModel *model, QWidget *parent = 0);
private:
    TargetPoseReachableAreaModel* mModel = nullptr;
};

#endif // TARGETPOSEREACHABLEAREAWIDGET_H
