#include "targetposereachableareawidget.h"

TargetPoseReachableAreaWidget::TargetPoseReachableAreaWidget(TargetPoseReachableAreaModel *model, QWidget *parent)
    : mModel(model), QWidget(parent)
{

    auto graph= new QtDataVisualization::Q3DScatter();

    QWidget *container = QWidget::createWindowContainer(graph);

    QHBoxLayout *hLayout = new QHBoxLayout(this);
    hLayout->addWidget(container, 1);

    this->setWindowTitle(QStringLiteral("Target Pose Reachable Area"));

    QSize screenSize = graph->screen()->size();
    container->setMinimumSize(QSize(screenSize.width() / 4, screenSize.height() / 3));
    container->setMaximumSize(screenSize);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container->setFocusPolicy(Qt::StrongFocus);
    ReachabilityAreaHandlerIn3D *modifier = new ReachabilityAreaHandlerIn3D(mModel, graph);

}
