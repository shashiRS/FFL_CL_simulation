#ifndef REACHABILITYAREAHANDLERIN3D_H
#define REACHABILITYAREAHANDLERIN3D_H

#include <QtDataVisualization/q3dscatter.h>
#include <QtDataVisualization/qabstract3dseries.h>
#include <QtGui/QFont>
#include "models/targetposereachableareamodel.h"

using namespace QtDataVisualization;

class ReachabilityAreaHandlerIn3D : public QObject
{
    Q_OBJECT
public:
    explicit ReachabilityAreaHandlerIn3D(TargetPoseReachableAreaModel *model, Q3DScatter *scatter);
    ~ReachabilityAreaHandlerIn3D();

    void addData();

public Q_SLOTS:


Q_SIGNALS:

private:
    Q3DScatter *m_graph;
    int m_fontSize;
    QAbstract3DSeries::Mesh m_style;
    bool m_smooth;
    int m_itemCount;
    float m_curveDivider;
    TargetPoseReachableAreaModel* mModel = nullptr;
};

#endif
