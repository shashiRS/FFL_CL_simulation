#include "reachabilityareahandlerin3d.h"
#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/q3dtheme.h>
#include <QtCore/qmath.h>
#include <QtCore/qrandom.h>
#include <QtWidgets/QComboBox>

using namespace QtDataVisualization;

ReachabilityAreaHandlerIn3D::ReachabilityAreaHandlerIn3D(TargetPoseReachableAreaModel *model, Q3DScatter *scatter)
    : mModel(model),
      m_graph(scatter)
{

    m_graph->setShadowQuality(QAbstract3DGraph::ShadowQualityNone);
    m_graph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFrontLow);
    QFont font = m_graph->activeTheme()->font();
    font.setPointSize(40.0f);
    m_graph->activeTheme()->setFont(font);

    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);
    series->setMesh(QAbstract3DSeries::MeshSphere);
    series->setBaseColor(QColor(0,255,0,100));
    m_graph->addSeries(series);



    QScatterDataProxy *proxy2 = new QScatterDataProxy;
    QScatter3DSeries *series2 = new QScatter3DSeries(proxy2);

    series2->setMesh(QAbstract3DSeries::MeshSphere);
    series2->setBaseColor(QColor(255,0,0,100));
    m_graph->addSeries(series2);


    addData();

}

ReachabilityAreaHandlerIn3D::~ReachabilityAreaHandlerIn3D()
{

}

void ReachabilityAreaHandlerIn3D::addData()
{
    // Configure the axes according to the data
    m_graph->axisX()->setTitle("X");
    m_graph->axisY()->setTitle("Y");
    m_graph->axisZ()->setTitle("Z");

    const auto& reachablePoses= mModel->getTargetReachablePoses();
    QScatterDataArray *dataArray = new QScatterDataArray;
    dataArray->reserve(reachablePoses.size());

    QScatterDataArray *dataArray2 = new QScatterDataArray;
    dataArray2->reserve(reachablePoses.size());

    for(const auto& entry: reachablePoses) {
        (entry.reachable? dataArray : dataArray2)->append(QScatterDataItem{entry.pose});
    }

    dataArray->shrink_to_fit();
    dataArray2->shrink_to_fit();

    m_graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);
    m_graph->seriesList().at(1)->dataProxy()->resetArray(dataArray2);
}
