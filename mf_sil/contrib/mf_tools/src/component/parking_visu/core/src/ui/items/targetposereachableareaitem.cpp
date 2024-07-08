#include "targetposereachableareaitem.h"

TargetPoseReachableAreaItem::TargetPoseReachableAreaItem(TargetPoseReachableAreaModel *model, QObject *parent) : QObject(parent) , mModel(model)
{
    connect(model,&TargetPoseReachableAreaModel::areaReady2D,this, &TargetPoseReachableAreaItem::displayArea);
}

void TargetPoseReachableAreaItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    qreal paintSizeX = mModel->getXstep();
    qreal paintSizeY = mModel->getYstep();

    float angleSelected= qDegreesToRadians(mModel->getYawAngleDegSelected());

    for(const auto& entry: mModel->getTargetReachablePoses()){
        if(qFuzzyCompare(entry.pose.z(),angleSelected)){
           QPointF topLeftPoint = entry.pose.toPointF() - QPointF(paintSizeX/2, paintSizeY/2);
           QPointF botRightPoint = topLeftPoint + QPointF(paintSizeX, paintSizeY);
           QRectF rectangle(topLeftPoint,botRightPoint);

           painter->fillRect(rectangle, getReachableEntryColor(entry));
        }
    }
}
QRectF TargetPoseReachableAreaItem::boundingRect() const
{

    return QRectF(mModel->getXstart(),mModel->getYstart(),
                  mModel->getXend()-mModel->getXstart(),mModel->getYend()-mModel->getYstart());

}

void TargetPoseReachableAreaItem::displayArea(bool display){
    prepareGeometryChange();
    setVisible(display);
}

QColor TargetPoseReachableAreaItem::getReachableEntryColor(const ReachablePoseEntry &entry)
{
    static const QColor colorReachable(0,255,0,100), colorNotReachable(255,0,0,100), colorCat {0, 0, 255, 100};
    if(!entry.reachable) {
        return colorNotReachable;
    } else {
        QColor colorReachableSpec = colorReachable.toHsv();
        colorReachableSpec.setHsv(colorReachableSpec.hue()-entry.nrStrokes*10,
                                  colorReachableSpec.saturation(),
                                  colorReachableSpec.value(),colorReachableSpec.alpha());
        return colorReachableSpec;
    }
}
