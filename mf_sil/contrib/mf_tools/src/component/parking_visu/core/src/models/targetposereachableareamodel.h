#ifndef TARGETPOSEREACHABLEAREAMODEL_H
#define TARGETPOSEREACHABLEAREAMODEL_H

#include <QObject>
#include <QPointF>
#include <QList>
#include <QVector3D>
#include <QtMath>

#include "models/enumproperty.h"

struct ReachablePoseEntry {
    QVector3D pose;
    bool reachable = false;
    int nrStrokes;
};

class TargetPoseReachableAreaModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(float xStart READ getXstart WRITE setXstart)
    Q_PROPERTY(float xEnd READ getXend WRITE setXend)
    Q_PROPERTY(float yStart READ getYstart WRITE setYstart)
    Q_PROPERTY(float yEnd READ getYend WRITE setYend)
    Q_PROPERTY(float xStep READ getXstep WRITE setXstep)
    Q_PROPERTY(float yStep READ getYstep WRITE setYstep)
    Q_PROPERTY(float yawAngleDegStart READ getYawAngleDegStart WRITE setYawAngleDegStart)
    Q_PROPERTY(float yawAngleDegEnd READ getYawAngleDegEnd WRITE setYawAngleDegEnd)
    Q_PROPERTY(float yawAngleDegStep READ getYawAngleDegStep WRITE setYawAngleDegStep)
    Q_PROPERTY(float yawAngleDegSelected READ getYawAngleDegSelected WRITE setYawAngleDegSelected)
    Q_PROPERTY(bool isReplanning READ isReplanning WRITE setIsReplanning)
    Q_PROPERTY(bool isVisible READ isAreaReady2D WRITE setAreaReady2D)

public:
    explicit TargetPoseReachableAreaModel(QObject *parent = nullptr);
    void reset(void);
    void setPoseInTargetReachableArea(const ReachablePoseEntry &entry);
    const QList<ReachablePoseEntry>& getTargetReachablePoses(void);
    void setAreaReady2D(bool areaReady);
    bool isAreaReady2D(void);

    float getXstart(void);
    float getXend(void);
    float getXstep(void);
    float getYstart(void);
    float getYend(void);
    float getYstep(void);
    float getYawAngleDegStart(void);
    float getYawAngleDegEnd(void);
    float getYawAngleDegStep(void);
    float getYawAngleDegSelected(void);
    EnumProperty getPlanTypeProp();
    bool isReplanning(void);

    void setXstart(float xStart);
    void setXend(float xEnd);
    void setXstep(float xStep);
    void setYstart(float yStart);
    void setYend(float yEnd);
    void setYstep(float yStep);
    void setYawAngleDegStart(float yawAngleStart);
    void setYawAngleDegEnd(float yawAngleEnd);
    void setYawAngleDegStep(float yawAngleStep);
    void setYawAngleDegSelected(float yawAngleSelected);
    void setPlanTypeProp(const EnumProperty& planType);
    void setIsReplanning(bool isReplaning);
    void TargetPoseReachableAreaModel::loadREACH(QString filename);
    void setSearchArea(float_t xStart,float_t xEnd,float_t yStart,float_t yEnd, float_t yawAngleStart, float_t yawAngleEnd);
signals:

    void areaReady2D(bool ready);

private:
    QList<ReachablePoseEntry> mTargetReachablePoses;
    bool mAreaReady2D;
    float_t mXstart= 0.0f;
    float_t mXend= 5.0f;
    float_t mYstart= 2.0f;
    float_t mYend= 5.0f;
    float_t mXStep= 0.25f;
    float_t mYStep= 0.25f;
    float_t mYawAngleDegStart = 0.0f;
    float_t mYawAngleDegEnd = 0.0f;
    float_t mYawAngleDegStep= 5.0f;
    float_t mYawAngleDegSelected = mYawAngleDegStart;
    bool mIsReplanning=false;
    bool mIsVisible = false;

};

#endif // TARGETPOSEREACHABLEAREAMODEL_H
