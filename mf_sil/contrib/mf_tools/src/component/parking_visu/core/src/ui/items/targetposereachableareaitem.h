#ifndef TARGETPOSEREACHABLEAREAITEM_H
#define TARGETPOSEREACHABLEAREAITEM_H

#include <QObject>
#include <QGraphicsItem>
#include <QPen>
#include <QPainter>

#include "models/targetposereachableareamodel.h"

class TargetPoseReachableAreaItem : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit TargetPoseReachableAreaItem(TargetPoseReachableAreaModel *model,QObject *parent = nullptr);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;
    void displayArea(bool display);

signals:

private:

    TargetPoseReachableAreaModel* mModel = nullptr;

    static QColor getReachableEntryColor(const ReachablePoseEntry& entry);
};

#endif // TARGETPOSEREACHABLEAREAITEM_H
