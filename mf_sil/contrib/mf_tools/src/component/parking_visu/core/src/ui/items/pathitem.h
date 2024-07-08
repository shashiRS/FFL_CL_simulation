#ifndef PATHITEM_H
#define PATHITEM_H

#include <QObject>
#include <QGraphicsItem>
#include <QTransform>
#include <QList>

#include "models/trajectorysetmodel.h"

class PathItem : public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    explicit PathItem(TrajectorySetModel * model,  QObject *parent = nullptr);

    QRectF boundingRect() const;

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget);

signals:

public slots:
    void onPathsChanged();

private:
    TrajectorySetModel* mModel = nullptr;
};

#endif // PATHITEM_H
