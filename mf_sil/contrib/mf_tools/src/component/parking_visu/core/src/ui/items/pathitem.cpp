#include "pathitem.h"

#include <QPainter>
#include <QDebug>
#include <QMargins>
#include <QVector2D>

#include <algorithm>

static const std::vector<QColor> pathColors = {
    QColor("blue"), QColor("magenta"), QColor("red"),
                          QColor("darkRed"), QColor("darkCyan"), QColor("darkMagenta"),
                          QColor("green"), QColor("darkGreen"), QColor("yellow"),
                          QColor("cyan")
};

PathItem::PathItem(TrajectorySetModel *model, QObject *parent)
    : QObject(parent)
    , mModel(model)
{
    connect(model, &TrajectorySetModel::trajectoriesChanged, this, &PathItem::onPathsChanged);
    // need to use lambda, since parameters do not match
    connect(model, &TrajectorySetModel::displayOffsetChanged, this, [this]{prepareGeometryChange();});
}

QRectF PathItem::boundingRect() const
{
    qreal minx = 10000, miny = 10000, maxx = -10000, maxy = -100000;

    if(mModel->getTrajectorySet().empty()) {
        return QRectF();
    }

    const auto displayOffset = mModel->getDisplayOffset();
    bool hasPoint = false;

    for(auto path: mModel->getTrajectorySet()) {
        for(const auto& entry: path.poses) {
            auto p = entry.trans.map(displayOffset);

            if (hasPoint) {
                minx = std::min(minx, p.x());
                maxx = std::max(maxx, p.x());
                miny = std::min(miny, p.y());
                maxy = std::max(maxy, p.y());
            } else {
                hasPoint = true;
                minx = p.x();
                maxx = p.x();
                miny = p.y();
                maxy = p.y();
            }
        }
    }

    if (hasPoint) {
        return QRectF(minx, miny, maxx - minx, maxy - miny) + QMarginsF(0.5, 0.5, 0.5, 0.5);
    } else {
        return QRectF();
    }
}

void PathItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    const auto displayOffset = mModel->getDisplayOffset();
    QPolygonF pathPoint;
    pathPoint.append(QPointF(0, 0.03) + displayOffset);
    pathPoint.append(QPointF(0.03, 0) + displayOffset);
    pathPoint.append(QPointF(0, -0.03) + displayOffset);

    painter->setPen(Qt::NoPen);

    // qDebug() << "+++++++++++++++++++++++++++++++++++++++++++++++++++";

    size_t colorIndex = 0;
    for(const auto& path: mModel->getTrajectorySet()) {
        QColor defaultColor {colorIndex < pathColors.size() ?
                        pathColors.at(colorIndex) :
                        QColor(rand()&0xFF, rand()&0xFF, rand()&0xFF)};

        const bool isSelected = (&path.poses == mModel->getSelectedTrajectory());
        if(!isSelected) {
            // only selected path has color
            defaultColor.setRgb(128, 128, 128, 128);
            painter->setBrush(defaultColor);
        }

        for(const auto& point: path.poses) {
            if(isSelected) {
                painter->setBrush(point.velocity_mps < 0 ? defaultColor : QColor::fromHsvF(std::min(point.velocity_mps, 1.0F), 1.0, 0.8));
            }
            painter->drawPolygon(point.trans.map(pathPoint));
        }
        colorIndex++;
    }
}

void PathItem::onPathsChanged()
{
    prepareGeometryChange();

    int size = 0;
    for (auto& path : mModel->getTrajectorySet()) {size += path.poses.size();}
    setVisible(size > 0);

    qDebug() << "paths changed";
}
