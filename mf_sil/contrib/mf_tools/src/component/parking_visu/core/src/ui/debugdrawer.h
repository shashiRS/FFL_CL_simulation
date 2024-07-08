#ifndef DEBUGDRAWER_H
#define DEBUGDRAWER_H

#include <QObject>
#include <QGraphicsItem>

#include "models/parkingscenemodel.h"


class DebugDrawer : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool debugCaptionsVisible READ areCaptionsVisible WRITE setCaptionsVisible)
public:
    explicit DebugDrawer(QGraphicsScene* scene, QObject* parent, ParkingSceneModel* model);

    void drawPoint(QString const& name, float x, float y);
    void drawLine(QString const& name, float x1, float y1, float x2, float y2);
    void drawCircle(QString const& name, float x, float y, float r);
    void drawPolygon(QString const& name, QPolygonF const& poly);

    void clearAll();

    void setCaptionsVisible(bool visible);

    bool areCaptionsVisible() const {
        return mCaptionRootItem->isVisible();
    }

    ParkingSceneModel* getParkingModel() {
        return mParkingModel;
    }
signals:

public slots:

private:
    QGraphicsItem* mRootItem;
    QGraphicsItem* mCaptionRootItem;

    std::map<QString, std::pair<std::unique_ptr<QGraphicsItem>, std::unique_ptr<QGraphicsSimpleTextItem>>> mItemMap;

    ParkingSceneModel* mParkingModel = nullptr;

    /**
     * @brief Retrieve item from mItemMap with given name
     *
     * If item does not exist or has wrong type, it is created.
     *
     * @tparam T graphics item class, needs to have a T::Type value defined
     * @param name
     * @return non-null item pointer
     */
    template<class T>
    std::pair<T*, QGraphicsSimpleTextItem*> getCreateItem(QString const& name);
};

#endif // DEBUGDRAWER_H
