#ifndef TAPOSDDEBUGITEM_H
#define TAPOSDDEBUGITEM_H

#include <QObject>
#include <QGraphicsItem>

#include <models/taposddebugmodel.h>
#include <ap_common/ap_common_generated_types.h>

class TaposdDebugItem: public QObject, public QGraphicsItem
{
    Q_OBJECT
public:
    TaposdDebugItem(TaposdDebugModel* model, QGraphicsItem* parent = nullptr);

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    QRectF boundingRect() const;

protected slots:
    void onTaposdDataChanged();
    void onTaposdVisibleChanged(bool visible);

private:
    void createMaxComfBoxPoly(const ap_tp::ParkingBoxDebugInfo& boxInfo);

    TaposdDebugModel* mModel = nullptr;
};

#endif // TAPOSDDEBUGITEM_H
