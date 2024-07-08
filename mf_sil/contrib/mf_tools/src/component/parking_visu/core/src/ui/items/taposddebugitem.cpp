#include "taposddebugitem.h"
#include <QGraphicsPolygonItem>
#include <QBrush>
#include <QPen>

#include <ap_tp/ap_tp_generated_types.h>


TaposdDebugItem::TaposdDebugItem(TaposdDebugModel *model, QGraphicsItem *parent):
    QGraphicsItem (parent),
    mModel(model)
{
    connect(mModel, &TaposdDebugModel::taposdDataChanged, this, &TaposdDebugItem::onTaposdDataChanged);
    connect(mModel, &TaposdDebugModel::visibleChanged, this, &TaposdDebugItem::onTaposdVisibleChanged);
}

void TaposdDebugItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
}

QRectF TaposdDebugItem::boundingRect() const
{
    return QRectF();
}

void TaposdDebugItem::onTaposdDataChanged()
{
    while(!childItems().empty()) {
        delete childItems().front();
    }

    const ap_tp::TAPOSDDebugPort& data = mModel->getData();

    for (uint8_t i = 0U; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU; i++) {
        if (data.pbDebugBackwards[i].maxParkingBox.numValidPoints_nu > 0U) {
            createMaxComfBoxPoly(data.pbDebugBackwards[i]);
        }
        if (data.pbDebugForwards[i].maxParkingBox.numValidPoints_nu > 0U) {
            createMaxComfBoxPoly(data.pbDebugForwards[i]);
        }
    }
}

void TaposdDebugItem::onTaposdVisibleChanged(bool visible)
{
    setVisible(visible);
}

void TaposdDebugItem::createMaxComfBoxPoly(const ap_tp::ParkingBoxDebugInfo& boxInfo)
{
    QPolygonF polyMax;
    QPolygonF polyComf;
    for (uint8_t i = 0U; i < ap_tp::AP_TP_Const::AP_T_MAX_NUM_SMPL_PBOX_VERT_NU; i++) {
        polyMax.append(QPointF{static_cast<qreal>(boxInfo.maxParkingBox.posX_m[i]),
                               static_cast<qreal>(boxInfo.maxParkingBox.posY_m[i])});
        polyComf.append(QPointF{static_cast<qreal>(boxInfo.comfParkingBox.posX_m[i]),
                                static_cast<qreal>(boxInfo.comfParkingBox.posY_m[i])});
    }

    auto polyItemMax = new QGraphicsPolygonItem(polyMax, this);
    polyItemMax->setPen(Qt::NoPen);
    polyItemMax->setBrush(QColor(255, 0, 0, 50));

    auto polyItemComf = new QGraphicsPolygonItem(polyComf, this);
    polyItemComf->setPen(Qt::NoPen);
    polyItemComf->setBrush(QColor(0, 255, 0, 150));
}
