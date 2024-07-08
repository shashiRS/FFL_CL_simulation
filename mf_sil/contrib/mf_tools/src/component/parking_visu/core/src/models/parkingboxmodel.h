#ifndef PARKINGBOXMODEL_H
#define PARKINGBOXMODEL_H

#include <QObject>
#include <ap_common/ap_common_generated_types.h>
#include <si/parking_box_serializable.h>
#include <si/virtual_line_serializable.h>
#include "models/enumproperty.h"
#include <QPointF>

class ParkingBoxModel : public QObject
{
    Q_PROPERTY(int parkingBoxID READ getParkingBoxID WRITE setParkingBoxID NOTIFY shapeChanged)
    Q_PROPERTY(int existenceProb_perc READ getExistenceProb WRITE setExistenceProb NOTIFY shapeChanged)
    Q_PROPERTY(EnumProperty parkingScenario_nu READ getParkingScenario WRITE setParkingScenario NOTIFY shapeChanged)

    Q_OBJECT
public:
    explicit ParkingBoxModel(QObject *parent = nullptr);

    void setBoxData(const si::ParkingBoxSerializable& data);
    const si::ParkingBoxSerializable& getBoxData() const {
        return mBoxData;
    }

    int getParkingBoxID() const;
    void setParkingBoxID(int parkingBoxID);

    int getExistenceProb() const;
    void setExistenceProb(int existenceProb_perc);

    EnumProperty getParkingScenario() const;
    void setParkingScenario(EnumProperty parkingScenario_nu);

    si::VirtualLineSerializable& getVirtualLine(uint8_t index);
    void setVirtualLinePoints(uint8_t index, QPointF point1, QPointF point2);
    void setVirtualLinePoint(uint8_t vlIndex, uint8_t pIndex, QPointF point);
    bool isVirtualLineVisible(uint8_t vlIndex) const;

signals:
    void shapeChanged();
    void virtualLinesChanged();


private:
    si::ParkingBoxSerializable mBoxData;
};

#endif // PARKINGBOXMODEL_H
