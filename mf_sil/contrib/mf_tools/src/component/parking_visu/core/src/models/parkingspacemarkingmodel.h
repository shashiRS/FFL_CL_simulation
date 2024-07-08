#ifndef PARKINGSPACEMARKINGMODEL_H
#define PARKINGSPACEMARKINGMODEL_H

#ifndef ULTRASONIC_ONLY

#include <QObject>
#include <QPointF>
#include "models/enumproperty.h"
#include <si/parking_space_markings_serializable.h>

class ParkingSpaceMarkingModel : public QObject
{
    Q_PROPERTY(int existenceProb_perc READ getExistanceProbability WRITE setExistanceProbability NOTIFY markingChanged)
    Q_PROPERTY(float width READ getWidth WRITE setWidth NOTIFY markingChanged)
    Q_PROPERTY(EnumProperty type_nu READ getMarkingType WRITE setMarkingType NOTIFY markingChanged)

    Q_OBJECT
public:
    explicit ParkingSpaceMarkingModel(QObject *parent = nullptr);

    const si::ParkingSpaceMarkingsSerializable& getMarking() const {
        return mData;
    }

    void setMarking(const si::ParkingSpaceMarkingsSerializable &marking);

    void setLinePosition(const QPointF &p1, const QPointF &p2);

    EnumProperty getMarkingType() const;
    void setMarkingType(const EnumProperty markingType);

    bool isVisible() const;
    void setVisible(bool visible);

    uint8_t getExistanceProbability() const;
    void setExistanceProbability(uint8_t existProb);

    float getWidth() const;
    void setWidth(float width);

signals:
    void markingChanged();

public slots:

private:
    si::ParkingSpaceMarkingsSerializable mData;
};

#endif // ULTRASONIC_ONLY
#endif // PARKINGSPACEMARKINGMODEL_H