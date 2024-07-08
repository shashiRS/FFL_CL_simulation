#ifndef STATICOBSTACLEMODEL_H
#define STATICOBSTACLEMODEL_H

#include <QObject>
#include <QPolygon>

#include <si/static_object_serializable.h>

#include "enumproperty.h"

class StaticObstacleModel : public QObject
{
    Q_OBJECT
public:
    explicit StaticObstacleModel(QObject *parent = nullptr);

    Q_PROPERTY(EnumProperty heightType READ heightType WRITE setHeightTypeProp NOTIFY dataChanged)
    Q_PROPERTY(bool visible READ isVisible WRITE setVisible NOTIFY visibilityChanged)
#ifndef ULTRASONIC_ONLY
        Q_PROPERTY(EnumProperty refObjClass READ getObjClass WRITE setObjClass NOTIFY dataChanged)
#endif

    void setStructureData(si::StaticObjectSerializable const& structure);
    const si::StaticObjectSerializable& getStructureData() const {
        return mStructureData;
    }

    void updateShapePolygon(const si::StaticObjectSerializable& object);

    EnumProperty heightType() const {
        return mStructureData.objHeightClass_nu;
    }

    si::StaticObjHeigthType getHeightType() const {
        return mStructureData.objHeightClass_nu;
    }

    void setHeightType(si::StaticObjHeigthType height);
    void setHeightTypeProp(const EnumProperty& prop);

#ifndef ULTRASONIC_ONLY
    EnumProperty getObjClass() const;
    void setObjClass(const EnumProperty& prop);
#endif
    bool isVisible() const {
        return mStructureData.existenceProb_perc > 0;
    }
    void setVisible(bool visible);

signals:
    void shapeChanged(int previousPointCount);
    void visibilityChanged(bool visible);
    void dataChanged();

private:
    si::StaticObjectSerializable mStructureData;

    QPolygonF mShape;
};

#endif // STATICOBSTACLEMODEL_H
