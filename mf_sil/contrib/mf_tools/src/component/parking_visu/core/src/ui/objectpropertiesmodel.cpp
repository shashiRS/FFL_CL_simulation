#include "objectpropertiesmodel.h"

#include "models/staticobstaclemodel.h"
#include "models/parkingboxmodel.h"
#ifndef ULTRASONIC_ONLY
#include "models/parkingspacemarkingmodel.h"
#endif

#include <QMetaProperty>
#include <QDebug>

ObjectPropertiesModel::ObjectPropertiesModel(ParkingSceneModel* model, QObject *parent)
    : QAbstractItemModel(parent)
    , mModel(model)
{
    connect(mModel, &ParkingSceneModel::selectedObjectChanged, this, &ObjectPropertiesModel::onSelectedObjectChanged);
}

int ObjectPropertiesModel::rowCount(const QModelIndex &parent) const
{
    auto selected = mModel->getSelectedObject();
    if(!parent.isValid()) { // this is top level row count
        int propCount = getSelectedObjectPropCount();
        if(dynamic_cast<StaticObstacleModel*>(selected)) {
            propCount += 1;
        } else if(dynamic_cast<ParkingBoxModel*>(selected)) {
            propCount += 1 + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_DELIMITERS_NU;
#ifndef ULTRASONIC_ONLY
        } else if(dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
            propCount += 1;
#endif
        }
        return propCount;
    }
    // This is row count of child items
    // Only pseudo-properties can have child items
    // => check if parent is pseudo-property root item, then return the number
    // or corresponding pseudo property children
    else if(auto obstacle = dynamic_cast<StaticObstacleModel*>(selected)) {
        if (parent.row() == getSelectedObjectPropCount() && !parent.internalId()) {
            // StaticObstacleModel has only the shape pseudo property
            return obstacle->getStructureData().objShape_m.actualSize;
        }
    } else if(dynamic_cast<ParkingBoxModel*>(selected)) {
        if (parent.row() == getSelectedObjectPropCount() && !parent.internalId()) {
            // ParkingBoxModel has shape as first pseudo property
            return ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU;
        } else if (parent.row() > getSelectedObjectPropCount()
                   && parent.row() <= getSelectedObjectPropCount() + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU
                   && !parent.internalId()) {
            return 2;
        } else if (parent.row() > getSelectedObjectPropCount() + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU
                   && parent.row() <= getSelectedObjectPropCount() + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_DELIMITERS_NU
                   && !parent.internalId()) {
            // and some delimiters as following pseudo properties
            // delimiters have 4 entries
            return 4;
        }
#ifndef ULTRASONIC_ONLY
    } else if(dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
        if (parent.row() == getSelectedObjectPropCount() && !parent.internalId()) {
            // ParkingSpaceMarkingModel has only Line as pseudo property
            // Line has two entries
            return 2;
        }
#endif
    }

    return 0;
}

int ObjectPropertiesModel::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent)

    return 2;
}

/**
 * @return Convenience function to get property string
 */
static QString getEnumString(const EnumProperty& e) {
    auto description = e.getDescription();
    if(description) {
        return description->nameMap.value(e.asInt(), "<unknown>");
    } else {
        return "<unknown>";
    }
}

static QPointF getQPoint(const cml::Vec2Df& point) {
    return QPointF(point.x(), point.y());
}

static QString getPointString(const QPointF& point) {
    return QString::number(point.x()) + "; " + QString::number(point.y());
}

QVariant ObjectPropertiesModel::data(const QModelIndex &index, int role) const
{
    // return empty if index invalid, role is not display or edit, or if no object is selected

    if (!index.isValid())
        return QVariant();

    if ((role != Qt::DisplayRole) && (role != Qt::EditRole))
        return QVariant();


    const auto selected = mModel->getSelectedObject();
    if(!selected)
        return QVariant();


    if (index.internalId()) { // this is a pseudo-property sub-item
        // data depends on selected object type
        if(auto obstacle = dynamic_cast<StaticObstacleModel*>(selected)) {
            // Obstacles have only shape as pseudo-property
            // => item row is point index

            if(index.column() == 0)
                return "Point " + QString::number(index.row());

            const auto point = getQPoint(obstacle->getStructureData().objShape_m.array[index.row()]);

            if(role == Qt::DisplayRole) {
                return getPointString(point);
            } else {
                return point;
            }
        } else if(auto pbox = dynamic_cast<ParkingBoxModel*>(selected)) {
            // Pseudo-property root at getSelectedObjectPropCount() is shape,
            // following roots are delimiters

            // col 0: name
            if(index.column() == 0) {
                if(index.parent().row() == getSelectedObjectPropCount())
                    // this child belongs to the shape
                    return "Point " + QString::number(index.row());
                else if (index.parent().row() > getSelectedObjectPropCount()
                         && index.parent().row() <= getSelectedObjectPropCount() + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU) {
                    // Child items of VirtualLine
                    switch (index.row()) {
                    case 0:
                        return "Point 0";
                    case 1:
                        return "Point 1";
                    default:
                        break;
                    }
                }
                else {
                    // This child belongs to a delimiter
                    switch (index.row()) {
                    case 0:
                        return "indexInList";
                    case 1:
                        return "delimiterType";
                    case 2:
                        return "delimitingSide";
                    case 3:
                        return "virtLineIdx_nu";
                    default:
                        break;
                    }
                }
            }
            // col 1 : value
            else {
                if(index.parent().row() == getSelectedObjectPropCount()) {
                    // shape pseudo property
                    auto point = getQPoint(pbox->getBoxData().slotCoordinates_m.array[index.row()]);

                    if(role == Qt::DisplayRole) {
                        return getPointString(point);
                    } else {
                        return point;
                    }
                } else if (index.parent().row() > getSelectedObjectPropCount()
                           && index.parent().row() <= getSelectedObjectPropCount() + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU) {
                    // Child items of VirtualLine

                    // delimiter pseudo property, starts at getSelectedObjectPropCount()+1
                    int lineIndex = index.parent().row() - getSelectedObjectPropCount() - 1;
                    if(lineIndex < 0 || lineIndex >= ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU)
                        return QVariant();

                    switch (index.row()) {
                    case 0:
                    case 1:
                    {
                        QPointF point;
                        auto parkbox = pbox->getBoxData();
                        if (lineIndex < parkbox.numVirtualLines_nu) {
                            point = getQPoint(pbox->getBoxData().virtualLines[lineIndex].virtLineVertices_m.array[index.row()]);
                        } else {
                            point = QPointF{};
                        }

                        if(role == Qt::DisplayRole) {
                            return getPointString(point);
                        } else {
                            return point;
                        }
                    }
                    default:
                        return QVariant();
                    }
                } else {
                    // delimiter pseudo property, starts at getSelectedObjectPropCount()+1
                    int boxIndex = index.parent().row() - getSelectedObjectPropCount() - 1 - ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU;
                    if(boxIndex < 0 || boxIndex >= ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_DELIMITERS_NU)
                        return QVariant();

                    // qDebug() << "delimiter box " << boxIndex << "field" << index.row();
                    switch (index.row()) {
                    case 0:
                        return pbox->getBoxData().delimiters[boxIndex].indexInList_nu;
#ifndef ULTRASONIC_ONLY
                    case 1: {
                        const auto val = pbox->getBoxData().delimiters[boxIndex].delimiterType_nu;
                        if(role == Qt::EditRole) {
                            return QVariant::fromValue(EnumProperty(val));
                        }
                        return getEnumString(val);
                    }
#endif
                    case 2: {
                        const auto val = pbox->getBoxData().delimiters[boxIndex].delimitingSide_nu;
                        if(role == Qt::EditRole) {
                            return QVariant::fromValue(EnumProperty(val));
                        }
                        return getEnumString(val);
                    }
                    case 3:
                        return pbox->getBoxData().delimiters[boxIndex].virtLineIdx_nu;
                    default:
                        return QVariant();
                    }
                }
            }
        }
#ifndef ULTRASONIC_ONLY
        else if(auto marking = dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
            // Parking space markings have only shape as pseudo-property
            // => item row is point index

            if(index.column() == 0)
                return "Point " + QString::number(index.row());

            const auto point = getQPoint(marking->getMarking().pos_m.array[index.row()]);

            if(role == Qt::DisplayRole) {
                return getPointString(point);
            } else {
                return point;
            }
        }
#endif
    } else if(index.row() >= getSelectedObjectPropCount()) {
        // pseudo-property root-item
        // only names (at column 0), value is always empty

        if(dynamic_cast<StaticObstacleModel*>(selected)) {
            if(index.column() == 0) {
                return "Shape";
            }
        } else if(dynamic_cast<ParkingBoxModel*>(selected)) {
            if(index.row() == getSelectedObjectPropCount() && index.column() == 0) {
                return "Shape";
            } else if(index.row() > getSelectedObjectPropCount()
                      && index.row() <= getSelectedObjectPropCount() + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU
                      && index.column() == 0) {
                return "VirtualLine " + QString::number(index.row() - getSelectedObjectPropCount() - 1);
            } else if(index.column() == 0) {
                return "Delimiter " + QString::number(index.row() - getSelectedObjectPropCount() - 1 - ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU);
            }
#ifndef ULTRASONIC_ONLY
        } else if(dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
            if(index.column() == 0) {
                return "Line";
            }
#endif
        }
    } else {
        // real object Q_PROPERTY
        // index.row is property index

        auto meta = selected->metaObject();
        if(index.row() < meta->propertyCount()) {
            if(index.column() == 0) {
                return meta->property(index.row()).name();
            } else if(index.column() == 1) {
                QVariant value = meta->property(index.row()).read(selected);

                // return direct value (even if enum) for editing since delegate knows what to do
                if(role == Qt::EditRole)
                    return value;

                // otherwise display points and enums as strings
                if(value.type() == QMetaType::QPointF) {
                    return getPointString(value.toPointF());
                } else if(value.canConvert<EnumProperty>()) {
                    return getEnumString(value.value<EnumProperty>());
                } else {
                    return value;
                }
            }
        }
    }

    return QVariant();
}

QModelIndex ObjectPropertiesModel::index(int row, int column, const QModelIndex &parent) const
{
    if (!hasIndex(row, column, parent))
        return QModelIndex();

    auto selected = mModel->getSelectedObject();

    if(!parent.isValid()) { // root item
        return createIndex(row, column);
    }
    // not root item -> special pseudo-property of the selected object
    else if (dynamic_cast<StaticObstacleModel*>(selected)) {
        // obstacles have only 1-level polygon coordinates as pseudo-props
        if (parent.internalId())
            return QModelIndex();

        // only last property is pseudo, others are real properties and thus don't have children
        if (parent.row() == getSelectedObjectPropCount()) {
            return createIndex(row, column, ITEM_STATIC_OBSTACLE_POLY);
        }
    } else if (dynamic_cast<ParkingBoxModel*>(selected)) {
        if (parent.internalId())
            return QModelIndex(); // Here we are in child layer

        // last 1 + MAX_NUM_PARKING_BOX_DELIMITERS props are pseudo
        if (parent.row() == getSelectedObjectPropCount()) {
            return createIndex(row, column, ITEM_PARKING_BOX_POLY);
        } else if(parent.row() > getSelectedObjectPropCount()
                  && parent.row() <= getSelectedObjectPropCount()+ ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU) {
            return createIndex(row, column, ITEM_PARKING_BOX_VIRTUAL_LINE_START + parent.row() - getSelectedObjectPropCount() - 1);
        } else if(parent.row() > getSelectedObjectPropCount()+ ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU
                  && parent.row() <= getSelectedObjectPropCount()+ ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU+ ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_DELIMITERS_NU) {
            return createIndex(row, column, ITEM_PARKING_BOX_DELIMITER_START + parent.row() - getSelectedObjectPropCount() - 1 - ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU);
        }
#ifndef ULTRASONIC_ONLY
    } else if (dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
        // parking space markings have only 1-level polygon coordinates as pseudo-props
        if (parent.internalId())
            return QModelIndex();

        // only last property is pseudo, others are real properties and thus don't have children
        if (parent.row() == getSelectedObjectPropCount()) {
            return createIndex(row, column, ITEM_PARKING_SPACE_MARKING);
        }
#endif
    }

    return QModelIndex();
}

QModelIndex ObjectPropertiesModel::parent(const QModelIndex &index) const
{
    if (!index.isValid())
        return QModelIndex();

    auto selected = mModel->getSelectedObject();
    if (index.internalId()) { // this is a pseudo-property item
        // parent depends on selected object type
        if(dynamic_cast<StaticObstacleModel*>(selected)) {
            // obstacles have only polygons as pseudo-prop
            return createIndex(getSelectedObjectPropCount(), 0);
        } else if(dynamic_cast<ParkingBoxModel*>(selected)) {
            // parking boxes have shape and delimiters as as pseudo-prop
            if(index.internalId() == ITEM_PARKING_BOX_POLY)
                return createIndex(getSelectedObjectPropCount(), 0);
            else if(index.internalId() >= ITEM_PARKING_BOX_VIRTUAL_LINE_START
                    && index.internalId() < ITEM_PARKING_BOX_VIRTUAL_LINE_START + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU)
                return createIndex(getSelectedObjectPropCount() + 1
                                   +index.internalId() - ITEM_PARKING_BOX_VIRTUAL_LINE_START, 0);
            else
                return createIndex(getSelectedObjectPropCount() + 1 + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU
                                   +index.internalId() - ITEM_PARKING_BOX_DELIMITER_START, 0);
#ifndef ULTRASONIC_ONLY
        } else if(dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
            // parking boxes have only polygons as pseudo-prop
            return createIndex(getSelectedObjectPropCount(), 0);
#endif
        }
    }

    return QModelIndex();
}

bool ObjectPropertiesModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
    bool set = false;
    if (index.isValid() && role == Qt::EditRole && index.column() == 1) {
        if(index.internalId()) { // pseudo-property sub-item
            auto selected = mModel->getSelectedObject();
            if(auto obstacle = dynamic_cast<StaticObstacleModel*>(selected)) {
                auto object = obstacle->getStructureData();
                if(value.type() == QMetaType::QPointF &&  static_cast<uint8_t>(index.row()) < object.objShape_m.actualSize) {
                    QPointF p = value.toPointF();
                    cml::Vec2Df v(p.x(), p.y());
                    if(!(object.objShape_m.array[index.row()] == v)) {
                        object.objShape_m.array[index.row()] = cml::Vec2Df_POD{ v.x(), v.y() };
                        obstacle->updateShapePolygon(object);
                        set = true;
                    }
                }
            } else if(auto pbox = dynamic_cast<ParkingBoxModel*>(selected)) {
                if(index.internalId() == ITEM_PARKING_BOX_POLY) {
                    auto shape = pbox->getBoxData();
                    if(value.type() == QMetaType::QPointF && index.row() < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU) {
                        QPointF p = value.toPointF();
                        cml::Vec2Df v(p.x(), p.y());
                        if(!(shape.slotCoordinates_m.array[index.row()] == v)) {
                            shape.slotCoordinates_m.array[index.row()] = cml::Vec2Df_POD{ v.x(), v.y() };
                            pbox->setBoxData(shape);
                            set = true;
                        }
                    }
                } else if(index.internalId() >= ITEM_PARKING_BOX_VIRTUAL_LINE_START
                          && index.internalId() < ITEM_PARKING_BOX_VIRTUAL_LINE_START + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU) {
                    int lineIndex = index.internalId() - ITEM_PARKING_BOX_VIRTUAL_LINE_START;
                    auto data = pbox->getBoxData();
                    switch (index.row()) {
                    case 0:
                    case 1:
                    {
                        QPointF p = value.toPointF();
                        cml::Vec2Df v{ float32_t(p.x()), float32_t(p.y()) };
                        if (data.virtualLines[lineIndex].virtLineVertices_m.actualSize == 0 && index.row() == 0) {
                            data.virtualLines[lineIndex].virtLineVertices_m.array[0] = cml::Vec2Df_POD{ v.x(), v.y() };
                            data.virtualLines[lineIndex].virtLineVertices_m.array[1] = cml::Vec2Df_POD { 0.0F, 0.0F};
                            data.virtualLines[lineIndex].virtLineVertices_m.actualSize = 2U;
                            set = true;
                        } else if (data.virtualLines[lineIndex].virtLineVertices_m.actualSize == 0 && index.row() == 1) {
                            data.virtualLines[lineIndex].virtLineVertices_m.array[0] = cml::Vec2Df_POD{ 0.0F, 0.0F };
                            data.virtualLines[lineIndex].virtLineVertices_m.array[1] = cml::Vec2Df_POD{ v.x(), v.y() };
                            data.virtualLines[lineIndex].virtLineVertices_m.actualSize = 2U;
                            set = true;
                        } else if (data.virtualLines[lineIndex].virtLineVertices_m.actualSize == 2 &&
                            !(data.virtualLines[lineIndex].virtLineVertices_m.array[index.row()] == v)) {
                            data.virtualLines[lineIndex].virtLineVertices_m.array[index.row()] = cml::Vec2Df_POD{ v.x(), v.y() };
                            set = true;
                        }
                        break;
                    }
                    default:
                        break;
                    }

                    if(set) {
                        pbox->setBoxData(data);
                    }
                } else if(index.internalId() >= ITEM_PARKING_BOX_DELIMITER_START
                          && index.internalId() < ITEM_PARKING_BOX_DELIMITER_START + ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_DELIMITERS_NU) {
                    int delimiterIndex = index.internalId() - ITEM_PARKING_BOX_DELIMITER_START;
                    auto data = pbox->getBoxData();
                    switch (index.row()) {
                    case 0:
                        if(data.delimiters[delimiterIndex].indexInList_nu != value.toInt()) {
                            data.delimiters[delimiterIndex].indexInList_nu = value.toInt();
                            set = true;
                        }
                        break;
#ifndef ULTRASONIC_ONLY
                    case 1:
                        if(value.canConvert<EnumProperty>()) {
                            auto newValue = (si::DelimiterTypes)value.value<EnumProperty>().asInt();
                            if(newValue != data.delimiters[delimiterIndex].delimiterType_nu) {
                                data.delimiters[delimiterIndex].delimiterType_nu = newValue;
                                set = true;
                            }
                        }
                        break;
#endif
                    case 2:
                        if(value.canConvert<EnumProperty>()) {
                            auto newValue = (si::RelativeLocationToParkingBox)value.value<EnumProperty>().asInt();
                            if(newValue != data.delimiters[delimiterIndex].delimitingSide_nu) {
                                data.delimiters[delimiterIndex].delimitingSide_nu = newValue;
                                set = true;
                            }
                        }
                        break;
                    case 3:
                        if (data.delimiters[delimiterIndex].virtLineIdx_nu != value.toInt()) {
                            data.delimiters[delimiterIndex].virtLineIdx_nu = value.toInt();
                            set = true;
                        }
                        break;
                    default:
                        break;
                    }

                    if(set) {
                        pbox->setBoxData(data);
                    }
                }
            }
#ifndef ULTRASONIC_ONLY
             else if(auto marking = dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
                auto data = marking->getMarking();
                data.pos_m.actualSize = ap_common::AP_COMMON_TYPES_Consts::AP_G_NUM_POINTS_PER_PARKING_LINE_NU;
                if (value.type() == QMetaType::QPointF && index.row() < 2) {
                    QPointF p = value.toPointF();
                    cml::Vec2Df v(p.x(), p.y());
                    if (!(data.pos_m.array[index.row()] == v)) {
                        data.pos_m.array[index.row()] = cml::Vec2Df_POD{ v.x(), v.y() };
                        marking->setMarking(data);
                        set = true;
                    }
                }
            }
#endif
        } else {
            if(index.row() < getSelectedObjectPropCount()) { // real property
                auto selected = mModel->getSelectedObject();
                auto property = selected->metaObject()->property(index.row());

                if(property.read(selected) != value) {
                    selected->metaObject()->property(index.row()).write(selected, value);
                    set = true;
                }
            }
        }
    }

    if(set) {
        mModel->createHistoryPoint();
        emit dataChanged(index, index);
    }
    return set;
}

Qt::ItemFlags ObjectPropertiesModel::flags(const QModelIndex &index) const
{
    if(index.isValid() && index.column() == 1) {
        if(!index.internalId()) { // root item
            if(index.row() < getSelectedObjectPropCount()) {
                return Qt::ItemIsEditable | QAbstractItemModel::flags(index);
            }
        } else {
            // this is a pseudo-property sub-item
            auto selected = mModel->getSelectedObject();
            if(auto obstacle = dynamic_cast<StaticObstacleModel*>(selected)) {
                if(static_cast<uint8_t>(index.row()) < obstacle->getStructureData().objShape_m.actualSize) {
                    return Qt::ItemIsEditable | QAbstractItemModel::flags(index);
                }
            } else if(dynamic_cast<ParkingBoxModel*>(selected)) {
                return Qt::ItemIsEditable | QAbstractItemModel::flags(index);
#ifndef ULTRASONIC_ONLY
            }  else if(dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
                if(index.row() < 2) {
                    return Qt::ItemIsEditable | QAbstractItemModel::flags(index);
                }
#endif
            }
        }
    }

    return QAbstractItemModel::flags(index);
}

QVariant ObjectPropertiesModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if(role == Qt::DisplayRole && orientation == Qt::Horizontal) {
        if(section == 0)
            return "Property";
        else if(section == 1)
            return "Value";
        else
            return "";
    } else {
        return QAbstractItemModel::headerData(section, orientation, role);
    }
}

void ObjectPropertiesModel::onSelectedObjectChanged(QObject *selected, QObject *previous)
{
    beginResetModel();

    if(previous && !dynamic_cast<ParkingSceneModel*>(previous)) {
        // disconnect any signals connected from previous to this, except if this is the main parking scene,
        // since parking scene should keep onSelectedObjectChanged
        disconnect(previous, nullptr, this, nullptr);
    }

    if(selected && !dynamic_cast<ParkingSceneModel*>(selected)) {
        QMetaMethod updateSlot = metaObject()->method(
                metaObject()->indexOfSlot("onPropertyChanged()"));

        for(int i = 0; i < selected->metaObject()->propertyCount(); i++) {
            auto property = selected->metaObject()->property(i);
            if(property.hasNotifySignal()) {
                connect(selected, property.notifySignal(), this, updateSlot);
            }
        }
    }

    if(auto obstacle = dynamic_cast<StaticObstacleModel*>(selected)) {
        connect(obstacle, &StaticObstacleModel::shapeChanged, this, [this, obstacle] (int previousCount) {
            // emit rowsInserted()/rowsRemoved() signals if number of polygon points changed
            auto currentPointCount = obstacle->getStructureData().objShape_m.actualSize;
            auto rootIndex = createIndex(getSelectedObjectPropCount(), 0);
            if(currentPointCount > static_cast<uint8_t>(previousCount)) {
                emit beginInsertRows(rootIndex, previousCount, currentPointCount-1);
                emit endInsertRows();
            }
            // we cannot remove items since the original data inside the obstacle model
            // has already decreased the number of points, so rowCount() here would
            // also return the lower number of points and beginRemoveRows() would fail
            // for whatever reason it seems to be not necessary to call beginRemoveRows().
            // At some point in the past, this code actually worked, but then stopped working
            // You can try to git bisect to find the relevant commit.
            /*
            else if(currentPointCount < previousCount) {
                emit beginRemoveRows(rootIndex, currentPointCount, previousCount-1);
                emit endRemoveRows();
            }
            */

            // emit data change for all points
            auto indexStart = createIndex(0, 1, ITEM_STATIC_OBSTACLE_POLY);
            auto indexEnd = createIndex(currentPointCount, 1, ITEM_STATIC_OBSTACLE_POLY);
            emit dataChanged(indexStart, indexEnd);
        });
    } else if(auto pbox = dynamic_cast<ParkingBoxModel*>(selected)) {
        connect(pbox, &ParkingBoxModel::shapeChanged, this, [this] () {
            // emit data change for all points
            auto indexStart = createIndex(0, 1, ITEM_PARKING_BOX_POLY);
            auto indexEnd = createIndex(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU-1, 1, ITEM_PARKING_BOX_POLY);
            emit dataChanged(indexStart, indexEnd);
        });
#ifndef ULTRASONIC_ONLY
    } else if(auto marking = dynamic_cast<ParkingSpaceMarkingModel*>(selected)) {
        connect(marking, &ParkingSpaceMarkingModel::markingChanged, this, [this] () {
            // emit data change for all points
            auto indexStart = createIndex(0, 1, ITEM_PARKING_SPACE_MARKING);
            auto indexEnd = createIndex(1, 1, ITEM_PARKING_SPACE_MARKING);
            emit dataChanged(indexStart, indexEnd);
        });
#endif
    }

    endResetModel();
}

void ObjectPropertiesModel::onPropertyChanged()
{
    // qDebug() << "ObjectPropertiesModel::onPropertyChanged()";
    auto indexStart = createIndex(0, 1);
    auto indexEnd = createIndex(getSelectedObjectPropCount()-1, 1);
    emit dataChanged(indexStart, indexEnd);
}

int ObjectPropertiesModel::getSelectedObjectPropCount() const
{
    auto selected = mModel->getSelectedObject();
    return selected? selected->metaObject()->propertyCount() : 0;
}
