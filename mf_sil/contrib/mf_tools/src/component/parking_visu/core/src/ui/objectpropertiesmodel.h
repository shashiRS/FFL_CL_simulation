#ifndef OBJECTPROPERTIESMODEL_H
#define OBJECTPROPERTIESMODEL_H

#include <QAbstractItemModel>

#include "models/parkingscenemodel.h"

/**
 * Model for object properties as seen in the property window
 *
 * The first top level items always contain the Qt properties
 * defined by Q_PROPERTY()
 *
 * Additionally so called pseudo-properties are located just behind
 * the Qt properties. They can have child items and be dynamically
 * added or removed during execution (e.g. points of a polygon shape).
 *
 * Pseudo properties need to be handled manually by checking the selected
 * object type
 */
class ObjectPropertiesModel : public QAbstractItemModel
{
    Q_OBJECT

public:
    enum ItemType {
        ITEM_NONE = 0,
        ITEM_STATIC_OBSTACLE_POLY,
        ITEM_PARKING_BOX_POLY,
        ITEM_PARKING_SPACE_MARKING,
        ITEM_PARKING_BOX_DELIMITER_START = 1000,
        ITEM_PARKING_BOX_VIRTUAL_LINE_START = 1200
    };

    explicit ObjectPropertiesModel(ParkingSceneModel* model, QObject *parent = nullptr);

    // Basic functionality:
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    QModelIndex index(int row, int column, const QModelIndex &parent) const;
    QModelIndex parent(const QModelIndex &child) const;

    bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole) override;
    Qt::ItemFlags flags(const QModelIndex & index) const override ;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

public slots:
    /**
     * Called when the selected object has changed. Disconnects any signals from
     * previous object, and connects the update signals from real and pseudo properties
     * to the dataChanged() signal of this properties model.
     */
    void onSelectedObjectChanged(QObject* selected, QObject* previous);

    void onPropertyChanged();

private:
    ParkingSceneModel* mModel = nullptr;

    /**
     * @return The number of Qt properties for the currently selected object.
     *         0 if no object currently selected.
     */
    int getSelectedObjectPropCount() const;
};

#endif // OBJECTPROPERTIESMODEL_H
