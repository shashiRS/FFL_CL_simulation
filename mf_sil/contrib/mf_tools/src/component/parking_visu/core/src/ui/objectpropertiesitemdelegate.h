#ifndef OBJECTPROPERTIESITEMDELEGATE_H
#define OBJECTPROPERTIESITEMDELEGATE_H

#include <QItemDelegate>
#include <QModelIndex>
#include <QObject>
#include <QDoubleSpinBox>
#include <QPointF>

/// adapted from http://www.bogotobogo.com/Qt/Qt5_QTableView_QItemDelegate_ModelView_MVC.php

class ObjectPropertiesItemDelegate : public QItemDelegate
{
    Q_OBJECT
public:
    explicit ObjectPropertiesItemDelegate(QObject *parent = 0);

    // Create Editor when we construct MyDelegate
    QWidget* createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;

    // Then, we set the Editor
    void setEditorData(QWidget *editor, const QModelIndex &index) const;

    // When we modify data, this model reflect the change
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const;

    // Give the SpinBox the info on size and location
    void updateEditorGeometry(QWidget *editor, const QStyleOptionViewItem &option, const QModelIndex &index) const;

    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override;

private:
};

class PointFEditor: public QWidget {
    Q_OBJECT
public:
    PointFEditor(QWidget *parent = nullptr);

    friend class ObjectPropertiesItemDelegate;

    QPointF getPoint() const;

signals:
    void pointChanged();

private:
    QDoubleSpinBox* mSpinboxX = nullptr;
    QDoubleSpinBox* mSpinboxY = nullptr;
};

#endif // OBJECTPROPERTIESITEMDELEGATE_H
