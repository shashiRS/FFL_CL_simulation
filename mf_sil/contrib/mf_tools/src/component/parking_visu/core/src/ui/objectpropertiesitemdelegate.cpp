#include "objectpropertiesitemdelegate.h"

#include "models/enumproperty.h"

#include <QDoubleSpinBox>
#include <QComboBox>
#include <QDebug>
#include <QHBoxLayout>

ObjectPropertiesItemDelegate::ObjectPropertiesItemDelegate(QObject *parent)
    : QItemDelegate(parent)
{

}

// TableView need to create an Editor
// Create Editor when we construct ObjectPropertiesItemDelegate
// and return the Editor
QWidget* ObjectPropertiesItemDelegate::createEditor(QWidget *parent,
                                                    const QStyleOptionViewItem &option,
                                                    const QModelIndex &index) const
{
	if (index.column() != 1) {
		return nullptr;
	}

    auto value = index.model()->data(index, Qt::EditRole);
    if(value.canConvert<EnumProperty>()) {
        return new QComboBox(parent);
    } else if(value.type() == QMetaType::Float || value.type() == QMetaType::Double) {
        auto e = new QDoubleSpinBox(parent);
        e->setMinimum(-10000.0);
        e->setMaximum(10000.0);
        e->setSingleStep(0.1);
        e->setDecimals(4);
        return e;
    } else if(value.type() == QMetaType::QPointF) {
        return new PointFEditor(parent);
    } else{
        return QItemDelegate::createEditor(parent, option, index);
    }
}

// Then, we set the Editor
// Gets the data from Model and feeds the data to Editor
void ObjectPropertiesItemDelegate::setEditorData(QWidget *editor,
                                                 const QModelIndex &index) const
{
    // qDebug() << "setEditorData";

    // Get the value via index of the Model
    auto value = index.model()->data(index, Qt::EditRole);

    if(value.canConvert<EnumProperty>()) {
        QComboBox *comboBox = dynamic_cast<QComboBox*>(editor);

        auto e = value.value<EnumProperty>();
        auto description = e.getDescription();
        comboBox->clear();

        if(description) {
            for(auto key: description->nameMap.keys()) {
                comboBox->addItem(description->nameMap.value(key), QVariant(key));
            }

            comboBox->setCurrentText(description->nameMap.value(e.asInt()));
        }
    } else if(value.type() == QMetaType::Float || value.type() == QMetaType::Double) {
        QDoubleSpinBox *spinbox = dynamic_cast<QDoubleSpinBox*>(editor);

        // call setModelData() when spinbox value changes to have an instant
        // user input response
        // TODO this is a very dirty hack (using const_cast)
        spinbox->disconnect();

        // make sure to compare float values if type is float
        if(((value.type() == QMetaType::Float) && ((float)spinbox->value() != value.toFloat()))
                || ((value.type() == QMetaType::Double && spinbox->value() != value.toDouble())))
        {
            spinbox->setValue(value.toDouble());
        }

        // explicit type needed since QDoubleSpinBox::valueChanged has two overloads
        void (QDoubleSpinBox::* valueChanged)(double) = &QDoubleSpinBox::valueChanged;
        connect(spinbox, valueChanged, this, [this, editor, index]() {
             this->setModelData(editor, const_cast<QAbstractItemModel*>(index.model()), index);
        });
    } else if(value.type() == QMetaType::QPointF) {
        PointFEditor *pe = dynamic_cast<PointFEditor*>(editor);
        QPointF point = value.toPointF();

        // same dirty hack again
        pe->disconnect();

        // compare floats instead of qreal since we use float for polygon corners internally
        // otherwise there will be a tiny error making it seem that the values are acutally different
        if((float)point.x() != (float)pe->getPoint().x() || (float)point.y() != (float)pe->getPoint().y()) {
            pe->mSpinboxX->setValue(point.x());
            pe->mSpinboxY->setValue(point.y());
        }

        connect(pe, &PointFEditor::pointChanged, this, [this, editor, index] () {
            this->setModelData(editor, const_cast<QAbstractItemModel*>(index.model()), index);
        });
    } else {
        QItemDelegate::setEditorData(editor, index);
    }
}

// When we modify data, this model reflect the change
void ObjectPropertiesItemDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                                const QModelIndex &index) const
{
    auto value = index.model()->data(index, Qt::EditRole);

    if(value.canConvert<EnumProperty>()) {
        QComboBox *comboBox = dynamic_cast<QComboBox*>(editor);
        EnumProperty prop = value.value<EnumProperty>();
        prop.setAsInt(comboBox->currentData().toInt());

        model->setData(index, QVariant::fromValue(prop), Qt::EditRole);
    } else if(value.type() == QMetaType::Float || value.type() == QMetaType::Double) {
        QDoubleSpinBox *spinbox = dynamic_cast<QDoubleSpinBox*>(editor);
        model->setData(index, QVariant(spinbox->value()));
    } else if(value.type() == QMetaType::QPointF) {
        PointFEditor *pe = dynamic_cast<PointFEditor*>(editor);
        model->setData(index, QVariant(pe->getPoint()));
    } else {
        QItemDelegate::setModelData(editor, model, index);
    }

}

// Give the SpinBox the info on size and location
void ObjectPropertiesItemDelegate::updateEditorGeometry(QWidget *editor,
                                                        const QStyleOptionViewItem &option,
                                                        const QModelIndex &index) const
{
    Q_UNUSED(index)
    editor->setGeometry(option.rect);
}

bool ObjectPropertiesItemDelegate::editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index)
{
    // qDebug() << "editorEvent";
    return QItemDelegate::editorEvent(event, model, option, index);
}

PointFEditor::PointFEditor(QWidget *parent)
    : QWidget(parent)
{
    auto layout = new QHBoxLayout(this);
    layout->setMargin(0);
    layout->addWidget(mSpinboxX = new QDoubleSpinBox());
    layout->addWidget(mSpinboxY = new QDoubleSpinBox());
    setLayout(layout);

    mSpinboxX->setMinimum(-10000.0);
    mSpinboxX->setMaximum(10000.0);
    mSpinboxX->setSingleStep(0.1);
    mSpinboxX->setDecimals(4);

    mSpinboxY->setMinimum(-10000.0);
    mSpinboxY->setMaximum(10000.0);
    mSpinboxY->setSingleStep(0.1);
    mSpinboxY->setDecimals(4);

    // explicit type needed since QDoubleSpinBox::valueChanged has two overloads
    void (QDoubleSpinBox::* valueChanged)(double) = &QDoubleSpinBox::valueChanged;

    connect(mSpinboxX, valueChanged, this, [this] () {
        emit pointChanged();
    });

    connect(mSpinboxY, valueChanged, this, [this] () {
        emit pointChanged();
    });
}

QPointF PointFEditor::getPoint() const
{
    return QPointF(mSpinboxX->value(), mSpinboxY->value());
}
