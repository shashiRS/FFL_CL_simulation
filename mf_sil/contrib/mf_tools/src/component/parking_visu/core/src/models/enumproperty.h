#ifndef ENUMPROPERTY_H
#define ENUMPROPERTY_H

#include <QMap>
#include <QMetaType>

class EnumPropertyDescription
{
public:
    EnumPropertyDescription(const QMap<int, QString> &names);

    const QMap<int, QString> nameMap;

    template<class T>
    static const EnumPropertyDescription* getForType();
};


class EnumProperty {
public:
    template<class T>
    EnumProperty(T value) : mValue((int)value), mDescription(EnumPropertyDescription::getForType<T>())
    {}

    EnumProperty(const EnumProperty& other):
        mValue(other.mValue), mDescription(other.mDescription)
    {}

    EnumProperty()
    {}

    EnumProperty& operator=(const EnumProperty& other) {
        mValue = other.mValue;
        mDescription = other.mDescription;
        return *this;
    }

    const EnumPropertyDescription* getDescription() const {
        return mDescription;
    }

    int asInt() const {
        return mValue;
    }

    void setAsInt(int value) {
        if(mDescription && mDescription->nameMap.contains(value)) {
            mValue = value;
        }
    }
private:
    int mValue = 0;
    const EnumPropertyDescription* mDescription = nullptr;
};

Q_DECLARE_METATYPE(EnumProperty)

#endif // ENUMPROPERTY_H
