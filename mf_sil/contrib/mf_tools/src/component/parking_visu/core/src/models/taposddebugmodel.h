#ifndef TAPOSDDEBUGMODEL_H
#define TAPOSDDEBUGMODEL_H

#include <QObject>

#include <ap_tp/taposddebug_port.h>
#include <ap_common/sys_func_params.h>

class TaposdDebugModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool visible READ isVisible WRITE setVisible NOTIFY visibleChanged)

public:
    explicit TaposdDebugModel(QObject *parent = nullptr);

    void setData(const ap_tp::TAPOSDDebugPort& data);
    const ap_tp::TAPOSDDebugPort& getData() {
        return mTpdDebugData;
    }

    bool isVisible() const {
        return mVisible;
    }

    void setVisible(bool visible);

signals:
    void taposdDataChanged();
    void visibleChanged(bool visible);

public slots:

private:
    ap_tp::TAPOSDDebugPort mTpdDebugData;

    bool mVisible = true;
};

#endif // TAPOSDDEBUGMODEL_H
