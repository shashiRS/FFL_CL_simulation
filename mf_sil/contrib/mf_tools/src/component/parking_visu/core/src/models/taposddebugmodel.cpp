#include "taposddebugmodel.h"

TaposdDebugModel::TaposdDebugModel(QObject *parent) : QObject(parent)
{
    memset(&mTpdDebugData, 0, sizeof(mTpdDebugData));
}

void TaposdDebugModel::setData(const ap_tp::TAPOSDDebugPort &data)
{
    mTpdDebugData = data;
    emit taposdDataChanged();
}

void TaposdDebugModel::setVisible(bool visible)
{
    if(mVisible != visible) {
        mVisible = visible;
        emit visibleChanged(mVisible);
    }
}
