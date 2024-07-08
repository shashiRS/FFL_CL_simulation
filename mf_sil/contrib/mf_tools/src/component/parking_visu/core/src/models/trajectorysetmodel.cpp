#include "trajectorysetmodel.h"
#include <QDebug>
#include <cassert>

TrajectorySetModel::TrajectorySetModel(QObject *parent) : QObject(parent)
{

}

void TrajectorySetModel::setTrajectories(const QList<TrajectoryEntry> &trajectories)
{
    mTrajectorySet = trajectories;
    mSelectedTrajectoryIndex = mTrajectorySet.empty()? -1 : 0;
    mCurrentPoseIndex = mSelectedTrajectoryIndex < 0 || mTrajectorySet.at(mSelectedTrajectoryIndex).poses.empty() ? -1 : 0;

    emit currentTrajectoryUpdated();
    emit selectedTrajectoryIndexChanged(mSelectedTrajectoryIndex);
    emit currentPoseIndexChanged(mCurrentPoseIndex);
    emit currentPoseVelocityChanged(getCurrentPoseVelocity());
    emit trajectoriesChanged();
}

void TrajectorySetModel::addTrajectory(const TrajectoryEntry &trajectory)
{
    // replace trajectory if set
    for(int i = 0; i < mTrajectorySet.size(); i++) {
        auto& entry = mTrajectorySet[i];

        if(entry.name == trajectory.name) {
            entry.poses = trajectory.poses;

            if(mSelectedTrajectoryIndex == i) {
                mCurrentPoseIndex = entry.poses.empty() ? -1 : 0;
                emit currentTrajectoryUpdated();
                emit currentPoseIndexChanged(mCurrentPoseIndex);
                emit currentPoseVelocityChanged(getCurrentPoseVelocity());
            }

            emit trajectoriesChanged();
            return;
        }
    }

    // otherwise add a new trajectory
    mTrajectorySet.append(trajectory);

    if(mTrajectorySet.size() == 1) { // set was empty before
        mSelectedTrajectoryIndex = 0;
        mCurrentPoseIndex = mTrajectorySet.at(mSelectedTrajectoryIndex).poses.empty() ? -1 : 0;

        emit currentTrajectoryUpdated();
        emit selectedTrajectoryIndexChanged(mSelectedTrajectoryIndex);
        emit currentPoseIndexChanged(mCurrentPoseIndex);
        emit currentPoseVelocityChanged(getCurrentPoseVelocity());
    }

    emit trajectoriesChanged();
}

void TrajectorySetModel::removeTrajectory(int index)
{
    assert(index >= 0 && index < mTrajectorySet.size());

    mTrajectorySet.removeAt(index);

    if(mSelectedTrajectoryIndex > index) {
        mSelectedTrajectoryIndex--;
        emit selectedTrajectoryIndexChanged(mSelectedTrajectoryIndex);
    } else if(mSelectedTrajectoryIndex == index) {
        mSelectedTrajectoryIndex = mTrajectorySet.empty()? -1 : 0;
        mCurrentPoseIndex = mSelectedTrajectoryIndex < 0 || mTrajectorySet.at(mSelectedTrajectoryIndex).poses.empty() ? -1 : 0;

        emit currentTrajectoryUpdated();
        emit selectedTrajectoryIndexChanged(mSelectedTrajectoryIndex);
        emit currentPoseIndexChanged(mCurrentPoseIndex);
        emit currentPoseVelocityChanged(getCurrentPoseVelocity());
    }

    emit trajectoriesChanged();
}

void TrajectorySetModel::removeTrajectory(const QString &name)
{
    for(int i = 0; i < mTrajectorySet.size(); i++) {
        if(mTrajectorySet[i].name == name) {
            removeTrajectory(i);
            break;
        }
    }
}

float TrajectorySetModel::getCurrentPoseVelocity() const
{
    return mSelectedTrajectoryIndex < 0 || mCurrentPoseIndex < 0 ?
                -1.0 :
                mTrajectorySet.at(mSelectedTrajectoryIndex).poses.at(mCurrentPoseIndex).velocity_mps;
}

const Trajectory *TrajectorySetModel::getSelectedTrajectory() const
{
    return mSelectedTrajectoryIndex < 0 ? nullptr : &mTrajectorySet.at(mSelectedTrajectoryIndex).poses;
}

void TrajectorySetModel::setSelectedTrajectoryIndex(int index)
{
    if(mSelectedTrajectoryIndex != index) {
        mSelectedTrajectoryIndex = index < mTrajectorySet.size()? index : -1;
        mCurrentPoseIndex = mSelectedTrajectoryIndex < 0 || mTrajectorySet.at(mSelectedTrajectoryIndex).poses.empty() ? -1 : 0;

        emit selectedTrajectoryIndexChanged(mSelectedTrajectoryIndex);
        emit currentTrajectoryUpdated();
        emit currentPoseIndexChanged(mCurrentPoseIndex);
        emit currentPoseVelocityChanged(getCurrentPoseVelocity());
    }
}

void TrajectorySetModel::setCurrentPoseIndex(int index)
{
    if(mSelectedTrajectoryIndex >= 0 && mCurrentPoseIndex != index) {
        mCurrentPoseIndex = index < mTrajectorySet.at(mSelectedTrajectoryIndex).poses.size() ? index : -1;
        emit currentPoseIndexChanged(mCurrentPoseIndex);
        emit currentPoseVelocityChanged(getCurrentPoseVelocity());
    }
}

void TrajectorySetModel::setDisplayOffset(const QPointF &offset)
{
    if(mTrajectoryDisplayOffet != offset) {
        mTrajectoryDisplayOffet = offset;
        emit displayOffsetChanged(mTrajectoryDisplayOffet);
    }
}

