#ifndef TRAJECTORYSETMODEL_H
#define TRAJECTORYSETMODEL_H

#include <QObject>
#include <QList>
#include <QTransform>

struct TrajectoryPose {
    TrajectoryPose() = default;

    TrajectoryPose(const QTransform& trans, float velocity = -1.0F):
        trans(trans), velocity_mps(velocity) { }

    QTransform trans;
    float velocity_mps = -1.0F;
    float curvature_1pm = 0.0F;
};

typedef QList<TrajectoryPose> Trajectory;

struct TrajectoryEntry {
    QString name;
    Trajectory poses;
};

class TrajectorySetModel : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int selectedTrajectoryIndex READ getSelectedTrajectoryIndex WRITE setSelectedTrajectoryIndex NOTIFY selectedTrajectoryIndexChanged)
    Q_PROPERTY(int currentPoseIndex READ getCurrentPoseIndex WRITE setCurrentPoseIndex NOTIFY currentPoseIndexChanged)
    Q_PROPERTY(float PoseVelocity READ getCurrentPoseVelocity NOTIFY currentPoseVelocityChanged)
    Q_PROPERTY(QPointF displayOffset READ getDisplayOffset WRITE setDisplayOffset NOTIFY displayOffsetChanged)

public:
    explicit TrajectorySetModel(QObject *parent = nullptr);

    void setTrajectories(const QList<TrajectoryEntry>& trajectories = QList<TrajectoryEntry>());

    void setMainTrajectory(const Trajectory& trajectory) {
        addTrajectory({ TrajectoryEntry {"main traj", trajectory} });
    }

    void addTrajectory(const TrajectoryEntry& trajectory);

    void addTrajectory(const QString& name, const Trajectory& trajectory) {
        addTrajectory(TrajectoryEntry{name, trajectory});
    }

    void removeTrajectory(int index);

    void removeTrajectory(const QString& name);

    int getSelectedTrajectoryIndex() const {
        return mSelectedTrajectoryIndex;
    }

    int getCurrentPoseIndex() const {
        return mCurrentPoseIndex;
    }

    float getCurrentPoseVelocity() const;

    const QList<TrajectoryEntry>& getTrajectorySet() const {
        return mTrajectorySet;
    }

    /**
     * @return Selected trajectory or nullptr
     */
    const Trajectory* getSelectedTrajectory() const;

    QPointF getDisplayOffset() const {
        return mTrajectoryDisplayOffet;
    }


signals:
    void trajectoriesChanged();

    void selectedTrajectoryIndexChanged(int index);
    void currentPoseIndexChanged(int index);
    void currentTrajectoryUpdated();

    void currentPoseVelocityChanged(float);

    void displayOffsetChanged(const QPointF offset);

public slots:
    void setSelectedTrajectoryIndex(int index);
    void setCurrentPoseIndex(int index);
    void setDisplayOffset(const QPointF& offset);

private:
    QList<TrajectoryEntry> mTrajectorySet;

    int mSelectedTrajectoryIndex = -1; /// index in mTrajectorySet, invalid if negative
    int mCurrentPoseIndex = -1; /// index in mTrajectorySet[mSelectedTrajectoryIndex], invalid if negative

    QPointF mTrajectoryDisplayOffet; /// local coordinate offset for displaying the trajectory points
};

#endif // TRAJECTORYSETMODEL_H
