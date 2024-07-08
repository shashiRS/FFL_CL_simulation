#ifndef PARKINGSCENEMODEL_H
#define PARKINGSCENEMODEL_H

#include <QObject>

#include <mf_taposd/TAPOSD_Interface.h>
#include <MF_TRJPLA_TestDataHandler_wMetaData.h>
#include <si_types_JsonReader/SI_OUTPUT_TestDataHandler.h>
#include <ap_common/sys_func_params.h>

#include <si/ap_env_model_port.h>
#include <si/ap_parking_box_port.h>
#include <si/ego_motion_port.h>
#include <ap_tp/ap_tp_generated_types.h>

#include <QPolygonF>
#include <QPointF>
#include <QUrl>
#include <QTransform>
#include <QAbstractTableModel>
#include <QStringListModel>

#include <memory>
#include <array>
#include <cassert>

#include "debugvehiclemodel.h"
#include "targetposemodel.h"
#include "staticobstaclemodel.h"
#ifndef ULTRASONIC_ONLY
    #include "parkingspacemarkingmodel.h"
#endif
#include "parkingboxcollectionmodel.h"
#include "trajectorysetmodel.h"
#include "targetposereachableareamodel.h"
#include "taposddebugmodel.h"

namespace Json {
class Value;
}

namespace SI {
class SI_OUTPUT_JsonTestDataHandler;
}

namespace ap_tp {
	class MF_TRJPLA_TestDataHandler;
}

namespace ap_tpD {
	class MF_TAPOSD_TestDataHandler;
}

class ParkingSceneModel : public QObject
{
    Q_OBJECT

    Q_PROPERTY(int selectedTargetPose READ getSelectedTargetPose WRITE setSelectedTargetPose)
    Q_PROPERTY(float vehInflRadius_m READ getVehInflRadius_m WRITE setVehInflRadius_m NOTIFY vehInflRadiusChanged)
    Q_PROPERTY(int EMIndex READ getEMIndex WRITE selectEMbyIndex NOTIFY selectedEMIndexChanged)
    Q_PROPERTY(bool isReplanning READ isReplanning WRITE setIsReplanning)
    Q_PROPERTY(EnumProperty ReplanTrigger READ getReplanTrigger WRITE setReplanTrigger)

public:
    class ChangeGuard {
    public:
        ChangeGuard(ParkingSceneModel* model)
            : mModel(model) {
            mModel->mDoingBigChange++;
        }

        ChangeGuard(ChangeGuard&& rhs) {
            mModel = rhs.mModel;
            rhs.mModel = nullptr;
        }

        ChangeGuard& operator=(ChangeGuard&& rhs) {
            mModel = rhs.mModel;
            rhs.mModel = nullptr;
            return *this;
        }

        ChangeGuard(const ChangeGuard&) = delete;
        ChangeGuard& operator=(const ChangeGuard&) = delete;

        ~ChangeGuard() {
            if(mModel) {
                assert(mModel->mDoingBigChange > 0);
                mModel->mDoingBigChange--;
                mModel->onDataChanged();
                mModel = nullptr;
            }
        }
    private:
        ParkingSceneModel* mModel;
    };
    friend class ChangeGuard;

    explicit ParkingSceneModel(QObject *parent = nullptr);

    void loadEM(QString filename);

    /**
     * @brief saveEM
     * @param filename
     * @param forFakeEM if true, ego motion port and target pose will be omitted for FakeEM export
     */
    void saveEM(QString filename, bool forFakeEM = false);

    void saveReach(QString filename);

    void setEnvModelPort(const si::ApEnvModelPort  &em);
    const si::ApEnvModelPort *getEnvModelPort();

    void setEgoMotionPort(const si::EgoMotionPort& egoMotion);
    const si::EgoMotionPort *getEgoMotionPort();

    void setTargetPosesPort(const ap_tp::TargetPosesPort& target);
    const ap_tp::TargetPosesPort *getTargetPosesPort();

    void setParkingBoxPort(const si::ApParkingBoxPort& port);
    const si::ApParkingBoxPort* getParkingBoxPort();

    void ParkingSceneModel::setTrjplaMetaData(const ap_tp::TrjplaMetaData& metaData);
    const ap_tp::TrjplaMetaData *ParkingSceneModel::getTrjplaMetaData();

    DebugVehicleModel* getStartPoseModel() {
        return &mStartPoseModel;
    }

    TargetPoseModel* getTargetPoseModel(lsm_geoml::size_type i) {
        return mTargetPoseModels.at(i).get();
    }

    lsm_geoml::size_type numTargetPoseModels() const {
        return mTargetPoseModels.size();
    }

#ifndef ULTRASONIC_ONLY
    std::array<ParkingSpaceMarkingModel, ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_SPACE_MARKINGS_NU>& getParkingSpaceMarkingModels() {
        return mParkingSpaceMarkingModels;
    }
#endif

    std::array<StaticObstacleModel, ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU>& getStaticStructureModels() {
        return mStaticStructuresModels;
    }

    TargetPoseReachableAreaModel* getTargetPoseReachableAreaModel() {
        return &mTargetPoseReachableAreaModel;
    }

    void setTargetPoseReachableAreaReady2D(bool areaReady);
    void setTargetPoseReachableAreaReady3D();

    /*StaticObstacleCollectionModel* getObstaclesModel() {
        return &mStaticObjectsModel;
    } */

    ParkingBoxCollectionModel* getParkingBoxesModel() {
        return &mParkingBoxCollection;
    }
    void setSelectedObject(QObject *object);

    QObject* getSelectedObject() {
        return mSelectedObject;
    }

	void setSelectedTargetPoseIndex(int index);

	int* getSelectedTargetPoseIndex() {
		return &mSelectedPoseIndex;
	};

    inline void setTrajplaParams(ap_tp::FC_TRJPLA_Params const& params) {
        mTrajplaParams = std::make_shared<ap_tp::FC_TRJPLA_Params>(params);
    }

    inline const ap_tp::FC_TRJPLA_Params* getTrajplaParams() const {
        return mTrajplaParams.get();
    }

    void setVehicleParameters(ap_common::Vehicle_Params const& params);

    const ap_common::Vehicle_Params& getVehicleParams() const {
        return mVehicleParams;
    }

    int getSelectedTargetPose() const;
    void setSelectedTargetPose(int index);

    /**
     * @return selected target pose model or nullptr
     */
    TargetPoseModel* getSelectedTargetPoseModel();

    DebugVehicleModel* getDebugVehicleModel() {
        return &mDebugVehicle;
    }

    TrajectorySetModel* getTrajectorySetModel() {
        return &mTrajectoriesModel;
    }

    TaposdDebugModel* getTaposdDebugModel() {
        return &mTaposdDebugModel;
    }

    /**
     * @brief Captures current EM as a new point in redo/undo history
     */
    void createHistoryPoint();

    /**
     * @brief Go one step back in history if applicable
     */
    void undo();

    /**
     * @brief Go one step forward in history if applicable
     */
    void redo();

    /**
     * @return true if "simulation" is running, i.e. we have a valid path and replanning is possible; false if we don't have a valid path.
     */
    bool isFollowingPath() const;

    float getVehInflRadius_m() {
        return mVehInflRadiusOverride;
    }

    void setVehInflRadius_m(float radius);

    void notifyVehPoseChanged();
    void resetTargetPoseReachableArea(void);

    const QString& getLoadedEMFilename() const {
        return mLoadedEMFilename;
    }

    int getEMIndex() const {
        return mSelectedEMIndex;
    }

    void selectEMbyIndex(int index);


    EnumProperty getReplanTrigger() const {
        return mReplanTrigger;
    }

    void setReplanTrigger(const EnumProperty& prop);
       
    QAbstractListModel* getLoadedScenesModel();
    bool isReplanning(void);
    void setIsReplanning(bool isReplaning);

    /**
     * When doing a big change on the scene it is expected that the dataChanged signal will
     * be emitted multiple times upon every single change. Use this function to block
     * the signal emission. The signal will be emitted as soon as the guard is being destroyed (RAII).
     *
     * Example:
     *
     * {
     *    auto changeGuard = model.startBigChange(); // now data change signal is blocked
     *
     *    model.setEnvModelPort(...);
     *    model.setParkingBoxPort(...);
     *    model.getStaticStructureModels().at(0).setShape(...);
     *
     * } // "changeGuard" is destroyed, the dataChanged signal is finally emitted.
     *
     * Note that its ok to create multiple guard instances, the signal will be blocked
     * until the last guard object is destroyed.
     *
     * @return Change Guard object
     */
    ChangeGuard startBigChange();

    /**
     * @return true if a big change is currently in progress, otherwise false
     */
    bool isDoingBigChange() const {
        return mDoingBigChange > 0;
    }

public slots:
    void addNewSceneEntry();
    void removeSelectedSceneEntry();
    void onDataChanged();

signals:
    void vehicleParamsChanged(const ap_common::Vehicle_Params& params);
    void selectedTargetPoseChanged(int previousIndex, int currentIndex); // index is out of range if no pose is selected!

    void selectedObjectChanged(QObject* selected, QObject* previous);

    void loadedEMPathChanged(QString filename);

    void vehInflRadiusChanged(float radius);

    void targetPoseReachabilityAreaReady3D();

    void selectedEMIndexChanged(int index);

    void dataChanged();

private:
    // enum to string methods for comments in json files
    const char* ObjHeight_to_string(const si::StaticObjHeigthType value)
    {
        static const char* LUT[] = { "SO_HI_UNKNOWN", "SO_HI_WHEEL_TRAVERSABLE", "SO_HI_BODY_TRAVERSABLE", "SO_HI_DOOR_OPENABLE", "SO_HI_HIGH_OBSTACLE", "SO_HI_HANGING_OBJECT", "SO_HI_LOWER_BUMPER_HEIGHT", "MAX_NUM_HEIGHT_TYPES" };
        return LUT[static_cast<int>(value)];
    }
#ifndef ULTRASONIC_ONLY
    const char* DelimType_to_string(const si::DelimiterTypes value)
    {
        static const char* LUT[] = { "STATIC_STRUCTURE", "PARKING_SPACE_MARKING", "LANE_BOUNDARY", "MAX_NUM_PARKING_BOX_DELIMITER_TYPES" };
        return LUT[static_cast<int>(value)];
    }
    const char* PLT_to_string(const si::ParkingLineType value)
    {
        static const char* LUT[] = { "PLT_UNKNOWN", "PLT_WHITE", "PLT_BLUE", "PLT_RED", "PLT_YELLOW", "PLT_GREEN", "PLT_MAX_NUM_COLORS" };
        return LUT[static_cast<int>(value)];
    }
    const char* LBEst_to_string(const si::LaneBoundaryEstimationState value)
    {
        static const char* LUT[] = { "LB_INVALID", "LB_DETECTED", "LB_ESTIMATED_USING_STATIC_OBJECTS", "LB_VIRTUAL", "LB_MAX_NUM_ESTIMATION_STATES" };
        return LUT[static_cast<int>(value)];
    }
    const char* Lbt_to_string(const si::LaneBoundaryType value)
    {
        static const char* LUT[] = { "LB_UNCLASSIFIED", "LB_DASHED", "LB_DOTTED", "LB_CONTINUOUS", "LB_DOUBLE_CONTINUOUS", "LB_MAX_NUM_TYPES" };
        return LUT[static_cast<int>(value)];
    }
    const char* LPt_to_string(const si::LanePosType value)
    {
        static const char* LUT[] = { "EGO_LANE", "LEFT_OF_EGO_LANE", "RIGHT_OF_EGO_LANE", "MAX_NUM_LANE_POS_TYPES" };
        return LUT[static_cast<int>(value)];
    }
#endif
    const char* DelimSide_to_string(const si::RelativeLocationToParkingBox value)
    {
        static const char* LUT[] = { "UNDEFINED_EDGE", "ROAD_SIDE_EDGE", "RIGHT_EDGE", "CURB_SIDE_EDGE", "LEFT_EDGE", "INSIDE_ROAD_SIDE_EDGE", "INSIDE_RIGHT_EDGE", "INSIDE_CURB_SIDE_EDGE", "INSIDE_LEFT_EDGE", "RELATED_OTHERWISE", "MAX_NUM_PARKING_BOX_EDGE_TYPES" };
        return LUT[static_cast<int>(value)];
    }
    const char* TPType_to_string(const ap_tp::PoseType value)
    {
        static const char* LUT[] = { "T_PARALLEL_PARKING", "T_PERP_PARKING_FWD", "T_PERP_PARKING_BWD", "T_ANGLED_PARKING_STANDARD", "T_ANGLED_PARKING_REVERSE", "T_REM_MAN_FWD", "T_REM_MAN_BWD", "T_PERP_PARKING_OUT_FWD", "T_PERP_PARKING_OUT_BWD", "T_PAR_PARKING_OUT", "T_ANGLED_PARKING_STANDARD_OUT", "T_ANGLED_PARKING_REVERSE_OUT", "T_UNDO", "T_GP_FWD", "T_GP_BWD", "T_GP_OUT_FWD", "T_GP_OUT_BWD", "T_GP_FWD_AXIS", "T_GP_BWD_AXIS", "T_GP_OUT_FWD_AXIS", "T_GP_OUT_BWD_AXIS", "T_UNDEFINED" };
        return LUT[static_cast<int>(value)];
    }
    const char* TPPoseFailReason_to_string(const ap_tp::PoseFailReason value)
    {
        static const char* LUT[] = { "TAPOSD_PFR_NONE", "TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW", "TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT", "TAPOSD_PFR_MAXBOX_EXCEEDED", "TAPOSD_PFR_WHEEL_COLLISION", "TAPOSD_PFR_HIGH_OBJECT_COLLISION", "TAPOSD_PFR_UNKNOWN", "MAX_NUM_POSE_FAIL_TYPES" };
        return LUT[static_cast<int>(value)];
    }
    const char* TPSide_to_string(const ap_tp::TargetSide value)
    {
        static const char* LUT[] = { "TS_RIGHT_SIDE", "TS_LEFT_SIDE", "TS_IN_FRONT_RIGHT", "TS_IN_FRONT_CENTER", "TS_IN_FRONT_LEFT", "TS_IN_REAR_RIGHT", "TS_IN_REAR_CENTER", "TS_IN_REAR_LEFT", "TS_UNDEFINED_SIDE" };
        return LUT[static_cast<int>(value)];
    }
    const char* ReachStatus_to_string(const ap_tp::PoseReachableStatus value)
    {
        if (static_cast<uint8_t>(value) < 7) {
            static const char* LUT[] = { "TP_NOT_VALID", "TP_NOT_REACHABLE", "TP_FULLY_REACHABLE", "TP_SAFE_ZONE_REACHABLE", "TP_MANUAL_FWD_REACHABLE", "TP_MANUAL_BWD_REACHABLE", "MAX_NUM_POSE_REACHABLE_STATUS" };
            return LUT[static_cast<int>(value)];
        } else {
            return "TP_NOT_VALID";
        }
    }
    const char* FailR_to_string(const ap_tp::PlanningFailReason value)
    {
        if (static_cast<uint8_t>(value) < 6) {
            static const char* LUT[] = { "PFR_NONE", "PFR_TARGET_POSE_LOST", "PFR_PARKING_BOX_LOST", "PFR_INPUT_CORRUPTED", "PFR_REPLAN_FAIL", "MAX_NUM_PLANNING_FAIL_TYPES" };
            return LUT[static_cast<int>(value)];
        } else {
            return "PFR_NONE";
        }
    }
    const char* ReachedStatus_to_string(const ap_tp::PoseReachedStatus value)
    {
        static const char* LUT[] = { "NO_TP_REACHED_STATUS", "TP_REACHED", "TP_NOT_REACHED", "MAX_NUM_POSE_REACHED_STATUS_TYPES" };
        return LUT[static_cast<int>(value)];
    }
    const char* ParkingScen_to_string(const si::ParkingScenarioTypes value)
    {
        static const char* LUT[] = { "PARALLEL_PARKING", "PERPENDICULAR_PARKING", "ANGLED_PARKING_OPENING_TOWARDS_BACK", "ANGLED_PARKING_OPENING_TOWARDS_FRONT", "GARAGE_PARKING", "DIRECT_PARKING", "EXTERNAL_TAPOS_PARALLEL", "EXTERNAL_TAPOS_PERPENDICULAR", "EXTERNAL_TAPOS_PARALLEL_OUT", "EXTERNAL_TAPOS_PERPENDICULAR_OUT", "MAX_NUM_PARKING_SCENARIO_TYPES" };
        return LUT[static_cast<int>(value)];
    }
       
    const char* DrivingDir_to_string(const ap_common::DrivingDirection value)
    {
        static const char* LUT[] = { "DIRECTION_UNKNOWN", "STANDSTILL", "DRIVING_FORWARDS", "DRIVING_BACKWARDS" };
        return LUT[static_cast<int>(value)];
    }
    const char* SteeringDir_to_string(const ap_common::SteeringDirection value)
    {
        static const char* LUT[] = { "CIRCLE_LEFT", "CLOTHOID_LEFT_BACKEND", "CLOTHOID_LEFT_FRONTEND", "STRAIGHT", "CIRCLE_RIGHT", "CLOTHOID_RIGHT_BACKEND", "CLOTHOID_RIGHT_FRONTEND" };
        return LUT[static_cast<int>(value)];
    }

    bool mIsReplanning=false;
    struct SceneData {
        si::ApEnvModelPort emModel;
        ap_tp::TargetPosesPort targetPoseIn;
        si::EgoMotionPort egoMotion;
        si::ApParkingBoxPort parkingBoxPort;
        ap_tp::TrjplaMetaData metaData;

        SceneData() {
            reset();
        }

        SceneData(const si::ApEnvModelPort& emModel,
                  const ap_tp::TargetPosesPort& targetPoseIn,
                  const si::EgoMotionPort& egoMotion,
                  const si::ApParkingBoxPort& parkingBoxPort,
                  const ap_tp::TrjplaMetaData& trjplaMetaData)
            : emModel(emModel)
            , targetPoseIn(targetPoseIn)
            , egoMotion(egoMotion)
            , parkingBoxPort(parkingBoxPort)
            , metaData(trjplaMetaData)
        {
        }

        void reset() {
            memset(&emModel, 0, sizeof(emModel));
            memset(&targetPoseIn, 0, sizeof(targetPoseIn));
            memset(&egoMotion, 0, sizeof(egoMotion));
            memset(&parkingBoxPort, 0, sizeof(parkingBoxPort));
            memset(&metaData, 0, sizeof(metaData));
        }
    };

    struct SceneEntry {
        SceneEntry(const SceneData& sceneData)
            : sceneData(sceneData)
            , historyIndex(0)
        {
            history.push_back(sceneData);
        }

        void createHistoryPoint(const SceneData& data) {
            // erase future history since since we are writing new history
            if(historyIndex < history.size()-1) {
                auto iter = history.begin() + historyIndex + 1;
                history.erase(iter, history.end());
            }

            history.push_back(data);
            historyIndex = history.size()-1;
            //qDebug() << "History point, size = " << mHistory.size();
        }

        SceneData sceneData;
        QList<SceneData> history; /**< History of model data, just stores complete data */
        int historyIndex = 0; /**< index of currently active entry in mEMHistory */
    };

    /// Indicates whether a big data update is in progress (e.g. multiple models are set from outside
    /// So the dataChanged signal should not be emitted on every change
    int mDoingBigChange = 0;

    QList<SceneEntry> mSceneCollection;
    SceneEntry* mCurrentSceneEntry = nullptr;
    QStringListModel mLoadedScenesListModel;
    // TODO: make mSceneCollection and mLoadedScenesListModel a single model

    std::shared_ptr<ap_tp::FC_TRJPLA_Params> mTrajplaParams;

    DebugVehicleModel mStartPoseModel;

    // needs to have pointers since not default-constructible and non-copyable
    std::array<std::unique_ptr<TargetPoseModel>, ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_TARGET_POSES_NU> mTargetPoseModels;

#ifndef ULTRASONIC_ONLY
    std::array < ParkingSpaceMarkingModel, ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_SPACE_MARKINGS_NU> mParkingSpaceMarkingModels;

    const char* ObjClass_to_string(const si::StaticObjectClass value)
    {
        static const char* LUT[] = { "STAT_OBJ_UNCLASSIFIED_STRUCTURE", "STAT_OBJ_VEHICLE", "STAT_OBJ_WHEEL_STOPPER", "STAT_OBJ_POLE", "STAT_OBJ_CURB", "STAT_OBJ_MAX_NUM_TYPES" };
        return LUT[static_cast<int>(value)];
    }

#endif


    // StaticObstacleCollectionModel mStaticObjectsModel;
    std::array<StaticObstacleModel, ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU> mStaticStructuresModels;

    ParkingBoxCollectionModel mParkingBoxCollection;

    /** used to display debug vehicle poses produced by planner during planning */
    DebugVehicleModel mDebugVehicle;

    TrajectorySetModel mTrajectoriesModel;

    ap_common::Vehicle_Params mVehicleParams;

    TargetPoseReachableAreaModel mTargetPoseReachableAreaModel;

    TaposdDebugModel mTaposdDebugModel;

    QObject* mSelectedObject = 0;

    float mVehInflRadiusOverride = -1.0F;   /// Override of vehicle inflation radius in m. If negative, radius from TRJPLA params is taken.

    QString mLoadedEMFilename; // empty if no EM loaded from file

	int mSelectedPoseIndex = 0;
    int mSelectedEMIndex = 0;
    ap_tp::ReplanTrigger mReplanTrigger = ap_tp::ReplanTrigger::NO_TRIGGER_SET;

    /**
     * @brief Load model data from mHistory.at(mHistoryIndex)
     */
    void applyHistoryData();

	/**
	 * @brief
	 */
    void addSceneEntryFromJson(::Json::Value *rootVal, si::SI_OUTPUT_JsonTestDataHandler &jsonTestDataHandler, ap_tp::MF_TRJPLA_TestDataHandler_wMetaData &trjplaTestDataHandler);
    /**
     * Reload scene data from mCurrentSceneData
     */
    void reloadSceneData();
};

#endif // PARKINGSCENEMODEL_H
