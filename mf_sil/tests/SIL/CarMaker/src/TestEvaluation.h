#ifndef TEST_EVALUATION_HEADER_
#define TEST_EVALUATION_HEADER_

#include "ap_tp/ap_tp_generated_types.h"
#include "ap_trjctl/ap_trjctl_generated_types.h"
//#include "mf_common_types/Sys_Func_Params.h"
#include "si/parking_scenario_types.h"
#include "lsm_vedodo/odo_estimation_output_port.h"
#include "ap_psm/slot_ctrl_port.h"
#include "carMakerAPWrapper.h"
#include "TestEvaluationStruct.h"
#include "TestRunWrapper.h"
#include <list>
#include <vector>
#include "MfSilTypes.h"

constexpr unsigned int SCANNING_PATH_MAX_SIZE_NU = 20U;
static constexpr auto SEGMENTS_PARALLEL_MAX_ANGLE_THRESHOLD_RAD = 0.261799388f; // corresponds to 15 degrees

static constexpr auto AP_G_MAN_AREA_PAR_IN_D1_M   = 6.0f;
static constexpr auto AP_G_MAN_AREA_PAR_IN_D2_M   = 0.3f;
static constexpr auto AP_G_MAN_AREA_PAR_IN_D3_M   = 3.0f;
static constexpr auto AP_G_MAN_AREA_PAR_IN_D4_M   = 3.0f;
static constexpr auto AP_G_MAN_AREA_PAR_IN_D5_M   = 0.6f;

static constexpr auto AP_G_MAN_AREA_IN_PAR_D7_M   = 0.8f;
static constexpr auto AP_G_MAN_AREA_IN_PAR_D8_M   = 0.8f;

static constexpr auto AP_G_MAN_AREA_PER_B_IN_D1_M = 8.0f;
static constexpr auto AP_G_MAN_AREA_PER_B_IN_D3_M = 3.0f;
static constexpr auto AP_G_MAN_AREA_PER_B_IN_D4_M = 5.0f;
static constexpr auto AP_G_MAN_AREA_PER_B_IN_D5_M = 0.6f;

static constexpr auto AP_G_MAN_AREA_PER_F_IN_D1_M = 8.0f;
static constexpr auto AP_G_MAN_AREA_PER_F_IN_D3_M = 5.0f;
static constexpr auto AP_G_MAN_AREA_PER_F_IN_D4_M = 3.0f;
static constexpr auto AP_G_MAN_AREA_PER_F_IN_D5_M = 0.6f;

static constexpr auto AP_G_MAN_AREA_IN_PER_B_D7_PERC = 70.0f;
static constexpr auto AP_G_MAN_AREA_IN_PER_B_D8_PERC = 40.0f;

static constexpr auto AP_G_MAN_AREA_IN_PER_F_D7_PERC = 70.0f;
static constexpr auto AP_G_MAN_AREA_IN_PER_F_D8_PERC = 40.0f;

static constexpr auto AP_LO_MAX_TIME_UNSECURE_S = 0.2f;           //@impl{ L3_AP_241 }
static constexpr auto AP_LO_MAX_TIME_GEAR_SECURE_S = 0.1f;        //@impl{ L3_AP_244 }
static constexpr auto AP_LO_MAX_TIME_SECURE_S = 0.2f;             //@impl{ L3_AP_244 }
static constexpr auto AP_LO_MAX_TIME_GEAR_CHANGE_S = 0.1f;        //@impl{ L3_AP_242 }
static constexpr auto AP_LO_LONG_ACCELERATION_MAX_MPS2 = 0.5f;    //@impl{ L3_AP_243 }
static constexpr auto MAX_NO_STROKES = 11U;
static constexpr auto min_velocity = .1f;
static constexpr float32_t lookUpVel_mps[12]{ min_velocity / 3.6f, 2.0f / 3.6f, 3.6f / 3.6f, 5.0f / 3.6f, 6.2f / 3.6f,
    7.4f / 3.6f, 8.1f / 3.6f, 9.0f / 3.6f, 9.4f / 3.6f, 9.8f / 3.6f, 10.0f / 3.6f, 10.0f / 3.6f };
static constexpr float32_t lookUpDist[12]{ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 15.0 };

static constexpr auto TEST_EVAL_PARKINGBOX_TOLERANCE_CURB_ROAD = 0.03f;
static constexpr auto TEST_EVAL_PARKINGBOX_TOLERANCE_LEFT_RIGHT = 0.015f;

//ParkingSlot Index Convention
constexpr static LSM_GEOML::size_type ROAD_LEFT_CORNER_IDX{ 0U };
constexpr static LSM_GEOML::size_type ROAD_RIGHT_CORNER_IDX{ 1U };
constexpr static LSM_GEOML::size_type CURB_RIGHT_CORNER_IDX{ 2U };
constexpr static LSM_GEOML::size_type CURB_LEFT_CORNER_IDX{ 3U };

constexpr auto NO_STATIC_STRUCTURES = 8U;  //used from AUP_EVAL_STET_KPI_L1_L3.py where is calculated if the static object collides TP
constexpr auto EXISTENCE_PROB_PERC  = 80U; // used from AUP_EVAL_STET_KPI_L1_L3.py to check if the static object is detected

struct DifferenceBetween2Poses {
    float32_t longDistToTarget_m{};
    float32_t latDistToTarget_m{};
    float32_t yawDiffToTarget_rad{};
};


typedef struct TestEvaluationPort {
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> vehicleArea1Polygon;
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> vehicleArea2Polygon;
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU> maneuveringAreaPolygon;
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU> boundingBoxPolygon;
    LSM_GEOML::Polygon2D<SCANNING_PATH_MAX_SIZE_NU> egoVehScanningPathPolygon;
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> plannedEgoVehOutsideBoundariesPolygon;
    float32_t d5Distance_m[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU];
    bool plannedEgoVehOutsideBoundaries_bool;
    ap_tp::PlannedTraj plannedTrajDrivenPoint;
    float shortestDistanceToHighObject_m;
    int closestHighObjectId_nu;
    std::array<float32_t, MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM> shortestDistanceOdo;
    std::array<float32_t, MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM> shortestDistanceCM;
    int trajVertexIndexShortestDistHighObj_nu;
    float shortestDistanceToWheelTravObject_m;
    int closestWheelTravObjectId_nu;
    int trajVertexIndexShortestDistWheelTravObj_nu;
    float shortestDistanceToBodyTravObject_m;
    int closestBodyTravObjectId_nu;
    int trajVertexIndexShortestDistBodyTravObj_nu;
    const unsigned int scanningPathMaxSize_nu{ SCANNING_PATH_MAX_SIZE_NU };
    uint8_t vehicleArea1Polygon_size{ 0u };
    uint8_t vehicleArea2Polygon_size{ 0u };
    bool egoVehScanningPathPolygonRemoved_bool{ false };
    bool vehicleArea1OverlapsPb{ false };
    unsigned int numberOfStrokes;
    unsigned int numberOfStrokes_gear;
    unsigned int numberOfStrokes_newSegmentStarted;
    float32_t drivenDistanceOnStroke_m{ 0.0F };
    bool allowedManeuveringSpaceExceed_bool;
    float odoEstimationCMDiference;
    float32_t timeToPark;
    float32_t latDiffOptimalTP_FinalEgoPose_m{ -1.0f };
    float32_t longDiffOptimalTP_FinalEgoPose_m{ -1.0f };
    float32_t yawDiffOptimalTP_FinalEgoPose_deg{ -1.0f };
    float32_t latDiffOptimalTP_TargetPose_m{ -1.0f };
    float32_t longDiffOptimalTP_TargetPose_m{ -1.0f };
    float32_t yawDiffOptimalTP_TargetPose_deg{ -1.0f };
    float32_t latDiffErrorTreshold_m{ .0f };
    float32_t longDiffErrorTreshold_m{ .0f };
    float32_t yawDiffErrorTreshold_deg{ .0f };
    DifferenceBetween2Poses egoPoseTargetPoseDeviation{};
    LSM_GEOML::Pose finalTargetPose{};
    int car_outside_PB{ -1 }; //check if vehicle is inside of PB; -1 - NOT CALCULATED; 0 - Vehicle Inside PB; 1 - Vehicle Outside PB)
    int staticStructColidesTarget_Pose[NO_STATIC_STRUCTURES]{ -1,-1,-1,-1,-1,-1,-1,-1 };//check if staticStructure colides final vehicle Shape; -1 - NOT CALCULATED; 0 - NO COLLISION; 1 - COLLISION)
    unsigned int numberOfCrvSteps {0u};
    bool tooManyStrokes {false};
    float32_t latMaxDeviation_m{ .0f };     // Maximal deviation between ego vehicle position and target position in lateral direction
    float32_t longMaxDeviation_m{ .0f };    // Maximal deviation between ego vehicle position and target position in longitudinal direction
    float32_t yawMaxDeviation_rad{ .0f };   // Maximal deviation between ego vehicle yaw angle and target position yaw angle
    float32_t distanceToStopReqInterExtrapolTraj_m{ .0f };
}TestEvaluationPort_t;

class TestEvaluation {
    private:
      //Parameters for Optimal Target Pose evaluation from https://confluence-adas.zone2.agileci.conti.de/display/SYSAUP/KPIs 
        //Parallel Premium:
        static constexpr auto MAX_LAT_DIST_ERR_PARALLEL_PREM_M = 0.06f;
        static constexpr auto MAX_LONG_DIST_ERR_PARALLEL_PREM_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_PARALLEL_PREM_GRAD = 2.0f;
        //Perpendicular Premium:
        //TODO: Use specific KPI value for lines accurarcy (DFMFPT-2355) (MAX_LAT_DIST_ERR_PERP_PREM_LINES_M = 0.06f)
        static constexpr auto MAX_LAT_DIST_ERR_PERP_PREM_M = 0.09f;
        static constexpr auto MAX_LONG_DIST_ERR_PERP_PREM_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_PERP_PREM_GRAD = 2.0f;
        //Angled Premium:
        //TODO: This is a copy of perpenicular values (lat. KPIs for lines used)! (see DFMFPT-2355); Replace by Angled KPIs when available
        //TODO: Use specific KPI values for parking between objects accurarcy (see comment in Perpendicular Premium: DFMFPT-2355)
        static constexpr auto MAX_LAT_DIST_ERR_ANG_PREM_M = 0.06f;
        static constexpr auto MAX_LONG_DIST_ERR_ANG_PREM_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_PERP_ANG_GRAD = 2.0f;
        //Parallel CUS:
        static constexpr auto MAX_LAT_DIST_ERR_PARALLEL_CUS_M = 0.10f;
        static constexpr auto MAX_LONG_DIST_ERR_PARALLEL_CUS_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_PARALLEL_CUS_GRAD = 2.0f;
        //Perpendicular CUS:
        static constexpr auto MAX_LAT_DIST_ERR_PERP_CUS_M = 0.10f;
        static constexpr auto MAX_LONG_DIST_ERR_PERP_CUS_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_PERP_CUS_GRAD = 2.0f;
        //Angled CUS:
        //TODO: This is a copy of perpenicular values! (see DFMFPT-2355); Replace by Angled KPIs when available
        static constexpr auto MAX_LAT_DIST_ERR_ANG_CUS_M = 0.10f;
        static constexpr auto MAX_LONG_DIST_ERR_ANG_CUS_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_ANG_CUS_GRAD = 2.0f;
		//Parallel Performance:
		//TODO: Copied from Premium
        static constexpr auto MAX_LAT_DIST_ERR_PARALLEL_PERF_M = 0.06f;
        static constexpr auto MAX_LONG_DIST_ERR_PARALLEL_PERF_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_PARALLEL_PERF_GRAD = 2.0f;
        //Perpendicular Performance:
        //TODO: Copied from Premium
        static constexpr auto MAX_LAT_DIST_ERR_PERP_PERF_M = 0.09f;
        static constexpr auto MAX_LONG_DIST_ERR_PERP_PERF_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_PERP_PERF_GRAD = 2.0f;
        //Angled Performance:
        //TODO: Copied from Premium
        static constexpr auto MAX_LAT_DIST_ERR_ANG_PERF_M = 0.06f;
        static constexpr auto MAX_LONG_DIST_ERR_ANG_PERF_M = 0.20f;
        static constexpr auto MAX_YAW_ERR_ANG_PERF_GRAD = 2.0f;

        bool testEvaluationActive{ false };
        bool egoVehScanningPathDefined_bool;
        uint8_t currentTrajectoryIndex_nu;
        TestEvaluationPort testEvaluationPort;
        LSM_GEOML::Pose lastTargetPoseUsed;
        std::vector<LSM_GEOML::Pose> egoVehScanningPath;
        float xPosition_m_delay;
        float yPosition_m_delay;
        float yawAngle_rad_delay;
        unsigned char headUnitScreen_nu_delay;
        float odoCmRefEgoRaCur_m_calc;
        float odoCmRefxPosition_m_delay;
        float odoCmRefyPosition_m_delay;
        float odoCmRefyawAngEgoRaCur_rad_delay;
        float vehVelocityCur_mps_delay;
        int gearNo_delay;
        bool newSegmentStarted_Last{ false };
        cml::Vec2Df cornerEgoVehicle[4], initialCornerEgoVehicle[4];
        cml::Vec2Df CrossingParkingBoxCorner[9], CrossingEgoVehicleCorner[5];
        float xFrontPoint, xFrontPointParkingBox, xRearPointParkingBox, xRearPoint, yRoadPoint, ySidePoint, yEndParkingBoxPoint;
        cml::Vec2Df cornerPointEgoVehicle[4];
        cml::Vec2Df cornerPointParkingBox[4];
        float am_lo_rear, am_la_side, am_lo_front, am_la_road;
        float am_p_rear, am_p_end, am_p_front;
        // tire circumference estimation (tce) validation
        float cmCirc_m[4];
        float cmVel_mps;
        float cmLastRot_rad[4];
        float cmLastDist_m;
        float cmDistFiltered;
        float cmFirstDist_m;
        uint8_t selectedParkingBoxIdx{ 0U };

        bool targetPoseReachedStatusSet{ false };

        //TimeToPark
        float32_t curvature_last{ .0f };
        bool drivingForwardReq_nu_delay{ false };
        uint8_t noOfNewSegment { 0U };
        bool newSegmentStarted_nu_delay{ 0U };

        float32_t vel_accel_delay {.0f};
        float32_t distanceToStopReq_m_delay {.0f};
        float32_t distanceOnStroke{ .0f };                       // Variable to calculate the distance on stroke based on DistanceToStop variable
        std::vector<float32_t> inverseVelocityAtTrajPoints;
        std::vector<float32_t> distanceOnStrokeList;

        void shrinkManeuveringArea(const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> &egoVehicleShape,
            const bool parkingOnLeftSide_nu);

        LSM_GEOML::Polygon2D<4U> expandParkingBox(const LSM_GEOML::Polygon2D<4U> &slotCoordinates_m);

        void TestEvaluation::defineScanningPathPolygon(const ap_common::Vehicle_Params egoVehicleParams);

        void defineManeuveringArea(const ap_tp::PoseType &parkingScenarioType,
            const bool parkingOnLeftSide_nu);

        void TestEvaluation::defineBoundingBox(const LSM_GEOML::Pose &selectedTargetPose,
            const bool parkingOnLeftSide_nu,
            const ap_common::Vehicle_Params egoVehicleParams);

        void updateEgoVehScanningPath(const std::list<LSM_GEOML::Pose> &drivenPath,
            const LSM_GEOML::Pose &originTransformation,
            const bool parkingOnLeftSide_nu);

        void definePositionOnTrajectory(
            const ap_tp::PlannedTrajPort& plannedTrajPort,
            const LSM_GEOML::Pose& egoVehiclePose,
            PositionOnTrajectory& positionOnTrajectory);

        void createVehicleAreasPoints(const ap_tp::PoseType &parkingType,
            std::vector<cml::Vec2Df> &vehicleArea1Points, std::vector<cml::Vec2Df> &vehicleArea2Points,
            const ap_common::Vehicle_Params egoVehicleParams);

        void checkVehOutsidePB(const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU> &parkingBoxCoords,
            const ap_tp::PoseType &parkingType,
            const LSM_GEOML::Pose &egoVehiclePoseCMOrigin,
            const ap_tp::PlannedTraj &plannedTraj,
            const ap_common::Vehicle_Params egoVehicleParams);

        void TestEvaluation::timeToPark(const unsigned char headUnitScreen_nu,
            const ap_common::Vehicle_Params &vehicle_Params,
            const ap_tp::PlannedTrajPort &plannedTrajPort,
            const float32_t egoMotionFrontWheelAngle_rad,
            const float32_t egoMotionVelocity_mps,
            const ap_trjctl::MFControlStatusPort &mFControlStatusPort);

        void optimalTargetPoseEvaluation(const si::ParkingScenarioTypes &parkingScenarioType_nu,
            const LSM_GEOML::Pose &egoPossitionAtEndOfParkingManeuver);

        void runEveryCycle(const ap_tp::PlannedTrajPort& plannedTrajPort,
            const ap_common::CarMakerInterface& carMakerInterface,
            const ap_commonvehsigprovider::Gear& gearCur_nu,
            const bool hadGearReq_nu,
            const float32_t drivenDistance_m);

        static bool isWheelstopperDuringLastStrokeSituation(
            const ap_tp::PlannedTrajPort &plannedTrajPort,
            const ap_trjctl::TrajCtrlDebugPort &trajCtrlDebugPort);

    public:
        EvaluationPort evaluationPort;
        void resetVariablesTo0();

        void run(const ap_tp::PlannedTrajPort &plannedTrajPort,
            const ap_tp::TargetPosesPort &targetPosesPortCMOrigin,
            const ap_psm::SlotCtrlPort &slotCtrlPort,
            const si::ApEnvModelPort &envModelPort,
            const si::ApEnvModelPort &envModelPortCMOrigin,
            const si::ApParkingBoxPort &gParkingBoxPort,
            const si::ApParkingBoxPort &parkingBoxPortCMOrigin,
            const ap_common::CarMakerInterface &carMakerInterface,
            const lsm_vedodo::OdoEstimation &odoEstimationPort,
            const lsm_vedodo::OdoEstimation &odoEstimationPortCM,
            const ap_commonvehsigprovider::Gear &gearCur_nu,
            const std::list<LSM_GEOML::Pose> &drivenPath,
            const bool hadGearReq_nu,
            const bool parkingOnLeftSide_nu[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU],
            const float32_t egoMotionFrontWheelAngle_rad,
            const float32_t egoMotionVelocity_mps,
            const bool isFirstCycle,
            const ap_trjctl::MFControlStatusPort &mFControlStatusPort,
            const ap_trjctl::TrajCtrlDebugPort &trajCtrlDebugPort,
            const TrafficContour2D trafficContour2D[]);

        void registerCarMakerDVAs();

        void setNumberOfGearStrokes(const unsigned int &numberOfStrokesToSet);
};
#endif