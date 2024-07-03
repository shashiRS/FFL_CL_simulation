#pragma once

// needed to prevent that the Win10 header rpcndr.h defines the type boolean, which is also defined in geoml/cml_type.h
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include<queue>
#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#pragma warning( pop )
#include "Car/Car_Steer_Pfeffer.h"
#include "Vehicle/Sensor_Object.h"

#include "EpDelimiterManager.h"
#include "si/static_obj_heigth_type.h"
#include "si/perception_availability_port.h"
#include "si/coll_env_model_port.h"
#include "si/ap_env_model_port.h"
#include "si/ap_parking_box_port.h"
#include "geoml/LSM_Math.h"
#include "geoml/LSM_LookupTable.h"
#include "TestRunWrapper.h"
#include "geoml/AxisAlignedRectangular2D.h"
#include "lsm_vedodo/odo_estimation_output_port.h"
#include "ap_psm/reset_origin_request_port.h"



//#include "vec_vector.h"
#include <algorithm>
#include <map>
#include "SimCore.h"
#include <cstring>
#include <cmath>
#include "MfSilTypes.h"
//parameter from Doors requirements; to search the parameter name in SI component
// to do : read all these parameters from a json file as their values often change
constexpr float32_t AP_G_MIN_HEIGHT_OBSTACLE_M          { 0.02f }; // In DOORS a value of 0.04 is required. AP_G_MIN_HEIGHT_OBSTACLE_M is only used in SI Surrogate and CEM Surrogate (not in DF). 
                                                                   // A value of 0.04 would lead to a lot of curb misclassifications (SO_HI_UNKNOWN instead of SO_HI_WHEEL_TRAVERSABLE) in CM use cases. 
                                                                  // In order not to change the height of all curbs in CM uses cases, the value is set back to 0.02.
constexpr float32_t AP_G_MAX_HEIGHT_WHEEL_TRAVER_M      { 0.06f };
constexpr float32_t AP_G_MAX_HEIGHT_BODY_TRAVER_M       { 0.15f };
constexpr float32_t AP_G_MAX_HEIGHT_DOOR_OPENABLE_M     { 0.2f };
constexpr float32_t AP_G_MIN_HEIGHT_HANG_OBST_M         { 0.85f }; // to be clarify
constexpr float32_t AP_G_MAX_HEIGHT_HANG_OBST_M         { 1.6f };  // to be clarify
constexpr float32_t AP_G_MIN_DEPTH_HANG_OBST_M          { 0.1f };
constexpr float32_t LSCA_PREDICTED_TO_DELETED_TIME_US   { 5e6f };
constexpr float32_t LSCA_DYNAMIC_TO_STATIC_TIME_US      { 1e6f };
constexpr float32_t AP_G_ANG_MAX_ORI_ANGLE_RAD          { 0.96f };
constexpr float32_t AP_G_PERP_MAX_ORI_ANGLE_RAD         { 0.15f };

// # NotDefine - is not Define
// # StaticObject - the object is static
// # DynamicObject - the object is dynamic
// # ParkingBox - the object is Parking Box
// # LaneBoundary - the object is lane boundary
// # ParkingSpaceMarking - the object is a parking space marking
enum objectClass { NotDefine = 0U, StaticObject = 1U, DynamicObject = 2U, ParkingBox = 3U, LaneBoundary = 4U, ParkingSpaceMarking = 5U };

typedef struct trafficObjects {
    objectClass type; //type of the traffic Object ( NotDefine = 0U, StaticObject = 1U, DynamicObject = 2U, ParkingBox = 3U, LaneBoundary = 4U, ParkingSpaceMarking = 5U)
    objectClass type2; //for the case when an CarMaker traffic Object is used for 2 AUP traffic object (Curb - ParkingSpaceMarking; a cotour vehicle should be splitted in 2 object)
                       //type of the traffic Object ( NotDefine = 0U, StaticObject = 1U, DynamicObject = 2U, ParkingBox = 3U, LaneBoundary = 4U, ParkingSpaceMarking = 5U)
    uint8_t SWIdx; //Software ID --- refObjID_nu
    uint8_t SWIdx2; ///for the case when an CarMaker traffic Object is used for 2 AUP traffic object (Curb - ParkingSpaceMarking; a cotour vehicle should be splitted in 2 object)
                    //Software ID --- refObjID_nu
    uint64_t lastDynamicObjectDetectionTimestamp_us; // last time when the dynamic object was detected
    std::map<uint8_t, uint8_t> pbIdx2virtLineIdx_nu; // map parking box index to the index of the virtual line created for this traffic object and parking box
}trafficObjects_t;

/*
*
* @param[in]  objectHeight_m - height of traffic Object parameter (from CarMaker)
* @param[in]  distanceFromGroundToObjec_m - distance from the "ground" to traffic objcet (offset on Z axes of traffic Object from CarMaker)
*
* return the Height Type of Static Structure (MFEnvModelData::StaticStructHeightType)
*/
si::StaticObjHeigthType returnStaticStructHeightType(double objectHeight_m, double distanceFromGroundToObjec_m);

/****************************************************************************
* Forward declarations
*/

namespace AP_Common {
    struct CarMakerInterface;
}

// Type alias for Polygon2D with 4 vertices
using PolygonPB = LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU>;

class EnvironmentPerception {
    si::PerceptionAvailabilityPort mPerceptionAvailabilityPort;
    si::CollEnvModelPort mCollEnvModelPort;
    EpDelimiterManager mDelimiterManager{};
    uint8_t mNumVirtualLinesPB[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU];
    std::queue<si::ApEnvModelPort> qEnvModelPort;
    si::ApEnvModelPort tEnvModelPort;
    std::queue<si::ApParkingBoxPort> qParkingBoxPort;
    si::ApParkingBoxPort tParkingBoxPort;

    si::ApEnvModelPort envModelPort;
    si::ApParkingBoxPort parkingBoxPort;
    bool mEnableScanning_nu{ false };
    float mLatencyTime_ms{ 0.0 };
    float mLongOffset_m{ 0.0 };
    float mLatOffset_m{ 0.0 };
    uint8_t mStaticObjIdx{ 0U };
    uint8_t mParkingSMIdx{ 0U };
    uint8_t mLaneBoundIdx{ 0U };
    uint8_t mDynObjIdx{ 0U };
    uint8_t mNumParkingBoxes{ 0U };
    float32_t mEgoVehicleOrientationDuringScanning_rad[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU];
    trafficObjects mTrafficObjects[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU];

    void limitFieldOfView(const bool limitedFieldOfView);
    void addLatencyEffect(const float SensorsLatencyTime_ms, const bool enableLatencyEffect);
    void addStaticOffset(const float offsetX, const float offsetY);
    void resetDynamicObject(uint8_t idx);
    void resetStaticObject(uint8_t idx);
    void shiftUpDynamicObjectElements(uint8_t idxStartShifting, uint8_t idxStopShifting);
    void shiftUpStaticObjectElements(uint8_t idxStartShifting, uint8_t idxStopShifting);
    LSM_GEOML::Polygon2D<4U> calculateBoundingBoxFromObjectShape(const trafficObjects& trafficObject, const float32_t boxOrientation_rad);
    void setVirtualLineForDelimiter(trafficObjects& trafficObject,
        const bool parkingOnLeftSide_nu, uint8_t &numVirtualLinesPB,
        const uint8_t pbIdx, const float32_t orientationAngle_rad, const si::RelativeLocationToParkingBox delimitingSide);
    void updateStaticObject(const tTrafficObj* trafficObj, objectType objectType_nu, const TrafficContour2D trafficContour2D_t[],
        const float32_t offsetX, const float32_t offsetY, const si::StaticObjectClass staticObjectClass_nu, const float32_t inflationLength_m, const bool parkingOnLeftSide_nu[]);
public:
    static constexpr uint64_t SI_HIGH_CYCLE_TIME_MS = 33U;

    EnvironmentPerception();
    void Init(const bool enableLimitFieldOfView_nu, const bool enableLatencyEffect_nu, const float latencyEffectTime_s);
    const bool getScanEM();
    const float getLatencyEffect();
    const float getlongOffset_m();
    const float getLatOffset_m();
    si::ApEnvModelPort EnvironmentPerception::getEnvModelPort();
    si::ApParkingBoxPort EnvironmentPerception::getParkingBoxPort();
    si::PerceptionAvailabilityPort getPerceptionAvailabilityPort();
    si::CollEnvModelPort getCollEnvModelPort();
    void updateEnvironmentModelAndParkingBox(const bool parkingOnLeftSide_nu[],
        const float MIN_OBJ_HEIGHT_M,
        const bool isFirstCycle,
        int pathBeforeActIndex,
        uint8_t overwritePathBeforeFuncActivation_nu,
        LSM_GEOML::Pose pathBeforeFuncActivation0Pose,
        const TrafficContour2D trafficContour2D_t[],
        const float32_t inflationLength_m);
    void updatePerceptionAvailabilityPort(const bool isFirstCycle);
    void updateCollEnvModel(const si::ApEnvModelPort& envModelPort);

    void resetVariables();
    void updateEnvModelData(const unsigned char vedodoActive_nu,
        const bool parkingOnLeftSide_nu[],
        const float MIN_OBJ_HEIGHT_M,
        const int pathBeforeActIndex,
        const uint8_t overwritePathBeforeFuncActivation_nu,
        const LSM_GEOML::Pose &pathBeforeFuncActivation0Pose,
        const TrafficContour2D trafficContour2D_t[],
        const float32_t inflationLength_m,
        const bool isFirstCycle,
        const lsm_vedodo::OdoEstimation &odoEstimationPortCM,
        const lsm_vedodo::OdoEstimationOutputPort &odoEstimationOutput,
        si::ApEnvModelPort &lEnvModelPort,
        si::ApEnvModelPort &lEnvModelPortCMOrigin,
        si::CollEnvModelPort &collEnvModelPort,
        si::ApParkingBoxPort &lParkingBoxPort,
        si::ApParkingBoxPort &lParkingBoxPortCMOrigin,
        si::PerceptionAvailabilityPort &perceptionAvailabilityPort);
    void transformEnvModelData(const ap_psm::ResetOriginRequestPort &resetOriginRequestPort,
        ap_psm::ResetOriginRequestPort &prevResetOriginRequestPort,
        LSM_GEOML::Pose &lastTransformation,
        LSM_GEOML::Pose &inverseTransformation,
        si::ApEnvModelPort &lEnvModelPort,
        si::ApEnvModelPort &lEnvModelPortCMOrigin,        
        si::ApParkingBoxPort &lParkingBoxPort);
};