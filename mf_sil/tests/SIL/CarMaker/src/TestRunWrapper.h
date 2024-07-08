#ifndef _TEST_RUN_WRAPPER_
#define _TEST_RUN_WRAPPER_

#include "Platform_Types.h"
#include "MfSilTypes.h"
#include "ap_common/ap_common_types_consts.h"

// Forward declarations
struct tTrafficObj;
struct tTrafficObj_Cfg;
namespace cml
{
    struct Vec2Df_POD;
}


constexpr uint8_t MAX_NUMBER_OF_POINTS_READ_FROM_TESTRUN{ 16U };
constexpr uint8_t MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM{ 200U };

bool trfObjIsOdo(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsDyn(tTrafficObj const* _trfObj);
bool trfObjIsStatic(tTrafficObj const* _trfObj);
bool trfObjIsParkBox(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsODS(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsRoundStatic(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsRectStatic(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsCurbstone(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsWhlStp(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsRoadLaneMark(tTrafficObj_Cfg const* _trfObj);
bool trfObjIsParkLineMark(tTrafficObj_Cfg const* _trfObj);

struct TrafficContour2D {
    char *name;                                                   // name of the CM scenarios
    uint8_t nrRows;                                               // number of Rows of the contour points of traffic object
    uint8_t trafficId;                                            // ID of trafficObject 
    float32_t points[2][MAX_NUMBER_OF_POINTS_READ_FROM_TESTRUN];  // 16 - 2 object (8 points maxim for one object)
    bool isMirroringConstruction_nu;                              // flag that is a mirroring construction of contour points of traffic object
};

// # ParkingBoxRightSide - parking box on the right side of the road
// # ParkingBoxLeftSide - parking box on the left side of the road
// # CircleObstacle - circle shape obstacles
// # LaneMarking - use 2 points
// # Rect4Points - use the length and the width of object to draw it
// # Other - other cases
enum objectType { ParkingBoxRightSide, ParkingBoxLeftSide, CircleObstacle, LaneMarking, Rect4Points, Other };

/*
*
* @param[in]  trafficObj - traffic Object parameter (from CarMaker)
* @param[in]  offsetX_m - offset for X coordonate for Dynamic Environment
* @param[in]  offsetY_m - offset for Y coordonate for Dynamic Environment
* @param[in]  objectCarMakerType_nu - Carmaker object type (PB,Lane Marking,CircleForm,4Points,others)
* @param[out] [numberOfPoints_nu]points of Traffic Object
* return the number of Points used for contour (int)
*/
uint8_t calculateObjectContourPoints(
    const tTrafficObj* trafficObj,
    const TrafficContour2D &trafficContour2D_t,
    const float32_t offsetX_m,
    const float32_t offsetY_m,
    objectType objectCarMakerType_nu,
    cml::Vec2Df_POD worldPt_m[],
    const bool isSplited,
    bool& needToBeSplited,
    const uint8_t maxPointToSplit = ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU);

#endif // !_TEST_RUN_WRAPPER_


