#include "EnvironmentPerception.h"
#include "SiUtility.h"
#include<set>
#include "geoml/Polygon2D.h"
#include "pod_class_converter.h"

static lsm_geoml::Pose_POD getPose(const lsm_vedodo::OdoEstimation &odoEstimation) {
    return { odoEstimation.xPosition_m, odoEstimation.yPosition_m, odoEstimation.yawAngle_rad };
}

si::ParkingScenarioTypes determineParkingScenarioType(const si::ParkingBoxSerializable& parkingBox, const tTrafficObj* trafficObj, const float32_t egoScanningOrientation_rad)
{   
    const LSM_GEOML::Polygon2D<4U> pbShape = convert(parkingBox.slotCoordinates_m);
    const float32_t pbLongLengthSq = pbShape.getEdge(0).normSq() + pbShape.getEdge(2).normSq();
    const float32_t pbLatLengthSq = pbShape.getEdge(1).normSq() + pbShape.getEdge(3).normSq();
    const float32_t meanAngleOfPBSideEdges_rad = 0.5F*(LSM_GEOML::calcAngleBetweenVectors((-1.0F)*pbShape.getEdge(3), LSM_GEOML::getVecFromAngle(egoScanningOrientation_rad)) +
        LSM_GEOML::calcAngleBetweenVectors(pbShape.getEdge(1), LSM_GEOML::getVecFromAngle(egoScanningOrientation_rad)));
    if (strstr(trafficObj->Cfg.Info, "_GP")) { // Garage Parking scenarions are indicated by _GP in ParkingBox description
        return si::ParkingScenarioTypes::GARAGE_PARKING;
    }
    else if (pbLongLengthSq > pbLatLengthSq) /*if the pb is longer in longitudinal direction than in lateral direction, we consider it parallel */ {
        return si::ParkingScenarioTypes::PARALLEL_PARKING;
    }
    else if (meanAngleOfPBSideEdges_rad >= (LSM_GEOML::LSM_HALF_PI - AP_G_ANG_MAX_ORI_ANGLE_RAD) && meanAngleOfPBSideEdges_rad <= LSM_GEOML::LSM_HALF_PI - AP_G_PERP_MAX_ORI_ANGLE_RAD)
    {
        return si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_BACK;
    }
    else if (meanAngleOfPBSideEdges_rad <= (LSM_GEOML::LSM_HALF_PI + AP_G_ANG_MAX_ORI_ANGLE_RAD) && meanAngleOfPBSideEdges_rad >= LSM_GEOML::LSM_HALF_PI + AP_G_PERP_MAX_ORI_ANGLE_RAD)
    {
        return si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_FRONT;
    }
    else if (meanAngleOfPBSideEdges_rad > (LSM_GEOML::LSM_HALF_PI - AP_G_PERP_MAX_ORI_ANGLE_RAD) && meanAngleOfPBSideEdges_rad <= LSM_GEOML::LSM_HALF_PI + AP_G_PERP_MAX_ORI_ANGLE_RAD)
    {
        return si::ParkingScenarioTypes::PERPENDICULAR_PARKING;
    }
    // no scenario detected, return invalid value
    return  si::ParkingScenarioTypes::MAX_NUM_PARKING_SCENARIO_TYPES;
}

si::StaticObjHeigthType returnStaticObjHeightType(double objectHeight_m, double distanceFromGroundToObject_m) {
    //the height of the object including the distance from the "Ground" to Object
    float32_t objectHeightFromGround_m = static_cast<float32_t>(objectHeight_m + distanceFromGroundToObject_m);

    if (distanceFromGroundToObject_m >= AP_G_MIN_HEIGHT_HANG_OBST_M &&
        objectHeight_m > AP_G_MIN_DEPTH_HANG_OBST_M) {
        return si::StaticObjHeigthType::SO_HI_HANGING_OBJECT;
    }
    else if (objectHeightFromGround_m > AP_G_MIN_HEIGHT_OBSTACLE_M &&
        objectHeightFromGround_m < AP_G_MAX_HEIGHT_WHEEL_TRAVER_M) {
        return si::StaticObjHeigthType::SO_HI_WHEEL_TRAVERSABLE;
    }
    else if (objectHeightFromGround_m > AP_G_MIN_HEIGHT_OBSTACLE_M &&
        objectHeightFromGround_m < AP_G_MAX_HEIGHT_BODY_TRAVER_M) {
        return si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE;
    }
    else if (objectHeightFromGround_m > AP_G_MIN_HEIGHT_OBSTACLE_M &&
        objectHeightFromGround_m < AP_G_MAX_HEIGHT_DOOR_OPENABLE_M) {
        return si::StaticObjHeigthType::SO_HI_DOOR_OPENABLE;
    }
    else if (objectHeightFromGround_m >= AP_G_MAX_HEIGHT_DOOR_OPENABLE_M) {
        return si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE;
    }
    else {
        return si::StaticObjHeigthType::SO_HI_UNKNOWN;
    }
}

float32_t determineVirtualLineLengthThreshold(const si::RelativeLocationToParkingBox delimitingSide, const si::ParkingScenarioTypes parkingScenario_nu)
{
    float32_t minVirtualLineLength_m{ 0.0f };
    const auto &siParams = SiUtility::getSiParameters();
    switch (delimitingSide)
    {
    case si::RelativeLocationToParkingBox::INSIDE_LEFT_EDGE:
    case si::RelativeLocationToParkingBox::LEFT_EDGE:
    case si::RelativeLocationToParkingBox::INSIDE_RIGHT_EDGE:
    case si::RelativeLocationToParkingBox::RIGHT_EDGE:
        if (parkingScenario_nu == si::ParkingScenarioTypes::PERPENDICULAR_PARKING)
        {
            minVirtualLineLength_m = siParams.vlMinVirtualLineLengthPerpSide_m;
            break;
        }
        if (parkingScenario_nu == si::ParkingScenarioTypes::PARALLEL_PARKING)
        {
            minVirtualLineLength_m = siParams.vlMinVirtualLineLengthParSide_m;
        }
        break;
    case si::RelativeLocationToParkingBox::CURB_SIDE_EDGE:
    case si::RelativeLocationToParkingBox::INSIDE_CURB_SIDE_EDGE:
        minVirtualLineLength_m = siParams.vlMinVirtualLineLengthCurb_m;
        break;
    case si::RelativeLocationToParkingBox::ROAD_SIDE_EDGE:
    case si::RelativeLocationToParkingBox::INSIDE_ROAD_SIDE_EDGE:
        minVirtualLineLength_m = siParams.vlMinVirtualLineLengthRoad_m;
        break;
    default:
        break;
    }
    return minVirtualLineLength_m;
}

bool sortByY(const cml::Vec2Df &lhs, const cml::Vec2Df &rhs) { return lhs.y() > rhs.y(); }

bool sortByX(const cml::Vec2Df &lhs, const cml::Vec2Df &rhs) { return lhs.x() > rhs.x(); }

//---------------------------------------------------------------------------
//! fill the parkingBox delimiter info in tParkingBoxPort.parkingBoxes[].delimiters data struc
//! used for scan On testrun to don't overwrite previous value
//!
void fillDelimiterOfParkingBox(const uint8_t tmpIndexInList_nu, const si::DelimiterTypes tmpDelimiterType_nu, const si::RelativeLocationToParkingBox tmpDelimitingSide_nu,
    si::ParkingBoxDelimiter tmpParkingBoxDelimiter[])
{
    uint8_t delimiterIdx{ 0U };
    //for scanOn enable, to don't overwrite previous value; check if the index of traffic Object in delimiters port are the same with the index of traffic Object;
    for (uint8_t k = 0; k < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_DELIMITERS_NU; k++)
    {
        if ((tmpParkingBoxDelimiter[k].indexInList_nu == tmpIndexInList_nu &&
            tmpParkingBoxDelimiter[k].delimiterType_nu == tmpDelimiterType_nu) ||
            tmpParkingBoxDelimiter[k].delimiterType_nu == si::DelimiterTypes::MAX_NUM_PARKING_BOX_DELIMITER_TYPES)
        {
            delimiterIdx = k;
            break;
        }
    }

    tmpParkingBoxDelimiter[delimiterIdx].indexInList_nu = tmpIndexInList_nu;
    tmpParkingBoxDelimiter[delimiterIdx].delimiterType_nu = tmpDelimiterType_nu;
    tmpParkingBoxDelimiter[delimiterIdx].delimitingSide_nu = tmpDelimitingSide_nu;
}

EnvironmentPerception::EnvironmentPerception() {
    mEnableScanning_nu = false;
    mLatencyTime_ms = 100.0f;
    mLongOffset_m = 0.0f;
    mLatOffset_m = 0.0f;
}

void EnvironmentPerception::Init(const bool enableLimitFieldOfView_nu, const bool enableLatencyEffect_nu, const float latencyEffectTime_s)
{
    addLatencyEffect(latencyEffectTime_s*1000.0f, enableLatencyEffect_nu);
    addStaticOffset(0.0f, 0.0f);
    limitFieldOfView(enableLimitFieldOfView_nu);
}

void EnvironmentPerception::limitFieldOfView(const bool limitedFieldOfView = false)
{
    mEnableScanning_nu = limitedFieldOfView;
}

void EnvironmentPerception::addLatencyEffect(const float SensorsLatencyTime_ms = 0.0f, const bool enableLatencyEffect = false)
{
    if (enableLatencyEffect)
        mLatencyTime_ms = SensorsLatencyTime_ms;
    else
        mLatencyTime_ms = 0.0f;
}

void EnvironmentPerception::addStaticOffset(const float offsetX = 0.0f, const float offsetY = 0.0f)
{
    mLongOffset_m = offsetX;
    mLatOffset_m = offsetY;
}

const bool EnvironmentPerception::getScanEM() {
    return mEnableScanning_nu;
}

const float EnvironmentPerception::getLatencyEffect() {
    return mLatencyTime_ms;
}

const float EnvironmentPerception::getlongOffset_m() {
    return mLongOffset_m;
}

const float EnvironmentPerception::getLatOffset_m() {
    return mLatOffset_m;
}

si::ApEnvModelPort EnvironmentPerception::getEnvModelPort() {
    return envModelPort;
}

si::ApParkingBoxPort EnvironmentPerception::getParkingBoxPort() {
    return parkingBoxPort;
}

si::PerceptionAvailabilityPort EnvironmentPerception::getPerceptionAvailabilityPort() {
    return mPerceptionAvailabilityPort;
}

si::CollEnvModelPort EnvironmentPerception::getCollEnvModelPort() {
    return mCollEnvModelPort;
}

/*
* reset the Dynamic Object with Id - idx to default value
*
* @param[in]  idx - the id of Dynamic Object element do you want to reset/delete
*
*
*/
void EnvironmentPerception::resetDynamicObject(uint8_t idx) {
    tEnvModelPort.dynamicObjects[idx] = eco::create_default<si::DynamicObjectSerializable>();
    //when the object is reset, the measumentState is New - > set to MAX_NUM_MEASUREMENT_STATES
    tEnvModelPort.dynamicObjects[idx].measurementState_nu = si::ObjMeasurementState::MAX_NUM_MEASUREMENT_STATES;
    tEnvModelPort.dynamicObjects[idx].existenceProb_perc = 0U;
    tEnvModelPort.dynamicObjects[idx].objShape_m.actualSize = 0U;
}

/*
* reset the Static Object with Id - idx to default value
*
* @param[in]  idx - the id of Static Object element do you want to reset/delete
*
*
*/
void EnvironmentPerception::resetStaticObject(uint8_t idx) {
    tEnvModelPort.staticObjects[idx] = eco::create_default<si::StaticObjectSerializable>();
    tEnvModelPort.staticObjects[idx].existenceProb_perc = 0U;
    tEnvModelPort.staticObjects[idx].objShape_m.actualSize = 0U;
}

/*
* shift the Dynamic Object from the last Object that was detected (existenceProbability is 100) - idxStopShifting to the Object that should be reset(idxStartShifting)
*
* @param[in]  idxStartShifting - the id of Dynamic Object element do you want to reset/delete
* @param[in]  idxStopShifting - the id of the last Object from the dynamic object list that was detected (existenceProbability is 100)
*
*
*/
void EnvironmentPerception::shiftUpDynamicObjectElements(uint8_t idxStartShifting, uint8_t idxStopShifting) {
    if (idxStartShifting == idxStopShifting) {
        resetDynamicObject(idxStartShifting);
    }
    else {
        uint8_t idxForShifting;
        //check untill idxForShifting = idxStopShifting - 1 - because object with id idxStopShifting have existenceProb != 100
        for (idxForShifting = idxStartShifting; idxForShifting < idxStopShifting - 1; idxForShifting++) {
            tEnvModelPort.dynamicObjects[idxForShifting] = tEnvModelPort.dynamicObjects[idxForShifting + 1];
            /*tEnvModelPort.dynamicObjects[idxForShifting].refObjID_nu = idxForShifting;*/
        }
        resetDynamicObject(idxForShifting);
    }
}

/*
* shift the Static Object from the last Object from the Static object list that was detected (existenceProbability is 100) - idxStopShifting to the Object that should be reset(idxStartShifting)
*
* @param[in]  idxStartShifting - the id of Static Object element do you want to reset/delete
* @param[in]  idxStopShifting  - the id of the last Object from the Static object list that was detected (existenceProbability is 100)
*
*
*/
void EnvironmentPerception::shiftUpStaticObjectElements(uint8_t idxStartShifting, uint8_t idxStopShifting) {
    if (idxStartShifting == idxStopShifting) {
        resetStaticObject(idxStartShifting);
    }
    else {
        uint8_t idxForShifting;
        //check untill idxForShifting = idxStopShifting - 1 - because object with id idxStopShifting have existenceProb != 100
        for (idxForShifting = idxStartShifting; idxForShifting < idxStopShifting - 1; idxForShifting++) {
            tEnvModelPort.staticObjects[idxForShifting] = tEnvModelPort.staticObjects[idxForShifting + 1];
        }
        resetStaticObject(idxForShifting);
    }
}
/*
void removeLessImportantPoints(ap_common::PlanningObject<si::StaticObject> &inflatedObject) {
    while (inflatedObject.getSize() > ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU) {
        float32_t minimumDistance = std::numeric_limits<float32_t>::max();
        auto vertexToBeRemoved = inflatedObject.begin();

        for (auto it = inflatedObject.begin(); it < inflatedObject.end(); it++) {
            // Get the distances from the current vertex to the previous and the next vertex;
            // the first vertex is connected with the last one and the last vertex is connected with the first one
            const cml::Vec2Df vecToPrevPoint = ((it == inflatedObject.begin()) ? inflatedObject.back() : *std::prev(it)) - *it;
            const cml::Vec2Df vecToNextPoint = ((it == std::prev(inflatedObject.end())) ? inflatedObject.front() : *std::next(it)) - *it;
            const float32_t neighbourPointsDistance = vecToPrevPoint.norm() + vecToNextPoint.norm();

            if (neighbourPointsDistance < minimumDistance) {
                minimumDistance = neighbourPointsDistance;
                vertexToBeRemoved = it;
            }
        }
        inflatedObject.eraseAt(vertexToBeRemoved);
    }
}
*/
void inflateObject(si::StaticObjectSerializable &/*staticObj*/, const float32_t /*inflationLength*/)
{
    // Inflation of polygons was removed from mf_common
    // Agreement is to reenable it in a future point in time with some other methods here.


    //ap_common::PlanningObject<si::StaticObject> objToInflate;

    ////initialize an InflatedStaticObject with our SI object
    //objToInflate.setRefObj(&staticObj);

    ////if inflation succeed then populate the new SI object with inflated points
    //if (objToInflate.inflate(inflationLength) == true)
    //{
    //    //verify if the polygon of the objToInflate contains less or equal than 16 points.If not then reduce the number of points to 16.
    //    if (objToInflate.getSize() > ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU) {
    //        try {
    //            removeLessImportantPoints(objToInflate);
    //        }
    //        catch (std::exception e)
    //        {
    //            LogErrStr(EC_General, std::string("Inflation of the object exited with exception: " + std::string(e.what()) + "\n").c_str());
    //        }
    //    }

    //    // copy points of inflated object to the static object shape
    //    staticObj.objShape_m.clear();
    //    for (const auto &p : objToInflate) {
    //        staticObj.objShape_m.append(p);
    //    }
    //}
}

LSM_GEOML::Polygon2D<4U> EnvironmentPerception::calculateBoundingBoxFromObjectShape(const trafficObjects& trafficObject, const float32_t boxOrientation_rad) {
    const cml::Vec2Df longitAxis{ cos(boxOrientation_rad), sin(boxOrientation_rad) };
    const cml::Vec2Df lateralAxis{ -sin(boxOrientation_rad), cos(boxOrientation_rad) };
    const LSM_GEOML::Polygon2D<16U> obj1Shape = convert(tEnvModelPort.staticObjects[trafficObject.SWIdx].objShape_m);
    auto longitMinMax{ obj1Shape.projectToAxis(longitAxis) };
    auto lateralMinMax{ obj1Shape.projectToAxis(lateralAxis) };
    // If the trafficObject is splitted into two static objects, also consider the polygon of the second static object.
    if (objectClass::StaticObject == trafficObject.type2) {
        const LSM_GEOML::Polygon2D<16U> obj2Shape = convert(tEnvModelPort.staticObjects[trafficObject.SWIdx2].objShape_m);
        const auto longitMinMax2{ obj2Shape.projectToAxis(longitAxis) };
        const auto lateralMinMax2{ obj2Shape.projectToAxis(lateralAxis) };
        longitMinMax.first = std::min(longitMinMax.first, longitMinMax2.first);
        longitMinMax.second = std::max(longitMinMax.second, longitMinMax2.second);
        lateralMinMax.first = std::min(lateralMinMax.first, lateralMinMax2.first);
        lateralMinMax.second = std::max(lateralMinMax.second, lateralMinMax2.second);
    }
    LSM_GEOML::Polygon2D<4U> boundingBox{};
    boundingBox.append(longitMinMax.second * longitAxis + lateralMinMax.first * lateralAxis);   // front right
    boundingBox.append(longitMinMax.second * longitAxis + lateralMinMax.second * lateralAxis);  // front left
    boundingBox.append(longitMinMax.first * longitAxis + lateralMinMax.second * lateralAxis);   // rear left
    boundingBox.append(longitMinMax.first * longitAxis + lateralMinMax.first * lateralAxis);    // rear right
    return boundingBox;
}

void adjustParkingBoxSizeToVirtualLines(si::ParkingBoxSerializable& parkingBox) {

    const LSM_GEOML::Polygon2D<4U> pbShape = convert(parkingBox.slotCoordinates_m);
    const auto roadDir = pbShape.getEdge(0).getNormalized();
    const auto perpDir = cml::Vec2Df{ -roadDir.y(), roadDir.x() };
    auto pbRoadExtent = pbShape.projectToAxis(roadDir);    // first = left PB side; second = right PB side
    auto pbPerpExtent = pbShape.projectToAxis(perpDir);    // first = road side edge; second = curb side edge
    for (unsigned iDel = 0U; iDel < parkingBox.numValidDelimiters_nu; ++iDel) {
        auto& delimiter = parkingBox.delimiters[iDel];
        if (si::DelimiterTypes::STATIC_STRUCTURE == delimiter.delimiterType_nu &&
            delimiter.virtLineIdx_nu < parkingBox.numVirtualLines_nu) {

            LSM_GEOML::Polygon2D<2U> virtualLineAsPoly{}; // convert virtual line to a Polygon2D to use its projectToAxis() method
            virtualLineAsPoly.append(parkingBox.virtualLines[delimiter.virtLineIdx_nu].virtLineVertices_m.array[0]);
            virtualLineAsPoly.append(parkingBox.virtualLines[delimiter.virtLineIdx_nu].virtLineVertices_m.array[1]);
            const auto vlRoadExtent = virtualLineAsPoly.projectToAxis(roadDir);
            const auto vlPerpExtent = virtualLineAsPoly.projectToAxis(perpDir);

            // decrease length of parking box in road direction
            if ((si::RelativeLocationToParkingBox::LEFT_EDGE == delimiter.delimitingSide_nu || si::RelativeLocationToParkingBox::INSIDE_LEFT_EDGE == delimiter.delimitingSide_nu) &&
                // only consider left-side delimiters that do not reach into right side of the parking box
                vlRoadExtent.second < pbRoadExtent.second) {
                pbRoadExtent.first = std::max({ pbRoadExtent.first, vlRoadExtent.first,  vlRoadExtent.second });
                delimiter.delimitingSide_nu = si::RelativeLocationToParkingBox::LEFT_EDGE; // After shrinking the PB, the static object can not be anymore inside of the PB (not INSIDE_LEFT_EDGE).
            }
            else if ((si::RelativeLocationToParkingBox::RIGHT_EDGE == delimiter.delimitingSide_nu || si::RelativeLocationToParkingBox::INSIDE_RIGHT_EDGE == delimiter.delimitingSide_nu) &&
                // only consider right-side delimiters that do not reach into left side of the parking box
                vlRoadExtent.first > pbRoadExtent.first) {
                pbRoadExtent.second = std::min({ pbRoadExtent.second, vlRoadExtent.first, vlRoadExtent.second });
                delimiter.delimitingSide_nu = si::RelativeLocationToParkingBox::RIGHT_EDGE; // After shrinking the PB, the static object can not be anymore inside of the PB (not INSIDE_RIGHT_EDGE).
            }
            // Widen the parking box towards the road side
            pbPerpExtent.first = std::min({ pbPerpExtent.first, vlPerpExtent.first, vlPerpExtent.second });
        }
    }

    // construct new parking box
    parkingBox.slotCoordinates_m.array[0] = convert(static_cast<cml::Vec2Df>(pbRoadExtent.first * roadDir + pbPerpExtent.first * perpDir));
    parkingBox.slotCoordinates_m.array[1] = convert(static_cast<cml::Vec2Df>(pbRoadExtent.second * roadDir + pbPerpExtent.first * perpDir));
    parkingBox.slotCoordinates_m.array[2] = convert(static_cast<cml::Vec2Df>(pbRoadExtent.second * roadDir + pbPerpExtent.second * perpDir));
    parkingBox.slotCoordinates_m.array[3] = convert(static_cast<cml::Vec2Df>(pbRoadExtent.first * roadDir + pbPerpExtent.second * perpDir));
}

void EnvironmentPerception::setVirtualLineForDelimiter(trafficObjects& trafficObject,
    const bool parkingOnLeftSide_nu, uint8_t &numVirtualLinesPB,
    const uint8_t pbIdx, const float32_t orientationAngle_rad, const si::RelativeLocationToParkingBox delimitingSide)
{
    bool virtualLineSet{ false };
    //calculate the bounding box of the object
    auto angleDiffToAxes_rad = std::fmodf(orientationAngle_rad, LSM_GEOML::LSM_HALF_PI);
    if (std::fabsf(angleDiffToAxes_rad - LSM_GEOML::signum(angleDiffToAxes_rad) * LSM_GEOML::LSM_HALF_PI) < std::fabsf(angleDiffToAxes_rad)) {
        angleDiffToAxes_rad -= LSM_GEOML::signum(angleDiffToAxes_rad) * LSM_GEOML::LSM_HALF_PI;
    }
    const auto objectBoundingBox = calculateBoundingBoxFromObjectShape(trafficObject, angleDiffToAxes_rad);

    const float32_t minVirtualLineLength_m = determineVirtualLineLengthThreshold(delimitingSide, tParkingBoxPort.parkingBoxes[pbIdx].parkingScenario_nu);
    const auto updateVirtualLine = [&trafficObject, &numVirtualLinesPB, &virtualLineSet, pbIdx, minVirtualLineLength_m, this]
    (const cml::Vec2Df &vertex0, const cml::Vec2Df &vertex1) {
        if (static_cast<cml::Vec2Df>(vertex1 - vertex0).norm() >= minVirtualLineLength_m)
        {
            if (!trafficObject.pbIdx2virtLineIdx_nu.count(pbIdx)) { // if no virtual line was generated for this parking box yet
                trafficObject.pbIdx2virtLineIdx_nu.insert(std::make_pair(pbIdx, numVirtualLinesPB++));
            }
            const uint8_t virtualLineIdx = trafficObject.pbIdx2virtLineIdx_nu[pbIdx];
            tParkingBoxPort.parkingBoxes[pbIdx].virtualLines[virtualLineIdx].virtLineVertices_m.actualSize = 2U;
            tParkingBoxPort.parkingBoxes[pbIdx].virtualLines[virtualLineIdx].virtLineVertices_m.array[0] = convert(vertex0);
            tParkingBoxPort.parkingBoxes[pbIdx].virtualLines[virtualLineIdx].virtLineVertices_m.array[1] = convert(vertex1);
            virtualLineSet = true;
        }
    };

    switch (tParkingBoxPort.parkingBoxes[pbIdx].parkingScenario_nu)
    {
    case si::ParkingScenarioTypes::PARALLEL_PARKING:
        switch (delimitingSide)
        {
        case si::RelativeLocationToParkingBox::LEFT_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_LEFT_EDGE:
        case si::RelativeLocationToParkingBox::RIGHT_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_RIGHT_EDGE:
        case si::RelativeLocationToParkingBox::CURB_SIDE_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_CURB_SIDE_EDGE:
            // take road side of bounding box
            updateVirtualLine(objectBoundingBox[parkingOnLeftSide_nu ? 3 : 2], objectBoundingBox[parkingOnLeftSide_nu ? 0 : 1]);
            break;
        case si::RelativeLocationToParkingBox::ROAD_SIDE_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_ROAD_SIDE_EDGE:
            // take curb side of bounding box
            updateVirtualLine(objectBoundingBox[parkingOnLeftSide_nu ? 2 : 3], objectBoundingBox[parkingOnLeftSide_nu ? 1 : 0]);
            break;
        default:
            break; // no virtual line for other parking box edges
        }
        break;

    case si::ParkingScenarioTypes::PERPENDICULAR_PARKING:
    case si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_FRONT:
    case si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_BACK:
        switch (delimitingSide)
        {
        case si::RelativeLocationToParkingBox::LEFT_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_LEFT_EDGE:
            // take right side of bounding box
            updateVirtualLine(objectBoundingBox[parkingOnLeftSide_nu ? 0 : 2], objectBoundingBox[parkingOnLeftSide_nu ? 1 : 3]);
            break;
        case si::RelativeLocationToParkingBox::RIGHT_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_RIGHT_EDGE:
            // take left side of bounding box
            updateVirtualLine(objectBoundingBox[parkingOnLeftSide_nu ? 3 : 1], objectBoundingBox[parkingOnLeftSide_nu ? 2 : 0]);
            break;
        case si::RelativeLocationToParkingBox::CURB_SIDE_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_CURB_SIDE_EDGE:
            // take road side of bounding box
            updateVirtualLine(objectBoundingBox[parkingOnLeftSide_nu ? 3 : 2], objectBoundingBox[parkingOnLeftSide_nu ? 0 : 1]);
            break;
        case si::RelativeLocationToParkingBox::ROAD_SIDE_EDGE:
        case si::RelativeLocationToParkingBox::INSIDE_ROAD_SIDE_EDGE:
            // take curb side of bounding box
            updateVirtualLine(objectBoundingBox[parkingOnLeftSide_nu ? 2 : 3], objectBoundingBox[parkingOnLeftSide_nu ? 1 : 0]);
            break;
        default:
            break; // no virtual line for other parking box edges
        }
        break;

    default:
        break; // no virtual line for other ParkingScenarioTypes
    }

    if (virtualLineSet) { // only if the virtual line was set
                    // set virtual line index to parking box delimiter which already references this static object
        const uint8_t objIdx = trafficObject.SWIdx;
        for (auto &delimiter : tParkingBoxPort.parkingBoxes[pbIdx].delimiters) {
            if (objIdx == delimiter.indexInList_nu && si::DelimiterTypes::STATIC_STRUCTURE == delimiter.delimiterType_nu) {
                delimiter.virtLineIdx_nu = trafficObject.pbIdx2virtLineIdx_nu[pbIdx];
                break;
            }
        }
    }
}

void EnvironmentPerception::updateStaticObject(const tTrafficObj* trafficObj, objectType objectType_nu, const TrafficContour2D trafficContour2D_t[],
    const float32_t offsetX, const float32_t offsetY, const si::StaticObjectClass staticObjectClass_nu, const float32_t inflationLength_m, const bool parkingOnLeftSide_nu[]) {
    //check if the traffic object was saved in staticObject port
    if (mTrafficObjects[trafficObj->Cfg.Id].type != objectClass::StaticObject) {
        mTrafficObjects[trafficObj->Cfg.Id].SWIdx = mStaticObjIdx++;
        mTrafficObjects[trafficObj->Cfg.Id].type = objectClass::StaticObject;
    }
    const uint8_t objIdx = mTrafficObjects[trafficObj->Cfg.Id].SWIdx;
    tEnvModelPort.staticObjects[objIdx].refObjClass_nu = staticObjectClass_nu;
    tEnvModelPort.staticObjects[objIdx].existenceProb_perc = 100u;
    tEnvModelPort.staticObjects[objIdx].readFromNVRAM_nu = false;
    tEnvModelPort.staticObjects[objIdx].refObjID_nu = static_cast<uint16_t>(objIdx);

    //for the case when the number of 2d CarMaker Contour is bigger than 16
    bool needToBeSplited = false;
    /*transform the CarMaker Origin point*/
    tEnvModelPort.staticObjects[objIdx].objShape_m.actualSize = calculateObjectContourPoints(trafficObj,
        trafficContour2D_t[trafficObj->Cfg.Id],
        offsetX,
        offsetY,
        objectType_nu,
        tEnvModelPort.staticObjects[objIdx].objShape_m.array,
        false,
        needToBeSplited);
    //call the inflation function only if the variable from CarMaker is greater than 0
    if (inflationLength_m > 0.0f) {
        assert(false); // not supported because content of inflateObject method was commented because functionality removed from mf_common: contact Nicolas Stein; object inflation will be covered by SI in future
        inflateObject(tEnvModelPort.staticObjects[objIdx], inflationLength_m);
    }

    // determine all parking boxes of which this static object is a delimiter
    for (uint8_t iPB = 0U; iPB < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; ++iPB) {
        //check the delimiter only if the PB exist
        if (tParkingBoxPort.parkingBoxes[iPB].existenceProb_perc == 100u)
        {
            //check if the trafficObject(Obstacle-circle) is a delimiter of PB
            const LSM_GEOML::Polygon2D<16U> objShape = convert(tEnvModelPort.staticObjects[objIdx].objShape_m);
            const si::RelativeLocationToParkingBox delimiterType_nu = mDelimiterManager.determineDelimiterType(iPB, objShape, trafficObj->Cfg.Info,
                tParkingBoxPort.parkingBoxes[iPB].parkingScenario_nu);
            if (delimiterType_nu != si::RelativeLocationToParkingBox::UNDEFINED_EDGE &&
                delimiterType_nu != si::RelativeLocationToParkingBox::MAX_NUM_PARKING_BOX_EDGE_TYPES)
            {
                fillDelimiterOfParkingBox(objIdx,
                    si::DelimiterTypes::STATIC_STRUCTURE,
                    delimiterType_nu,
                    &(tParkingBoxPort.parkingBoxes[iPB].delimiters[0]));//TODO better solution

                setVirtualLineForDelimiter(mTrafficObjects[trafficObj->Cfg.Id],
                    parkingOnLeftSide_nu[iPB], mNumVirtualLinesPB[iPB], iPB, static_cast<float32_t>(trafficObj->Cfg.r_zyx[2]), delimiterType_nu);
            }
        }
    }

    //tEnvModelPort.staticStructures[objIdx].detectingSensors_nu[static_cast<uint8_t>(MFEnvModelData::SensorType::ULTRA_SONIC_SENSOR)] = true;
    tEnvModelPort.staticObjects[objIdx].objHeightClassConfidence_perc = 100u;
    tEnvModelPort.staticObjects[objIdx].objHeightClass_nu = returnStaticObjHeightType(trafficObj->Cfg.h, trafficObj->Cfg.zOff);

    if (needToBeSplited) {
        if (mTrafficObjects[trafficObj->Cfg.Id].type2 != objectClass::StaticObject) {
            mTrafficObjects[trafficObj->Cfg.Id].SWIdx2 = mStaticObjIdx++;
            mTrafficObjects[trafficObj->Cfg.Id].type2 = objectClass::StaticObject;
        }
        const uint8_t objIdx2 = mTrafficObjects[trafficObj->Cfg.Id].SWIdx2;

        tEnvModelPort.staticObjects[objIdx2].existenceProb_perc = 100u;
        tEnvModelPort.staticObjects[objIdx2].readFromNVRAM_nu = false;
        tEnvModelPort.staticObjects[objIdx2].refObjID_nu = static_cast<uint16_t>(objIdx2);
        tEnvModelPort.staticObjects[objIdx2].refObjClass_nu = si::StaticObjectClass::STAT_OBJ_UNCLASSIFIED_STRUCTURE;
        tEnvModelPort.staticObjects[objIdx2].objShape_m.actualSize = calculateObjectContourPoints(trafficObj,
            trafficContour2D_t[trafficObj->Cfg.Id],
            offsetX,
            offsetY,
            objectType::Other,
            tEnvModelPort.staticObjects[objIdx2].objShape_m.array,
            true,
            needToBeSplited);
        tEnvModelPort.staticObjects[objIdx2].objHeightClassConfidence_perc = 100u;
        tEnvModelPort.staticObjects[objIdx2].objHeightClass_nu = returnStaticObjHeightType(trafficObj->Cfg.h, trafficObj->Cfg.zOff);
        //call the inflation function only if the variable from CarMaker is greater than 0
        if (inflationLength_m > 0.0f) {
            inflateObject(tEnvModelPort.staticObjects[objIdx2], inflationLength_m);
        }
    }
}

void EnvironmentPerception::updateCollEnvModel(const si::ApEnvModelPort& localEnvModelPort) {
    mCollEnvModelPort.sSigHeader.uiTimeStamp = localEnvModelPort.sSigHeader.uiTimeStamp;
    mCollEnvModelPort.sSigHeader.eSigStatus = localEnvModelPort.sSigHeader.eSigStatus;
    mCollEnvModelPort.numberOfStaticObjects_u8 = std::min(localEnvModelPort.numberOfStaticObjects_u8, uint8_t(ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_STATIC_OBJ_NU));
    mCollEnvModelPort.numberOfDynamicObjects_u8 = std::min(localEnvModelPort.numberOfDynamicObjects_u8, uint8_t(ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_DYN_OBJECTS_NU));
    const LSM_GEOML::CoordinateTransformer2D transformToNewCS(localEnvModelPort.egoVehiclePoseForAP);

    // iterate through every static object
    for (unsigned iObj = 0U; iObj < std::min(uint8_t(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU), uint8_t(ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_STATIC_OBJ_NU)); ++iObj) {
        auto &collStatObj = mCollEnvModelPort.staticObjects[iObj];
        collStatObj = localEnvModelPort.staticObjects[iObj];
        if (collStatObj.existenceProb_perc > 0U) {
            // transform the vertices from the CarMaker coordinate system to the ego vehicle coordinate system
            for (lsm_geoml::size_type iVert{ 0U }; iVert < collStatObj.objShape_m.actualSize; ++iVert) {
                auto &objShapeVertex = collStatObj.objShape_m.array[iVert];
                objShapeVertex = convert(transformToNewCS.inverseTransform(objShapeVertex));
            }
        }
    }
    for (unsigned int object = 0; object < std::min(uint8_t(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_DYN_OBJECTS_NU), uint8_t(ap_common::AP_COMMON_TYPES_Consts::COLL_G_MAX_NUM_DYN_OBJECTS_NU)); object++) {
        auto &collDynObj = mCollEnvModelPort.dynamicObjects[object];
        collDynObj = localEnvModelPort.dynamicObjects[object];
        if (collDynObj.existenceProb_perc > 0U) {
            // transform the vertices from the CarMaker coordinate system to the ego vehicle coordinate system
            for (lsm_geoml::size_type iVert{ 0U }; iVert < collDynObj.objShape_m.actualSize; ++iVert) {
                auto &objShapeVertex = collDynObj.objShape_m.array[iVert];
                objShapeVertex = convert(transformToNewCS.inverseTransform(objShapeVertex));
            }
            // transform the velocity, acceleration and heading angle to ego vehicle coordinates
            collDynObj.vel_mps = convert(transformToNewCS.rotate(collDynObj.vel_mps, -transformToNewCS.getRefOrientation()));
            collDynObj.accel_mps2 = convert(transformToNewCS.rotate(collDynObj.accel_mps2, -transformToNewCS.getRefOrientation()));
            collDynObj.headingAngle_rad = LSM_GEOML::getAngularDiff(transformToNewCS.getRefOrientation(), collDynObj.headingAngle_rad);
        }
    }
    SiUtility::getInstance().sortStaticObjectsByDistance(mCollEnvModelPort);
    SiUtility::getInstance().setFirstObjOutDetZoneIdx(mCollEnvModelPort);
}

static void updateSignalHeader(const bool isFirstCycle, eco::SignalHeader& sigHeader) {
    const uint64_t timestamp_us = static_cast<uint64_t>(std::round(1e6 * SimCore.Time));
    sigHeader.uiTimeStamp = timestamp_us;
    if (isFirstCycle) {
        sigHeader.uiCycleCounter = 1U;
        sigHeader.uiMeasurementCounter = 1U;
        sigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
    }
    else {
        sigHeader.uiCycleCounter++;
        sigHeader.uiMeasurementCounter++;
    }
}

void EnvironmentPerception::updateEnvironmentModelAndParkingBox(const bool parkingOnLeftSide_nu[],
    const float MIN_OBJ_HEIGHT_M,
    const bool isFirstCycle,
    int pathBeforeActIndex,
    uint8_t overwritePathBeforeFuncActivation_nu,
    LSM_GEOML::Pose pathBeforeFuncActivation0Pose,
    const TrafficContour2D trafficContour2D_t[],
    const float32_t inflationLength_m)
{
    // TODO: remove these parameters from the method interface if they are really not needed anymore (align with Felix)
    pathBeforeFuncActivation0Pose;       /*silence unreferenced warning*/
    pathBeforeActIndex;                  /*silence unreferenced warning*/
    overwritePathBeforeFuncActivation_nu;/*silence unreferenced warning*/

    updateSignalHeader(isFirstCycle, tEnvModelPort.sSigHeader);
    updateSignalHeader(isFirstCycle, envModelPort.sSigHeader);
    updateSignalHeader(isFirstCycle, tParkingBoxPort.sSigHeader);
    updateSignalHeader(isFirstCycle, parkingBoxPort.sSigHeader);
    updateSignalHeader(isFirstCycle, mCollEnvModelPort.sSigHeader);

    if (isFirstCycle) {
        for (int iSens = 0; iSens < ObjectSensorCount; ++iSens) {
            ObjectSensor_Enable(iSens);
        }

        tEnvModelPort.resetOriginResult.resetCounter_nu = 0u;
        tEnvModelPort.resetOriginResult.originTransformation.x_dir = 0.0f;
        tEnvModelPort.resetOriginResult.originTransformation.y_dir = 0.0f;
        tEnvModelPort.resetOriginResult.originTransformation.yaw_rad = 0.0f;
        //set the tEnvModelPort.dynamicObjects.measurementState_nu to MAX_NUM_MEASUREMENT_STATES only in first cycle (default value is ObjMeasurementState::MEAS_STATE_NEW) keep the previous implementation
        //used to can determine the first time when the dynamic Objects is detected
        for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_DYN_OBJECTS_NU; i++) {
            tEnvModelPort.dynamicObjects[i].existenceProb_perc = 0u;
            tEnvModelPort.dynamicObjects[i].measurementState_nu = si::ObjMeasurementState::MAX_NUM_MEASUREMENT_STATES;
            tEnvModelPort.dynamicObjects[i].objShape_m.actualSize = 0U;
        }

        //Initalize data in parkingBoxPort
        for (auto& parkingBox : tParkingBoxPort.parkingBoxes) {
            parkingBox.parkingScenario_nu = si::ParkingScenarioTypes::MAX_NUM_PARKING_SCENARIO_TYPES;
            parkingBox.slotCoordinates_m.actualSize = 0U;
            for (auto& delimiter : parkingBox.delimiters) {
                delimiter.delimiterType_nu = si::DelimiterTypes::MAX_NUM_PARKING_BOX_DELIMITER_TYPES;
                delimiter.delimitingSide_nu = si::RelativeLocationToParkingBox::MAX_NUM_PARKING_BOX_EDGE_TYPES;
                delimiter.virtLineIdx_nu = ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_VIRTUAL_LINES_NU;
            }
            for (auto& virtualLine : parkingBox.virtualLines) {
                virtualLine.virtLineVertices_m.actualSize = 0U;
            }
        }

        //Initialize data in apEnvModelPort
        for (auto& parkingSpaceMarking : tEnvModelPort.parkingSpaceMarkings) {
            parkingSpaceMarking.type_nu = si::ParkingLineType::PLT_MAX_NUM_COLORS;
            parkingSpaceMarking.pos_m.actualSize = 0U;
        }
        for (auto& dynamicObject : tEnvModelPort.dynamicObjects) {
            dynamicObject.objClass_nu = si::DynamicObjectClass::DYN_OBJ_MAX_NUM_TYPES;
            dynamicObject.measurementState_nu = si::ObjMeasurementState::MAX_NUM_MEASUREMENT_STATES;
        }
        for (auto& staticObject : tEnvModelPort.staticObjects) {
            staticObject.refObjClass_nu = si::StaticObjectClass::STAT_OBJ_MAX_NUM_TYPES;
            staticObject.measurementPrinciple_nu = si::MeasurementPrinciples::MPT_MAX_NUM;
            staticObject.objTrend_nu = si::ObjectTrend::OBJ_TRND_MAX_NUM;
            staticObject.objHeightClass_nu = si::StaticObjHeigthType::MAX_NUM_HEIGHT_TYPES;
            staticObject.existenceProb_perc = 0U;
            staticObject.objShape_m.actualSize = 0U;
        }

        for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_SPACE_MARKINGS_NU; i++) {
            //tEnvModelPort.parkingSpaceMarkings[i].existenceProb_perc = 0;
            /*TODO read from line sensor*/
        }
        for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_LANE_BOUNDARIES_NU; i++) {
            tEnvModelPort.roadDescription.laneBoundaries[i].laneShape = si::createLaneShapeSerializable();
            tEnvModelPort.roadDescription.laneBoundaries[i].laneShape.actualSize = 0U;
            tEnvModelPort.roadDescription.laneBoundaries[i].estimationState_nu = si::LaneBoundaryEstimationState::LB_INVALID;
        }
        for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_LANE_BOUNDARIES_NU - 1; i++) {
            tEnvModelPort.roadDescription.lanes[i].laneValid_nu = false;
        }
        tEnvModelPort.roadDescription.lanes[0].laneValid_nu = true;
        tEnvModelPort.roadDescription.lanes[0].isOneWayLane_nu = true;
    }

    char *USSsensorName[12] = { "USS_SLF","USS_FL","USS_FML","USS_FMR","USS_FR","USS_SRF","USS_SRR","USS_RR","USS_RMR","USS_RML","USS_RL","USS_SLR" };
    const float32_t offsetX{ (float)Car.ConBdy1.v_1[0] * mLatencyTime_ms / 1000.0f + mLongOffset_m };
    const float32_t offsetY{ (float)Car.ConBdy1.v_1[1] * mLatencyTime_ms / 1000.0f + mLatOffset_m };

    // Assert that the object sensor data are not older than 1 ms (e.g. one CarMaker cycle delayed)
    for (int iSens = 0; iSens < ObjectSensorCount; ++iSens) {
        assert(std::abs(SimCore.Time - ObjectSensor[iSens].TimeStamp) < 0.0011);
    }

    for (unsigned int i = 0; i < (unsigned int)Traffic.nObjs; i++) {
        const tTrafficObj* trafficObj = Traffic_GetByTrfId(i);
        //check if the Object class is Pedestian or Bicycler or if the absolute value of velocity (on X axes or Y axes) is bigger than MIN_FLT_DIVISOR(1e-6)
        if ((strcmp(trafficObj->Cfg.Name, "Odo") != 0) &&
            ((trafficObj->Cfg.MotionKind == tMotionKind::MotionKind_2Wheel || trafficObj->Cfg.RCSClass == tRCSClass::RCS_Bicycle) || trafficObj->Cfg.MotionKind == tMotionKind::MotionKind_4Wheel ||
            (trafficObj->Cfg.MotionKind == tMotionKind::MotionKind_Pedestrian || trafficObj->Cfg.RCSClass == tRCSClass::RCS_Pedestrian) ||
                ((fabs(float32_t(trafficObj->v_0[0])) >= LSM_GEOML::MIN_FLT_DIVISOR || fabs(float32_t(trafficObj->v_0[1])) >= LSM_GEOML::MIN_FLT_DIVISOR))))
        {
            bool objectDetected = false;
            for each(char *sensorName in USSsensorName) {
                tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);
                //detected[0][idSensor] = ObjectUSSSensor[idSensor]->dtct;
                if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false)) {
                    //check if the CarMaker traffic object was previous cycle time a static object
                    if (mTrafficObjects[trafficObj->Cfg.Id].type == objectClass::StaticObject) {
                        for (uint8_t idxCheck = mTrafficObjects[trafficObj->Cfg.Id].SWIdx; idxCheck < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU; idxCheck++) {
                            if (tEnvModelPort.staticObjects[idxCheck].existenceProb_perc != 100) {
                                //if existence probability of the staticObject is not 100 , shift up in staticObject the elements and reset the last object with existence Prob 100
                                shiftUpStaticObjectElements(mTrafficObjects[trafficObj->Cfg.Id].SWIdx, idxCheck);
                                break;
                            }
                        }
                    }
                    //check if the traffic object was saved in dynamicObject port
                    if (mTrafficObjects[trafficObj->Cfg.Id].type != objectClass::DynamicObject) {
                        mTrafficObjects[trafficObj->Cfg.Id].SWIdx = mDynObjIdx++;
                        mTrafficObjects[trafficObj->Cfg.Id].type = objectClass::DynamicObject;
                    }
                    uint8_t objIdx = mTrafficObjects[trafficObj->Cfg.Id].SWIdx;

                    objectDetected = true;
                    tEnvModelPort.dynamicObjects[objIdx].refObjID_nu = objIdx;
                    tEnvModelPort.dynamicObjects[objIdx].existenceProb_perc = 100u;
                    tEnvModelPort.dynamicObjects[objIdx].classConfidence_perc = 100u;

                    switch (trafficObj->Cfg.MotionKind) {
                    case tMotionKind::MotionKind_4Wheel:
                        tEnvModelPort.dynamicObjects[objIdx].objClass_nu = si::DynamicObjectClass::DYN_OBJ_VEHICLE;
                        break;
                    case tMotionKind::MotionKind_2Wheel:
                        tEnvModelPort.dynamicObjects[objIdx].objClass_nu = si::DynamicObjectClass::DYN_OBJ_BICYCLE;
                        break;
                    case tMotionKind::MotionKind_Pedestrian:
                        tEnvModelPort.dynamicObjects[objIdx].objClass_nu = si::DynamicObjectClass::DYN_OBJ_PEDESTRIAN;
                        break;
                    default:
                        tEnvModelPort.dynamicObjects[objIdx].objClass_nu = si::DynamicObjectClass::DYN_OBJ_UNCLASSIFIED_STRUCTURE;
                        break;
                    }
                    //dummy variable (used for calculateObjectContourPoints function)
                    bool needToBeSplited = false;

                    //to check mirroring
                    tEnvModelPort.dynamicObjects[objIdx].objShape_m.actualSize = calculateObjectContourPoints(trafficObj,
                        trafficContour2D_t[i],
                        offsetX,
                        offsetY,
                        objectType::Rect4Points,
                        tEnvModelPort.dynamicObjects[objIdx].objShape_m.array,
                        false,
                        needToBeSplited);

                    float32_t cYaw = cos(tEnvModelPort.egoVehiclePoseForAP.yaw_rad);
                    float32_t sYaw = sin(tEnvModelPort.egoVehiclePoseForAP.yaw_rad);
                    float32_t xVel_mps = static_cast<float32_t>(trafficObj->v_0[0]);
                    float32_t yVel_mps = static_cast<float32_t>(trafficObj->v_0[1]);
                    float32_t xAccel_mps2 = static_cast<float32_t>(trafficObj->a_0[0]);
                    float32_t yAccel_mps2 = static_cast<float32_t>(trafficObj->a_0[1]);

                    tEnvModelPort.dynamicObjects[objIdx].vel_mps = { cYaw * xVel_mps + sYaw * yVel_mps,
                        -sYaw * xVel_mps + cYaw * yVel_mps };
                    tEnvModelPort.dynamicObjects[objIdx].accel_mps2 = { cYaw * xAccel_mps2 + sYaw * yAccel_mps2,
                        -sYaw * xAccel_mps2 + cYaw * yAccel_mps2 };

                    tEnvModelPort.dynamicObjects[objIdx].headingAngle_rad = static_cast<float32_t>(trafficObj->r_zyx[2]);

                    tEnvModelPort.dynamicObjects[objIdx].measurementState_nu = si::ObjMeasurementState::MEAS_STATE_MEASURED;

                    //set the current timestamp
                    mTrafficObjects[trafficObj->Cfg.Id].lastDynamicObjectDetectionTimestamp_us = tEnvModelPort.sSigHeader.uiTimeStamp;
                    //to skipp the for each loop when enableScanning_nu is not enable
                    if (mEnableScanning_nu == false) {
                        break;
                    }
                }
            }
            //check if object was detected by sensors during the simulation (if was detected once the measurementState_nu should not be MAX_NUM_MEASUREMENT_STATES)
            //it is also check if in the current cycle time the object was detected (objectDetected is false if is not detected in the current cycle time)
            if (objectDetected == false &&
                tEnvModelPort.dynamicObjects[mTrafficObjects[trafficObj->Cfg.Id].SWIdx].measurementState_nu != si::ObjMeasurementState::MAX_NUM_MEASUREMENT_STATES &&
                mTrafficObjects[trafficObj->Cfg.Id].lastDynamicObjectDetectionTimestamp_us != 0u) {
                //calculate the difference between the last time from the last detection by sensors of traffic object to current time
                uint64_t differenceTimeFromPreviousDetection = tEnvModelPort.sSigHeader.uiTimeStamp - mTrafficObjects[trafficObj->Cfg.Id].lastDynamicObjectDetectionTimestamp_us;
                //check if LSCA_PREDICTED_TO_DELETED_TIME_US is not passed from the last detection of object by sensors (if is not passed -> Predicted, if is passed -> Deleted)
                if (differenceTimeFromPreviousDetection <= (LSCA_PREDICTED_TO_DELETED_TIME_US)) {
                    tEnvModelPort.dynamicObjects[mTrafficObjects[trafficObj->Cfg.Id].SWIdx].measurementState_nu = si::ObjMeasurementState::MEAS_STATE_PREDICTED;
                }
                else {
                    //if LSCA_PREDICTED_TO_DELETED_TIME_US are passed from the last detection, element from Dynamic Object are reset/deleted
                    resetDynamicObject(mTrafficObjects[trafficObj->Cfg.Id].SWIdx);
                    //to can "add" next dyn obj
                    mDynObjIdx = 0;
                }
            }
        }
        /*Search for parking Boxes, define delimiters of parking boxes, fill the parkingBoxPort */
        else if ((trafficObj->Cfg.h < MIN_OBJ_HEIGHT_M) && (trafficObj->Cfg.Name[0] == 'P')) {
            for each(char *sensorName in USSsensorName) {
                tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);
                //detected[0][idSensor] = ObjectUSSSensor[idSensor]->dtct;
                if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false))
                {
					if (mNumParkingBoxes >= ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU) {
						break;
					}
                    //check if the traffic object was saved in ParkingBox port
                    if (mTrafficObjects[trafficObj->Cfg.Id].type != objectClass::ParkingBox) {
                        mTrafficObjects[trafficObj->Cfg.Id].SWIdx = mNumParkingBoxes++;
                        mTrafficObjects[trafficObj->Cfg.Id].type = objectClass::ParkingBox;
                    }
                    const uint8_t PBIdx = mTrafficObjects[trafficObj->Cfg.Id].SWIdx;

                    // set the ego-vehicle orientation when the parking box is detected for the first time
                    if (100u != tParkingBoxPort.parkingBoxes[PBIdx].existenceProb_perc) {
                        mEgoVehicleOrientationDuringScanning_rad[PBIdx] = static_cast<float32_t>(Car.Fr1.r_zyx[2]);
                    }
                    tParkingBoxPort.parkingBoxes[PBIdx].existenceProb_perc = 100u;
                    tParkingBoxPort.parkingBoxes[PBIdx].parkingBoxID_nu = (uint16_t)PBIdx;

                    //for the case when the number of 2d CarMaker Contour is bigger than 16
                    bool needToBeSplited = false;
                    //to calculate the 4 points edge of Parking Box
                    tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m.actualSize = calculateObjectContourPoints(trafficObj,
                        trafficContour2D_t[i],
                        offsetX,
                        offsetY,
                        (parkingOnLeftSide_nu[PBIdx] == false) ? objectType::ParkingBoxRightSide : objectType::ParkingBoxLeftSide,
                        tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m.array,
                        false,
                        needToBeSplited);

                    /*for (unsigned int k = 3; k < MFEnvModelData::MAX_NUM_PARKING_BOX_DELIMITERS; k++) {
                    tParkingBoxPort.parkingBoxes[PBIdx].delimiters[k].valid_nu = false;
                    }*/

                    tParkingBoxPort.parkingBoxes[PBIdx].parkingScenario_nu =
                        determineParkingScenarioType(tParkingBoxPort.parkingBoxes[PBIdx], trafficObj, mEgoVehicleOrientationDuringScanning_rad[PBIdx]);

                    // HACK to correct parking box point order for GarageParking
                    //
                    //         0----------3
                    //         |          |
                    //   Ego>  |          |
                    //         |          |
                    //         1----------2
                    if (si::ParkingScenarioTypes::GARAGE_PARKING == tParkingBoxPort.parkingBoxes[PBIdx].parkingScenario_nu)
                    {
                        const si::SlotCoordinates_mSerializable tmp = tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m;
                        //Road side left
                        tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m.array[0] = tmp.array[1];
                        tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m.array[1] = tmp.array[2];
                        tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m.array[2] = tmp.array[3];
                        tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m.array[3] = tmp.array[0];
                    }

                    //update the delimiter zones of the parking box
                    mDelimiterManager.updateDelimiterZones(PBIdx, convert(tParkingBoxPort.parkingBoxes[PBIdx].slotCoordinates_m));

                    // to add coments
                    //to skipp the for each loop when enableScanning_nu is not enable
                    if (mEnableScanning_nu == false) {
                        break;
                    }
                }
            }
        }
        /*Search for Vehicle Traffic object, fill the envModelPort.staticObjects*/
        //check if the velocity is smaller than MIN_FLT_DIVISOR(1e-6)
        else if (trafficObj->Cfg.h > MIN_OBJ_HEIGHT_M &&
            (strcmp(trafficObj->Cfg.Name, "Odo") != 0) &&
            (trafficObj->Cfg.Name[0] == 'T') &&
            (fabs(float32_t(trafficObj->v_0[0])) <= LSM_GEOML::MIN_FLT_DIVISOR || fabs(float32_t(trafficObj->v_0[1])) <= LSM_GEOML::MIN_FLT_DIVISOR)) { //T=Vehicle Object
                //define a variable to check if the previous object type was Dynamic and now should be Static
            bool shouldRemainDynamicObject = false;
            //check if CarMaker traffic Object was previous a dynamic Object
            if (mTrafficObjects[trafficObj->Cfg.Id].type == objectClass::DynamicObject) {
                //calculate the difference between the last time from the last detection by sensors of traffic object to current time
                uint64_t differenceTimeFromPreviousDetection = tEnvModelPort.sSigHeader.uiTimeStamp - mTrafficObjects[trafficObj->Cfg.Id].lastDynamicObjectDetectionTimestamp_us;
                //check if LSCA_DYNAMIC_TO_STATIC_TIME_US are passed from the previous time where the traffic object was detected as a Dynamic Object
                if (differenceTimeFromPreviousDetection > LSCA_DYNAMIC_TO_STATIC_TIME_US) {
                    //keep until the interface will be changed and number of dynamic object(MAX_NUM_DYN_OBJECTS) is not 1
#pragma warning( push )
#pragma warning( disable : 4127 )
                    if (ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_DYN_OBJECTS_NU == 1) {
                        //reset the dynamicObject strucure
                        resetDynamicObject(mTrafficObjects[trafficObj->Cfg.Id].SWIdx);
                        mDynObjIdx = 0;
                        mTrafficObjects[trafficObj->Cfg.Id].type = objectClass::NotDefine;
                        //if LSCA_DYNAMIC_TO_STATIC_TIME_US are passed, change the object class from Dynamic to Static
                        shouldRemainDynamicObject = false;
                    }
#pragma warning( pop )
                    else {
                        for (uint8_t idxCheck = mTrafficObjects[trafficObj->Cfg.Id].SWIdx; idxCheck < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_DYN_OBJECTS_NU; idxCheck++) {
                            if (tEnvModelPort.dynamicObjects[idxCheck].existenceProb_perc != 100) {
                                //if existence probability of the staticObject is not 100 , shift up in staticObject the elements and reset the last object with existence Prob 100
                                shiftUpDynamicObjectElements(mTrafficObjects[trafficObj->Cfg.Id].SWIdx, idxCheck);
                                mDynObjIdx = 0;
                                mTrafficObjects[trafficObj->Cfg.Id].type = objectClass::NotDefine;
                                break;
                            }
                        }
                    }
                }
                else {
                    //if LSCA_DYNAMIC_TO_STATIC_TIME_US are NOT passed, the object class should not changed
                    shouldRemainDynamicObject = true;
                }
            }
            if (shouldRemainDynamicObject == false) {
                for each(char *sensorName in USSsensorName) {
                    tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);
                    //detected[0][idSensor] = ObjectUSSSensor[idSensor]->dtct;
                    if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false)) {
                        //staticStructureCall
                        EnvironmentPerception::updateStaticObject(trafficObj, objectType::Other, trafficContour2D_t,
                            offsetX, offsetY, si::StaticObjectClass::STAT_OBJ_VEHICLE, inflationLength_m, parkingOnLeftSide_nu);
                        //to skipp the for each loop when enableScanning_nu is not enable
                        if (mEnableScanning_nu == false) {
                            break;
                        }
                    }
                }
            }
        }
        /*Search for circle Obststacle Traffic object, fill the envModelPort.staticObjects*/
        else if (strstr(trafficObj->Cfg.Name, "Obs") && strstr(trafficObj->Cfg.Info, "circle")) {//Obsatcle - circle
            for each(char *sensorName in USSsensorName) {
                tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);
                if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false))
                {
                    EnvironmentPerception::updateStaticObject(trafficObj, objectType::CircleObstacle, trafficContour2D_t,
                        offsetX, offsetY, si::StaticObjectClass::STAT_OBJ_UNCLASSIFIED_STRUCTURE, inflationLength_m, parkingOnLeftSide_nu);
                    //to skipp the for each loop when enableScanning_nu is not enable
                    if (mEnableScanning_nu == false) {
                        break;
                    }
                }
            }
        }
        /*Search for rectangular Obststacle, Wall or guardrail limiter Traffic object, fill the envModelPort.staticObjects*/
        else if ((strstr(trafficObj->Cfg.Name, "Obs") && strstr(trafficObj->Cfg.Info, "rect"))
            || (strstr(trafficObj->Cfg.Name, "Lim") && (strstr(trafficObj->Cfg.Info, "wall") || strstr(trafficObj->Cfg.Info, "guard")))) { // Obstacle - rectangular
            for each(char *sensorName in USSsensorName) {
                tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);
                //detected[0][idSensor] = ObjectUSSSensor[idSensor]->dtct;
                if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false))
                {
                    EnvironmentPerception::updateStaticObject(trafficObj, objectType::Other, trafficContour2D_t,
                        offsetX, offsetY, si::StaticObjectClass::STAT_OBJ_UNCLASSIFIED_STRUCTURE, inflationLength_m, parkingOnLeftSide_nu);
                    //to skipp the for each loop when enableScanning_nu is not enable
                    if (mEnableScanning_nu == false) {
                        break;
                    }
                }
            }
        }
        /*Search for Curbstone limiter, fill the envModelPort.staticObjects*/
        else if (strstr(trafficObj->Cfg.Name, "Lim") && strstr(trafficObj->Cfg.Info, "curb_")) { //Limiter- Curbstone
            for each(char *sensorName in USSsensorName) {
                tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);

                if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false))
                {
                    EnvironmentPerception::updateStaticObject(trafficObj, objectType::Other, trafficContour2D_t,
                        offsetX, offsetY, si::StaticObjectClass::STAT_OBJ_CURB, inflationLength_m, parkingOnLeftSide_nu);
                }
                //to skipp the for each loop when enableScanning_nu is not enable
                if (mEnableScanning_nu == false) {
                    break;
                }
            }
        }
		/*Search for Wheel stoper limiter, fill the envModelPort.staticObjects*/
		else if (strstr(trafficObj->Cfg.Name, "Whs") && strstr(trafficObj->Cfg.Info, "Wheelstopper")) { //Limiter- Wheel Stopper
			for each(char *sensorName in USSsensorName) {
				tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);

				if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false))
				{
					EnvironmentPerception::updateStaticObject(trafficObj, objectType::Other, trafficContour2D_t,
						offsetX, offsetY, si::StaticObjectClass::STAT_OBJ_WHEEL_STOPPER, inflationLength_m, parkingOnLeftSide_nu);
				}
				//to skipp the for each loop when enableScanning_nu is not enable
				if (mEnableScanning_nu == false) {
					break;
				}
			}
		}
        /*Search for road line marking object, fill the envModelPort.roadDescription.laneBoundaries*/
        else if (strstr(trafficObj->Cfg.Name, "Lim") && strstr(trafficObj->Cfg.Info, "lim_Rd_P - line"))
        {
            for each(char *sensorName in USSsensorName) {
                tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);
                //detected[0][idSensor] = ObjectUSSSensor[idSensor]->dtct;
                if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false))
                {
                    //check if the traffic object was saved in laneBoundary port
                    if (mTrafficObjects[trafficObj->Cfg.Id].type != objectClass::LaneBoundary) {
                        mTrafficObjects[trafficObj->Cfg.Id].SWIdx = mLaneBoundIdx++;
                        mTrafficObjects[trafficObj->Cfg.Id].type = objectClass::LaneBoundary;
                    }
                    uint8_t objIdx = mTrafficObjects[trafficObj->Cfg.Id].SWIdx;

                    tEnvModelPort.roadDescription.laneBoundaries[objIdx].estimationState_nu = si::LaneBoundaryEstimationState::LB_DETECTED;
                    tEnvModelPort.roadDescription.laneBoundaries[objIdx].type_nu = si::LaneBoundaryType::LB_UNCLASSIFIED; // to be implemented (A.Bubu)
                    //for the case when the number of 2d CarMaker Contour is bigger than 16
                    bool needToBeSplited = false;

                    tEnvModelPort.roadDescription.laneBoundaries[objIdx].laneShape.actualSize = calculateObjectContourPoints(trafficObj,
                        trafficContour2D_t[i],
                        offsetX,
                        offsetY,
                        objectType::Other,
                        tEnvModelPort.roadDescription.laneBoundaries[objIdx].laneShape.array,
                        false,
                        needToBeSplited);

                    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_LANE_BOUND_PTS_NU> objectPolygon;
                    for (unsigned idxObjVer = 0; idxObjVer < tEnvModelPort.roadDescription.laneBoundaries[objIdx].laneShape.actualSize; ++idxObjVer)
                    {
                        objectPolygon.append(tEnvModelPort.roadDescription.laneBoundaries[objIdx].laneShape.array[idxObjVer]);
                    }

                    for (uint8_t iPB = 0U; iPB < mNumParkingBoxes; ++iPB) {
                        //check the delimiter only if the PB exist
                        if (tParkingBoxPort.parkingBoxes[iPB].existenceProb_perc == 100u)
                        {
                            //check if the trafficObject(road space marking) is a delimiter of PB
                            const auto delimiterType_nu = mDelimiterManager.determineDelimiterType(iPB, objectPolygon,
                                trafficObj->Cfg.Info, tParkingBoxPort.parkingBoxes[iPB].parkingScenario_nu);
                            if (delimiterType_nu != si::RelativeLocationToParkingBox::UNDEFINED_EDGE &&
                                delimiterType_nu != si::RelativeLocationToParkingBox::MAX_NUM_PARKING_BOX_EDGE_TYPES)
                            {
                                fillDelimiterOfParkingBox(objIdx,
                                    si::DelimiterTypes::LANE_BOUNDARY,
                                    delimiterType_nu,
                                    &(tParkingBoxPort.parkingBoxes[iPB].delimiters[0]));//TODO better solution
                            }
                        }
                    }

                    //to skipp the for each loop when enableScanning_nu is not enable
                    if (mEnableScanning_nu == false) {
                        break;
                    }
                }
            }
        }
        /*Search for parking line marking object, fill the envModelPort.parkingSpaceMarkings*/
        else if (strstr(trafficObj->Cfg.Name, "Lim") && strstr(trafficObj->Cfg.Info, "- line")
            // exclude external and roadside lines (named Lim??e and Lim??r, respectively) for TestRuns with double lines (US_Scenarios)
            && !strstr(trafficObj->Cfg.Name, "e") && !strstr(trafficObj->Cfg.Name, "r"))
        {
            for each(char *sensorName in USSsensorName) {
                tObjectSensorObj* ObjectUSSSensor = ObjectSensor_GetObjectForName(sensorName, trafficObj->Cfg.Name);
                //detected[0][idSensor] = ObjectUSSSensor[idSensor]->dtct;
                if (ObjectUSSSensor->dtct || (mEnableScanning_nu == false))
                {
                    //check if the traffic object was saved in ParkingSpaceMarking port
                    if (mTrafficObjects[trafficObj->Cfg.Id].type != objectClass::ParkingSpaceMarking) {
                        mTrafficObjects[trafficObj->Cfg.Id].SWIdx = mParkingSMIdx++;
                        mTrafficObjects[trafficObj->Cfg.Id].type = objectClass::ParkingSpaceMarking;
                    }
                    uint8_t objIdx = mTrafficObjects[trafficObj->Cfg.Id].SWIdx;

                    //calculateObjectContourPoints(trafficObj, trafficContour2D_t[i], offsetX, offsetY, tEnvModelPort.parkingSpaceMarkings[mParkingSMIdx].pos_m, 2, objectType::LaneMarking);
                    //for the case when the number of 2d CarMaker Contour is bigger than 16
                    bool needToBeSplited = false;
                    tEnvModelPort.parkingSpaceMarkings[objIdx].pos_m.actualSize = calculateObjectContourPoints(trafficObj,
                        trafficContour2D_t[i],
                        offsetX,
                        offsetY,
                        objectType::LaneMarking,
                        tEnvModelPort.parkingSpaceMarkings[objIdx].pos_m.array,
                        false,
                        needToBeSplited);

                    LSM_GEOML::Polygon2D<2u> objectPolygon;
                    objectPolygon.append(tEnvModelPort.parkingSpaceMarkings[objIdx].pos_m.array[0]);
                    objectPolygon.append(tEnvModelPort.parkingSpaceMarkings[objIdx].pos_m.array[1]);

                    tEnvModelPort.parkingSpaceMarkings[objIdx].existenceProb_perc = 100u;
                    tEnvModelPort.parkingSpaceMarkings[objIdx].width_m = float32_t(trafficObj->Cfg.w);
                    tEnvModelPort.parkingSpaceMarkings[objIdx].type_nu = si::ParkingLineType::PLT_WHITE;

                    for (uint8_t iPB = 0U; iPB < mNumParkingBoxes; ++iPB) {
                        //check the delimiter only if the PB exist
                        if (tParkingBoxPort.parkingBoxes[iPB].existenceProb_perc == 100u)
                        {
                            //check if the trafficObject(parkingSpace Mark) is a delimiter of PB
                            const auto delimiterType_nu = mDelimiterManager.determineDelimiterType(iPB, objectPolygon,
                                trafficObj->Cfg.Info, tParkingBoxPort.parkingBoxes[iPB].parkingScenario_nu);
                            if (delimiterType_nu != si::RelativeLocationToParkingBox::UNDEFINED_EDGE &&
                                delimiterType_nu != si::RelativeLocationToParkingBox::MAX_NUM_PARKING_BOX_EDGE_TYPES)
                            {
                                fillDelimiterOfParkingBox(objIdx,
                                    si::DelimiterTypes::PARKING_SPACE_MARKING,
                                    delimiterType_nu,
                                    &(tParkingBoxPort.parkingBoxes[iPB].delimiters[0]));//TODO better solution
                            }
                        }
                    }

                    //to skipp the for each loop when enableScanning_nu is not enable
                    if (mEnableScanning_nu == false) {
                        break;
                    }
                }
            }
        }
    }

    /* TO DEBUG
    if (SimCore.Time > 15)
    int b = 6;
    */
    //
    uint8_t countNrOfStaticObj = 0U;
    uint8_t countNrOfDynObj = 0U;
    uint8_t countNrOfParkSpaceMarking = 0U;
    uint8_t countNrOfPBObj = 0U;
    for (uint8_t i = 0U; i < mStaticObjIdx; i++)
        if (tEnvModelPort.staticObjects[i].existenceProb_perc == 100u)
            countNrOfStaticObj++;
    for (uint8_t i = 0U; i < mDynObjIdx; i++)
        if (tEnvModelPort.dynamicObjects[i].existenceProb_perc == 100u)
            countNrOfDynObj++;
    for (uint8_t i = 0U; i < mParkingSMIdx; i++)
        if (tEnvModelPort.parkingSpaceMarkings[i].existenceProb_perc == 100u)
            countNrOfParkSpaceMarking++;
    for (uint8_t i = 0U; i < mNumParkingBoxes; i++) {
        if (tParkingBoxPort.parkingBoxes[i].existenceProb_perc > 0U) countNrOfPBObj++;
        uint8_t countNrOfDelimiters = 0U;
        for (auto& delimiter : tParkingBoxPort.parkingBoxes[i].delimiters) {
            if (delimiter.delimitingSide_nu != si::RelativeLocationToParkingBox::MAX_NUM_PARKING_BOX_EDGE_TYPES) countNrOfDelimiters++;
        }
        tParkingBoxPort.parkingBoxes[i].numValidDelimiters_nu = countNrOfDelimiters;
        uint8_t countNrOfVirtualLines = 0U;
        for (auto& virtualLine : tParkingBoxPort.parkingBoxes[i].virtualLines) {
            if (virtualLine.virtLineVertices_m.actualSize) countNrOfVirtualLines++;
        }
        tParkingBoxPort.parkingBoxes[i].numVirtualLines_nu = countNrOfVirtualLines;

    }

    tEnvModelPort.numberOfStaticObjects_u8 = countNrOfStaticObj;
    tEnvModelPort.numberOfDynamicObjects_u8 = countNrOfDynObj;
    tEnvModelPort.numberOfParkMarkings_u8 = countNrOfParkSpaceMarking;
    tParkingBoxPort.numValidParkingBoxes_nu = countNrOfPBObj;

    // Adjust parking box size according to virtual lines (only after filling of tParkingBoxPort is finished).
    // This is needed since otherwise the parking box, which is statically defined in the TestRun, is not compatible to the virtual lines,
    // that are dynamically derived from the inflated static objects and adjusted to the ego-vehicle scanning direction.
    for (unsigned int iPB = 0; iPB < mNumParkingBoxes; iPB++) {
        if (tParkingBoxPort.parkingBoxes[iPB].existenceProb_perc == 100u && tParkingBoxPort.parkingBoxes[iPB].numVirtualLines_nu > 0U &&
            tParkingBoxPort.parkingBoxes[iPB].parkingScenario_nu != si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_BACK &&
            tParkingBoxPort.parkingBoxes[iPB].parkingScenario_nu != si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_FRONT) {
            adjustParkingBoxSizeToVirtualLines(tParkingBoxPort.parkingBoxes[iPB]);
        }
    }

    //// Save path unitl EM provides it to pathBeforeFuncActivation
    //float minDistPoses_m = 5.0f;
    //LSM_GEOML::Pose& currentEgoPose = mEgoMotionPort.egoVehiclePose;
    //cml::Vec2Df deltaPos{ 0.0F, 0.0F };
    //if (pathBeforeActIndex) {
    // deltaPos = currentEgoPose.pos - tEnvModelPort.pathBeforeFuncActivation.poses[pathBeforeActIndex - 1].pos ;

    //}
    //if (pathBeforeActIndex == 0u) {
    //    tEnvModelPort.pathBeforeFuncActivation.poses[pathBeforeActIndex] = currentEgoPose;
    //    pathBeforeActIndex++;
    //} else if (deltaPos.norm() >= minDistPoses_m) {
    //    // Move array elements - 1 index to have a free field at the end
    //    if (pathBeforeActIndex == MFEnvModelData::MAX_NUM_POSES_BEFORE_ACTIVATION) {
    //        for (uint8_t i = 0u; i < MFEnvModelData::MAX_NUM_POSES_BEFORE_ACTIVATION - 1u; i++) {
    //            tEnvModelPort.pathBeforeFuncActivation.poses[i] = tEnvModelPort.pathBeforeFuncActivation.poses[i + 1u];
    //        }
    //        pathBeforeActIndex -= 1u;
    //    }
    //    tEnvModelPort.pathBeforeFuncActivation.poses[pathBeforeActIndex] = currentEgoPose;
    //    pathBeforeActIndex++;
    //}
    //if (overwritePathBeforeFuncActivation_nu) {
    //    tEnvModelPort.pathBeforeFuncActivation.poses[0u].pos[0u] = pathBeforeFuncActivation0PoseX_m;
    //    tEnvModelPort.pathBeforeFuncActivation.poses[0u].pos[1u] = pathBeforeFuncActivation0PoseY_m;
    //    tEnvModelPort.pathBeforeFuncActivation.poses[0u].yaw_rad = pathBeforeFuncActivation0PoseYaw_rad;
    //    tEnvModelPort.pathBeforeFuncActivation.numValidPoses_nu = 1u;
    //    for (uint8_t i = 1u; i < MFEnvModelData::MAX_NUM_POSES_BEFORE_ACTIVATION; i++) {
    //        tEnvModelPort.pathBeforeFuncActivation.poses[i].pos[0u] = 0.0f;
    //        tEnvModelPort.pathBeforeFuncActivation.poses[i].pos[1u] = 0.0f;
    //        tEnvModelPort.pathBeforeFuncActivation.poses[i].yaw_rad = 0.0f;
    //    }
    //}

    if (mLatencyTime_ms > 0.0f) {
        qEnvModelPort.push(tEnvModelPort);
        if (std::round(mLatencyTime_ms / SI_HIGH_CYCLE_TIME_MS) < qEnvModelPort.size()) { /*queue is "full" --> copy delayed value*/
            envModelPort = qEnvModelPort.front();
            qEnvModelPort.pop();
        }
        else if (qEnvModelPort.size() == 1) { /*first cycle--> copy valid initial values*/
            envModelPort = qEnvModelPort.front();
        }
    }
    else {
        envModelPort = tEnvModelPort;
    }

    if (mLatencyTime_ms > 0.0f) {
        qParkingBoxPort.push(tParkingBoxPort);
        if (std::round(mLatencyTime_ms / SI_HIGH_CYCLE_TIME_MS) < qParkingBoxPort.size()) { /*queue is "full" --> copy delayed value*/
            parkingBoxPort = qParkingBoxPort.front();
            qParkingBoxPort.pop();
        }
        else if (qParkingBoxPort.size() == 1) { /*first cycle--> copy valid initial values*/
            parkingBoxPort = qParkingBoxPort.front();
        }
    }
    else {
        parkingBoxPort = tParkingBoxPort;
    }
}

void EnvironmentPerception::updatePerceptionAvailabilityPort(const bool isFirstCycle) {
    updateSignalHeader(isFirstCycle, mPerceptionAvailabilityPort.sSigHeader);
    if (isFirstCycle) {
        for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_SV_CAMS_NU; i++) {
            mPerceptionAvailabilityPort.statusSVCams_nu[i] = si::AvailabilityStatus::ITEM_AVAILABLE;
        }
        for (unsigned int i = 0; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_US_SENSORS_NU; i++) {
            mPerceptionAvailabilityPort.statusUSSensors_nu[i] = si::AvailabilityStatus::ITEM_AVAILABLE;
        }
        mPerceptionAvailabilityPort.statusEnvModel_nu = si::AvailabilityStatus::ITEM_AVAILABLE;
    }
}

void EnvironmentPerception::updateEnvModelData(const unsigned char vedodoActive_nu,
    const bool parkingOnLeftSide_nu[],
    const float MIN_OBJ_HEIGHT_M,
    const int pathBeforeActIndex,
    const uint8_t overwritePathBeforeFuncActivation_nu,
    const LSM_GEOML::Pose &pathBeforeFuncActivation0Pose,
    const TrafficContour2D trafficContour2D_t[],
    const float32_t inflationLength_m,
    const bool isFirstCycle,
    const lsm_vedodo::OdoEstimation &odoEstimationPortCM,
    const lsm_vedodo::OdoEstimationOutputPort &odoEstimationOutputPort,
    si::ApEnvModelPort &lEnvModelPort,
    si::ApEnvModelPort &lEnvModelPortCMOrigin,
    si::CollEnvModelPort &collEnvModelPort,
    si::ApParkingBoxPort &lParkingBoxPort,
    si::ApParkingBoxPort &lParkingBoxPortCMOrigin,
    si::PerceptionAvailabilityPort &perceptionAvailabilityPort) {
    //Update AUP core interface
    updateEnvironmentModelAndParkingBox(parkingOnLeftSide_nu,
        MIN_OBJ_HEIGHT_M,
        isFirstCycle,
        pathBeforeActIndex,
        overwritePathBeforeFuncActivation_nu,
        pathBeforeFuncActivation0Pose,
        trafficContour2D_t,
        inflationLength_m);
    lEnvModelPort = getEnvModelPort();
    lEnvModelPortCMOrigin = getEnvModelPort();

    // update egoVehiclePoseForAP only here and not in environmentPerception.updateEnvironmentModelAndParkingBox()
    // in order to avoid latency effect for this quantity
    lEnvModelPort.egoVehiclePoseForAP = getPose(vedodoActive_nu ? odoEstimationOutputPort.odoEstimation : odoEstimationPortCM);
    lEnvModelPortCMOrigin.egoVehiclePoseForAP = lEnvModelPort.egoVehiclePoseForAP;

    // set transformationToOdometry to no transformation
    lEnvModelPort.transformationToOdometry = lsm_geoml::Pose_POD{ 0.0F,0.0F,0.0F };
    lEnvModelPortCMOrigin.transformationToOdometry = lsm_geoml::Pose_POD{ 0.0F,0.0F,0.0F };

    updateCollEnvModel(lEnvModelPort);
    collEnvModelPort = getCollEnvModelPort();

    lParkingBoxPort = getParkingBoxPort();
    lParkingBoxPortCMOrigin = getParkingBoxPort();

    updatePerceptionAvailabilityPort(isFirstCycle);
    perceptionAvailabilityPort = getPerceptionAvailabilityPort();
}

void EnvironmentPerception::transformEnvModelData(const ap_psm::ResetOriginRequestPort &resetOriginRequestPort,
    ap_psm::ResetOriginRequestPort &prevResetOriginRequestPort,
    LSM_GEOML::Pose &lastTransformation,
    LSM_GEOML::Pose &inverseTransformation,
    si::ApEnvModelPort &lEnvModelPort,
    si::ApEnvModelPort &lEnvModelPortCMOrigin,
    si::ApParkingBoxPort &lParkingBoxPort) {
    if (resetOriginRequestPort.resetCounter_nu != prevResetOriginRequestPort.resetCounter_nu) {
        //Store last transformation; TODO: Cover also cases for multiple reset
        if (resetOriginRequestPort.resetCounter_nu > 1) lastTransformation = prevResetOriginRequestPort.transformation;
        //Store previous transformation
        prevResetOriginRequestPort = resetOriginRequestPort;
        switch (prevResetOriginRequestPort.resetOrigin_nu) {
        case ap_psm::ResetOriginType::RRT_NONE:
            //TODO: not expected to have a change in counter but no type
            assert(0);
            break;
        case ap_psm::ResetOriginType::RRT_RESET_PSI:
            prevResetOriginRequestPort.transformation.yaw_rad = lEnvModelPort.egoVehiclePoseForAP.yaw_rad;
            prevResetOriginRequestPort.transformation.x_dir = 0.0F;
            prevResetOriginRequestPort.transformation.y_dir = 0.0F;
            break;
        case ap_psm::ResetOriginType::RRT_RESET_XY:
            prevResetOriginRequestPort.transformation = lEnvModelPort.egoVehiclePoseForAP;
            prevResetOriginRequestPort.transformation.yaw_rad = 0.0F;
            break;
        case ap_psm::ResetOriginType::RRT_RESET_XY_PSI:
            prevResetOriginRequestPort.transformation = lEnvModelPort.egoVehiclePoseForAP;
            break;
        case ap_psm::ResetOriginType::RRT_RESET_CUSTOM:
        default:
            //keep copied origin
            break;
        }
    }
    if (prevResetOriginRequestPort.resetCounter_nu > 0) {
        LSM_GEOML::CoordinateTransformer2D transformToNewCS(prevResetOriginRequestPort.transformation);
        inverseTransformation = transformToNewCS.inverseTransform({ LSM_GEOML::Pose(resetOriginRequestPort.transformation) });
        //transform ego motion
        lEnvModelPort.egoVehiclePoseForAP = convert(transformToNewCS.inverseTransform(lEnvModelPort.egoVehiclePoseForAP));
        lEnvModelPortCMOrigin.egoVehiclePoseForAP = convert(transformToNewCS.inverseTransform(lEnvModelPortCMOrigin.egoVehiclePoseForAP));
        //TODO: egoMotionPort.accel_mps2
        //      egoMotionPort.rearWheelAngle_rad

        LSM_GEOML::CoordinateTransformer2D transformToLastCS(lastTransformation);
        lEnvModelPort.resetOriginResult.originTransformation = convert(transformToLastCS.inverseTransform(prevResetOriginRequestPort.transformation));
        lEnvModelPortCMOrigin.resetOriginResult.originTransformation = lEnvModelPort.resetOriginResult.originTransformation;
        lEnvModelPort.resetOriginResult.resetCounter_nu = prevResetOriginRequestPort.resetCounter_nu;
        lEnvModelPortCMOrigin.resetOriginResult.resetCounter_nu = prevResetOriginRequestPort.resetCounter_nu;
        //transform envmodel
        //envModelPort.dynamicObjects
        for (auto& dynOb : lEnvModelPort.dynamicObjects) {
            if (dynOb.existenceProb_perc > 0u) {
                //TODO: dynOb.objShape.accel_mps2
                //      dynOb.objShape.covMatrix
                dynOb.headingAngle_rad = LSM_GEOML::radMod(dynOb.headingAngle_rad - prevResetOriginRequestPort.transformation.yaw_rad);
                for (lsm_geoml::size_type iPos{ 0U }; iPos < dynOb.objShape_m.actualSize; ++iPos) {
                    auto& curPos = dynOb.objShape_m.array[iPos];
                    curPos = convert(transformToNewCS.inverseTransform(cml::Vec2Df(curPos)));
                }
                float32_t cYaw = cos(prevResetOriginRequestPort.transformation.yaw_rad);
                float32_t sYaw = sin(prevResetOriginRequestPort.transformation.yaw_rad);
                float32_t xVel_mps = dynOb.vel_mps.x_dir;
                float32_t yVel_mps = dynOb.vel_mps.y_dir;
                float32_t xAccel_mps2 = dynOb.accel_mps2.x_dir;
                float32_t yAccel_mps2 = dynOb.accel_mps2.y_dir;

                dynOb.vel_mps.x_dir = cYaw * xVel_mps + sYaw * yVel_mps;
                dynOb.vel_mps.y_dir = -sYaw * xVel_mps + cYaw * yVel_mps;

                dynOb.accel_mps2.x_dir = cYaw * xAccel_mps2 + sYaw * yAccel_mps2;
                dynOb.accel_mps2.y_dir = -sYaw * xAccel_mps2 + cYaw * yAccel_mps2;
            }
        }
        //envModelPort.parkingSpaceMarkings
        for (auto& psm : lEnvModelPort.parkingSpaceMarkings) {
            //TODO: psm.covMatrix
            for (lsm_geoml::size_type iPos{ 0U }; iPos < psm.pos_m.actualSize; ++iPos) {
                psm.pos_m.array[iPos] = convert(transformToNewCS.inverseTransform(psm.pos_m.array[iPos]));
            }
        }

        //envModelPort.roadDescription
        for (auto& lb : lEnvModelPort.roadDescription.laneBoundaries) {
            for (lsm_geoml::size_type iLs{ 0U }; iLs < lb.laneShape.actualSize; ++iLs) {
                lb.laneShape.array[iLs] = convert(transformToNewCS.inverseTransform(lb.laneShape.array[iLs]));
            }
        }
        //envModelPort.staticObjects
        for (auto& statObj : lEnvModelPort.staticObjects) {
            //TODO statObj.shape.covMatrix
            for (lsm_geoml::size_type iOs{ 0U }; iOs < statObj.objShape_m.actualSize; ++iOs) {
                statObj.objShape_m.array[iOs] = convert(transformToNewCS.inverseTransform(statObj.objShape_m.array[iOs]));
            }
        }

        //transform parking box and virtual lines
        for (uint8 iPb{ 0U }; iPb < lParkingBoxPort.numValidParkingBoxes_nu; ++iPb) {
            for (lsm_geoml::size_type iCoord{ 0U }; iCoord < lParkingBoxPort.parkingBoxes[iPb].slotCoordinates_m.actualSize; ++iCoord) {
                auto& pbpPos = lParkingBoxPort.parkingBoxes[iPb].slotCoordinates_m.array[iCoord];
                pbpPos = convert(transformToNewCS.inverseTransform(pbpPos));
            }
            for (uint8 iVL{ 0U }; iVL < lParkingBoxPort.parkingBoxes[iPb].numVirtualLines_nu; ++iVL) {
                for (lsm_geoml::size_type iVert{ 0U }; iVert < lParkingBoxPort.parkingBoxes[iPb].virtualLines[iVL].virtLineVertices_m.actualSize; ++iVert) {
                    auto& vlVertex = lParkingBoxPort.parkingBoxes[iPb].virtualLines[iVL].virtLineVertices_m.array[iVert];
                    vlVertex = convert(transformToNewCS.inverseTransform(vlVertex));
                }
            }
        }
    }

    // update transformationToOdometry
    lEnvModelPort.transformationToOdometry = prevResetOriginRequestPort.transformation;
    lEnvModelPortCMOrigin.transformationToOdometry = prevResetOriginRequestPort.transformation;
}

void EnvironmentPerception::resetVariables() {
    mPerceptionAvailabilityPort = eco::create_default<si::PerceptionAvailabilityPort>();
    mPerceptionAvailabilityPort.uiVersionNumber = si::createPerceptionAvailabilityPort_InterfaceVersion().PerceptionAvailabilityPort_VERSION;
    mCollEnvModelPort = eco::create_default<si::CollEnvModelPort>();
    mCollEnvModelPort.uiVersionNumber = si::createCollEnvModelPort_InterfaceVersion().CollEnvModelPort_VERSION;
    std::fill(std::begin(mNumVirtualLinesPB), std::end(mNumVirtualLinesPB), (uint8_t)0U);
    tEnvModelPort = eco::create_default<si::ApEnvModelPort>();
    tEnvModelPort.uiVersionNumber = si::createApEnvModelPort_InterfaceVersion().ApEnvModelPort_VERSION;
    tParkingBoxPort = eco::create_default<si::ApParkingBoxPort>();
    tParkingBoxPort.uiVersionNumber = si::createApParkingBoxPort_InterfaceVersion().ApParkingBoxPort_VERSION;

    envModelPort = eco::create_default<si::ApEnvModelPort>();
    envModelPort.uiVersionNumber = si::createApEnvModelPort_InterfaceVersion().ApEnvModelPort_VERSION;
    parkingBoxPort = eco::create_default<si::ApParkingBoxPort>();
    parkingBoxPort.uiVersionNumber = si::createApParkingBoxPort_InterfaceVersion().ApParkingBoxPort_VERSION;
    mEnableScanning_nu = false;
    mLatencyTime_ms = 0.0;
    mLongOffset_m = 0.0;
    mLatOffset_m = 0.0;
    mStaticObjIdx = 0U;
    mParkingSMIdx = 0U;
    mLaneBoundIdx = 0U;
    mDynObjIdx = 0U;
    mNumParkingBoxes = 0U;
    std::fill(std::begin(mEgoVehicleOrientationDuringScanning_rad), std::end(mEgoVehicleOrientationDuringScanning_rad), 0.0F);
    std::fill(std::begin(mTrafficObjects), std::end(mTrafficObjects), trafficObjects{});

    std::queue<si::ApEnvModelPort>().swap(qEnvModelPort);
    std::queue<si::ApParkingBoxPort>().swap(qParkingBoxPort);
}
