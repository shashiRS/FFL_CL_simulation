#include "EpDelimiterManager.h"
#include "SiUtility.h"
#include <Log.h> // CarMaker logging

//ParkingSlot Index Convention
constexpr static lsm_geoml::size_type ROAD_LEFT_CORNER_IDX{ 0U };
constexpr static lsm_geoml::size_type ROAD_RIGHT_CORNER_IDX{ 1U };
constexpr static lsm_geoml::size_type CURB_RIGHT_CORNER_IDX{ 2U };
constexpr static lsm_geoml::size_type CURB_LEFT_CORNER_IDX{ 3U };

// adopted from si_core component SiHighMiniEmManager.cpp,
// see https://github-am.geo.conti.de/ADAS/si_core/blob/release/8.0.28/src/component/SiHighMiniEmManager.cpp#L281
template<uint8_t objectPolySize>
float32_t EpDelimiterManager::getScoreLowComplexity(const DelimiterZone& delimiterZone, const LSM_GEOML::Polygon2D<objectPolySize> &objectPolygon) const
{
    float32_t minProjection{ std::numeric_limits<float32_t>::max() };
    float32_t maxProjection{ std::numeric_limits<float32_t>::lowest() };
    for (const auto& pt_nu : objectPolygon)
    {
        const float32_t proj_m = pt_nu.scalarProduct(delimiterZone.projectionAxis);
        if (proj_m < minProjection)
        {
            minProjection = proj_m;
        }
        if (proj_m > maxProjection)
        {
            maxProjection = proj_m;
        }
    }
    const float32_t score_m{ std::abs(std::max(delimiterZone.minMaxProjection.first, minProjection)
     - std::min(delimiterZone.minMaxProjection.second, maxProjection)) * delimiterZone.scoreFactor };
    return score_m;
}

// adopted from si_core component SiHighMiniEmManager.cpp,
// see https://github-am.geo.conti.de/ADAS/si_core/blob/release/8.0.28/src/component/SiHighMiniEmManager.cpp#L303
template<uint8_t objectPolySize>
float32_t EpDelimiterManager::getScoreHighComplexity(const DelimiterZone& delimiterZone, const LSM_GEOML::Polygon2D<objectPolySize> &objectPolygon) const
{
    float32_t minProjection{ std::numeric_limits<float32_t>::max() };
    float32_t maxProjection{ std::numeric_limits<float32_t>::lowest() };
    if (delimiterZone.useVanishingPoint)
    {
        const LSM_GEOML::LineSegment2D outerSegment{ delimiterZone.zone[0U], delimiterZone.zone[1U] };
        const float32_t maxDistsq{ delimiterZone.zone.getEdge(0U).normSq() };
        const float32_t maxDist{ std::sqrt(maxDistsq) };
        for (const auto& pt_nu : objectPolygon)
        {
            const LSM_GEOML::LineSegment2D currentEdge{ pt_nu, delimiterZone.vanishingPoint };
            const auto result{ outerSegment.intersectWith(currentEdge) };
            if (std::get<1>(result) == false)
            {
                LogErrF(EC_General, "EpDelimiterManager: Something went wrong in getScoreHighComplexity. Point: ( %f , %f ). VanishingPoint: ( %f , %f ).",
                    pt_nu.x(), pt_nu.y(), delimiterZone.vanishingPoint.x(), delimiterZone.vanishingPoint.y());
            }
            else
            {
                const float32_t dist1{ cml::Vec2Df{ delimiterZone.zone[0U] - std::get<0>(result) }.normSq() };
                const float32_t dist2{ cml::Vec2Df{ delimiterZone.zone[1U] - std::get<0>(result) }.normSq() };
                float32_t proj_m{};
                if (dist1 > maxDistsq)
                {
                    proj_m = maxDist;
                }
                else if (dist2 > maxDistsq)
                {
                    proj_m = 0.0F;
                }
                else
                {
                    proj_m = std::sqrt(dist1);
                }
                if (proj_m < minProjection)
                {
                    minProjection = proj_m;
                }
                if (proj_m > maxProjection)
                {
                    maxProjection = proj_m;
                }
            }
        }
        minProjection = std::max(0.0F, minProjection);
        maxProjection = std::min(maxProjection, maxDist);
        const float32_t score_m{ std::abs(maxProjection - minProjection) };
        //score_m *= delimiterZone.scoreFactor;
        return score_m;
    }
    else
    {
        const float32_t score_m{ getScoreLowComplexity(delimiterZone, objectPolygon) };
        return score_m;
    }
}

//---------------------------------------------------------------------------
//! Check if the traffic Object is a delimiter of ParkingBox (if DelimiterZone polygon overlaps Traffic Object polygon)
//! Returns the parking Box Delimiter type (if the object is not a delimiter function return UNDEFINED_EDGE)
template<uint8_t objectPolySize>
si::RelativeLocationToParkingBox EpDelimiterManager::determineDelimiterType(const uint8_t pbIdx, const LSM_GEOML::Polygon2D<objectPolySize>& objectPolygon,
    const char* trafficObjDescription, const si::ParkingScenarioTypes parkingScenario_nu)
{
    const DelimiterZoneSet& delimiterZoneSet = mDelimiterZoneSets[pbIdx];
    // For garage parking, the delimiter classification based on the CarMaker traffic object description suffix can be disabled by this definition.
#ifndef EP_GP_DISABLE_DELIM_TOBJ_DESCR
    if (si::ParkingScenarioTypes::GARAGE_PARKING == parkingScenario_nu)
    {
        const bool isInsideDelimiter = objectPolygon.doPolygonsOverlap(delimiterZoneSet.inside.zone);
        if ((strstr(trafficObjDescription, "lim_P_Left"))) {
            return isInsideDelimiter ? si::RelativeLocationToParkingBox::INSIDE_LEFT_EDGE : si::RelativeLocationToParkingBox::LEFT_EDGE;
        }
        else if ((strstr(trafficObjDescription, "lim_P_Right"))) {
            return isInsideDelimiter ? si::RelativeLocationToParkingBox::INSIDE_RIGHT_EDGE : si::RelativeLocationToParkingBox::RIGHT_EDGE;
        }
        else if ((strstr(trafficObjDescription, "lim_P_End"))) {
            return isInsideDelimiter ? si::RelativeLocationToParkingBox::INSIDE_CURB_SIDE_EDGE : si::RelativeLocationToParkingBox::CURB_SIDE_EDGE;
        }
        else if ((strstr(trafficObjDescription, "lim_P_Front"))) {
            return isInsideDelimiter ? si::RelativeLocationToParkingBox::INSIDE_ROAD_SIDE_EDGE : si::RelativeLocationToParkingBox::ROAD_SIDE_EDGE;
        }
        else {
            return si::RelativeLocationToParkingBox::UNDEFINED_EDGE;
        }
    }
#endif

    // Internal object representation adding a bounding box and a custom doOverlap(DelimiterZone) method.
    class InternalObject
    {
        const LSM_GEOML::Polygon2D<objectPolySize>& mObjectPolygon;
        LSM_GEOML::AxisAlignedRectangular2D mBoundingBox{ true };
    public:
        InternalObject(const LSM_GEOML::Polygon2D<objectPolySize>& objectPolygon) : mObjectPolygon(objectPolygon)
        {
            mBoundingBox.fit(objectPolygon);
        }
        const LSM_GEOML::AxisAlignedRectangular2D& getBoundingBox() const { return mBoundingBox; }
        bool doOverlap(const DelimiterZone& delimiterZone) const
        {
            const bool overlap{ mBoundingBox.doOverlap(delimiterZone.rect) && mObjectPolygon.doPolygonsOverlap(delimiterZone.zone) };
            return overlap;
        }
    };

    // delimiter classification partly adopted from si_core component SiHighMiniEmManager.cpp,
    // see https://github-am.geo.conti.de/ADAS/si_core/blob/ea11b53475ef8912a6966bdaa654461fc89428f6/src/component/SiHighMiniEmManager.cpp#L356
#ifndef NO_SCENE_INTERPRETATION
    const auto &siParams = SiUtility::getSiParameters();
    const float32_t requProjectionRatio_nu{ siParams.minRequiredDelimiterProjectionRatio_nu };
    const boolean delZonesUseHighComplexityGetScore{ siParams.delZonesUseHighComplexityGetScore };
#else
    const float32_t requProjectionRatio_nu{ 1.5F };
    const boolean delZonesUseHighComplexityGetScore{ b_FALSE };
#endif

    const float32_t ARBITRARY_SCORE_GREATER_ZERO_M{ 1.0F };
    si::RelativeLocationToParkingBox currentObjectRelativeLocationToParkingBox{ si::RelativeLocationToParkingBox::UNDEFINED_EDGE };
    const InternalObject object{ objectPolygon };
    //prefilter
    if (object.getBoundingBox().doOverlap(delimiterZoneSet.all))
    {
        // inside zone - "All objects touching the "inside zone" shall be marked as inside delimiter."
        const bool isInside_nu{ object.doOverlap(delimiterZoneSet.inside) };

        const bool overlapsCurbside{ object.doOverlap(delimiterZoneSet.curb) };
        const bool overlapsRoadside{ object.doOverlap(delimiterZoneSet.road) };
        const bool overlapsLeft{ object.doOverlap(delimiterZoneSet.left) };
        const bool overlapsRight{ object.doOverlap(delimiterZoneSet.right) };
        const uint8_t overlapCount{ static_cast<uint8_t>(
                                            static_cast<uint8_t>(overlapsCurbside) +
                                            static_cast<uint8_t>(overlapsRoadside) +
                                            static_cast<uint8_t>(overlapsLeft) +
                                            static_cast<uint8_t>(overlapsRight)) };

        //object is associated to no zone
        if (overlapCount == 0U)
        {
            if (isInside_nu)
            {
                currentObjectRelativeLocationToParkingBox = si::RelativeLocationToParkingBox::RELATED_OTHERWISE;
            }
        }
        //object is associated to all zones
        else if (overlapCount == 4U)
        {
            currentObjectRelativeLocationToParkingBox = si::RelativeLocationToParkingBox::RELATED_OTHERWISE;
        }
        //final object association not clear
        else
        {
            // Could have 1, 2 or 3 zone overlaps

            //Steer association via scores
            float32_t curbScore_m{ 0.0F };
            float32_t roadScore_m{ 0.0F };
            float32_t leftScore_m{ 0.0F };
            float32_t rightScore_m{ 0.0F };

            //1 overlap is trivial - set the ONE overlap zone to a dummy value
            if (overlapCount == 1U)
            {
                curbScore_m = overlapsCurbside ? ARBITRARY_SCORE_GREATER_ZERO_M : curbScore_m;
                roadScore_m = overlapsRoadside ? ARBITRARY_SCORE_GREATER_ZERO_M : roadScore_m;
                leftScore_m = overlapsLeft ? ARBITRARY_SCORE_GREATER_ZERO_M : leftScore_m;
                rightScore_m = overlapsRight ? ARBITRARY_SCORE_GREATER_ZERO_M : rightScore_m;
            }
            //2 overlaps: select score to determine which is the better fit in case of neighbours. 
            else if (overlapCount == 2U)
            {
                //Neighbouring case
                if (!((overlapsCurbside && overlapsRoadside) || (overlapsLeft && overlapsRight)))
                {
                    if (overlapsCurbside)
                    {
                        curbScore_m = delZonesUseHighComplexityGetScore ?
                            getScoreHighComplexity(delimiterZoneSet.curb, objectPolygon) :
                            getScoreLowComplexity(delimiterZoneSet.curb, objectPolygon);
                    }
                    if (overlapsRoadside)
                    {
                        roadScore_m = delZonesUseHighComplexityGetScore ?
                            getScoreHighComplexity(delimiterZoneSet.road, objectPolygon) :
                            getScoreLowComplexity(delimiterZoneSet.road, objectPolygon);
                    }
                    if (overlapsLeft)
                    {
                        leftScore_m = delZonesUseHighComplexityGetScore ?
                            getScoreHighComplexity(delimiterZoneSet.left, objectPolygon) :
                            getScoreLowComplexity(delimiterZoneSet.left, objectPolygon);
                    }
                    if (overlapsRight)
                    {
                        rightScore_m = delZonesUseHighComplexityGetScore ?
                            getScoreHighComplexity(delimiterZoneSet.right, objectPolygon) :
                            getScoreLowComplexity(delimiterZoneSet.right, objectPolygon);
                    }
                }
            }
            //3 overlaps - take the zone that is surrounded by 2 others (might be score-based as well if required later)
            else
            {
                if (!overlapsRoadside)
                {
                    curbScore_m = ARBITRARY_SCORE_GREATER_ZERO_M;
                }
                else if (!overlapsRight)
                {
                    leftScore_m = ARBITRARY_SCORE_GREATER_ZERO_M;
                }
                else if (!overlapsLeft)
                {
                    rightScore_m = ARBITRARY_SCORE_GREATER_ZERO_M;
                }
                else
                {
                    roadScore_m = ARBITRARY_SCORE_GREATER_ZERO_M;
                }
            }

            //Determine type by score
            //left is best
            if ((leftScore_m > requProjectionRatio_nu * rightScore_m) &&
                (leftScore_m > requProjectionRatio_nu * curbScore_m) &&
                (leftScore_m > requProjectionRatio_nu * roadScore_m))
            {
                currentObjectRelativeLocationToParkingBox = isInside_nu ? si::RelativeLocationToParkingBox::INSIDE_LEFT_EDGE : si::RelativeLocationToParkingBox::LEFT_EDGE;
            }
            //right is best
            else if ((rightScore_m > requProjectionRatio_nu * leftScore_m) &&
                (rightScore_m > requProjectionRatio_nu * curbScore_m) &&
                (rightScore_m > requProjectionRatio_nu * roadScore_m))
            {
                currentObjectRelativeLocationToParkingBox = isInside_nu ? si::RelativeLocationToParkingBox::INSIDE_RIGHT_EDGE : si::RelativeLocationToParkingBox::RIGHT_EDGE;
            }
            //road is best
            else if ((roadScore_m > requProjectionRatio_nu * leftScore_m) &&
                (roadScore_m > requProjectionRatio_nu * rightScore_m) &&
                (roadScore_m > requProjectionRatio_nu * curbScore_m))
            {
                currentObjectRelativeLocationToParkingBox = isInside_nu ? si::RelativeLocationToParkingBox::INSIDE_ROAD_SIDE_EDGE : si::RelativeLocationToParkingBox::ROAD_SIDE_EDGE;
            }
            //curb is best
            else if ((curbScore_m > requProjectionRatio_nu * leftScore_m) &&
                (curbScore_m > requProjectionRatio_nu * rightScore_m) &&
                (curbScore_m > requProjectionRatio_nu * roadScore_m))
            {
                currentObjectRelativeLocationToParkingBox = isInside_nu ? si::RelativeLocationToParkingBox::INSIDE_CURB_SIDE_EDGE : si::RelativeLocationToParkingBox::CURB_SIDE_EDGE;
            }
            else
            {
                //no clear result
                currentObjectRelativeLocationToParkingBox = si::RelativeLocationToParkingBox::RELATED_OTHERWISE;
            }
        }
    }
    return currentObjectRelativeLocationToParkingBox;
}

// Instantiate concrete implementations of the public template function, see https://isocpp.org/wiki/faq/templates#separate-template-fn-defn-from-decl
// objectPolySize=2U for si::ParkingSpaceMarkings
template si::RelativeLocationToParkingBox EpDelimiterManager::determineDelimiterType(const uint8_t pbIdx, const LSM_GEOML::Polygon2D<2U>& objectPolygon,
    const char* trafficObjDescription, const si::ParkingScenarioTypes parkingScenario_nu);
// objectPolySize=ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU for si::StaticObject
template si::RelativeLocationToParkingBox EpDelimiterManager::determineDelimiterType(const uint8_t pbIdx, const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU>& objectPolygon,
    const char* trafficObjDescription, const si::ParkingScenarioTypes parkingScenario_nu);
// objectPolySize=ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_LANE_BOUND_PTS_NU for si::LaneBoundary
template si::RelativeLocationToParkingBox EpDelimiterManager::determineDelimiterType(const uint8_t pbIdx, const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_LANE_BOUND_PTS_NU>& objectPolygon,
    const char* trafficObjDescription, const si::ParkingScenarioTypes parkingScenario_nu);

// delimiter zone calculation adopted from si_core component SiHighMiniEmManager.cpp,
// see https://github-am.geo.conti.de/ADAS/si_core/blob/release/8.0.28/src/component/SiHighMiniEmManager.cpp#L446
void EpDelimiterManager::updateDelimiterZones(const uint8_t pbIdx, const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU>& parkingBoxShape)
{
    DelimiterZoneSet& delimiterZoneSet = mDelimiterZoneSets[pbIdx];
    const DelimiterZoneCalcInfo data{ getDelimiterZoneCalcInfo(parkingBoxShape) };
    createLeftAndRightZone(data, delimiterZoneSet.left, delimiterZoneSet.right);
    delimiterZoneSet.curb = createCurbsideZone(data);
    delimiterZoneSet.road = createRoadsideZone(data);
    delimiterZoneSet.inside = createInsideZone(data);

    delimiterZoneSet.all.fit(parkingBoxShape);
    delimiterZoneSet.all.add(delimiterZoneSet.left.zone);
    delimiterZoneSet.all.add(delimiterZoneSet.right.zone);
    delimiterZoneSet.all.add(delimiterZoneSet.road.zone);
    delimiterZoneSet.all.add(delimiterZoneSet.curb.zone);
    //delimiterZoneSet.inside.zone is encapsulated by parkingSpace and the other zones -> not required

    // Update delimiter zones for plotting in IPG Movie
    SiUtility::DelimiterZones delimiterZones{};
    delimiterZones.curbsideZone = delimiterZoneSet.curb.zone;
    delimiterZones.roadsideZone = delimiterZoneSet.road.zone;
    delimiterZones.rightsideZone = delimiterZoneSet.right.zone;
    delimiterZones.leftsideZone = delimiterZoneSet.left.zone;
    delimiterZones.insideZone = delimiterZoneSet.inside.zone;
    SiUtility::getInstance().updateDelimiterZones(pbIdx, delimiterZones);
}

EpDelimiterManager::DelimiterZoneCalcInfo EpDelimiterManager::getDelimiterZoneCalcInfo(
    const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU>& slotCoordinates_m)
{
    DelimiterZoneCalcInfo data{};
    data.roadLeft = &slotCoordinates_m[ROAD_LEFT_CORNER_IDX];
    data.curbLeft = &slotCoordinates_m[CURB_LEFT_CORNER_IDX];
    data.roadRight = &slotCoordinates_m[ROAD_RIGHT_CORNER_IDX];
    data.curbRight = &slotCoordinates_m[CURB_RIGHT_CORNER_IDX];

    data.shiftToLeftRoadSide = cml::Vec2Df{ *data.roadLeft - *data.roadRight }.getNormalized();
    data.shiftToLeftCurbSide = cml::Vec2Df{ *data.curbLeft - *data.curbRight }.getNormalized();
    data.shiftToRoadLeftSide = cml::Vec2Df{ *data.roadLeft - *data.curbLeft }.getNormalized();
    data.shiftToRoadRightSide = cml::Vec2Df{ *data.roadRight - *data.curbRight }.getNormalized();

    const float32_t roadLeftCornerAngleSin{ std::sin(data.shiftToLeftRoadSide.getAngleWith(data.shiftToRoadLeftSide)) };
    const float32_t roadRightCornerAngleSin{ std::sin(data.shiftToLeftRoadSide.getAngleWith(cml::Vec2Df(-data.shiftToRoadRightSide))) };
    const float32_t curbLeftCornerAngleSin{ std::sin(data.shiftToLeftCurbSide.getAngleWith(cml::Vec2Df(-data.shiftToRoadLeftSide))) };
    const float32_t curbRightCornerAngleSin{ std::sin(data.shiftToLeftCurbSide.getAngleWith(data.shiftToRoadRightSide)) };

    if ((roadLeftCornerAngleSin < LSM_GEOML::MIN_FLT_DIVISOR) || (roadRightCornerAngleSin < LSM_GEOML::MIN_FLT_DIVISOR)
        || (curbLeftCornerAngleSin < LSM_GEOML::MIN_FLT_DIVISOR) || (curbRightCornerAngleSin < LSM_GEOML::MIN_FLT_DIVISOR))
    {
        LogErrStr(EC_General, "EpDelimiterManager: Sin of SlotCornerAngle is 0. Slot like this should not appear.");
    }
    else
    {
        data.roadLeftFactor = (1.0F / roadLeftCornerAngleSin);
        data.roadRightFactor = (1.0F / roadRightCornerAngleSin);
        data.curbLeftFactor = (1.0F / curbLeftCornerAngleSin);
        data.curbRightFactor = (1.0F / curbRightCornerAngleSin);
    }
    return data;
}

//!
//! \brief       create CurbSide Zone of Parking Box
//!
EpDelimiterManager::DelimiterZone EpDelimiterManager::createCurbsideZone(const DelimiterZoneCalcInfo& data) const
{
    // get dimensions of curbside Zone
    float32_t const thDelimiterDist_m = 1.0F;
    float32_t const curbsideZoneDepthFactor = 0.35F;

    //totally untested and assumingly not yet working - especially for left/right side scanning and driving beyond slot
    const cml::Vec2Df shiftToRoadRightUnnormalized{ *data.roadRight - *data.curbRight };
    const cml::Vec2Df shiftToRoadLeftUnnormalized{ *data.roadLeft - *data.curbLeft };

    const float32_t slotDepthLeft_m{ shiftToRoadLeftUnnormalized.norm() };
    const float32_t slotDepthRight_m{ shiftToRoadRightUnnormalized.norm() };

    const cml::Vec2Df vec0{ *data.curbRight - (data.shiftToRoadRightSide * (thDelimiterDist_m * data.curbRightFactor)) };
    const cml::Vec2Df vec1{ *data.curbLeft - (data.shiftToRoadLeftSide * (thDelimiterDist_m * data.curbLeftFactor)) };
    const cml::Vec2Df vec2{ *data.curbLeft + (data.shiftToRoadLeftSide * (curbsideZoneDepthFactor * slotDepthLeft_m)) };
    const cml::Vec2Df vec3{ *data.curbRight + (data.shiftToRoadRightSide * (curbsideZoneDepthFactor * slotDepthRight_m)) };

    DelimiterZone result;
    result.zone = Quadrangle2D{ vec0, vec1, vec2, vec3 };
    setDelimiterZoneParameters(result);
    return result;
}

//!
//! \brief       create RoadSide Zone of Parking Box
//!
EpDelimiterManager::DelimiterZone EpDelimiterManager::createRoadsideZone(const DelimiterZoneCalcInfo& data) const
{
    // get dimensions of lateral side zone
    float32_t const roadsideZoneDepth_m = 1.7F;
    float32_t const thDelimiterDist_m = 1.0F;
    if (roadsideZoneDepth_m < thDelimiterDist_m)
    {
        LogErrStr(EC_General, "EpDelimiterManager: Parameter roadsideZoneDepth_m is smaller than thDelimiterDist_m");
    }

    //totally untested and assumingly not yet working - especially for left/right side scanning and driving beyond slot
    const cml::Vec2Df vec0{ *data.roadLeft + (data.shiftToRoadLeftSide * (thDelimiterDist_m * data.roadLeftFactor)) };
    const cml::Vec2Df vec1{ *data.roadRight + (data.shiftToRoadRightSide * (thDelimiterDist_m * data.roadRightFactor)) };
    const cml::Vec2Df vec2{ *data.roadRight + (data.shiftToRoadRightSide * ((thDelimiterDist_m - roadsideZoneDepth_m) * data.roadRightFactor)) };
    const cml::Vec2Df vec3{ *data.roadLeft + (data.shiftToRoadLeftSide * ((thDelimiterDist_m - roadsideZoneDepth_m) * data.roadLeftFactor)) };

    DelimiterZone result;
    result.zone = Quadrangle2D{ vec0, vec1, vec2, vec3 };
    setDelimiterZoneParameters(result);
    return result;
}

//!
//! \brief       create Lateral Zone of Parking Box
//!                                                 3-----------2
//!                                                 |           |
//!                                Left side-->     |    Le     |  < -- Right side
//!                                                 |           |
//!                       y   ^                     0-----------1
//!                           |
//!     veh coord             .  --->              [ego >]
//!     syst (ref                   x
//!     rear axle)                                  1-------------0
//!                                                 |             |
//!                              Right side-->      |      Ri     |   <-Left side
//!                                                 |             |
//!                                                 2-------------3
//!
void EpDelimiterManager::createLeftAndRightZone(const DelimiterZoneCalcInfo& data,
    DelimiterZone& inoutLeft, DelimiterZone& inoutRight) const
{
    // get dimensions of lateralSide Zone
    float32_t const delimiterDist = 1.0F;
    float32_t const lateralZoneWidth = 1.7F;

    if (lateralZoneWidth < delimiterDist)
    {
        LogErrStr(EC_General, "EpDelimiterManager: Parameter lateralZoneWidth_m is smaller than thDelimiterDist_m");
    }

    // vec0 and vec1 belong to outer long side of the LEFT delimiter - required by convention
    const cml::Vec2Df vecL0{ *data.curbLeft + ((data.shiftToLeftCurbSide - data.shiftToRoadLeftSide) * (delimiterDist * data.curbLeftFactor)) };
    const cml::Vec2Df vecL1{ *data.roadLeft + ((data.shiftToLeftRoadSide + data.shiftToRoadLeftSide) * (delimiterDist * data.roadLeftFactor)) };
    const cml::Vec2Df vecL2{ *data.roadLeft +
        (((data.shiftToLeftRoadSide * (delimiterDist - lateralZoneWidth)) + (data.shiftToRoadLeftSide * delimiterDist)) * data.roadLeftFactor) };
    const cml::Vec2Df vecL3{ *data.curbLeft +
        (((data.shiftToLeftCurbSide * (delimiterDist - lateralZoneWidth)) - (data.shiftToRoadLeftSide * delimiterDist)) * data.curbLeftFactor) };

    inoutLeft.zone = Quadrangle2D{ vecL0, vecL1, vecL2, vecL3 };
    setDelimiterZoneParameters(inoutLeft);

    // vec0 and vec1 belong to outer long side of the Right delimiter - required by convention
    const cml::Vec2Df vecR0{ *data.roadRight - ((data.shiftToLeftRoadSide - data.shiftToRoadRightSide) * (delimiterDist * data.roadRightFactor)) };
    const cml::Vec2Df vecR1{ *data.curbRight - ((data.shiftToLeftCurbSide + data.shiftToRoadRightSide) * (delimiterDist * data.curbRightFactor)) };
    const cml::Vec2Df vecR2{ *data.curbRight
        - (((data.shiftToLeftCurbSide * (delimiterDist - lateralZoneWidth)) + (data.shiftToRoadRightSide * delimiterDist)) * data.curbRightFactor) };
    const cml::Vec2Df vecR3{ *data.roadRight
        - (((data.shiftToLeftRoadSide * (delimiterDist - lateralZoneWidth)) - (data.shiftToRoadRightSide * delimiterDist)) * data.roadRightFactor) };

    inoutRight.zone = Quadrangle2D{ vecR0, vecR1, vecR2, vecR3 };
    setDelimiterZoneParameters(inoutRight);
}

//!
//! \brief       create Inside Zone of Parking Box
//!
EpDelimiterManager::DelimiterZone EpDelimiterManager::createInsideZone(const DelimiterZoneCalcInfo& data) const
{
    // get dimensions of lateralSide Zone
#ifndef NO_SCENE_INTERPRETATION
    const auto &siParams = SiUtility::getSiParameters();
    float32_t const insideZoneDist_m = siParams.insideZoneDist_m;
#else
    float32_t const insideZoneDist_m = 0.15F;
#endif

    //basically take the parking space and shrink it
    const cml::Vec2Df vec0{ *data.roadLeft + ((-data.shiftToRoadLeftSide - data.shiftToLeftRoadSide) * (insideZoneDist_m * data.roadLeftFactor)) };
    const cml::Vec2Df vec1{ *data.roadRight + ((-data.shiftToRoadRightSide + data.shiftToLeftRoadSide) * (insideZoneDist_m * data.roadRightFactor)) };
    const cml::Vec2Df vec2{ *data.curbRight + ((data.shiftToRoadRightSide + data.shiftToLeftCurbSide) * (insideZoneDist_m * data.curbRightFactor)) };
    const cml::Vec2Df vec3{ *data.curbLeft + ((data.shiftToRoadLeftSide - data.shiftToLeftCurbSide) * (insideZoneDist_m * data.curbLeftFactor)) };

    DelimiterZone result;
    result.zone = Quadrangle2D{ vec0, vec1, vec2, vec3 };
    //The inside Zone does not need the other parameters except for the rect
    result.rect.fit(result.zone);
    return result;
}

void EpDelimiterManager::setDelimiterZoneParameters(DelimiterZone & delimiterZone) const
{
    // get SI parameter delZonesUseHighComplexityGetScore
#ifndef NO_SCENE_INTERPRETATION
    const boolean delZonesUseHighComplexityGetScore{ SiUtility::getSiParameters().delZonesUseHighComplexityGetScore };
#else
    const boolean delZonesUseHighComplexityGetScore{ b_FALSE };
#endif

    delimiterZone.rect.fit(delimiterZone.zone);
    const cml::Vec2Df edge3{ cml::Vec2Df{delimiterZone.zone.getEdge(3U).getNormalized()} };
    const cml::Vec2Df edge1{ cml::Vec2Df{delimiterZone.zone.getEdge(1U).getNormalized()} };
    const cml::Vec2Df sideEdgeDir{ cml::Vec2Df{ edge3 - edge1 }.getNormalized() };
    const cml::Vec2Df oldProjectionAxis{ delimiterZone.zone.getEdge(0U).getNormalized() };
    delimiterZone.projectionAxis = cml::Vec2Df{ -sideEdgeDir.y(), sideEdgeDir.x() };
    delimiterZone.minMaxProjection = delimiterZone.zone.projectToAxis(delimiterZone.projectionAxis);
    const float32_t angle_rad{ delimiterZone.projectionAxis.getAngleWith(oldProjectionAxis) };
    const float32_t cosAngleAbs{ std::abs(std::cos(angle_rad)) };
    if (cosAngleAbs < LSM_GEOML::MIN_FLT_DIVISOR)
    {
        LogErrStr(EC_General, "EpDelimiterManager: cosAngleAbs in setDelimiterZoneParameters is 0.");
        delimiterZone.scoreFactor = 1.0F;
    }
    else
    {
        delimiterZone.scoreFactor = (1.0F / cosAngleAbs);
    }
    if (delZonesUseHighComplexityGetScore == b_TRUE)
    {
        const LSM_GEOML::LineSegment2D edgeLS1{ delimiterZone.zone[1U], delimiterZone.zone[2U] };
        const LSM_GEOML::LineSegment2D edgeLS3{ delimiterZone.zone[3U], delimiterZone.zone[0U] };
        auto result{ edgeLS1.intersectWith(edgeLS3) };
        if (std::get<1>(result) == true)
        {
            delimiterZone.vanishingPoint = std::get<0>(result);
            // For side edges that are almost parallel the vanishing point variant should not be used to save performance
            // Therefore only use the vanishing point variant if the vanishing point is relatively close to the delimiter zone
            constexpr float32_t sqDistanceUseVanishingPoint_m2{ 10000.0F };
            if (cml::Vec2Df{ delimiterZone.vanishingPoint - delimiterZone.zone[0U] }.normSq() < sqDistanceUseVanishingPoint_m2)
            {
                delimiterZone.useVanishingPoint = true;
            }
            else
            {
                delimiterZone.useVanishingPoint = false;
            }
        }
        else
        {
            delimiterZone.useVanishingPoint = false;
        }
    }
}