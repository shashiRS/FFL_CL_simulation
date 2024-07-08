#include "TestEvaluation.h"
#include <pod_class_converter.h>
#include "geoml/LSM_Math.h"
#include "geoml/LSM_LookupTable.h"
#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include "Car/Car.h"
#pragma warning( pop )
#include "MathUtils.h"
#include "DataDict.h"
#include "InfoUtils.h"
#include "Traffic.h"
#include <string>
#include <fstream>

void TestEvaluation::resetVariablesTo0() {

    // Check in the SimParameters Infofile whether the pyBase test evaluation is running.
    testEvaluationActive = iEntryExists(SimCore.TestRig.SimParam.Inf, "PyBaseEvalActive") && iGetInt(SimCore.TestRig.SimParam.Inf, "PyBaseEvalActive");

    egoVehScanningPathDefined_bool = false;
    testEvaluationPort.plannedEgoVehOutsideBoundaries_bool = false;
    lastTargetPoseUsed = { 0.0f, 0.0f, 0.0f };
    egoVehScanningPath.clear();

    std::fill(testEvaluationPort.vehicleArea1Polygon.begin(), testEvaluationPort.vehicleArea1Polygon.end(), cml::Vec2Df{ 0.0f,0.0f });
    testEvaluationPort.vehicleArea1Polygon.clear();
    std::fill(testEvaluationPort.vehicleArea2Polygon.begin(), testEvaluationPort.vehicleArea2Polygon.end(), cml::Vec2Df{ 0.0f,0.0f });
    testEvaluationPort.vehicleArea2Polygon.clear();
    std::fill(testEvaluationPort.maneuveringAreaPolygon.begin(), testEvaluationPort.maneuveringAreaPolygon.end(), cml::Vec2Df{ 0.0f,0.0f });
    testEvaluationPort.maneuveringAreaPolygon.clear();
    std::fill(testEvaluationPort.boundingBoxPolygon.begin(), testEvaluationPort.boundingBoxPolygon.end(), cml::Vec2Df{ 0.0f,0.0f });
    testEvaluationPort.boundingBoxPolygon.clear();
    std::fill(testEvaluationPort.egoVehScanningPathPolygon.begin(), testEvaluationPort.egoVehScanningPathPolygon.end(), cml::Vec2Df{ 0.0f,0.0f });
    testEvaluationPort.egoVehScanningPathPolygon.clear();
    std::fill(testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.begin(), testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.end(), cml::Vec2Df{ 0.0f,0.0f });
    testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.clear();

    std::fill(std::begin(testEvaluationPort.d5Distance_m), std::end(testEvaluationPort.d5Distance_m), 0.0f);

    currentTrajectoryIndex_nu = 0U;
    testEvaluationPort.plannedTrajDrivenPoint = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
    testEvaluationPort.shortestDistanceToHighObject_m = -1.0f;
    testEvaluationPort.closestHighObjectId_nu = -1;
    std::fill(std::begin(testEvaluationPort.shortestDistanceCM), std::end(testEvaluationPort.shortestDistanceCM), -1.0f);
    std::fill(std::begin(testEvaluationPort.shortestDistanceOdo), std::end(testEvaluationPort.shortestDistanceOdo), -1.0f);
    testEvaluationPort.trajVertexIndexShortestDistHighObj_nu = -1;
    testEvaluationPort.shortestDistanceToWheelTravObject_m = -1.0f;
    testEvaluationPort.closestWheelTravObjectId_nu = -1;
    testEvaluationPort.trajVertexIndexShortestDistWheelTravObj_nu = -1;
    testEvaluationPort.shortestDistanceToBodyTravObject_m = -1.0f;
    testEvaluationPort.closestBodyTravObjectId_nu = -1;
    testEvaluationPort.trajVertexIndexShortestDistBodyTravObj_nu = -1;
    testEvaluationPort.vehicleArea1Polygon_size = 0u;
    testEvaluationPort.vehicleArea2Polygon_size = 0u;
    testEvaluationPort.egoVehScanningPathPolygonRemoved_bool = false;
    testEvaluationPort.vehicleArea1OverlapsPb = false ;
    testEvaluationPort.latDiffOptimalTP_FinalEgoPose_m      = -1.0f;
    testEvaluationPort.longDiffOptimalTP_FinalEgoPose_m     = -1.0f;
    testEvaluationPort.yawDiffOptimalTP_FinalEgoPose_deg    = -1.0f;
    testEvaluationPort.latDiffOptimalTP_TargetPose_m   = -1.0f;
    testEvaluationPort.longDiffOptimalTP_TargetPose_m  = -1.0f;
    testEvaluationPort.yawDiffOptimalTP_TargetPose_deg = -1.0f;
    testEvaluationPort.latDiffErrorTreshold_m   = .0f;
    testEvaluationPort.longDiffErrorTreshold_m  = .0f;
    testEvaluationPort.yawDiffErrorTreshold_deg = .0f;
    testEvaluationPort.egoPoseTargetPoseDeviation = {};
    testEvaluationPort.finalTargetPose = {};
    testEvaluationPort.car_outside_PB = -1;
    std::fill(std::begin(testEvaluationPort.staticStructColidesTarget_Pose), std::end(testEvaluationPort.staticStructColidesTarget_Pose), -1);

    //TimeToPark
    curvature_last = 0.0f;
    drivingForwardReq_nu_delay = false;
    noOfNewSegment = 0U;
    newSegmentStarted_nu_delay = false;
    testEvaluationPort.timeToPark = .0f;

    vel_accel_delay = 0.0f;
    distanceToStopReq_m_delay = 0.0f;
    distanceOnStroke = .0f;
    inverseVelocityAtTrajPoints.clear();
    distanceOnStrokeList.clear();

    //AlBu
    xPosition_m_delay = 0.0f;
    yPosition_m_delay = 0.0f;
    yawAngle_rad_delay = 0.0f;
    headUnitScreen_nu_delay = 0;
    odoCmRefEgoRaCur_m_calc = 0.0f;
    testEvaluationPort.odoEstimationCMDiference = 0.0f;
    odoCmRefxPosition_m_delay = 0.0f;
    odoCmRefyPosition_m_delay = 0.0f;
    odoCmRefyawAngEgoRaCur_rad_delay = 0.0f;
    vehVelocityCur_mps_delay = 0.0f;

    // tce validation
    for (unsigned int i = 0U; i < 4U; i++)
    {
        cmCirc_m[i] = 0.0f;
        cmLastRot_rad[i] = 0.0f;
    }
    cmVel_mps = 0.0f;
    cmLastDist_m = 0.0f;
    cmDistFiltered = 0.0f;
    cmFirstDist_m = 0.0f;

    testEvaluationPort.numberOfStrokes = 0;
    testEvaluationPort.numberOfStrokes_gear = 0;
    testEvaluationPort.numberOfStrokes_newSegmentStarted = 0;
    testEvaluationPort.drivenDistanceOnStroke_m = 0.0F;
    testEvaluationPort.allowedManeuveringSpaceExceed_bool = false;
    gearNo_delay = 0;
    newSegmentStarted_Last= false;
    am_lo_rear = 20.0f;
    am_la_side = 3.0f;
    am_lo_front = 0.2f;
    am_la_road = 0.6f;
    am_p_rear = 2.0f;
    am_p_end = 0.3f;
    am_p_front = 2.0f;

    for (unsigned int i = 0U; i < 4U; i++)
    {
        cornerEgoVehicle[i].x() = 0;
        cornerEgoVehicle[i].y() = 0;
        initialCornerEgoVehicle[i].x() = 0;
        initialCornerEgoVehicle[i].y() = 0;
    }
    for (unsigned int i = 0U; i < 9U; i++)
    {
        CrossingParkingBoxCorner[i].x() = 0;
        CrossingParkingBoxCorner[i].y() = 0;
    }

    evaluationPort.n_strokes_max_nu = 0U;
    evaluationPort.parkingManeuver_nu = Parking_Maneuver::NOT_SPECIFIED;
    evaluationPort.scopeBase_nu = false;
    evaluationPort.t_sim_max_s = .0f;
    evaluationPort.useCase_nu = false;
    evaluationPort.v_max_mps = .0f;
    evaluationPort.optimalTargetPose.pose = LSM_GEOML::Pose{};
    evaluationPort.optimalTargetPose.valid = false;
    evaluationPort.latMaxDeviation_m = -1.f;

    targetPoseReachedStatusSet = false;
    selectedParkingBoxIdx = 0U;

    testEvaluationPort.numberOfCrvSteps = 0U;
    testEvaluationPort.tooManyStrokes = false;

    testEvaluationPort.latMaxDeviation_m = .0f;
    testEvaluationPort.longMaxDeviation_m = .0f;
    testEvaluationPort.yawMaxDeviation_rad = .0f;
};

static cml::Vec2Df abs(cml::Vec2Df v) { return cml::Vec2Df{ fabsf(v.x()), fabsf(v.y()) }; }
static bool sortByYDescending(const cml::Vec2Df &lhs, const cml::Vec2Df &rhs) { return lhs.y() > rhs.y(); }
static bool sortByYAscending(const cml::Vec2Df &lhs, const cml::Vec2Df &rhs) { return lhs.y() < rhs.y(); }

void TestEvaluation::setNumberOfGearStrokes(const unsigned int &numberOfStrokesToSet) {
    testEvaluationPort.numberOfStrokes_gear = numberOfStrokesToSet;
}

static void transformLocalToWorld(
    const double originX_m,
    const double originY_m,
    const double orientation_rad,
    const double localX_m,
    const double localY_m,
    cml::Vec2Df& worldPt_m)
{
    double cYaw = cos(orientation_rad);
    double sYaw = sin(orientation_rad);
    worldPt_m[0] = float(originX_m + cYaw * localX_m - sYaw * localY_m);
    worldPt_m[1] = float(originY_m + sYaw * localX_m + cYaw * localY_m);
}

template<uint8_t polySize>
LSM_GEOML::Polygon2D<polySize> returnBoundingBox(const LSM_GEOML::Polygon2D<polySize>& polygonPoints, const LSM_GEOML::Pose& origin) {
    LSM_GEOML::Polygon2D<polySize> boundingBox{};
    for (auto& point : polygonPoints) {
        cml::Vec2Df transformedPoint{};
        transformLocalToWorld(origin.Pos().x(), origin.Pos().y(), origin.Yaw_rad(), point.x(), point.y(), transformedPoint);
        boundingBox.append(transformedPoint);
    }
    return boundingBox;
}

// copied from mf_trjctl/MFCPAI_PositioningAndInterpolation.cpp with small adaptations
void TestEvaluation::definePositionOnTrajectory(
    const ap_tp::PlannedTrajPort& plannedTrajPort,
    const LSM_GEOML::Pose& egoVehiclePose,
    PositionOnTrajectory& positionOnTrajectory)
{
    if (plannedTrajPort.trajValid_nu) {

        //Initilazie value before orthogonal projection loop.(Needed to save information about performed forward step during orthogonal projection.)
        bool stepedForwardOrthogonalProj{ false };

        //Check whether more than one valid trajectory points; otherwise an orthogonal projection cannot be performed.
        int8_t secondToLastTrajectoryIndex_nu{ static_cast<int8_t>(static_cast<int8_t>(plannedTrajPort.numValidCtrlPoints_nu) - 2) };
        assert(secondToLastTrajectoryIndex_nu >= static_cast<int8_t>(0U));

        /*
        *   Define start index for orthogonal projection:
        *   Default use case: number of valid trajectory points is reduced in plannedTrajPort.numValidCtrlPoints_nu
        */
        //If last cycle trajectory index_nu is equal to or larger than the number of valid trajectory points
        //use the second to last trajectory point as start index.
        if ((secondToLastTrajectoryIndex_nu >= 0) && (static_cast<int8_t>(currentTrajectoryIndex_nu) > secondToLastTrajectoryIndex_nu)) {
            currentTrajectoryIndex_nu = static_cast<uint8_t>(secondToLastTrajectoryIndex_nu);
        }
        //If a new trajectory segment was sent in planned trajectory set start index to 0 to continue at the start point of the new segment.
        else if (plannedTrajPort.newSegmentStarted_nu) {

            currentTrajectoryIndex_nu = 0U;
        }
        //Otherwise (default scenario while maneuvering on aa trajectory) do nothing and continue with last cycle index as start index.
        else {
        }

        //Loop for orthogonal projection (see comments below for details)
        for (uint8_t i{ 0U }; (i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_TRAJ_CTRL_POINTS) && ((currentTrajectoryIndex_nu + 1U) < ap_tp::AP_TP_Const::AP_P_MAX_NUM_TRAJ_CTRL_POINTS); i++) {

            // Vector between the trajectory points with index i0 and i0+1
            cml::Vec2Df vec_iCur_iCur1{
                plannedTrajPort.plannedTraj[currentTrajectoryIndex_nu + 1U].xTrajRAReq_m - plannedTrajPort.plannedTraj[currentTrajectoryIndex_nu].xTrajRAReq_m,
                plannedTrajPort.plannedTraj[currentTrajectoryIndex_nu + 1U].yTrajRAReq_m - plannedTrajPort.plannedTraj[currentTrajectoryIndex_nu].yTrajRAReq_m
            };
            // Vector between the trajectory point with index i0 current and the ego position
            cml::Vec2Df vec_iCur_ego{
                (egoVehiclePose.Pos().x() - plannedTrajPort.plannedTraj[currentTrajectoryIndex_nu].xTrajRAReq_m),
                (egoVehiclePose.Pos().y() - plannedTrajPort.plannedTraj[currentTrajectoryIndex_nu].yTrajRAReq_m)
            };
            // Calculate intermediate value of ego position between trajectory points with index i0 and i0+1
            float32_t dPath{ vec_iCur_iCur1.scalarProduct(vec_iCur_ego) / std::max(LSM_GEOML::MIN_FLT_DIVISOR, vec_iCur_iCur1.normSq()) };

            //If the orthogonal projection point is (positive) outside the two considered trajectory points:
            if (dPath > 1.00F) {
                //Save information about performed forward step during orthogonal projection.
                stepedForwardOrthogonalProj = true;

                //If the(positive) end of the trajectory is reached, use the second to last trajectory point as result and stop the orthogonal projection loop
                if ((secondToLastTrajectoryIndex_nu >= 0) && (currentTrajectoryIndex_nu == secondToLastTrajectoryIndex_nu)) {
                    positionOnTrajectory.intermediateValue_perc = 1.0F;
                    positionOnTrajectory.intermediateValue_raw_perc = dPath;
                    positionOnTrajectory.currentTrajectoryIndex = currentTrajectoryIndex_nu;
                    positionOnTrajectory.outsideTrajectoryStart_nu = false;
                    positionOnTrajectory.outsideTrajectoryEnd_nu = true;
                    break;
                }
                //If the (positive) end of the trajectory is NOT reached, restart the orthogonal projection loop with the next trajectory points
                else {
                    currentTrajectoryIndex_nu = static_cast<uint8_t>(currentTrajectoryIndex_nu + 1U);
                }
            }
            //If the orthogonal projection point is (negative) outside the two considered trajectory points or
            //mCurrentTrajectoryIndex_nu is equal to or larger than the number of valid trajectory points :
            else if ((dPath < 0.0F) || ((secondToLastTrajectoryIndex_nu >= 0) && (static_cast<int8_t>(currentTrajectoryIndex_nu) > secondToLastTrajectoryIndex_nu))) {
                //If the(negative) end of the trajectory is reached, use the first trajectory point as result and stop the orthogonal projection loop
                if (currentTrajectoryIndex_nu == 0U) {
                    positionOnTrajectory.intermediateValue_perc = 0.0F;
                    positionOnTrajectory.intermediateValue_raw_perc = dPath;
                    positionOnTrajectory.currentTrajectoryIndex = currentTrajectoryIndex_nu;
                    positionOnTrajectory.outsideTrajectoryStart_nu = true;
                    positionOnTrajectory.outsideTrajectoryEnd_nu = false;
                    break;
                }
                //Forward step performed before -> avoid back stepping -> set intermediate trajectory point as output and stop the orthogonal projection loop.
                else if (stepedForwardOrthogonalProj) {
                    positionOnTrajectory.intermediateValue_perc = 0.0F;
                    positionOnTrajectory.intermediateValue_raw_perc = 0.0F;
                    positionOnTrajectory.currentTrajectoryIndex = currentTrajectoryIndex_nu;
                    positionOnTrajectory.outsideTrajectoryStart_nu = false;
                    positionOnTrajectory.outsideTrajectoryEnd_nu = false;
                    break;
                }

                //If the (negative) end of the trajectory is NOT reached, restart the orthogonal projection loop with the previous trajectory points
                else if (currentTrajectoryIndex_nu > 0U) {
                    currentTrajectoryIndex_nu = static_cast<uint8_t>(currentTrajectoryIndex_nu - 1U);
                }
                else {}
            }
            //If dPath is between 0 and 1 use the current index as result and stop the orthogonal projection loop
            else {
                positionOnTrajectory.intermediateValue_perc = dPath;
                positionOnTrajectory.intermediateValue_raw_perc = dPath;
                positionOnTrajectory.currentTrajectoryIndex = currentTrajectoryIndex_nu;
                positionOnTrajectory.outsideTrajectoryStart_nu = false;
                positionOnTrajectory.outsideTrajectoryEnd_nu = false;
                break;
            }
        }

        //Check for discontinuty at current/next trajectoryIndex
        positionOnTrajectory.discontinuityAtCurrentTrajectoryIndex = (positionOnTrajectory.currentTrajectoryIndex == plannedTrajPort.stepInTrajAfterIdx_nu);
    }
}

// copied and adapted from SiLowWrapper.cpp
static std::vector<LSM_GEOML::Pose> removeLessImportantPoints(const std::vector<LSM_GEOML::Pose>& drivenPath, const unsigned int maxArrayLength) {
    std::vector<LSM_GEOML::Pose> minimalDrivenPath = drivenPath;
    while (minimalDrivenPath.size() > maxArrayLength) {
        float32_t minimumDistance = std::numeric_limits<float32_t>::max();
        auto vertexToBeRemoved = minimalDrivenPath.begin();

        for (auto it = std::next(minimalDrivenPath.begin()); it < std::prev(minimalDrivenPath.end()); it++) {
            const cml::Vec2Df vecToPrevPoint = (*std::prev(it)).Pos() - (*it).Pos();
            const cml::Vec2Df vecToNextPoint = (*std::next(it)).Pos() - (*it).Pos();
            const float32_t neighbourPointsDistance = vecToPrevPoint.norm() + vecToNextPoint.norm();

            if (neighbourPointsDistance < minimumDistance) {
                minimumDistance = neighbourPointsDistance;
                vertexToBeRemoved = it;
            }
        }
        minimalDrivenPath.erase(vertexToBeRemoved);
    }
    return minimalDrivenPath;
}

template<uint8_t polygonSize>
static bool vertexInPolygon(const cml::Vec2Df &vertex, const LSM_GEOML::Polygon2D<polygonSize> &polygon) {
    int nvert = polygon.getSize();
    int i, j;
    bool c = false;

    for (i = 0, j = nvert - 1; i < nvert; j = i++) {
        if (((polygon[i].y() > vertex.y()) != (polygon[j].y() > vertex.y())) &&
            (vertex.x() < (polygon[j].x() - polygon[i].x()) * (vertex.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + polygon[i].x()))
            c = !c;
    }
    return c;
}

void TestEvaluation::updateEgoVehScanningPath(const std::list<LSM_GEOML::Pose> &drivenPath,
    const LSM_GEOML::Pose &originTransformation,
    const bool parkingOnLeftSide_nu)
{
    bool lastPointLeftToPBAdded_bool = false;
    LSM_GEOML::Pose lastPointLeftToPB{ 0.0f, 0.0f, 0.0f };
    const LSM_GEOML::LineSegment2D pbSegment = parkingOnLeftSide_nu ?
        LSM_GEOML::LineSegment2D{ testEvaluationPort.maneuveringAreaPolygon[0], testEvaluationPort.maneuveringAreaPolygon[3] } :
        LSM_GEOML::LineSegment2D{ testEvaluationPort.maneuveringAreaPolygon[2], testEvaluationPort.maneuveringAreaPolygon[1] };

    for (auto &vertex : drivenPath) {
        auto egoVehVertexCMOrigin = vertex;
        egoVehVertexCMOrigin.Pos() += originTransformation.Pos();
        egoVehVertexCMOrigin.Yaw_rad() += originTransformation.Yaw_rad();
        const LSM_GEOML::LocationPointToSegment_t egoVehPosLocation = pbSegment.directionOfPoint(egoVehVertexCMOrigin.Pos());

        // save all points that are in the right side of the pb
        if (egoVehPosLocation == LSM_GEOML::LocationPointToSegment_t::ON_SEGMENT ||
            egoVehPosLocation == LSM_GEOML::LocationPointToSegment_t::RIGHT_OF_SEGMENT) {
            // add the last point left to the pb
            if (!lastPointLeftToPBAdded_bool) {
                egoVehScanningPath.push_back(lastPointLeftToPB);
                lastPointLeftToPBAdded_bool = true;
            }
            egoVehScanningPath.push_back(egoVehVertexCMOrigin);
        }
        else {
            lastPointLeftToPB = egoVehVertexCMOrigin;
        }
    }
    if (originTransformation.Pos().x() > egoVehScanningPath.back().Pos().x()) {
        egoVehScanningPath.push_back(originTransformation);
    }

    egoVehScanningPathDefined_bool = true;
}

void TestEvaluation::defineBoundingBox(const LSM_GEOML::Pose &selectedTargetPose,
    const bool parkingOnLeftSide_nu,
    const ap_common::Vehicle_Params egoVehicleParams) {
    testEvaluationPort.boundingBoxPolygon.clear();

    const auto leftBBSegmentExtension_m = - egoVehicleParams.AP_V_WIDTH_M / 2;
    const auto roadsideBBSegmentExtension_m = egoVehicleParams.AP_V_LENGTH_M - egoVehicleParams.AP_V_OVERHANG_REAR_M;
    const auto rightBBSegmentExtension_m = egoVehicleParams.AP_V_WIDTH_M / 2;
    const auto curbsideBBSegmentExtension_m = -egoVehicleParams.AP_V_OVERHANG_REAR_M;

    // build the boundingBoxPolygon from the target pose coordinates
    cml::Vec2Df translatedPoint;
    transformLocalToWorld(selectedTargetPose.Pos().x(), selectedTargetPose.Pos().y(), selectedTargetPose.Yaw_rad(),
        curbsideBBSegmentExtension_m, leftBBSegmentExtension_m, translatedPoint);
    testEvaluationPort.boundingBoxPolygon.append(translatedPoint);

    transformLocalToWorld(selectedTargetPose.Pos().x(), selectedTargetPose.Pos().y(), selectedTargetPose.Yaw_rad(),
        roadsideBBSegmentExtension_m, leftBBSegmentExtension_m, translatedPoint);
    testEvaluationPort.boundingBoxPolygon.append(translatedPoint);

    transformLocalToWorld(selectedTargetPose.Pos().x(), selectedTargetPose.Pos().y(), selectedTargetPose.Yaw_rad(),
        roadsideBBSegmentExtension_m, rightBBSegmentExtension_m, translatedPoint);
    testEvaluationPort.boundingBoxPolygon.append(translatedPoint);

    transformLocalToWorld(selectedTargetPose.Pos().x(), selectedTargetPose.Pos().y(), selectedTargetPose.Yaw_rad(),
        curbsideBBSegmentExtension_m, rightBBSegmentExtension_m, translatedPoint);
    testEvaluationPort.boundingBoxPolygon.append(translatedPoint);

    // sort the points from boundingBoxPolygon in order to respect the parking box vertices order
    if (parkingOnLeftSide_nu) {
        std::sort(testEvaluationPort.boundingBoxPolygon.begin(), testEvaluationPort.boundingBoxPolygon.end(), sortByYAscending);
        if (testEvaluationPort.boundingBoxPolygon[1].x() < testEvaluationPort.boundingBoxPolygon[0].x()) {
            std::swap(testEvaluationPort.boundingBoxPolygon[1], testEvaluationPort.boundingBoxPolygon[0]);
        }
        if (testEvaluationPort.boundingBoxPolygon[2].x() < testEvaluationPort.boundingBoxPolygon[3].x()) {
            std::swap(testEvaluationPort.boundingBoxPolygon[2], testEvaluationPort.boundingBoxPolygon[3]);
        }
    }
    else {
        std::sort(testEvaluationPort.boundingBoxPolygon.begin(), testEvaluationPort.boundingBoxPolygon.end(), sortByYDescending);
        if (testEvaluationPort.boundingBoxPolygon[1].x() > testEvaluationPort.boundingBoxPolygon[0].x()) {
            std::swap(testEvaluationPort.boundingBoxPolygon[1], testEvaluationPort.boundingBoxPolygon[0]);
        }
        if (testEvaluationPort.boundingBoxPolygon[2].x() > testEvaluationPort.boundingBoxPolygon[3].x()) {
            std::swap(testEvaluationPort.boundingBoxPolygon[2], testEvaluationPort.boundingBoxPolygon[3]);
        }
    }
}

void TestEvaluation::defineManeuveringArea(const ap_tp::PoseType &parkingScenarioType,
    const bool parkingOnLeftSide_nu)
{
    float32_t leftPBSegmentExtension_m{ 0.0f }, roadsidePBSegmentExtension_m{ 0.0f }, rightPBSegmentExtension_m{ 0.0f }, curbsidePBSegmentExtension_m{ 0.0f };
    const auto parkingSideCoef_nu = parkingOnLeftSide_nu ? -1.0f : 1.0f;
    const auto boundingBoxOrientation_rad = static_cast<cml::Vec2Df>(-parkingSideCoef_nu * testEvaluationPort.boundingBoxPolygon.getEdge(0)).getAngle();
    cml::Vec2Df translatedPoint;

    if (parkingScenarioType == ap_tp::PoseType::T_PARALLEL_PARKING) {
        const auto leftParkingSideThreshold_m = parkingOnLeftSide_nu ? AP_G_MAN_AREA_PAR_IN_D3_M : AP_G_MAN_AREA_PAR_IN_D4_M;
        const auto rightParkingSideThreshold_m = parkingOnLeftSide_nu ? AP_G_MAN_AREA_PAR_IN_D4_M : AP_G_MAN_AREA_PAR_IN_D3_M;
        leftPBSegmentExtension_m = parkingSideCoef_nu * leftParkingSideThreshold_m;
        roadsidePBSegmentExtension_m = parkingSideCoef_nu * AP_G_MAN_AREA_PAR_IN_D1_M;
        rightPBSegmentExtension_m = -parkingSideCoef_nu * rightParkingSideThreshold_m;
        curbsidePBSegmentExtension_m = -parkingSideCoef_nu * AP_G_MAN_AREA_PAR_IN_D5_M;
    }
    else if (parkingScenarioType == ap_tp::PoseType::T_PERP_PARKING_BWD || parkingScenarioType == ap_tp::PoseType::T_ANGLED_PARKING_REVERSE) {
        const auto leftParkingSideThreshold_m = parkingOnLeftSide_nu ? AP_G_MAN_AREA_PER_B_IN_D3_M : AP_G_MAN_AREA_PER_B_IN_D4_M;
        const auto rightParkingSideThreshold_m = parkingOnLeftSide_nu ? AP_G_MAN_AREA_PER_B_IN_D4_M : AP_G_MAN_AREA_PER_B_IN_D3_M;
        leftPBSegmentExtension_m = parkingSideCoef_nu * leftParkingSideThreshold_m;
        roadsidePBSegmentExtension_m = parkingSideCoef_nu * AP_G_MAN_AREA_PER_B_IN_D1_M;
        rightPBSegmentExtension_m = -parkingSideCoef_nu * rightParkingSideThreshold_m;
        curbsidePBSegmentExtension_m = -parkingSideCoef_nu * AP_G_MAN_AREA_PER_B_IN_D5_M;
    }
    else if (parkingScenarioType == ap_tp::PoseType::T_PERP_PARKING_FWD || parkingScenarioType == ap_tp::PoseType::T_ANGLED_PARKING_STANDARD) {
        const auto leftParkingSideThreshold_m = parkingOnLeftSide_nu ? AP_G_MAN_AREA_PER_F_IN_D3_M : AP_G_MAN_AREA_PER_F_IN_D4_M;
        const auto rightParkingSideThreshold_m = parkingOnLeftSide_nu ? AP_G_MAN_AREA_PER_F_IN_D4_M : AP_G_MAN_AREA_PER_F_IN_D3_M;
        leftPBSegmentExtension_m = parkingSideCoef_nu * leftParkingSideThreshold_m;
        roadsidePBSegmentExtension_m = parkingSideCoef_nu * AP_G_MAN_AREA_PER_F_IN_D1_M;
        rightPBSegmentExtension_m = -parkingSideCoef_nu * rightParkingSideThreshold_m;
        curbsidePBSegmentExtension_m = -parkingSideCoef_nu * AP_G_MAN_AREA_PER_F_IN_D5_M;
    }

    transformLocalToWorld(testEvaluationPort.boundingBoxPolygon[0].x(), testEvaluationPort.boundingBoxPolygon[0].y(), boundingBoxOrientation_rad,
        leftPBSegmentExtension_m, roadsidePBSegmentExtension_m, translatedPoint);
    testEvaluationPort.maneuveringAreaPolygon.append(translatedPoint);

    transformLocalToWorld(testEvaluationPort.boundingBoxPolygon[1].x(), testEvaluationPort.boundingBoxPolygon[1].y(), boundingBoxOrientation_rad,
        rightPBSegmentExtension_m, roadsidePBSegmentExtension_m, translatedPoint);
    testEvaluationPort.maneuveringAreaPolygon.append(translatedPoint);

    transformLocalToWorld(testEvaluationPort.boundingBoxPolygon[2].x(), testEvaluationPort.boundingBoxPolygon[2].y(), boundingBoxOrientation_rad,
        rightPBSegmentExtension_m, curbsidePBSegmentExtension_m, translatedPoint);
    testEvaluationPort.maneuveringAreaPolygon.append(translatedPoint);

    transformLocalToWorld(testEvaluationPort.boundingBoxPolygon[3].x(), testEvaluationPort.boundingBoxPolygon[3].y(), boundingBoxOrientation_rad,
        leftPBSegmentExtension_m, curbsidePBSegmentExtension_m, translatedPoint);
    testEvaluationPort.maneuveringAreaPolygon.append(translatedPoint);
}

void TestEvaluation::defineScanningPathPolygon(const ap_common::Vehicle_Params egoVehicleParams)
{
    const auto maxArrayLength_nu = SCANNING_PATH_MAX_SIZE_NU / 2;
    const auto smallerDrivenPath = removeLessImportantPoints(egoVehScanningPath, maxArrayLength_nu);
    unsigned int iSP = 0U;
    testEvaluationPort.egoVehScanningPathPolygon.setSize(SCANNING_PATH_MAX_SIZE_NU);
    // iterate from the end to the start in order to build the polygon CCW
    for (int iDP = static_cast<int>(smallerDrivenPath.size()) - 1; iDP >= 0; --iDP) {
        cml::Vec2Df translatedPoint;
        // extend the driven area at the end in order to include the ego vehicle shape
        const float32_t addToLastPoint = (iSP == 0U) ?
            egoVehicleParams.AP_V_LENGTH_M - egoVehicleParams.AP_V_OVERHANG_REAR_M + 0.01f :
            0.0f;

        // build the driven area polygon by translating the driven path up and down
        //                          ---------------------
        //   --------------    =>
        //                          ---------------------
        transformLocalToWorld(smallerDrivenPath[iDP].Pos().x(), smallerDrivenPath[iDP].Pos().y(), smallerDrivenPath[iDP].Yaw_rad(), addToLastPoint,
            egoVehicleParams.AP_V_WIDTH_M / 2 + AP_G_MAN_AREA_PAR_IN_D2_M, translatedPoint);
        testEvaluationPort.egoVehScanningPathPolygon[iSP] = translatedPoint;

        transformLocalToWorld(smallerDrivenPath[iDP].Pos().x(), smallerDrivenPath[iDP].Pos().y(), smallerDrivenPath[iDP].Yaw_rad(), addToLastPoint,
            -(egoVehicleParams.AP_V_WIDTH_M / 2 + AP_G_MAN_AREA_PAR_IN_D2_M), translatedPoint);
        testEvaluationPort.egoVehScanningPathPolygon[maxArrayLength_nu * 2 - iSP - 1] = translatedPoint;

        iSP++;
    }
}

void TestEvaluation::createVehicleAreasPoints(const ap_tp::PoseType &parkingType,
    std::vector<cml::Vec2Df> &vehicleArea1Points, std::vector<cml::Vec2Df> &vehicleArea2Points,
    const ap_common::Vehicle_Params egoVehicleParams)
{
    if (parkingType == ap_tp::PoseType::T_PERP_PARKING_BWD) {
        const auto d_7_m = (AP_G_MAN_AREA_IN_PER_B_D7_PERC * egoVehicleParams.AP_V_LENGTH_M) / 100;
        const auto d_8_m = (AP_G_MAN_AREA_IN_PER_B_D8_PERC * egoVehicleParams.AP_V_LENGTH_M) / 100;

        vehicleArea1Points.push_back(cml::Vec2Df(d_7_m - egoVehicleParams.AP_V_OVERHANG_REAR_M, egoVehicleParams.AP_V_WIDTH_M / 2));
        vehicleArea2Points.push_back(cml::Vec2Df(d_8_m - egoVehicleParams.AP_V_OVERHANG_REAR_M, egoVehicleParams.AP_V_WIDTH_M / 2));

        for (unsigned iPt = 0U; iPt < egoVehicleParams.AP_V_NUM_STANDARD_SHAPE_PTS; iPt++){
            if (egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt] < d_7_m - egoVehicleParams.AP_V_OVERHANG_REAR_M)
            {
                vehicleArea1Points.push_back(cml::Vec2Df(egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt], egoVehicleParams.AP_V_STANDARD_SHAPE_Y_M[iPt]));
                if (egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt] < d_8_m - egoVehicleParams.AP_V_OVERHANG_REAR_M)
                {
                    vehicleArea2Points.push_back(cml::Vec2Df(egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt], egoVehicleParams.AP_V_STANDARD_SHAPE_Y_M[iPt]));
                }
            }
        }
        vehicleArea1Points.push_back(cml::Vec2Df(d_7_m - egoVehicleParams.AP_V_OVERHANG_REAR_M, - egoVehicleParams.AP_V_WIDTH_M / 2));
           vehicleArea2Points.push_back(cml::Vec2Df(d_8_m - egoVehicleParams.AP_V_OVERHANG_REAR_M, -egoVehicleParams.AP_V_WIDTH_M / 2));

    }
    else if (parkingType == ap_tp::PoseType::T_PERP_PARKING_FWD) {
        const auto d_7_m = (AP_G_MAN_AREA_IN_PER_F_D7_PERC * egoVehicleParams.AP_V_LENGTH_M) / 100;
        const auto d_8_m = (AP_G_MAN_AREA_IN_PER_F_D8_PERC * egoVehicleParams.AP_V_LENGTH_M) / 100;

        const auto frontVehiclePointX_m = egoVehicleParams.AP_V_LENGTH_M - egoVehicleParams.AP_V_OVERHANG_REAR_M;
        vehicleArea1Points.push_back(cml::Vec2Df(frontVehiclePointX_m - d_7_m, - egoVehicleParams.AP_V_WIDTH_M / 2));
        vehicleArea2Points.push_back(cml::Vec2Df(frontVehiclePointX_m - d_8_m, - egoVehicleParams.AP_V_WIDTH_M / 2));

        for (unsigned iPt = 1U; iPt < egoVehicleParams.AP_V_NUM_STANDARD_SHAPE_PTS; iPt++)
        {
            if (egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt] > frontVehiclePointX_m - d_7_m)
            {
                vehicleArea1Points.push_back(cml::Vec2Df(egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt], egoVehicleParams.AP_V_STANDARD_SHAPE_Y_M[iPt]));
                if (egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt] > frontVehiclePointX_m - d_8_m)
                {
                    vehicleArea2Points.push_back(cml::Vec2Df(egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt], egoVehicleParams.AP_V_STANDARD_SHAPE_Y_M[iPt]));
                }
            }
        }
        vehicleArea1Points.push_back(cml::Vec2Df(egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[0], egoVehicleParams.AP_V_STANDARD_SHAPE_Y_M[0]));
        vehicleArea2Points.push_back(cml::Vec2Df(egoVehicleParams.AP_V_STANDARD_SHAPE_X_M[0], egoVehicleParams.AP_V_STANDARD_SHAPE_Y_M[0]));

        vehicleArea1Points.push_back(cml::Vec2Df(frontVehiclePointX_m - d_7_m, egoVehicleParams.AP_V_WIDTH_M / 2));
        vehicleArea2Points.push_back(cml::Vec2Df(frontVehiclePointX_m - d_8_m, egoVehicleParams.AP_V_WIDTH_M / 2));
    }
    testEvaluationPort.vehicleArea1Polygon_size = static_cast<uint8_t>(vehicleArea1Points.size());
    testEvaluationPort.vehicleArea2Polygon_size = static_cast<uint8_t>(vehicleArea2Points.size());
}

void TestEvaluation::checkVehOutsidePB(const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU> &parkingBoxCoords,
    const ap_tp::PoseType &parkingType,
    const LSM_GEOML::Pose &egoVehiclePoseCMOrigin,
    const ap_tp::PlannedTraj &plannedTraj,
    const ap_common::Vehicle_Params egoVehicleParams)
{
    static std::vector<cml::Vec2Df> vehicleArea1Points;
    static std::vector<cml::Vec2Df> vehicleArea2Points;

    // create vehicle mask for both areas (VehicleArea1 and VehicleArea2)
    if (testEvaluationPort.vehicleArea1Polygon_size == 0 || testEvaluationPort.vehicleArea2Polygon_size == 0)
    {
        vehicleArea1Points.clear();
        vehicleArea2Points.clear();

        createVehicleAreasPoints(parkingType, vehicleArea1Points, vehicleArea2Points, egoVehicleParams);
    }

    if (!testEvaluationPort.vehicleArea1OverlapsPb)
    {
        std::fill(testEvaluationPort.vehicleArea1Polygon.begin(), testEvaluationPort.vehicleArea1Polygon.end(), cml::Vec2Df{ 0.0f,0.0f });
        testEvaluationPort.vehicleArea1Polygon.clear();
        for (unsigned int idxVA1 = 0U; idxVA1 < testEvaluationPort.vehicleArea1Polygon_size; ++idxVA1)
        {
            cml::Vec2Df translatedPoint;
            transformLocalToWorld(egoVehiclePoseCMOrigin.Pos().x(), egoVehiclePoseCMOrigin.Pos().y(),
                egoVehiclePoseCMOrigin.Yaw_rad(), vehicleArea1Points[idxVA1].x(),
                vehicleArea1Points[idxVA1].y(), translatedPoint);
            testEvaluationPort.vehicleArea1Polygon.append(translatedPoint);
        }
        // check if parkingBox contains VehicleArea1
        if (parkingBoxCoords.contains(testEvaluationPort.vehicleArea1Polygon) == LSM_GEOML::PolygonContainmentType::THIS_CONTAINS_OTHER)
        {
            testEvaluationPort.vehicleArea1OverlapsPb = true;
        }
    }
    else if (!testEvaluationPort.plannedEgoVehOutsideBoundaries_bool)
    {
        std::fill(testEvaluationPort.vehicleArea1Polygon.begin(), testEvaluationPort.vehicleArea1Polygon.end(), cml::Vec2Df{ 0.0f,0.0f });
        testEvaluationPort.vehicleArea1Polygon.clear();
        std::fill(testEvaluationPort.vehicleArea2Polygon.begin(), testEvaluationPort.vehicleArea2Polygon.end(), cml::Vec2Df{ 0.0f,0.0f });
        testEvaluationPort.vehicleArea2Polygon.clear();

        for (unsigned int idxVA2 = 0U; idxVA2 < testEvaluationPort.vehicleArea2Polygon_size; ++idxVA2)
        {
            cml::Vec2Df translatedPoint;
            transformLocalToWorld(plannedTraj.xTrajRAReq_m, plannedTraj.yTrajRAReq_m,
                plannedTraj.yawReq_rad, vehicleArea2Points[idxVA2].x(),
                vehicleArea2Points[idxVA2].y(), translatedPoint);
            testEvaluationPort.vehicleArea2Polygon.append(translatedPoint);
        }
        //check if vehicleArea2 exceeds the parkingBox
        if (parkingBoxCoords.contains(testEvaluationPort.vehicleArea2Polygon) != LSM_GEOML::PolygonContainmentType::THIS_CONTAINS_OTHER)
        {
            testEvaluationPort.plannedEgoVehOutsideBoundaries_bool = true;
        }
    }
}

void TestEvaluation::shrinkManeuveringArea(const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> &egoVehicleShape,
    const bool parkingOnLeftSide_nu)
{
    // local coordinate system of parking box (Do not use the maneuvering area for this,
    // since then numerical errors become too large because the maneuvering area can change every cycle.)
    const auto roadDir = testEvaluationPort.boundingBoxPolygon.getEdge(0).getNormalized();
    const auto perpDir = testEvaluationPort.boundingBoxPolygon.getEdge(1).getNormalized();
    // project to axes of local coordinate system
    auto manAreaRoadExtent = testEvaluationPort.maneuveringAreaPolygon.projectToAxis(roadDir);    // first = left PB side; second = right PB side
    auto manAreaPerpExtent = testEvaluationPort.maneuveringAreaPolygon.projectToAxis(perpDir);    // first = road side edge; second = curb side edge
    const auto egoVehicleShapeRoadExtent = egoVehicleShape.projectToAxis(roadDir);
    const auto egoVehicleShapePerpExtent = egoVehicleShape.projectToAxis(perpDir);

    manAreaPerpExtent.first = std::max(manAreaPerpExtent.first, egoVehicleShapePerpExtent.first - AP_G_MAN_AREA_IN_PAR_D7_M);
    if (parkingOnLeftSide_nu) {
        manAreaRoadExtent.second = std::min(manAreaRoadExtent.second, egoVehicleShapeRoadExtent.second + AP_G_MAN_AREA_IN_PAR_D8_M);
    }
    else {
        manAreaRoadExtent.first = std::max(manAreaRoadExtent.first, egoVehicleShapeRoadExtent.first - AP_G_MAN_AREA_IN_PAR_D8_M);
    }

    // construct new maneuvering area
    testEvaluationPort.maneuveringAreaPolygon[0] = manAreaRoadExtent.first * roadDir + manAreaPerpExtent.first * perpDir;
    testEvaluationPort.maneuveringAreaPolygon[1] = manAreaRoadExtent.second * roadDir + manAreaPerpExtent.first * perpDir;
    testEvaluationPort.maneuveringAreaPolygon[2] = manAreaRoadExtent.second * roadDir + manAreaPerpExtent.second * perpDir;
    testEvaluationPort.maneuveringAreaPolygon[3] = manAreaRoadExtent.first * roadDir + manAreaPerpExtent.second * perpDir;
}

LSM_GEOML::Polygon2D<4U> TestEvaluation::expandParkingBox(const LSM_GEOML::Polygon2D<4U> &slotCoordinates_m)
{
    const auto& roadLeft{ slotCoordinates_m[ROAD_LEFT_CORNER_IDX] };
    const auto& roadRight{ slotCoordinates_m[ROAD_RIGHT_CORNER_IDX] };
    const auto& curbRight{ slotCoordinates_m[CURB_RIGHT_CORNER_IDX] };
    const auto& curbLeft{ slotCoordinates_m[CURB_LEFT_CORNER_IDX] };

    // anti-clockwise
    const auto normalizedDirectionVecRoadSide{ cml::Vec2Df{ roadRight - roadLeft }.getNormalized() };
    const auto normalizedDirectionVecRightSide{ cml::Vec2Df{ curbRight - roadRight }.getNormalized() };
    const auto normalizedDirectionVecCurbSide{ cml::Vec2Df{ curbLeft - curbRight }.getNormalized() };
    const auto normalizedDirectionVecLeftSide{ cml::Vec2Df{ roadLeft - curbLeft }.getNormalized() };

    /*expand parking box*/
    const cml::Vec2Df expandedRoadLeft{ roadLeft + normalizedDirectionVecLeftSide * TEST_EVAL_PARKINGBOX_TOLERANCE_CURB_ROAD - normalizedDirectionVecRoadSide * TEST_EVAL_PARKINGBOX_TOLERANCE_LEFT_RIGHT };
    const cml::Vec2Df expandedRoadRight{ roadRight + normalizedDirectionVecRoadSide * TEST_EVAL_PARKINGBOX_TOLERANCE_LEFT_RIGHT - normalizedDirectionVecRightSide * TEST_EVAL_PARKINGBOX_TOLERANCE_CURB_ROAD };
    const cml::Vec2Df expandedCurbRight{ curbRight + normalizedDirectionVecRightSide * TEST_EVAL_PARKINGBOX_TOLERANCE_CURB_ROAD - normalizedDirectionVecCurbSide * TEST_EVAL_PARKINGBOX_TOLERANCE_LEFT_RIGHT };
    const cml::Vec2Df expandedCurbLeft{ curbLeft + normalizedDirectionVecCurbSide * TEST_EVAL_PARKINGBOX_TOLERANCE_LEFT_RIGHT - normalizedDirectionVecLeftSide * TEST_EVAL_PARKINGBOX_TOLERANCE_CURB_ROAD };

    // return expanded parking box
    LSM_GEOML::Polygon2D<4U> expandedParkingBox{ expandedRoadLeft, expandedRoadRight, expandedCurbRight, expandedCurbLeft };
    return expandedParkingBox;
}

/**
*   @brief Determine Time to Park at beginning of Automated Vehicle Guidcance (A)
*
*   @param[in] curvature_first           Curvature of first trajectory point
*   @param[in] initial_steer_angle_rad   Ego steer angle at beginnong of maneuver
*   @param[in] wheelbase_m               Ego vehicle wheelbase
*   @param[in] standstillSteerVel_radps  Standstill steering velocity (at wheels)
*
*   @return Time to Park at beginning of Automated Vehicle Guidcance.
*
*   @impl{L3_AP_241}
*/
const float32_t returnTimeAtBeginning(const float32_t curvature_first, const float32_t  initial_steer_angle_rad, const float32_t wheelbase_m, const float32_t standstillSteerVel_radps) {

    const float32_t gear_change_initial = .0f;
    const float32_t initial_steer_angle_request_rad = std::atan(wheelbase_m * curvature_first);
    const float32_t standstill_steering_time_initial = std::fabs(initial_steer_angle_request_rad - initial_steer_angle_rad) / standstillSteerVel_radps;
    const float32_t secure_change_time_initial = AP_LO_MAX_TIME_UNSECURE_S;

    return std::max({ gear_change_initial, standstill_steering_time_initial, secure_change_time_initial });
}

/**
*   @brief Determine Time to Park at end of Automated Vehicle Guidcance (D)
*
*   @param[in] curvature_last            Curvature of last trajectory point
*   @param[in] wheelbase_m               Ego vehicle wheelbase
*   @param[in] standstillSteerVel_radps  Standstill steering velocity (at wheels)
*
*   @return Time to Park at end of Automated Vehicle Guidcance.
*
*   @impl{L3_AP_244}
*/
const float32_t returnTimeAtEnding(const float32_t curvature_last, const float32_t wheelbase_m, const float32_t standstillSteerVel_radps) {
    const float32_t  final_steer_angle_rad = .0f;
    const float32_t gear_change_time_final = AP_LO_MAX_TIME_GEAR_SECURE_S;
    const float32_t secure_change_time_final = AP_LO_MAX_TIME_SECURE_S;
    const float32_t last_steer_angle_request_rad = std::atan(wheelbase_m * curvature_last);
    const float32_t standstill_steering_time_final = std::fabs(last_steer_angle_request_rad - final_steer_angle_rad) / standstillSteerVel_radps;

    return std::max({ gear_change_time_final, secure_change_time_final, standstill_steering_time_final });
}

/**
*   @brief Determine Time to Park between two strokes of Automated Vehicle Guidcance (B)
*
*   @param[in] curvature_last            Curvature of last trajectory point of old stroke
*   @param[in] curvature_next            Curvature of first trajectory point of new stroke
*   @param[in] isDriveDirectionChanged   Indicator for driving direction change between strokes
*   @param[in] wheelbase_m               Ego vehicle awheelbase
*   @param[in] standstillSteerVel_radps  Standstill steering velocity (at wheels)
*
*   @return Time to Park between two strokes of Automated Vehicle Guidance
*
*   @impl{L3_AP_242}
*/
const float32_t returnTimeBetween2Strokes(const float32_t curvature_last, const float32_t curvature_next, const bool isDriveDirectionChanged, const float32_t  wheelbase_m, const float32_t standstillSteerVel_radps)
{
    const float32_t secure_change_time_stroke_x = .0f;
    const float32_t gear_change_time_stroke_x = (isDriveDirectionChanged) ? AP_LO_MAX_TIME_GEAR_CHANGE_S : .0f;
    const float32_t last_steer_angle_request_rad = std::atan(wheelbase_m * curvature_last);
    const float32_t next_steer_angle_request_rad = std::atan(wheelbase_m * curvature_next);
    const float32_t standstill_steering_time_stroke_x = std::fabs(next_steer_angle_request_rad - last_steer_angle_request_rad)
        / standstillSteerVel_radps;

    return std::max({ secure_change_time_stroke_x, gear_change_time_stroke_x, standstill_steering_time_stroke_x });
}

/**
*   @brief Calculate the minimum velocity request values at the next trajectory point
*
*   @param[in] distanceToStopReq_m        Distance from current trajectory point to the end of current stroke
*   @param[in] absoluteDistanceToStop     Absolute difference between current distance to stop value and the previous value
*   @param[in] velocityLimiReq_mps        VelocityLimit/ target vehicle velocity.(*used as longitudinal reference value in case of velocity control; *used as limit for the vehicle velocity in case of distance control)s
*   @param[in] lastVel_accel_delay        last value of reachableVelocityAtNextTrajPoint
*
*   @return minimum velocity request values at the next trajectory point
*
*   @impl{L3_AP_243}
*/
const float32_t calculateReachableVelocityAtNextTrajPoint(const float32_t distanceToStopReq_m, const float32_t absoluteDistanceToStop, const float32_t velocityLimiReq_mps,
    const float32_t lastVel_accel_delay) {

    const float32_t vel_dist = LSM_GEOML::lookupTable(distanceToStopReq_m, lookUpDist, lookUpVel_mps, 12U);
    const float32_t vel_min = std::min(velocityLimiReq_mps, vel_dist);
    const float32_t timeToNextSample_s = (-lastVel_accel_delay / AP_LO_LONG_ACCELERATION_MAX_MPS2 + std::sqrt(lastVel_accel_delay * lastVel_accel_delay /
        (AP_LO_LONG_ACCELERATION_MAX_MPS2*AP_LO_LONG_ACCELERATION_MAX_MPS2) +
        2 * (absoluteDistanceToStop / AP_LO_LONG_ACCELERATION_MAX_MPS2)));
    const float32_t vel_next_max = lastVel_accel_delay + AP_LO_LONG_ACCELERATION_MAX_MPS2 * timeToNextSample_s;

    return std::min(vel_next_max, vel_min);
}

/*
*   @brief Calculate the integral of a vector (Y) where the spacing between data points can not be uniform (X) using the Trapezoidal numerical integration (https://de.mathworks.com/help/matlab/ref/trapz.html)
*
*   @param[in] Y           Numeric data, specified as a vector
*   @param[in] X           Point spacing
*
*   @return Numerical integration via the trapezoid method of Y with respect to the coordinates specified by X
*
*   @impl{L3_AP_243}
*/
float32_t returnNumericalIntegration(const std::vector<float32_t>& X, const std::vector<float32_t>& Y) {
    float32_t trapz = .0f;
    if (X.size() == Y.size() && X.size() > 1u)
    {
        float32_t sum = (X[1] - X[0])*(Y[1] + Y[0]);
        for (int idx{ 1 }; idx < X.size() - 1; ++idx) {
            sum += (X[idx + 1] - X[idx]) * (Y[idx] + Y[idx + 1]);
        }
        trapz = sum / 2;
    }
    else
    {
        trapz = .0f;
    }
    return trapz;
}

/*
*   @brief Calculate time to park
*
*   @param[in] headUnitScreen_nu_delay                headUnitScreen_nu signal delayed
*   @param[in] headUnitScreen_nu                      HMI screen content depending on system status
*   @param[in] vehicle_Params                         Vehicle parameters
*   @param[in] plannedTraj                            Calculated Trajectory from the Trajectory Planning with EM based velocity limit.
*   @param[in] newSegmentStarted_nu                   Flag that a new segment (e.g. new stroke) was calculated
*   @param[in] drivingForwardReq_nu                   Define the requested driving direction.
*   @param[in] egoMotionFrontWheelAngle_rad           Front wheel angle
*
*
*   @impl{L3_AP_173}
*/
void TestEvaluation::timeToPark(const unsigned char headUnitScreen_nu,
    const ap_common::Vehicle_Params &vehicle_Params,
    const ap_tp::PlannedTrajPort &plannedTrajPort,
    const float32_t egoMotionFrontWheelAngle_rad,
    const float32_t egoMotionVelocity_mps,
    const ap_trjctl::MFControlStatusPort &mFControlStatusPort)
{
    //Beginning of parking maneuver
    if (headUnitScreen_nu_delay != static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE) &&
        headUnitScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE))
    {
        // Determine Time to Park at beginning of Automated Vehicle Guidcance
        auto timeToParkA = returnTimeAtBeginning(plannedTrajPort.plannedTraj[0].crvRAReq_1pm, // use the first trajectory point
            egoMotionFrontWheelAngle_rad, vehicle_Params.AP_V_WHEELBASE_M,
            vehicle_Params.AP_V_COMF_STEER_ANG_VEL_RADPS);
        testEvaluationPort.timeToPark += timeToParkA;
    }

    //Ending of parking maneuver (+last stroke)
    if ((headUnitScreen_nu_delay == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE)) &&
        (headUnitScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_FINISHED)))
    {
        auto timeToParkC = returnNumericalIntegration(distanceOnStrokeList, inverseVelocityAtTrajPoints);
        auto timeToParkD = returnTimeAtEnding(curvature_last,
            vehicle_Params.AP_V_WHEELBASE_M,
            vehicle_Params.AP_V_COMF_STEER_ANG_VEL_RADPS);
        testEvaluationPort.timeToPark += timeToParkC + timeToParkD;
    }

    //During maneuvering determine timeToParkB and timeToParkC and add to timeToPark for each stroke (last stroke not added to timeToPark)
    //@impl{L3_AP_242}
    //@impl{L3_AP_243}
    if (headUnitScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE))
    {
        //Consider only once that a new Segment is started
        if (plannedTrajPort.newSegmentStarted_nu &&
            !newSegmentStarted_nu_delay) {
            //Ignore first stroke for calculating the TimeToParkB because for first stroke covered by TimeToParkA
            if (noOfNewSegment > 0) {
                const bool isDriveDirectionChanged = (drivingForwardReq_nu_delay != static_cast<bool>(plannedTrajPort.drivingForwardReq_nu)) ? true : false;
                auto timeToParkB = returnTimeBetween2Strokes(curvature_last,
                    plannedTrajPort.plannedTraj[0].crvRAReq_1pm, // use the first trajectory point
                    isDriveDirectionChanged,
                    vehicle_Params.AP_V_WHEELBASE_M,
                    vehicle_Params.AP_V_COMF_STEER_ANG_VEL_RADPS);
                auto timeToParkC = returnNumericalIntegration(distanceOnStrokeList, inverseVelocityAtTrajPoints);
                testEvaluationPort.timeToPark += timeToParkB + timeToParkC;
            }
            ++noOfNewSegment;

            inverseVelocityAtTrajPoints.clear();
            distanceOnStrokeList.clear();
            vel_accel_delay = min_velocity;   // Minimum velocity 0.1 m/s
            distanceOnStroke = .0f;
            distanceToStopReq_m_delay = plannedTrajPort.plannedTraj[2].distanceToStopReq_m; // use the third trajectory point

        }
        uint8_t idxTrajPoint { 2u }; // using the third value of plannedTraj points

        if (noOfNewSegment > 0) {
            //check if is the end of a stroke
            if (mFControlStatusPort.longitudinalControlFinished_nu) { idxTrajPoint = plannedTrajPort.numValidCtrlPoints_nu - 1; }
            //otherwise use the third point
            else { idxTrajPoint = 2u; }
            //calculate the difference between last value of distanceToStopReq_m and current
            const float32_t diffBetweenCurrentAndLastDistanceToStopValue_m = distanceToStopReq_m_delay - plannedTrajPort.plannedTraj[idxTrajPoint].distanceToStopReq_m;
            //determine the velocity based on acceleraion limitiation (only when vehicle is moving)
            if (((std::fabs(diffBetweenCurrentAndLastDistanceToStopValue_m) > LSM_CML::MIN_FLT_DIVISOR)
                || distanceOnStrokeList.empty()) &&
                (std::abs(egoMotionVelocity_mps) > min_velocity))
            {
                distanceOnStroke += std::fabs(diffBetweenCurrentAndLastDistanceToStopValue_m);
                vel_accel_delay = calculateReachableVelocityAtNextTrajPoint(plannedTrajPort.plannedTraj[idxTrajPoint].distanceToStopReq_m,
                    std::fabs(diffBetweenCurrentAndLastDistanceToStopValue_m),
                    plannedTrajPort.plannedTraj[idxTrajPoint].velocityLimitReq_mps,
                    vel_accel_delay);

                auto inverseVelocityAtTrajPointsValue = 1.f / vel_accel_delay;
                inverseVelocityAtTrajPoints.push_back(inverseVelocityAtTrajPointsValue);
                distanceOnStrokeList.push_back(distanceOnStroke);
            }
        }
        curvature_last = plannedTrajPort.plannedTraj[idxTrajPoint].crvRAReq_1pm;
        distanceToStopReq_m_delay = plannedTrajPort.plannedTraj[idxTrajPoint].distanceToStopReq_m;
        drivingForwardReq_nu_delay = plannedTrajPort.drivingForwardReq_nu;
        newSegmentStarted_nu_delay = plannedTrajPort.newSegmentStarted_nu;
    }
}

static int8_t isTargetPoseOutsidePB(const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU> &slotCoordinates_m,
    const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> &boundingBox,const ap_tp::PoseType selcectedPoseType_nu) {

    //check if it is not a Park Out scenario
    if ((selcectedPoseType_nu != ap_tp::PoseType::T_PAR_PARKING_OUT) &&
        (selcectedPoseType_nu != ap_tp::PoseType::T_PERP_PARKING_OUT_BWD) &&
        (selcectedPoseType_nu != ap_tp::PoseType::T_PERP_PARKING_OUT_FWD)) {
        return (slotCoordinates_m.contains(boundingBox) != LSM_GEOML::PolygonContainmentType::THIS_CONTAINS_OTHER) ? 1:0;
    }
    else {
        return -1;
    }
}
static int8_t isTargetPoseInCollision(const si::StaticObjectSerializable &staticObject,
    const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> &boundingBox) {
    if ((staticObject.existenceProb_perc > EXISTENCE_PROB_PERC) &&
        (   (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_UNKNOWN) ||
            (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_DOOR_OPENABLE) ||
            (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE) ||
            (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_HANGING_OBJECT) ||
            (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_LOWER_BUMPER_HEIGHT))){

            return (convert(staticObject.objShape_m).doPolygonsOverlap(boundingBox)) ? 1 : 0;
    }
    else {
        return -1;
    }
}

void TestEvaluation::optimalTargetPoseEvaluation(const si::ParkingScenarioTypes &parkingScenarioType_nu,
    const LSM_GEOML::Pose &egoPossitionAtEndOfParkingManeuver) {
    //check if the OptimalTargetPose is defined
    if (evaluationPort.optimalTargetPose.valid) {

        //set the optimalTargetPose treshold for evaluation
        if (parkingScenarioType_nu == si::ParkingScenarioTypes::PARALLEL_PARKING) {
#ifdef VARIANT_CUS_ONLY
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_PARALLEL_CUS_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_PARALLEL_CUS_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_PARALLEL_CUS_GRAD;
#elif defined (VARIANT_PERFORMANCE)
			testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_PARALLEL_PERF_M;
			testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_PARALLEL_PERF_M;
			testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_PARALLEL_PERF_GRAD;
#else
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_PARALLEL_PREM_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_PARALLEL_PREM_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_PARALLEL_PREM_GRAD;
#endif // VARIANT_CUS_ONLY

        }
        else if (parkingScenarioType_nu == si::ParkingScenarioTypes::PERPENDICULAR_PARKING) {
#ifdef VARIANT_CUS_ONLY
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_PERP_CUS_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_PERP_CUS_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_PERP_CUS_GRAD * LSM_GEOML::LSM_PI / 180.0f;
#elif defined (VARIANT_PERFORMANCE)
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_PERP_PERF_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_PERP_PERF_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_PERP_PERF_GRAD * LSM_GEOML::LSM_PI / 180.0f;
#else
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_PERP_PREM_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_PERP_PREM_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_PERP_PREM_GRAD;
#endif // VARIANT_CUS_ONLY
        }
        else if (parkingScenarioType_nu == si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_BACK ||
            parkingScenarioType_nu == si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_FRONT) {
#ifdef VARIANT_CUS_ONLY
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_ANG_CUS_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_ANG_CUS_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_ANG_CUS_GRAD * LSM_GEOML::LSM_PI / 180.0f;
#elif defined (VARIANT_PERFORMANCE)
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_ANG_PERF_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_ANG_PERF_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_ANG_PERF_GRAD * LSM_GEOML::LSM_PI / 180.0f;
#else
            testEvaluationPort.latDiffErrorTreshold_m = MAX_LAT_DIST_ERR_ANG_PREM_M;
            testEvaluationPort.longDiffErrorTreshold_m = MAX_LONG_DIST_ERR_ANG_PREM_M;
            testEvaluationPort.yawDiffErrorTreshold_deg = MAX_YAW_ERR_PERP_ANG_GRAD;
#endif // VARIANT_CUS_ONLY
        }
        else {
            //do nothing
        }

        const LSM_GEOML::CoordinateTransformer2D transformToOTPOrigin(evaluationPort.optimalTargetPose.pose);

        //transform ego Pose at the end of Parking to OTP origin to calculate the difference between them
        const LSM_GEOML::Pose finalEgoPoseOTPOrigin = transformToOTPOrigin.inverseTransform(egoPossitionAtEndOfParkingManeuver);

        testEvaluationPort.longDiffOptimalTP_FinalEgoPose_m = std::fabs(finalEgoPoseOTPOrigin.Pos().x());
        testEvaluationPort.latDiffOptimalTP_FinalEgoPose_m = std::fabs(finalEgoPoseOTPOrigin.Pos().y());
        testEvaluationPort.yawDiffOptimalTP_FinalEgoPose_deg = std::fabs(finalEgoPoseOTPOrigin.Yaw_rad() * 180.0f / LSM_GEOML::LSM_PI);


        //calculate the difference beween the Optimal Target pose and Target Pose at the end of parking maneuver
        //during/at end of maneuvering the selected/chosen target pose is always at index 0 of targetPosesPort.targetPoses
        //transform Target Pose at the end of Parking to OTP origin to calculate the difference between them
        const LSM_GEOML::Pose targetPoseOTPOrigin = transformToOTPOrigin.inverseTransform(testEvaluationPort.finalTargetPose);

        testEvaluationPort.longDiffOptimalTP_TargetPose_m = std::fabs(targetPoseOTPOrigin.Pos().x());
        testEvaluationPort.latDiffOptimalTP_TargetPose_m = std::fabs(targetPoseOTPOrigin.Pos().y());
        testEvaluationPort.yawDiffOptimalTP_TargetPose_deg = std::fabs(targetPoseOTPOrigin.Yaw_rad() * 180.0f / LSM_GEOML::LSM_PI);
    }
}

//a method that assigns value to previousStep or currentStep
void assigneValueToCorrectStep(int& previousStep, int& currentStep, unsigned int value) {
    if (previousStep == -1) {
        previousStep = value;
    }
    else if (currentStep == -1) {
        currentStep = value;
    }
    else {
        previousStep = currentStep;
        currentStep = value;
    }
}

static LSM_GEOML::Pose getPose(const lsm_vedodo::OdoEstimation &odoEstimation) {
    return LSM_GEOML::Pose{ odoEstimation.xPosition_m, odoEstimation.yPosition_m, odoEstimation.yawAngle_rad };
}

static void evaluateShortestDistanceToTrafficObjects(const lsm_vedodo::OdoEstimation& odoEstimation,
                                                     const lsm_vedodo::OdoEstimation& odoEstimationCM,
                                                     const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU>& vehicleContour,
                                                     const TrafficContour2D trafficContour2D[],
                                                     std::array<float32_t, MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM>& shortestDistanceOdo,
                                                     std::array<float32_t, MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM>& shortestDistanceCM)
{
    const auto egoVehicleShapeOdo = returnBoundingBox(vehicleContour, getPose(odoEstimation));
    const auto egoVehicleShapeCM = returnBoundingBox(vehicleContour, getPose(odoEstimationCM));

    for (unsigned int i{ 0U }; i < std::min((unsigned int)Traffic.nObjs, (unsigned int)MAX_NUMBER_OF_TRAFFIC_OBJECTS_CM); ++i) {
        const tTrafficObj* trafficObj = Traffic_GetByTrfId(i);

        constexpr lsm_geoml::size_type OBJ_POLY_MAX_SIZE{ 30U };  //maximal amount of connected polygon points before split
        cml::Vec2Df_POD objectContourPoints[OBJ_POLY_MAX_SIZE];
        const bool isSplit{ false };
        bool needToBeSplit{ false };

        objectType objectTyp;
        if (trfObjIsOdo(&trafficObj->Cfg) || trfObjIsParkBox(&trafficObj->Cfg) || trfObjIsODS(&trafficObj->Cfg) ||
            trfObjIsRoadLaneMark(&trafficObj->Cfg) || trfObjIsParkLineMark(&trafficObj->Cfg)) {
            continue;
        } 
        else if (trfObjIsStatic(trafficObj) || trfObjIsDyn(trafficObj) || trfObjIsCurbstone(&trafficObj->Cfg) ||
            trfObjIsWhlStp(&trafficObj->Cfg) || trfObjIsRectStatic(&trafficObj->Cfg)) {
            objectTyp = objectType::Other;
        }
        else if (trfObjIsRoundStatic(&trafficObj->Cfg)) {
            objectTyp = objectType::CircleObstacle;
        }
        else {
            continue;
        }
        const uint8_t numContourPoints = calculateObjectContourPoints(trafficObj, trafficContour2D[i], 0.0f, 0.0f, objectTyp, objectContourPoints, isSplit, needToBeSplit, OBJ_POLY_MAX_SIZE);
        LSM_GEOML::Polygon2D<OBJ_POLY_MAX_SIZE> trafficObjectShape{};
        for (uint8_t iPt{ 0U }; iPt < numContourPoints; ++iPt) {
            trafficObjectShape.append(objectContourPoints[iPt]);
        }

        shortestDistanceOdo[i] = egoVehicleShapeOdo.calcDistance(trafficObjectShape);
        shortestDistanceCM[i] = egoVehicleShapeCM.calcDistance(trafficObjectShape);
    }
}

void computeTRJPLAnumberOfCrvSteps(ap_tp::PlannedTrajPort plannedTrajPort, ap_common::Vehicle_Params vehicle_Params, unsigned int& numberOfCrvSteps) {
    // because we only need the last 2 JUMP or NO_JUMP values we can use 2 variables and a function to assign the correnct values
    int previousStep = -1;
    int currentStep = -1;
    const unsigned int JUMP = 1;
    const unsigned int NO_JUMP = 0;
    for (unsigned int i = 1; i < ap_tp::AP_TP_Const::AP_P_MAX_NUM_TRAJ_CTRL_POINTS; i++) {
        const float distToNextSample = sqrtf(fabs(pow(plannedTrajPort.plannedTraj[i].xTrajRAReq_m - plannedTrajPort.plannedTraj[i - 1].xTrajRAReq_m, 2) + \
            pow(plannedTrajPort.plannedTraj[i].yTrajRAReq_m - plannedTrajPort.plannedTraj[i - 1].yTrajRAReq_m, 2)));

        if (vehicle_Params.AP_V_WHEELBASE_M * plannedTrajPort.plannedTraj[i].velocityLimitReq_mps == 0) {
            assigneValueToCorrectStep(previousStep, currentStep, NO_JUMP);
        }
        else {
            const float curvatureDelta = fabs(plannedTrajPort.plannedTraj[i].crvRAReq_1pm - plannedTrajPort.plannedTraj[i - 1].crvRAReq_1pm);
            const float allowedCurvatureDelta = (vehicle_Params.AP_V_MAX_STEER_ANG_VEL_RADPS * distToNextSample) / (vehicle_Params.AP_V_WHEELBASE_M * plannedTrajPort.plannedTraj[i - 1].velocityLimitReq_mps);
            //Workarround to allow 5 times the allowed curvature step as defined in requirement to ignore small curvature steps (see DFMFPT-2197)
            const float allowedCurvatureDeltaTolerate = 5 * allowedCurvatureDelta;
            if (curvatureDelta > allowedCurvatureDeltaTolerate) {
                assigneValueToCorrectStep(previousStep, currentStep, JUMP);
            }
            else {
                assigneValueToCorrectStep(previousStep, currentStep, NO_JUMP);
            }
        }
        if (previousStep == 0 && currentStep == 1) {
            numberOfCrvSteps++;
        }
    }
}

void ComparenumberOfStrokes(const unsigned int numberOfStrokes, const unsigned int maxNumberOfStrokes, bool& tooManyStrokes) {
#ifdef VARIANT_CUS_ONLY
    std::string filePath = "..\\Report_Generator\\NoOfStrokes_TRJPLA_CUS.txt";
#elif defined (VARIANT_PERFORMANCE)
    std::string filePath = "..\\Report_Generator\\NoOfStrokes_TRJPLA_Performance.txt";
#else
    std::string filePath = "..\\Report_Generator\\NoOfStrokes_TRJPLA.txt";
#endif // VARIANT_CUS_ONLY
    std::string tempFilePath = "..\\Report_Generator\\temp.txt";
    std::string line;
    std::string curName = SimCore.TestRun.Name;
    curName = curName.substr(curName.find("/") + 1);
    unsigned long n_strokes_max;

    std::ifstream MyReadFile(filePath);
    bool found = false;

    while (std::getline(MyReadFile, line)) {
        if (line.find(curName) != std::string::npos) {
            found = true;
            std::string text = curName + " numberOfStrokes = ";
            n_strokes_max = std::stoul(line.replace(0, text.length(), ""));
            //for most Testruns the maximum allowed number of strokes is defined in the Testrun Description; if it was not yet set to a specific value it will be MAX_NO_STROKES, larger or 0 (see MAX_NO_STROKES)
            //the check whether the number of needed strokes was increased compared to last time shall only be activated in case the maximum number of strokes is not defined in the Testrun Description (>=MAX_NO_STROKES || 0)
            if ((maxNumberOfStrokes >= MAX_NO_STROKES) || (maxNumberOfStrokes == 0U)) {
                //compare the recorded max number of strokes against the current number. If the curent number is greater that the recorded one, set the trigger to true
                if (numberOfStrokes > n_strokes_max) {
                    tooManyStrokes = true;
                }
            }
            break;
        }
    }
    MyReadFile.close();

    std::string new_line = curName + " numberOfStrokes = " + std::to_string(numberOfStrokes) + "\n";
    //if a line coresponding to the current test scenario was not found in the text file add a new line coresponding to the current test scenario
    if (!found) {
        std::ofstream MyWriteFile(filePath, std::ofstream::app);
        MyWriteFile << new_line;
        MyWriteFile.close();
    }
    //if a line was found but the current max number of strokes si smaller that the one recorded in the file, update the file with the smaller number
    //this is done by creating a temporary text file in wich we copy all the lines from NoOfStrokes_TRJPLA.txt except for the one we want to update,
    //delete the old file and rename the temporaty file to NoOfStrokes_TRJPLA.txt
    else if (numberOfStrokes < n_strokes_max) {
        std::ifstream MyReadFile(filePath);
        std::ofstream MyWriteFile(tempFilePath, std::ofstream::trunc);
        while (std::getline(MyReadFile, line)) {
            if (line.find(curName) == std::string::npos) {
                MyWriteFile << line << "\n";
            }
            else {
                MyWriteFile << new_line;
            }
        }
        MyReadFile.close();
        MyWriteFile.close();
        remove(filePath.c_str());
        rename(tempFilePath.c_str(), filePath.c_str());
    }
}
void TestEvaluation::runEveryCycle(
    const ap_tp::PlannedTrajPort& plannedTrajPort,
    const ap_common::CarMakerInterface& carMakerInterface,
    const ap_commonvehsigprovider::Gear& gearCur_nu,
    const bool hadGearReq_nu,
    const float32_t drivenDistance_m)
{
    //Determine Number of Strokes (based on gear change R vs D); increase numberOfStrokes if gear changes from R to D or D to R; during active (remote) maneuver
    if (hadGearReq_nu &&
        ((carMakerInterface.headUnitScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE)) ||
        (carMakerInterface.remoteScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE)))
        )
    {
        if ((gearNo_delay != static_cast<uint8_t>(ap_commonvehsigprovider::Gear::GEAR_D)) && (gearCur_nu == ap_commonvehsigprovider::Gear::GEAR_D)) //check if previos value was not GEAR_D
            testEvaluationPort.numberOfStrokes_gear++;
        else if ((gearNo_delay != static_cast<uint8_t>(ap_commonvehsigprovider::Gear::GEAR_R)) && (gearCur_nu == ap_commonvehsigprovider::Gear::GEAR_R))//check if previos value was not GEAR_R
            testEvaluationPort.numberOfStrokes_gear++;
    }
    if ((gearCur_nu == ap_commonvehsigprovider::Gear::GEAR_R) || (gearCur_nu == ap_commonvehsigprovider::Gear::GEAR_D)) //for number of strokes only R<->D relevant
    {
        gearNo_delay = static_cast<uint8_t>(gearCur_nu);
    }

    //Determine Number of Strokes (based on newSegmentStarted flag); increase numberOfStrokes_newSegmentStarted in case of rising edge in plannedTrajPort.newSegmentStarted_nu
    if (plannedTrajPort.newSegmentStarted_nu && (!newSegmentStarted_Last)) {
        testEvaluationPort.numberOfStrokes_newSegmentStarted++;
    }
    newSegmentStarted_Last = plannedTrajPort.newSegmentStarted_nu;

    //Set number of strokes to maximum of gear change and newSegmentStarted_nu rising edge based determination of strokes:
    const unsigned oldNumberOfStrokes{ testEvaluationPort.numberOfStrokes };
    testEvaluationPort.numberOfStrokes = std::max(testEvaluationPort.numberOfStrokes_gear, testEvaluationPort.numberOfStrokes_newSegmentStarted);

    // Determine the driven distance on the current stroke
    static float32_t drivenDistanceCurrentStrokeStarted_m{ 0.0F };
    if (testEvaluationPort.numberOfStrokes != oldNumberOfStrokes) {
        drivenDistanceCurrentStrokeStarted_m = drivenDistance_m;
    }
    testEvaluationPort.drivenDistanceOnStroke_m = (0U == testEvaluationPort.numberOfStrokes) ? 0.0F : (drivenDistance_m - drivenDistanceCurrentStrokeStarted_m);
}

void TestEvaluation::run(const ap_tp::PlannedTrajPort &plannedTrajPort,
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
    const TrafficContour2D trafficContour2D[])
{
    // always executed to have some signals (e.g. AP.numberOfStrokes, AP.drivenDistanceOnStroke_m) always available
    runEveryCycle(plannedTrajPort, carMakerInterface, gearCur_nu, hadGearReq_nu, odoEstimationPortCM.drivenDistance_m);

    const uint64_t timeStamp_ms = static_cast<uint64_t>(std::round(SimCore.Time * 1000));
    // For better performance, prepare quantities for the test evaluation only if the latter is running
    // and only every 10 ms, which is the sample period for the CarMaker result files.
    if ((!testEvaluationActive) || ((timeStamp_ms % 10U) != 0U)) {
        return;
    }

    //calculate Time to Park (impl{L3_AP_173})
    timeToPark(carMakerInterface.headUnitScreen_nu,
        carMakerInterface.vehicle_Params,
        plannedTrajPort,
        egoMotionFrontWheelAngle_rad,
        egoMotionVelocity_mps,
        mFControlStatusPort);

    //cml::Vec2Df cornerPointEgoVehicle[4];
    //cml::Vec2Df cornerPointParkingBox[4];
    //cml::Vec2Df CrossingParkingBoxCorner[9], CrossingEgoVehicleCorner[5];
    //cml::Vec2Df initialCornerEgoVehicle[4];
    //cml::Vec2Df cornerEgoVehicle[5];
    cml::Vec2Df pbDiag_m[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU];
    cml::Vec2Df evDiag_m[ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU];
    //float am_lo_rear= 20, am_la_side=3, am_lo_front=0.2, am_la_road=0.6;
    //float am_p_rear=2, am_p_end=0.3, am_p_front=2;

    const float CarLenght = 4.767f;
    const float CarWidth = 1.832f;
    /*float xFrontPoint,xFrontPointParkingBox,xRearPointParkingBox,xRearPoint,yRoadPoint,ySidePoint,yEndParkingBoxPoint;*/
    float x_pb1, y_pb1, x_pb2, y_pb2, x_ev3, y_ev3, x_ev4, y_ev4, crossPointX, crossPointY;
    //ParkingBoxes references are calculated in "updateTestrunParkingBox" function
    // Define all for corner points of a ParkingBox

    transformLocalToWorld(Car.Fr1.t_0[0], Car.Fr1.t_0[1], Car.Fr1.r_zyx[2], CarLenght, CarWidth, cornerEgoVehicle[0]);
    /*transform rear left*/
    transformLocalToWorld(Car.Fr1.t_0[0], Car.Fr1.t_0[1], Car.Fr1.r_zyx[2], 0.0, CarWidth, cornerEgoVehicle[1]);
    /*transform rear right*/
    transformLocalToWorld(Car.Fr1.t_0[0], Car.Fr1.t_0[1], Car.Fr1.r_zyx[2], 0.0, 0.0, cornerEgoVehicle[2]);
    /*transform front right*/
    transformLocalToWorld(Car.Fr1.t_0[0], Car.Fr1.t_0[1], Car.Fr1.r_zyx[2], CarLenght, 0.0, cornerEgoVehicle[3]);

    if (carMakerInterface.headUnitScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE) && headUnitScreen_nu_delay != static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE))
    {
        initialCornerEgoVehicle[0] = cornerEgoVehicle[0];
        initialCornerEgoVehicle[1] = cornerEgoVehicle[1];
        initialCornerEgoVehicle[2] = cornerEgoVehicle[2];
        initialCornerEgoVehicle[3] = cornerEgoVehicle[3];

    }

    if (gParkingBoxPort.numValidParkingBoxes_nu > 0)
    {
        for (unsigned int i = 0; i < gParkingBoxPort.numValidParkingBoxes_nu; ++i)
        {
            //front point of AMS (only Vehicle)
            evDiag_m[i] = initialCornerEgoVehicle[0] - initialCornerEgoVehicle[3];
            if (evDiag_m[i][0] > 0)
                cornerPointEgoVehicle[0] = initialCornerEgoVehicle[0];
            else
                cornerPointEgoVehicle[0] = initialCornerEgoVehicle[3];

            //rear point of AMS (only Vehicle)
            evDiag_m[i] = abs(initialCornerEgoVehicle[1]) - abs(initialCornerEgoVehicle[2]);
            if (evDiag_m[i][1] > 0)
                cornerPointEgoVehicle[1] = initialCornerEgoVehicle[1];
            else
                cornerPointEgoVehicle[1] = initialCornerEgoVehicle[2];
            if (parkingOnLeftSide_nu[i] == false) {
                //side point of AMS (only Vehicle)
                evDiag_m[i] = initialCornerEgoVehicle[2] - initialCornerEgoVehicle[3];
                if (evDiag_m[i][0] > 0)
                    cornerPointEgoVehicle[2] = initialCornerEgoVehicle[2];
                else
                    cornerPointEgoVehicle[2] = initialCornerEgoVehicle[3];

                //road point of AMS (only Vehicle)
                evDiag_m[i] = abs(initialCornerEgoVehicle[0]) - abs(initialCornerEgoVehicle[1]);
                if (evDiag_m[i][1] > 0)
                    cornerPointEgoVehicle[3] = initialCornerEgoVehicle[0];
                else
                    cornerPointEgoVehicle[3] = initialCornerEgoVehicle[1];


                //front point of PB
                pbDiag_m[i] = cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[0]) - cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3]);
                if (pbDiag_m[i][0] > 0)
                    cornerPointParkingBox[0] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[0];
                else
                    cornerPointParkingBox[0] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3];

                //rear extrem point of PB
                pbDiag_m[i] = abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[1]) - abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2]);
                if (pbDiag_m[i][1] > 0)
                    cornerPointParkingBox[1] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[1];
                else
                    cornerPointParkingBox[1] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2];

                //end extrem point of PB
                pbDiag_m[i] = cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2]) - cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3]);
                if (pbDiag_m[i][0] > 0)
                    cornerPointParkingBox[2] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2];
                else
                    cornerPointParkingBox[2] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3];

                //up extrem point of PB // doesn't matter
                pbDiag_m[i] = abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2]) - abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3]);
                if (pbDiag_m[i][1] > 0)
                    cornerPointParkingBox[3] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2];
                else
                    cornerPointParkingBox[3] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3];
            }
            else {
                //side point of AMS (only Vehicle)
                evDiag_m[i] = initialCornerEgoVehicle[0] - initialCornerEgoVehicle[1];
                if (evDiag_m[i][0] > 0)
                    cornerPointEgoVehicle[2] = initialCornerEgoVehicle[0];
                else
                    cornerPointEgoVehicle[2] = initialCornerEgoVehicle[1];

                //road point of AMS (only Vehicle)
                evDiag_m[i] = abs(initialCornerEgoVehicle[2]) - abs(initialCornerEgoVehicle[3]);
                if (evDiag_m[i][1] > 0)
                    cornerPointEgoVehicle[3] = initialCornerEgoVehicle[2];
                else
                    cornerPointEgoVehicle[3] = initialCornerEgoVehicle[3];

                //front point of PB
                pbDiag_m[i] = cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2]) - cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[1]);
                if (pbDiag_m[i][0] > 0)
                    cornerPointParkingBox[0] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2];
                else
                    cornerPointParkingBox[0] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[1];

                //rear extrem point of PB
                pbDiag_m[i] = abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3]) - abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[0]);
                if (pbDiag_m[i][1] > 0)
                    cornerPointParkingBox[1] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3];
                else
                    cornerPointParkingBox[1] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[0];

                //end extrem point of PB
                pbDiag_m[i] = cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2]) - cml::Vec2Df(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3]);
                if (pbDiag_m[i][0] > 0)
                    cornerPointParkingBox[2] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2];
                else
                    cornerPointParkingBox[2] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3];

                //up extrem point of PB // doesn't matter
                pbDiag_m[i] = abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[1]) - abs(gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[0]);
                if (pbDiag_m[i][1] > 0)
                    cornerPointParkingBox[3] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[1];
                else
                    cornerPointParkingBox[3] = gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[0];
            }
        }
    }

    //front x position of an allowed maneuvering Space
    xFrontPoint = cornerPointEgoVehicle[0].x() + am_lo_front;

    //front PB x position of an allowed maneuvering Space
    xFrontPointParkingBox = cornerPointParkingBox[0].x() + am_p_front;

    //rear PB x position of an allowed maneuvering Space
    xRearPointParkingBox = cornerPointParkingBox[1].x() - am_p_rear;

    //rear x position of an allowed maneuvering Space
    xRearPoint = cornerPointEgoVehicle[1].x() - am_lo_rear;

    //road y position of an allowed maneuvering Space
    yRoadPoint = cornerPointEgoVehicle[2].y() - am_la_road; //y axes rightEgoVehicle is a negative value

    //side y position of an allowed maneuvering Space
    ySidePoint = cornerPointEgoVehicle[3].y() + am_la_side;

    //end PB y position of an allowed maneuvering Space
    yEndParkingBoxPoint = cornerPointParkingBox[1].y() - am_p_end;//y axes rearParkingBox is a negative value

    CrossingParkingBoxCorner[0].x() = xFrontPoint;           // front-side Corner
    CrossingParkingBoxCorner[0].y() = ySidePoint;            // front-side Corner
    CrossingParkingBoxCorner[1].x() = xFrontPointParkingBox; // frontParkingBox-side Corner
    CrossingParkingBoxCorner[1].y() = ySidePoint;            // frontParkingBox-side Corner

    CrossingParkingBoxCorner[2].x() = xFrontPointParkingBox; // frontParkingBox-end Corner
    CrossingParkingBoxCorner[2].y() = yEndParkingBoxPoint;   // frontParkingBox-end Corner
    CrossingParkingBoxCorner[3].x() = xRearPointParkingBox;  // rearParkingBox -end Corner
    CrossingParkingBoxCorner[3].y() = yEndParkingBoxPoint;   // rearParkingBox -end Corner

    CrossingParkingBoxCorner[4].x() = xRearPointParkingBox;  // rearParkingBox-side Corner
    CrossingParkingBoxCorner[4].y() = ySidePoint;            // rearParkingBox-side Corner
    CrossingParkingBoxCorner[5].x() = xRearPoint;            // rear-side Corner
    CrossingParkingBoxCorner[5].y() = ySidePoint;            // rear-side Corner

    CrossingParkingBoxCorner[6].x() = xRearPoint;           // front-road Corner
    CrossingParkingBoxCorner[6].y() = yRoadPoint;           // front-road Corner
    CrossingParkingBoxCorner[7].x() = xFrontPoint;          // rear-road Corner
    CrossingParkingBoxCorner[7].y() = yRoadPoint;           // rear-road Corner

                                                            //to make x0y0-x0y1 line
    CrossingParkingBoxCorner[8].x() = xFrontPoint;          // front-side Corner
    CrossingParkingBoxCorner[8].y() = ySidePoint;           // front-side Corner

    CrossingEgoVehicleCorner[0] = cornerEgoVehicle[0];
    CrossingEgoVehicleCorner[1] = cornerEgoVehicle[1];
    CrossingEgoVehicleCorner[2] = cornerEgoVehicle[2];
    CrossingEgoVehicleCorner[3] = cornerEgoVehicle[3];
    CrossingEgoVehicleCorner[4] = cornerEgoVehicle[0];

    int collision_flag = 0;
    for (int i = 0; i <= 7; i++)
    {
        for (int j = 0; j <= 3; j++)
        {
            x_pb1 = CrossingParkingBoxCorner[i].x();
            y_pb1 = CrossingParkingBoxCorner[i].y();
            x_pb2 = CrossingParkingBoxCorner[i + 1].x();
            y_pb2 = CrossingParkingBoxCorner[i + 1].y();

            x_ev3 = CrossingEgoVehicleCorner[j].x();
            y_ev3 = CrossingEgoVehicleCorner[j].y();
            x_ev4 = CrossingEgoVehicleCorner[j + 1].x();
            y_ev4 = CrossingEgoVehicleCorner[j + 1].y();

            //calculate crosspoint of two lines
            crossPointX = ((x_pb1*y_pb2 - y_pb1 * x_pb2) * (x_ev3 - x_ev4) - (x_pb1 - x_pb2) * (x_ev3*y_ev4 - y_ev3 * x_ev4)) / ((x_pb1 - x_pb2)*(y_ev3 - y_ev4) - (y_pb1 - y_pb2)*(x_ev3 - x_ev4));
            crossPointY = ((x_pb1*y_pb2 - y_pb1 * x_pb2) * (y_ev3 - y_ev4) - (y_pb1 - y_pb2) * (x_ev3*y_ev4 - y_ev3 * x_ev4)) / ((x_pb1 - x_pb2)*(y_ev3 - y_ev4) - (y_pb1 - y_pb2)*(x_ev3 - x_ev4));

            //Check if the crosspoint is on the lines
            // for(k=0;k<x1;k++) //for i in range(len(x1)): (python syntax)
            // {
            //if (min(x1[k], x2[k]) < crossPointX[k] < max(x1[k], x2[k])) && (min(x3[k], x4[k]) < crossPointX[k] < max (x3[k], x4[k])) &&
            //    (min(y1[k], y2[k]) < crossPointY[k] < max(y1[k], y2[k])) && (min(y3[k], y4[k]) < crossPointY[k] < max (y3[k], y4[k]))
            if (((std::min(x_pb1, x_pb2) < crossPointX) && (crossPointX < std::max(x_pb1, x_pb2)))\
                && ((std::min(x_ev3, x_ev4) < crossPointX) && (crossPointX < std::max(x_ev3, x_ev4))) \
                && ((std::min(y_pb1, y_pb2) < crossPointY) && (crossPointY < std::max(y_pb1, y_pb2))) && \
                ((std::min(y_ev3, y_ev4) < crossPointY) && (crossPointY < std::max(y_ev3, y_ev4))))
            {
                collision_flag = 1;
                break;
            }
            // }
        }
    }
    if (collision_flag == 1 && headUnitScreen_nu_delay == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE))
        testEvaluationPort.allowedManeuveringSpaceExceed_bool = 1;
    else
        testEvaluationPort.allowedManeuveringSpaceExceed_bool = 0;

    //maximum error from starting after standstill
    float xPosition_m_diff = 0.0f;
    float yPosition_m_diff = 0.0f;
    float odoCmRefxPosition_m_diff = 0.0f;
    float odoCmRefyPosition_m_diff = 0.0f;
    float egoRACur_m_calc = 0.0f;

    //detect standstill and calculate the error.
    if ((fabs(odoEstimationPortCM.longiVelocity_mps) > 0.001f) && fabs(odoCmRefEgoRaCur_m_calc) < 0.2f) {
        xPosition_m_diff = odoEstimationPort.xPosition_m - xPosition_m_delay;
        yPosition_m_diff = odoEstimationPort.yPosition_m - yPosition_m_delay;

        odoCmRefxPosition_m_diff = odoEstimationPortCM.xPosition_m - odoCmRefxPosition_m_delay;
        odoCmRefyPosition_m_diff = odoEstimationPortCM.yPosition_m - odoCmRefyPosition_m_delay;

        egoRACur_m_calc = sqrt(xPosition_m_diff * xPosition_m_diff + yPosition_m_diff * yPosition_m_diff);
        odoCmRefEgoRaCur_m_calc = sqrt(odoCmRefxPosition_m_diff * odoCmRefxPosition_m_diff + odoCmRefyPosition_m_diff * odoCmRefyPosition_m_diff);

        testEvaluationPort.odoEstimationCMDiference = fabs(egoRACur_m_calc - odoCmRefEgoRaCur_m_calc);

    }
    else if (fabs(odoEstimationPortCM.longiVelocity_mps) < 0.001f) {
        odoCmRefEgoRaCur_m_calc = 0;

        xPosition_m_delay = odoEstimationPort.xPosition_m;
        yPosition_m_delay = odoEstimationPort.yPosition_m;
        yawAngle_rad_delay = odoEstimationPort.yawAngle_rad;

        odoCmRefxPosition_m_delay = odoEstimationPortCM.xPosition_m;
        odoCmRefyPosition_m_delay = odoEstimationPortCM.yPosition_m;
        odoCmRefyawAngEgoRaCur_rad_delay = odoEstimationPortCM.yawAngle_rad;
    }
    else {
        testEvaluationPort.odoEstimationCMDiference = 0.0F;
    }

    vehVelocityCur_mps_delay = odoEstimationPortCM.longiVelocity_mps;

    // tyre circumference estimation debugging
    for (int i = 0; i < 4; i++)
    {
        const float filtConst = 0.99f;

        if (false == isFirstCycle)
        {
            cmDistFiltered = (float)((filtConst * cmLastDist_m) + ((1 - filtConst) * (-Car.Fr1.t_0[0])));

            cmCirc_m[i] = (float)(cmDistFiltered / Car.Tire[i].rot * 2 * M_PI);
            //   cmCirc_m[i] = (Car.Tire[i].rot - cmLastRot_rad[i]) / 0.001f;  // calculate rotational speed:

            cmVel_mps = (cmDistFiltered - cmLastDist_m) / 0.001f;  // Car.Distance signal not plausible resp. not linear at constant speed
        }
        else
        {
            cmDistFiltered = (float)(-Car.Fr1.t_0[0]);
        }

        // used to calculate rotational speed:
        cmLastRot_rad[i] = (float)(Car.Tire[i].rot);
        cmLastDist_m = cmDistFiltered;

    }

    //Set vehicle contour
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> vehicleContourPoints{};
    for (uint8_t iPt = 0;  iPt < carMakerInterface.vehicle_Params.AP_V_NUM_STANDARD_SHAPE_PTS; ++iPt) {
        vehicleContourPoints.append(cml::Vec2Df(carMakerInterface.vehicle_Params.AP_V_STANDARD_SHAPE_X_M[iPt], carMakerInterface.vehicle_Params.AP_V_STANDARD_SHAPE_Y_M[iPt]));
    }

    //calculate shortest distance of traffic objects to ego vehicle
    evaluateShortestDistanceToTrafficObjects(odoEstimationPort, odoEstimationPortCM, vehicleContourPoints, trafficContour2D, testEvaluationPort.shortestDistanceOdo, testEvaluationPort.shortestDistanceCM);

    if (plannedTrajPort.trajValid_nu) {
        testEvaluationPort.shortestDistanceToHighObject_m = std::numeric_limits<float32_t>::max();
        testEvaluationPort.shortestDistanceToWheelTravObject_m = std::numeric_limits<float32_t>::max();
        testEvaluationPort.shortestDistanceToBodyTravObject_m = std::numeric_limits<float32_t>::max();
        PositionOnTrajectory positionOnTraj;
        static ap_tp::PoseType parkingType;
        testEvaluationPort.plannedEgoVehOutsideBoundaries_bool = false;
        LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> egoVehicleShape{};

        for (unsigned int iTP = 0U; iTP < targetPosesPortCMOrigin.numValidPoses; iTP++) {
            if (targetPosesPortCMOrigin.targetPoses[iTP].pose_ID == slotCtrlPort.planningCtrlCommands.apChosenTargetPoseId_nu) {
                parkingType = targetPosesPortCMOrigin.targetPoses[iTP].type;
                const auto relatedParkingBoxID = targetPosesPortCMOrigin.targetPoses[iTP].relatedParkingBoxID;
                for (uint8_t iPB = 0U; iPB < parkingBoxPortCMOrigin.numValidParkingBoxes_nu; ++iPB) {
                    if (parkingBoxPortCMOrigin.parkingBoxes[iPB].parkingBoxID_nu == relatedParkingBoxID) {
                        selectedParkingBoxIdx = iPB;
                        break;
                    }
                }

                // build the maneuvering area if needed
                if (static_cast<cml::Vec2Df>(lastTargetPoseUsed.Pos() - LSM_GEOML::Pose(targetPosesPortCMOrigin.targetPoses[iTP].pose).Pos()).norm() > std::numeric_limits<float32_t>::epsilon()) {
                    defineBoundingBox(targetPosesPortCMOrigin.targetPoses[iTP].pose,parkingOnLeftSide_nu[selectedParkingBoxIdx], carMakerInterface.vehicle_Params);
                    defineManeuveringArea(targetPosesPortCMOrigin.targetPoses[iTP].type, parkingOnLeftSide_nu[selectedParkingBoxIdx]);
                    lastTargetPoseUsed = targetPosesPortCMOrigin.targetPoses[iTP].pose;
                }
                break;
            }
        }

        // build the driven area only once
        if (!egoVehScanningPathDefined_bool) {
            updateEgoVehScanningPath(drivenPath, envModelPortCMOrigin.resetOriginResult.originTransformation, parkingOnLeftSide_nu[selectedParkingBoxIdx]);

            defineScanningPathPolygon(carMakerInterface.vehicle_Params);
        }
        const LSM_GEOML::Pose egoVehiclePoseCMOrigin = getPose(carMakerInterface.vedodoActive_nu ? odoEstimationPort : odoEstimationPortCM);

        egoVehicleShape = returnBoundingBox(vehicleContourPoints, egoVehiclePoseCMOrigin);

        const LSM_GEOML::PolygonContainmentType toShrink = testEvaluationPort.maneuveringAreaPolygon.contains(egoVehicleShape);

        // if ego vehicle is completely in the maneuvering area, shrink it if necessary
        if (toShrink == LSM_GEOML::PolygonContainmentType::THIS_CONTAINS_OTHER) {
            if (parkingType == ap_tp::PoseType::T_PARALLEL_PARKING) {
                shrinkManeuveringArea(egoVehicleShape,
                parkingOnLeftSide_nu[selectedParkingBoxIdx]);
            }

            if (!testEvaluationPort.egoVehScanningPathPolygonRemoved_bool) {
                testEvaluationPort.egoVehScanningPathPolygonRemoved_bool = true;
                std::fill(testEvaluationPort.egoVehScanningPathPolygon.begin(), testEvaluationPort.egoVehScanningPathPolygon.end(), cml::Vec2Df{ 0.0f,0.0f });
                testEvaluationPort.egoVehScanningPathPolygon.clear();
            }
        }

        // iterate through all valid points of the calculated trajectory
        for (unsigned vertex = 0; vertex < plannedTrajPort.numValidCtrlPoints_nu; vertex++) {
            LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> plannedEgoVehicleShape;
            LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_WHEEL_SHAPE_MAX_SIZE_NU> wheelsShapes[ap_common::AP_COMMON_TYPES_Consts::AP_V_NUM_WHEELS_NU];
            // build the Polygon2D for the ego vehicle shape
            LSM_GEOML::Pose plannedTrajPose(plannedTrajPort.plannedTraj[vertex].xTrajRAReq_m, plannedTrajPort.plannedTraj[vertex].yTrajRAReq_m, plannedTrajPort.plannedTraj[vertex].yawReq_rad);
            plannedEgoVehicleShape = returnBoundingBox(vehicleContourPoints, plannedTrajPose);

            // check if the planned ego vehicle exceeds at least one time the allowed areas
            if (!testEvaluationPort.plannedEgoVehOutsideBoundaries_bool) {
                for (int iPlanned = plannedEgoVehicleShape.getSize() - 1; iPlanned >= 0; --iPlanned) {
                    // check if at least one point of the planned ego vehicle is out of both maneuvering and driven area
                    // vehicleArea1OverlapsPb is set to true only if is perpendicular parking and vehicleArea1 overlaps the PB
                    if (!testEvaluationPort.vehicleArea1OverlapsPb && !vertexInPolygon(plannedEgoVehicleShape[iPlanned], testEvaluationPort.maneuveringAreaPolygon) &&
                        ((!testEvaluationPort.egoVehScanningPathPolygonRemoved_bool && !vertexInPolygon(plannedEgoVehicleShape[iPlanned], testEvaluationPort.egoVehScanningPathPolygon)) || testEvaluationPort.egoVehScanningPathPolygonRemoved_bool)) {
                        testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon = plannedEgoVehicleShape;
                        testEvaluationPort.plannedEgoVehOutsideBoundaries_bool = true;
                        break;
                    }
                }
                if (!testEvaluationPort.plannedEgoVehOutsideBoundaries_bool && (parkingType == ap_tp::PoseType::T_PERP_PARKING_BWD || parkingType == ap_tp::PoseType::T_PERP_PARKING_FWD)) {
                    checkVehOutsidePB(convert(parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].slotCoordinates_m),
                        parkingType, egoVehiclePoseCMOrigin, plannedTrajPort.plannedTraj[vertex], carMakerInterface.vehicle_Params);
                }
            }
            // build the polygons for the ego vehicle wheels shapes
            for (unsigned iPt = 0; iPt < carMakerInterface.vehicle_Params.AP_V_WHEEL_SHAPE_SIZE_NU; iPt++) {
                cml::Vec2Df translatedPoint;
                transformLocalToWorld(plannedTrajPort.plannedTraj[vertex].xTrajRAReq_m, plannedTrajPort.plannedTraj[vertex].yTrajRAReq_m,
                    plannedTrajPort.plannedTraj[vertex].yawReq_rad, carMakerInterface.vehicle_Params.AP_V_FL_WHEEL_SHAPE_X_M[iPt],
                    carMakerInterface.vehicle_Params.AP_V_FL_WHEEL_SHAPE_Y_M[iPt], translatedPoint);
                wheelsShapes[0].append(translatedPoint);

                transformLocalToWorld(plannedTrajPort.plannedTraj[vertex].xTrajRAReq_m, plannedTrajPort.plannedTraj[vertex].yTrajRAReq_m,
                    plannedTrajPort.plannedTraj[vertex].yawReq_rad, carMakerInterface.vehicle_Params.AP_V_RL_WHEEL_SHAPE_X_M[iPt],
                    carMakerInterface.vehicle_Params.AP_V_RL_WHEEL_SHAPE_Y_M[iPt], translatedPoint);
                wheelsShapes[1].append(translatedPoint);

                transformLocalToWorld(plannedTrajPort.plannedTraj[vertex].xTrajRAReq_m, plannedTrajPort.plannedTraj[vertex].yTrajRAReq_m,
                    plannedTrajPort.plannedTraj[vertex].yawReq_rad, carMakerInterface.vehicle_Params.AP_V_RR_WHEEL_SHAPE_X_M[iPt],
                    carMakerInterface.vehicle_Params.AP_V_RR_WHEEL_SHAPE_Y_M[iPt], translatedPoint);
                wheelsShapes[2].append(translatedPoint);

                transformLocalToWorld(plannedTrajPort.plannedTraj[vertex].xTrajRAReq_m, plannedTrajPort.plannedTraj[vertex].yTrajRAReq_m,
                    plannedTrajPort.plannedTraj[vertex].yawReq_rad, carMakerInterface.vehicle_Params.AP_V_FR_WHEEL_SHAPE_X_M[iPt],
                    carMakerInterface.vehicle_Params.AP_V_FR_WHEEL_SHAPE_Y_M[iPt], translatedPoint);
                wheelsShapes[3].append(translatedPoint);
            }

            // iterate through all valid static objects in order to calculate the distance between all the high objects and the ego vehicle shape
            for (unsigned iObj = 0U; iObj < envModelPortCMOrigin.numberOfStaticObjects_u8; ++iObj) {
                const auto &staticObject = envModelPortCMOrigin.staticObjects[iObj];
                if (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE) {
                    const float32_t objToEgoVehDistance = convert(staticObject.objShape_m).calcDistance(plannedEgoVehicleShape);
                    // if a smaller distance than the previous one is found, save the distance, object id and the index of the trajectory point
                    if (objToEgoVehDistance < testEvaluationPort.shortestDistanceToHighObject_m) {
                        testEvaluationPort.shortestDistanceToHighObject_m = objToEgoVehDistance;
                        testEvaluationPort.closestHighObjectId_nu = staticObject.refObjID_nu;
                        testEvaluationPort.trajVertexIndexShortestDistHighObj_nu = vertex;
                    }
                }
                else if (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE ||
                    staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_WHEEL_TRAVERSABLE) {
                    bool excludeStaticObject_bool = false;
                    for (unsigned iPb = 0U; iPb < gParkingBoxPort.numValidParkingBoxes_nu; ++iPb) {
                        bool staticObjectFound_bool = false;

                        for (unsigned iDel = 0U; iDel < gParkingBoxPort.parkingBoxes[iPb].numValidDelimiters_nu; ++iDel) {
                            const auto &delimiter = gParkingBoxPort.parkingBoxes[iPb].delimiters[iDel];

                            if (
#ifndef VARIANT_CUS_ONLY // si::DelimiterTypes does not exist for CUS-only
                                delimiter.delimiterType_nu == si::DelimiterTypes::STATIC_STRUCTURE &&
#endif
                                delimiter.indexInList_nu == iObj) {
                                staticObjectFound_bool = true;

                                if (delimiter.delimitingSide_nu == si::RelativeLocationToParkingBox::ROAD_SIDE_EDGE) {
                                    excludeStaticObject_bool = true;
                                    break;
                                }
                                else if (delimiter.delimitingSide_nu == si::RelativeLocationToParkingBox::INSIDE_CURB_SIDE_EDGE ||
                                    delimiter.delimitingSide_nu == si::RelativeLocationToParkingBox::INSIDE_ROAD_SIDE_EDGE) {
                                    const auto parkingBoxRoadSideEdge = convert(gParkingBoxPort.parkingBoxes[iPb].slotCoordinates_m).getEdge(0);
                                    cml::Vec2Df longestObjectSegment{ 0.0f, 0.0f };

                                    // get the longest segment of the static object shape polygon
                                    const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> staticObjectPoly{ convert(staticObject.objShape_m) };
                                    for (unsigned iPt = 0U; iPt < staticObjectPoly.getSize(); iPt++) {
                                        if (staticObjectPoly.getEdge(iPt).norm() > longestObjectSegment.norm()) {
                                            longestObjectSegment = staticObjectPoly.getEdge(iPt);
                                        }
                                    }

                                    // calculate the cos angle between the road side edge of the parking box and the longest segment of the static object
                                    const float32_t cosAngle = abs(cos(LSM_GEOML::calcAngleBetweenVectors(parkingBoxRoadSideEdge, longestObjectSegment)));

                                    // if the cosine between the segments is 1 => the segments are perfect parallel
                                    // the cosAngle value will decrease by the cosine angle between the segments
                                    if (cosAngle > cos(SEGMENTS_PARALLEL_MAX_ANGLE_THRESHOLD_RAD)) {
                                        excludeStaticObject_bool = true;
                                    }
                                }
                                break;
                            }
                        }
                        if (staticObjectFound_bool) break;
                    }
                    if (!excludeStaticObject_bool) {
                        // calculate the distance between body traversable objects and the ego vehicle wheel shape
                        if (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE) {
                            for (unsigned iWheels = 0; iWheels < ap_common::AP_COMMON_TYPES_Consts::AP_V_NUM_WHEELS_NU; iWheels++) {
                                const auto objToEgoVehDistance = convert(staticObject.objShape_m).calcDistance(wheelsShapes[iWheels]);
                                if (objToEgoVehDistance < testEvaluationPort.shortestDistanceToBodyTravObject_m) {
                                    testEvaluationPort.shortestDistanceToBodyTravObject_m = objToEgoVehDistance;
                                    testEvaluationPort.closestBodyTravObjectId_nu = staticObject.refObjID_nu;
                                    testEvaluationPort.trajVertexIndexShortestDistBodyTravObj_nu = vertex;
                                }
                            }
                        }
                        // calculate the distance between wheel traversable objects and the ego vehicle wheel shape
                        else if (staticObject.objHeightClass_nu == si::StaticObjHeigthType::SO_HI_WHEEL_TRAVERSABLE) {
                            for (unsigned iWheels = 0; iWheels < ap_common::AP_COMMON_TYPES_Consts::AP_V_NUM_WHEELS_NU; iWheels++) {
                                const auto objToEgoVehDistance = convert(staticObject.objShape_m).calcDistance(wheelsShapes[iWheels]);
                                if (objToEgoVehDistance < testEvaluationPort.shortestDistanceToWheelTravObject_m) {
                                    testEvaluationPort.shortestDistanceToWheelTravObject_m = objToEgoVehDistance;
                                    testEvaluationPort.closestWheelTravObjectId_nu = staticObject.refObjID_nu;
                                    testEvaluationPort.trajVertexIndexShortestDistWheelTravObj_nu = vertex;
                                }
                            }
                        }
                    }
                }
            }
        }

        if (!testEvaluationPort.plannedEgoVehOutsideBoundaries_bool) {
            // if ego vehicle is completely in the maneuvering area or driven path -> reset plannedEgoVehOutsideBoundariesPolygon
            if (testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.getSize() != 0U &&
                !(testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon[0] == cml::Vec2Df{ 0.0f,0.0f }) &&
                !(testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon[1] == cml::Vec2Df{ 0.0f,0.0f })) {
                std::fill(testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.begin(), testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.end(), cml::Vec2Df{ 0.0f,0.0f });
            }
        }
        else
        {
            if (testEvaluationPort.vehicleArea1Polygon.getSize() != 0U &&
                !(testEvaluationPort.vehicleArea1Polygon[0] == cml::Vec2Df{ 0.0f,0.0f }) &&
                !(testEvaluationPort.vehicleArea1Polygon[1] == cml::Vec2Df{ 0.0f,0.0f })) {
                std::fill(testEvaluationPort.vehicleArea1Polygon.begin(), testEvaluationPort.vehicleArea1Polygon.end(), cml::Vec2Df{ 0.0f,0.0f });
            }
        }
        testEvaluationPort.shortestDistanceToHighObject_m = (testEvaluationPort.shortestDistanceToHighObject_m == std::numeric_limits<float32_t>::max()) ?
            -1.0f : testEvaluationPort.shortestDistanceToHighObject_m;
        testEvaluationPort.shortestDistanceToWheelTravObject_m = (testEvaluationPort.shortestDistanceToWheelTravObject_m == std::numeric_limits<float32_t>::max()) ?
            -1.0f : testEvaluationPort.shortestDistanceToWheelTravObject_m;
        testEvaluationPort.shortestDistanceToBodyTravObject_m = (testEvaluationPort.shortestDistanceToBodyTravObject_m == std::numeric_limits<float32_t>::max()) ?
            -1.0f : testEvaluationPort.shortestDistanceToBodyTravObject_m;

        definePositionOnTrajectory(plannedTrajPort, egoVehiclePoseCMOrigin, positionOnTraj);
        if (!positionOnTraj.outsideTrajectoryStart_nu) {
            testEvaluationPort.plannedTrajDrivenPoint = positionOnTraj.outsideTrajectoryEnd_nu ?
                plannedTrajPort.plannedTraj[positionOnTraj.currentTrajectoryIndex + 1] :
                plannedTrajPort.plannedTraj[positionOnTraj.currentTrajectoryIndex];
        }
    }
    //calculate the distance range in which a valid trajectory to a parking slot should be offered by AUP core
    LSM_GEOML::LineSegment2D sideEdgeOfPB = { cml::Vec2Df{ 0.0f, 0.0f },cml::Vec2Df{ 0.0f,0.0f } };
    if (gParkingBoxPort.numValidParkingBoxes_nu > 0)
    {
        for (unsigned int i = 0; i < gParkingBoxPort.numValidParkingBoxes_nu; i++) {
            if (parkingOnLeftSide_nu[i]){
                sideEdgeOfPB = { gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[1],gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[2] };
            }
            else {
                sideEdgeOfPB = { gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[0], gParkingBoxPort.parkingBoxes[i].slotCoordinates_m.array[3] };
            }
            cml::Vec2Df backOfTheCarCoord;
            transformLocalToWorld(envModelPort.egoVehiclePoseForAP.x_dir, envModelPort.egoVehiclePoseForAP.y_dir, envModelPort.egoVehiclePoseForAP.yaw_rad,
                -carMakerInterface.vehicle_Params.AP_V_OVERHANG_REAR_M, 0.0f, backOfTheCarCoord);
            testEvaluationPort.d5Distance_m[i] = (-1)*sideEdgeOfPB.pointToExtLineDistance(backOfTheCarCoord);
            // if the the car passed the right side edge of PB(when parking on left side)  or the left side edge of PB(when parking on right side)
            if (backOfTheCarCoord.x() > sideEdgeOfPB.p1().x()) {
                testEvaluationPort.d5Distance_m[i] = (-1)*testEvaluationPort.d5Distance_m[i];
            }
        }
    }

    //set the maximum deviation values between ego vehicle pose and target pose for evaluation
    if (parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].parkingScenario_nu == si::ParkingScenarioTypes::PARALLEL_PARKING) {
        testEvaluationPort.latMaxDeviation_m = carMakerInterface.trajplaSysFuncParams.AP_G_PAR_MAX_DEVIATION_LAT_M;
        testEvaluationPort.longMaxDeviation_m = carMakerInterface.trajplaSysFuncParams.AP_G_PAR_MAX_DEVIATION_LONG_M;
        testEvaluationPort.yawMaxDeviation_rad = carMakerInterface.trajplaSysFuncParams.AP_G_PAR_MAX_DEVIATION_ANGL_RAD;
    }
    else if (parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].parkingScenario_nu == si::ParkingScenarioTypes::PERPENDICULAR_PARKING) {
        testEvaluationPort.latMaxDeviation_m = carMakerInterface.trajplaSysFuncParams.AP_G_PER_MAX_DEVIATION_LAT_M;
        testEvaluationPort.longMaxDeviation_m = carMakerInterface.trajplaSysFuncParams.AP_G_PER_MAX_DEVIATION_LONG_M;
        testEvaluationPort.yawMaxDeviation_rad = carMakerInterface.trajplaSysFuncParams.AP_G_PER_MAX_DEVIATION_ANGL_RAD;
    }
    else if (parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].parkingScenario_nu == si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_BACK ||
        parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].parkingScenario_nu == si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_FRONT) {
        testEvaluationPort.latMaxDeviation_m = carMakerInterface.trajplaSysFuncParams.AP_G_PER_MAX_DEVIATION_LAT_M;
        testEvaluationPort.longMaxDeviation_m = carMakerInterface.trajplaSysFuncParams.AP_G_PER_MAX_DEVIATION_LONG_M;
        testEvaluationPort.yawMaxDeviation_rad = carMakerInterface.trajplaSysFuncParams.AP_G_PER_MAX_DEVIATION_ANGL_RAD;
    }
    else {
        //do nothing
    }
    //in case of a wheelstopper information during last stroke set the allowed ego pose to target pose deviation to larger value because here an overshoot is intended and target pose in axis mode
    if (isWheelstopperDuringLastStrokeSituation(plannedTrajPort, trajCtrlDebugPort)) {
        testEvaluationPort.longMaxDeviation_m = 9.99f;
    }
    //in case of testrun defines latMaxDeviation_m overwrite the default threshold
    if (evaluationPort.latMaxDeviation_m != -1.f) {
        testEvaluationPort.latMaxDeviation_m = evaluationPort.latMaxDeviation_m;
    }

    //Detect when reachedStatus set first time
    if ((targetPosesPortCMOrigin.selectedPoseData.reachedStatus != ap_tp::PoseReachedStatus::NO_TP_REACHED_STATUS) && (!targetPoseReachedStatusSet)) {
        //Store target pose
        testEvaluationPort.finalTargetPose = targetPosesPortCMOrigin.targetPoses[0].pose;
        targetPoseReachedStatusSet = true;
    }

    //Determine taposd debug deviation based on stored target pose data and current ego pose
    if (targetPoseReachedStatusSet) {
        const LSM_GEOML::CoordinateTransformer2D transformToTPOrigin(testEvaluationPort.finalTargetPose);
        const lsm_vedodo::OdoEstimation odoEstimationPortToUse = carMakerInterface.vedodoActive_nu ? odoEstimationPort : odoEstimationPortCM;
        const LSM_GEOML::Pose egoPoseTPOrigin = transformToTPOrigin.inverseTransform(LSM_GEOML::Pose(odoEstimationPortToUse.xPosition_m, odoEstimationPortToUse.yPosition_m, odoEstimationPortToUse.yawAngle_rad));
        //Store to testEvaluationPort deviation of ego Pose for KPI
        testEvaluationPort.egoPoseTargetPoseDeviation.longDistToTarget_m = egoPoseTPOrigin.Pos().x();
        testEvaluationPort.egoPoseTargetPoseDeviation.latDistToTarget_m = egoPoseTPOrigin.Pos().y();
        testEvaluationPort.egoPoseTargetPoseDeviation.yawDiffToTarget_rad = egoPoseTPOrigin.Yaw_rad();
    }


    //implement car_outside_PB
    //check if at the end of parking maneuver the ego vehicle shape (construct using the TP) is inside of Parking Box
    const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> egoVehicleShape_TP = returnBoundingBox(vehicleContourPoints, (targetPoseReachedStatusSet)? testEvaluationPort.finalTargetPose: targetPosesPortCMOrigin.targetPoses[0].pose);

    // add parking box tolerance
    auto expandedParkingBox = convert(parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].slotCoordinates_m);
    if (parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].slotCoordinates_m.actualSize == 4U) {
        expandedParkingBox = TestEvaluation::expandParkingBox(convert(parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].slotCoordinates_m));
    }

    // check if vehicle is inside of PB (0 - Vehicle Inside PB ; 1 - Vehicle Outside PB)
    testEvaluationPort.car_outside_PB = isTargetPoseOutsidePB(expandedParkingBox, egoVehicleShape_TP, targetPosesPortCMOrigin.targetPoses[0].type);

    //check if the final ego vehicle shape with TP as origin colides the static structure
    for (unsigned int idx_static_struc = 0U; idx_static_struc < NO_STATIC_STRUCTURES; ++idx_static_struc) {
        //check if the final ego vehicle shape colides the static structure (0 - NO COLLISION; 1 - COLLISION)
        testEvaluationPort.staticStructColidesTarget_Pose[idx_static_struc] = isTargetPoseInCollision(envModelPortCMOrigin.staticObjects[idx_static_struc], egoVehicleShape_TP);
     }

    //determine the end of parking maneuver
    if (carMakerInterface.headUnitScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_FINISHED)
        && headUnitScreen_nu_delay == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_ACTIVE)) {
        // save the possition of ego Vehicle at the end of parking maneuver
        const lsm_vedodo::OdoEstimation odoEstimationPortToUse = carMakerInterface.vedodoActive_nu ? odoEstimationPort : odoEstimationPortCM;
        const LSM_GEOML::Pose egoPossitionAtEndOfParkingManeuver = LSM_GEOML::Pose(odoEstimationPortToUse.xPosition_m, odoEstimationPortToUse.yPosition_m, odoEstimationPortToUse.yawAngle_rad);
        //OptimalTargetPose evaluation
        optimalTargetPoseEvaluation(parkingBoxPortCMOrigin.parkingBoxes[selectedParkingBoxIdx].parkingScenario_nu,
            egoPossitionAtEndOfParkingManeuver);
    }
    
    computeTRJPLAnumberOfCrvSteps(plannedTrajPort, carMakerInterface.vehicle_Params, testEvaluationPort.numberOfCrvSteps);

    if (carMakerInterface.headUnitScreen_nu == static_cast<uint8_t>(mf_hmih::ScreenHeadUnit::MANEUVER_FINISHED)) {
        ComparenumberOfStrokes(testEvaluationPort.numberOfStrokes, evaluationPort.n_strokes_max_nu, testEvaluationPort.tooManyStrokes);
    }

    //determine the distance to stop extrapolated based on trajectory data and mf_control debug data
    //by default use trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m
    testEvaluationPort.distanceToStopReqInterExtrapolTraj_m = trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m;
    //in case of a wheelstopper information during last stroke deactivate KPI by lower limit 0m of testEvaluationPort.distanceToStopReqInterExtrapolTraj_m because here an overshoot is intended
    if (isWheelstopperDuringLastStrokeSituation(plannedTrajPort,trajCtrlDebugPort) &&
        (trajCtrlDebugPort.distanceToStopReqInterExtrapolTraj_m < 0.0f))
    {
        testEvaluationPort.distanceToStopReqInterExtrapolTraj_m = 0.0f;
    }

    //set  headUnitScreen_nu_delay
    headUnitScreen_nu_delay = carMakerInterface.headUnitScreen_nu;
}

bool TestEvaluation::isWheelstopperDuringLastStrokeSituation(
    const ap_tp::PlannedTrajPort &plannedTrajPort,
    const ap_trjctl::TrajCtrlDebugPort &trajCtrlDebugPort
) {
    return (plannedTrajPort.isLastSegment_nu &&
        ((ap_tp::DrivingResistanceType::WHEEL_STOPPER == plannedTrajPort.drivingResistance[0].type_nu) ||
        (ap_tp::DrivingResistanceType::WHEEL_STOPPER == plannedTrajPort.drivingResistance[1].type_nu) ||
            (ap_tp::DrivingResistanceType::WHEEL_STOPPER == plannedTrajPort.drivingResistance[2].type_nu) ||
            (ap_tp::DrivingResistanceType::WHEEL_STOPPER == plannedTrajPort.drivingResistance[3].type_nu)));
}


void TestEvaluation::registerCarMakerDVAs()
{
    DDefUChar(NULL, "AP.testEvaluationActive", "", (uint8_t*)&testEvaluationActive, DVA_None);

    tDDefault *plannedEgoVehOutsideBoundaries = DDefaultCreate("AP.plannedEgoVehOutsideBoundariesPolygon");
    testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.setSize(testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.getMaxSize());
    for (unsigned int i = 0; i < testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.getSize(); i++) {
        DDefPrefix(plannedEgoVehOutsideBoundaries, "AP.plannedEgoVehOutsideBoundariesPolygon.polygonPoint_%d.x", i);
        DDefFloat(plannedEgoVehOutsideBoundaries, "", "m", &testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon[i].x(), DVA_None);
        DDefPrefix(plannedEgoVehOutsideBoundaries, "AP.plannedEgoVehOutsideBoundariesPolygon.polygonPoint_%d.y", i);
        DDefFloat(plannedEgoVehOutsideBoundaries, "", "m", &testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon[i].y(), DVA_None);
    }
    testEvaluationPort.plannedEgoVehOutsideBoundariesPolygon.setSize(0U);

    tDDefault *vehicleArea1Boundaries = DDefaultCreate("AP.vehicleArea1Polygon");
    testEvaluationPort.vehicleArea1Polygon.setSize(testEvaluationPort.vehicleArea1Polygon.getMaxSize());
    for (unsigned int i = 0; i < testEvaluationPort.vehicleArea1Polygon.getSize(); i++) {
        DDefPrefix(vehicleArea1Boundaries, "AP.vehicleArea1Polygon.polygonPoint_%d.x", i);
        DDefFloat(vehicleArea1Boundaries, "", "m", &testEvaluationPort.vehicleArea1Polygon[i].x(), DVA_None);
        DDefPrefix(vehicleArea1Boundaries, "AP.vehicleArea1Polygon.polygonPoint_%d.y", i);
        DDefFloat(vehicleArea1Boundaries, "", "m", &testEvaluationPort.vehicleArea1Polygon[i].y(), DVA_None);
    }
    DDefUChar(NULL, "AP.vehicleArea1Polygon.size", "", &testEvaluationPort.vehicleArea1Polygon_size, DVA_None);
    testEvaluationPort.vehicleArea1Polygon.setSize(0U);

    tDDefault *vehicleArea2Boundaries = DDefaultCreate("AP.vehicleArea2Polygon");
    testEvaluationPort.vehicleArea2Polygon.setSize(testEvaluationPort.vehicleArea2Polygon.getMaxSize());
    for (unsigned int i = 0; i < testEvaluationPort.vehicleArea2Polygon.getSize(); i++) {
        DDefPrefix(vehicleArea2Boundaries, "AP.vehicleArea2Polygon.polygonPoint_%d.x", i);
        DDefFloat(vehicleArea2Boundaries, "", "m", &testEvaluationPort.vehicleArea2Polygon[i].x(), DVA_None);
        DDefPrefix(vehicleArea2Boundaries, "AP.vehicleArea2Polygon.polygonPoint_%d.y", i);
        DDefFloat(vehicleArea2Boundaries, "", "m", &testEvaluationPort.vehicleArea2Polygon[i].y(), DVA_None);
    }
    DDefUChar(NULL, "AP.vehicleArea2Polygon.size", "", &testEvaluationPort.vehicleArea2Polygon_size, DVA_None);
    testEvaluationPort.vehicleArea2Polygon.setSize(0U);


    tDDefault *drivenArea = DDefaultCreate("AP.drivenArea");
    testEvaluationPort.egoVehScanningPathPolygon.setSize(testEvaluationPort.scanningPathMaxSize_nu);
    for (unsigned int i = 0; i < testEvaluationPort.scanningPathMaxSize_nu; i++) {
        DDefPrefix(drivenArea, "AP.drivenArea.polygonPoint_%d.x", i);
        DDefFloat(drivenArea, "", "m", &testEvaluationPort.egoVehScanningPathPolygon[i].x(), DVA_None);
        DDefPrefix(drivenArea, "AP.drivenArea.polygonPoint_%d.y", i);
        DDefFloat(drivenArea, "", "m", &testEvaluationPort.egoVehScanningPathPolygon[i].y(), DVA_None);
    }
    testEvaluationPort.egoVehScanningPathPolygon.setSize(0U);

    tDDefault *maneuverArea = DDefaultCreate("AP.maneuverArea");
    testEvaluationPort.maneuveringAreaPolygon.setSize(testEvaluationPort.maneuveringAreaPolygon.getMaxSize());
    for (unsigned int i = 0; i < testEvaluationPort.maneuveringAreaPolygon.getSize(); i++) {
        DDefPrefix(maneuverArea, "AP.maneuverArea.polygonPoint_%d.x", i);
        DDefFloat(maneuverArea, "", "m", &testEvaluationPort.maneuveringAreaPolygon[i].x(), DVA_None);
        DDefPrefix(maneuverArea, "AP.maneuverArea.polygonPoint_%d.y", i);
        DDefFloat(maneuverArea, "", "m", &testEvaluationPort.maneuveringAreaPolygon[i].y(), DVA_None);
    }
    testEvaluationPort.maneuveringAreaPolygon.setSize(0U);

    tDDefault *boundingBox = DDefaultCreate("AP.boundingBox");
    testEvaluationPort.boundingBoxPolygon.setSize(testEvaluationPort.boundingBoxPolygon.getMaxSize());
    for (unsigned int i = 0; i < testEvaluationPort.boundingBoxPolygon.getSize(); i++) {
        DDefPrefix(boundingBox, "AP.boundingBox.polygonPoint_%d.x", i);
        DDefFloat(boundingBox, "", "m", &testEvaluationPort.boundingBoxPolygon[i].x(), DVA_None);
        DDefPrefix(boundingBox, "AP.boundingBox.polygonPoint_%d.y", i);
        DDefFloat(boundingBox, "", "m", &testEvaluationPort.boundingBoxPolygon[i].y(), DVA_None);
    }
    testEvaluationPort.boundingBoxPolygon.setSize(0U);

    tDDefault *d5 = DDefaultCreate("d5.");
    for (int idx = 0; idx < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; ++idx) {
        DDefPrefix(d5, "AP.d5Distance_m[%d]", idx);
        DDefFloat(d5, "", "", &testEvaluationPort.d5Distance_m[idx], DVA_None);
    }

    DDefUChar(NULL, "AP.egoVehScanningPathPolygonRemoved_bool", "", (uint8_t*)&testEvaluationPort.egoVehScanningPathPolygonRemoved_bool, DVA_None);
    DDefUChar(NULL, "AP.plannedEgoVehOutsideBoundaries_bool", "", (uint8_t*)&testEvaluationPort.plannedEgoVehOutsideBoundaries_bool, DVA_None);
    DDefUChar(NULL, "AP.vehicleArea1OverlapsPb", "", (uint8_t*)&testEvaluationPort.vehicleArea1OverlapsPb, DVA_None);
    DDefUInt(NULL, "AP.scanningPathMaxSize_nu", "", (unsigned int*)&testEvaluationPort.scanningPathMaxSize_nu, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajDrivenPoint.xTrajRAReq_m", "m", &testEvaluationPort.plannedTrajDrivenPoint.xTrajRAReq_m, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajDrivenPoint.yTrajRAReq_m", "m", &testEvaluationPort.plannedTrajDrivenPoint.yTrajRAReq_m, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajDrivenPoint.yawReq_rad", "rad", &testEvaluationPort.plannedTrajDrivenPoint.yawReq_rad, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajDrivenPoint.crvRAReq_1pm", "1/m", &testEvaluationPort.plannedTrajDrivenPoint.crvRAReq_1pm, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajDrivenPoint.distanceToStopReq_m", "m", &testEvaluationPort.plannedTrajDrivenPoint.distanceToStopReq_m, DVA_None);
    DDefFloat(NULL, "AP.plannedTrajDrivenPoint.velocityLimitReq_mps", "mps", &testEvaluationPort.plannedTrajDrivenPoint.velocityLimitReq_mps, DVA_None);
    DDefFloat(NULL, "AP.shortestDistanceToHighObject_m", "m", &testEvaluationPort.shortestDistanceToHighObject_m, DVA_None);
    DDefInt(NULL, "AP.closestHighObjectId_nu", "", &testEvaluationPort.closestHighObjectId_nu, DVA_None);
    for (uint8_t i = 0U; i < testEvaluationPort.shortestDistanceOdo.size(); i++) {
        DDefPrefix(d5, "AP.testEvaluation.shortestDistanceOdo[%d]", i);
        DDefFloat(d5, "", "m", &testEvaluationPort.shortestDistanceOdo[i], DVA_None);
        DDefPrefix(d5, "AP.testEvaluation.shortestDistanceCM[%d]", i);
        DDefFloat(d5, "", "m", &testEvaluationPort.shortestDistanceCM[i], DVA_None);
    }
    DDefaultDelete(d5);
    DDefInt(NULL, "AP.trajVertexIndexShortestDistHighObj_nu", "", &testEvaluationPort.trajVertexIndexShortestDistHighObj_nu, DVA_None);
    DDefFloat(NULL, "AP.shortestDistanceToWheelTravObject_m", "m", &testEvaluationPort.shortestDistanceToWheelTravObject_m, DVA_None);
    DDefInt(NULL, "AP.closestWheelTravObjectId_nu", "", &testEvaluationPort.closestWheelTravObjectId_nu, DVA_None);
    DDefInt(NULL, "AP.trajVertexIndexShortestDistWheelTravObj_nu", "", &testEvaluationPort.trajVertexIndexShortestDistWheelTravObj_nu, DVA_None);
    DDefFloat(NULL, "AP.shortestDistanceToBodyTravObject_m", "m", &testEvaluationPort.shortestDistanceToBodyTravObject_m, DVA_None);
    DDefInt(NULL, "AP.closestBodyTravObjectId_nu", "", &testEvaluationPort.closestBodyTravObjectId_nu, DVA_None);
    DDefInt(NULL, "AP.trajVertexIndexShortestDistBodyTravObj_nu", "", &testEvaluationPort.trajVertexIndexShortestDistBodyTravObj_nu, DVA_None);
    DDefUInt(NULL, "AP.numberOfStrokes", "", &testEvaluationPort.numberOfStrokes, DVA_None);
    DDefUInt(NULL, "AP.numberOfStrokes_gear", "", &testEvaluationPort.numberOfStrokes_gear, DVA_None);
    DDefUInt(NULL, "AP.numberOfStrokes_newSegmentStarted", "", &testEvaluationPort.numberOfStrokes_newSegmentStarted, DVA_None);
    DDefFloat(NULL, "AP.drivenDistanceOnStroke_m", "m", &testEvaluationPort.drivenDistanceOnStroke_m, DVA_None);
    DDefUChar(NULL, "AP.allowedManeuveringSpaceExceed_bool", "", (uint8_t*)&testEvaluationPort.allowedManeuveringSpaceExceed_bool, DVA_None);

    DDefFloat(NULL, "AP.odoEstimationCMDiference", "", &testEvaluationPort.odoEstimationCMDiference, DVA_None);

    DDefFloat(NULL, "AP.testEvaluationPort.TimeToPark_s", "s", &testEvaluationPort.timeToPark, DVA_None);
    DDefUInt(NULL, "AP.evaluationPort.n_strokes_max_nu", "", (unsigned int *)&evaluationPort.n_strokes_max_nu, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.t_sim_max_s", "s", &evaluationPort.t_sim_max_s, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.v_max_mps", "m/s", &evaluationPort.v_max_mps, DVA_None);
    DDefUChar(NULL, "AP.evaluationPort.scopeBase_nu", "", (uint8_t *)&evaluationPort.scopeBase_nu, DVA_None);
    DDefUChar(NULL, "AP.evaluationPort.useCase_nu", "", (uint8_t *)&evaluationPort.useCase_nu, DVA_None);
    DDefUChar(NULL, "AP.evaluationPort.parkingManeuver_nu", "", (uint8_t *)&evaluationPort.parkingManeuver_nu, DVA_None);
    DDefUChar(NULL, "AP.evaluationPort.OptimalTargetPose.isPresent_nu", "", (uint8_t *)&evaluationPort.optimalTargetPose.valid, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.OptimalTargetPose.x_m", "m", &evaluationPort.optimalTargetPose.pose.Pos().x(), DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.OptimalTargetPose.y_m", "m", &evaluationPort.optimalTargetPose.pose.Pos().y(), DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.OptimalTargetPose.yaw_rad", "rad", &evaluationPort.optimalTargetPose.pose.Yaw_rad(), DVA_None);

    DDefFloat(NULL, "AP.testEvaluation.latDiffOptimalTP_FinalEgoPose_m", "m", &testEvaluationPort.latDiffOptimalTP_FinalEgoPose_m, DVA_None);
    DDefFloat(NULL, "AP.testEvaluation.longDiffOptimalTP_FinalEgoPose_m", "m", &testEvaluationPort.longDiffOptimalTP_FinalEgoPose_m, DVA_None);
    DDefFloat(NULL, "AP.testEvaluation.yawDiffOptimalTP_FinalEgoPose_deg", "deg", &testEvaluationPort.yawDiffOptimalTP_FinalEgoPose_deg, DVA_None);

    DDefFloat(NULL, "AP.testEvaluation.latDiffOptimalTP_TargetPose_m", "m", &testEvaluationPort.latDiffOptimalTP_TargetPose_m, DVA_None);
    DDefFloat(NULL, "AP.testEvaluation.longDiffOptimalTP_TargetPose_m", "m", &testEvaluationPort.longDiffOptimalTP_TargetPose_m, DVA_None);
    DDefFloat(NULL, "AP.testEvaluation.yawDiffOptimalTP_TargetPose_deg", "deg", &testEvaluationPort.yawDiffOptimalTP_TargetPose_deg, DVA_None);

    DDefFloat(NULL, "AP.testEvaluation.latDiffErrorTreshold_m", "m", &testEvaluationPort.latDiffErrorTreshold_m, DVA_None);
    DDefFloat(NULL, "AP.testEvaluation.longDiffErrorTreshold_m", "m", &testEvaluationPort.longDiffErrorTreshold_m, DVA_None);
    DDefFloat(NULL, "AP.testEvaluation.yawDiffErrorTreshold_deg", "deg", &testEvaluationPort.yawDiffErrorTreshold_deg, DVA_None);
    DDefFloat(NULL, "AP.testEvaluation.vel_accel_delay_mps", "m/s", &vel_accel_delay, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.egoPoseTargetPoseDeviation.longDistToTarget_m", "m", &testEvaluationPort.egoPoseTargetPoseDeviation.longDistToTarget_m, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.egoPoseTargetPoseDeviation.latDistToTarget_m", "m", &testEvaluationPort.egoPoseTargetPoseDeviation.latDistToTarget_m, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.egoPoseTargetPoseDeviation.yawDiffToTarget_rad", "rad", &testEvaluationPort.egoPoseTargetPoseDeviation.yawDiffToTarget_rad, DVA_None);

    DDefFloat(NULL, "AP.evaluationPort.finalTargetPose.x_m", "m", &testEvaluationPort.finalTargetPose.Pos().x(), DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.finalTargetPose.y_m", "m", &testEvaluationPort.finalTargetPose.Pos().y(), DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.finalTargetPose.yaw_rad", "rad", &testEvaluationPort.finalTargetPose.Yaw_rad(), DVA_None);

    DDefFloat(NULL, "AP.evaluationPort.distanceToStopReqInterExtrapolTraj_m", "m", &testEvaluationPort.distanceToStopReqInterExtrapolTraj_m, DVA_None);

    DDefInt(NULL, "AP.evaluationPort.car_outside_PB", "", &testEvaluationPort.car_outside_PB, DVA_None);
    tDDefault *staticColl = DDefaultCreate("staticColl.");
    for (int idx = 0; idx < NO_STATIC_STRUCTURES; ++idx) {
        DDefPrefix(staticColl, "AP.evaluationPort.staticStructColidesTarget_Pose_%d", idx);
        DDefInt(staticColl, "", "", &testEvaluationPort.staticStructColidesTarget_Pose[idx], DVA_None);
    }
    DDefaultDelete(staticColl);

    DDefUInt(NULL, "AP.evaluationPort.TRJPLA_numberOfCrvSteps", "", &testEvaluationPort.numberOfCrvSteps, DVA_None);
    DDefUChar(NULL, "AP.currentNrOfStrokesGreaterThanMaxNrOfSrokes_bool", "", (uint8_t*)&testEvaluationPort.tooManyStrokes, DVA_None);
    
    DDefFloat(NULL, "AP.evaluationPort.longMaxDeviation_m", "m", &testEvaluationPort.longMaxDeviation_m, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.latMaxDeviation_m", "m", &testEvaluationPort.latMaxDeviation_m, DVA_None);
    DDefFloat(NULL, "AP.evaluationPort.yawMaxDeviation_rad", "rad", &testEvaluationPort.yawMaxDeviation_rad, DVA_None);
}
