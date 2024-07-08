#include "CemSurrogate.h"
#include "geoml/AxisAlignedRectangular2D.h"
#include <algorithm> // std::sort
#include <set>
#include <queue> // std::priority_queue
#include <fstream> // TODO delete this
// CarMaker headers
#include <SimCore.h>
#include <Car/Car.h>
#include <Vehicle/Sensor_FSpace.h>
#include <algorithm>
#include <cctype>
#include <string>
namespace VCEM {

#define PI 3.14159265359f;
    // Width of the convex polygon hulls in meters.
    static constexpr float32_t HULL_WIDTH_M = 0.05F;
    // Minimal distance between two polygon points in meters.
    static constexpr float32_t MIN_POINT_DISTANCE_M = 0.04F;
    // Maximum number of points for a convex hull that is stored internally in the CEM surrogate model
    static constexpr unsigned MAX_NUM_PTS_HULL_INTERNAL{ 30U };
    // MAX_NUM_PTS_HULL defined in CemSurrogate header!
    static constexpr float32_t CURB_OBJECT_HEIGHT_M = 0.25F;
    static constexpr float32_t POLE_LENGTH_THRESHOLD_M = 0.2F;
    static constexpr float32_t POLE_WIDTH_THRESHOLD_M = 0.2F;
    // distance threshold from the traffic object to ego vehicle for less precise point detection of the camera
    static constexpr float32_t CAMERA_UNCERTAINTY_DISTANCE_THRESHOLD_M = 2.5F;
    // minimum angle threshold where a detected point should have a camera uncertainty
    static constexpr double CAMERA_FOV_EDGE_MIN_ANGLE = 1.13446401;
    // maximum angle threshold where a detected point should have a camera uncertainty
    static constexpr double CAMERA_FOV_EDGE_MAX_ANGLE = 1.57079633;
    static constexpr float32_t AP_G_MAX_HEIGHT_BODY_TRAVER_M = 0.15F;
    static constexpr float32_t AP_G_MAX_HEIGHT_DOOR_OPENABLE_M = 0.2F;

    static constexpr unsigned int SEGMENT_STARTING_INDEX = 10000U;

    /** 2D Cross product of OA and OB vectors
    *
    * @note: It effectively calculates the z-component of their 3D cross product. Returns
    *        a positive value if OAB makes a counter-clockwise turn, negative for a clockwise
    *        turn and zero is the points are collinear.
    *
    * @return z-component of the 3D cross product.
    */
    static float32_t cross(const cml::Vec2Df& origin, const cml::Vec2Df& a, const cml::Vec2Df& b) {
        const cml::Vec2Df originToA = a - origin;
        const cml::Vec2Df originToB = b - origin;
        return originToA.crossProduct(originToB);
    }

    // Returns a list of points on the convex hull in counter-clockwise order.
    // Note: the last point in the returned list is the same as the first one.
    std::vector<DetectedPoint> CemSurrogate::convex_hull(std::vector<DetectedPoint> &points) {
        const size_t n = points.size();
        size_t k = 0;
        if (n <= 3) {
            return points;
        }
        std::vector<DetectedPoint> hull(2 * n);

        // Sort points lexicographically according to its position
        std::sort(points.begin(), points.end(), [](const DetectedPoint &a, const DetectedPoint &b) {
            return a.getPosition() < b.getPosition();
        });

        // Build lower hull
        for (size_t i = 0; i < n; ++i) {
            while (k >= 2 && cross(hull[k - 2].getPosition(), hull[k - 1].getPosition(), points[i].getPosition()) <= 0) {
                k--;
            }
            hull[k++] = points[i];
        }

        // Build upper hull
        for (size_t i = n - 1, t = k + 1; i > 0; --i) {
            while (k >= t && cross(hull[k - 2].getPosition(), hull[k - 1].getPosition(), points[i - 1].getPosition()) <= 0) {
                k--;
            }
            hull[k++] = points[i - 1];
        }

        hull.resize(k - 1);
        return hull;
    }

    static cml::Vec2Df calcNormalVector(const cml::Vec2Df &detectedPoint, const cml::Vec2Df &tangent, const cml::Vec2Df &sensorPos) {
        const cml::Vec2Df normalVec = cml::Vec2Df(tangent.y(), -tangent.x()).getNormalized();
        const float32_t dotProduct = normalVec.scalarProduct(detectedPoint - sensorPos);
        return -static_cast<float32_t>(LSM_GEOML::signum(dotProduct)) * normalVec;
    }

    // calculate two additional points with a fixed distance to a detected point in the normal direction.
    void CemSurrogate::addOppositePointsTo(std::vector<DetectedPoint> &pointsPerObj) {
        if (pointsPerObj.size() > 0) {
            const auto &firstPoint = pointsPerObj.front();
            const auto &lastPoint = pointsPerObj.back();
            auto firstOppositePoint = firstPoint;
            firstOppositePoint.positionGroundTruth_m -= HULL_WIDTH_M * firstPoint.normalVector_m;
            firstOppositePoint.normalVector_m = -firstPoint.normalVector_m;
            if (pointsPerObj.size() > 1) {
                auto lastOppositePoint = lastPoint;
                lastOppositePoint.positionGroundTruth_m -= HULL_WIDTH_M * lastPoint.normalVector_m;
                lastOppositePoint.normalVector_m = -lastPoint.normalVector_m;
                pointsPerObj.push_back(lastOppositePoint);
            }
            pointsPerObj.push_back(firstOppositePoint);
        }
    }

    float32_t CemSurrogate::calcTriangleArea(const std::vector<DetectedPoint>& convexHull, const size_t index, const size_t prevInd, const size_t nextInd) {
        return 0.5F * std::abs(cross(convexHull[index].getPosition(), convexHull[prevInd].getPosition(), convexHull[nextInd].getPosition()));
    }

    // Reduces the number of points in the convex hull polygons by applying the Visvalingam�Whyatt algorithm.
    // The implementation is optimized by using a min-heap to sort the points according to their added area
    // in order to avoid a quadratic computational complexity in the number of polygon points.
    void CemSurrogate::reduceNumberOfPoints(std::vector<DetectedPoint> &convexHull, std::vector<DetectedPoint> &convexHullInternal) {
        struct HullVertex
        {
            float32_t triangleArea;
            bool areaValid{ true };
            size_t index, prevIndex, nextIndex;
            HullVertex(size_t index_, size_t prevIndex_, size_t nextIndex_, float32_t triangleArea_)
                : index(index_), prevIndex(prevIndex_), nextIndex(nextIndex_), triangleArea(triangleArea_) { }
        };
        auto cmpHullVertices = [](const HullVertex* left, const HullVertex* right) {
            return (left->triangleArea > right->triangleArea || (left->triangleArea == right->triangleArea && left->index < right->index));
        };

        assert(MAX_NUM_PTS_HULL_INTERNAL >= MAX_NUM_PTS_HULL);
        // at least three points are required to form a convex hull with a non-zero area
        if (convexHull.size() <= 3U) {
            return;
        }

        std::vector<HullVertex> hullVertices{};
        hullVertices.reserve(convexHull.size());
        // min-heap always having the HullVertex with the smallest triangle area at the top
        std::priority_queue<HullVertex*, std::vector<HullVertex*>, decltype(cmpHullVertices)> minAreaHeap(cmpHullVertices);
        for (size_t i = 0; i < convexHull.size(); ++i) {
            const size_t iCyclicPrev = (i == 0) ? convexHull.size() - 1 : i - 1;
            const size_t iCyclicNext = (i == convexHull.size() - 1) ? 0 : i + 1;
            const float32_t triangleArea = calcTriangleArea(convexHull, i, iCyclicPrev, iCyclicNext);
            hullVertices.push_back(HullVertex(i, iCyclicPrev, iCyclicNext, triangleArea));
            minAreaHeap.push(&hullVertices.back());
        }

        std::vector<size_t> idsVerticesToRemove{};
        std::vector<size_t> idsVerticesToRemoveInternal{};
        HullVertex* vertexToRemove = minAreaHeap.top();
        minAreaHeap.pop();
        // reduce number of convexHull points below MAX_NUM_PTS_HULL and remove even more points if their added area to the polygon is below 10 mm^2
        while (convexHull.size() - idsVerticesToRemove.size() > MAX_NUM_PTS_HULL || vertexToRemove->triangleArea < 0.00001F) {
            // update the neighboring vertices of the vertexToRemove
            HullVertex& prevVertex = hullVertices[vertexToRemove->prevIndex];
            prevVertex.nextIndex = vertexToRemove->nextIndex;
            prevVertex.areaValid = false;
            HullVertex& nextVertex = hullVertices[vertexToRemove->nextIndex];
            nextVertex.prevIndex = vertexToRemove->prevIndex;
            nextVertex.areaValid = false;

            // only remember vertices for removal, because directly removing them would change the indices in convexHull,
            // which would interfere with the triangle area re-calculation
            idsVerticesToRemove.push_back(vertexToRemove->index);
            if (convexHullInternal.size() - idsVerticesToRemove.size() == MAX_NUM_PTS_HULL_INTERNAL) {
                idsVerticesToRemoveInternal = idsVerticesToRemove;
            }

            // select next possible vertex candidate to remove
            if (minAreaHeap.empty()) {
                break;
            }
            vertexToRemove = minAreaHeap.top();
            minAreaHeap.pop();
            while (!vertexToRemove->areaValid) {
                // update vertex with invalid area and push it to the min-heap again
                vertexToRemove->triangleArea = calcTriangleArea(convexHull, vertexToRemove->index, vertexToRemove->prevIndex, vertexToRemove->nextIndex);
                vertexToRemove->areaValid = true;
                minAreaHeap.push(vertexToRemove);
                // select new vertex to remove
                vertexToRemove = minAreaHeap.top();
                minAreaHeap.pop();
            }
        }

        if (!idsVerticesToRemove.empty()) {
            std::sort(idsVerticesToRemove.begin(), idsVerticesToRemove.end(), std::greater<size_t>());
            for (size_t ind : idsVerticesToRemove) {
                convexHull.erase(convexHull.begin() + ind);
            }
        }
        if (!idsVerticesToRemoveInternal.empty()) {
            std::sort(idsVerticesToRemoveInternal.begin(), idsVerticesToRemoveInternal.end(), std::greater<size_t>());
            for (size_t ind : idsVerticesToRemoveInternal) {
                convexHullInternal.erase(convexHullInternal.begin() + ind);
            }
        }
    }

    bool CemSurrogate::hasDisappeared(CemObject& cemObject, const uint64_t cycleTime_ms)
    {
        // If dynamic object movement is enabled, dynamic objects should not disappear. They move around in the map instead.
        if (mConfig.enableDynamicObjectMovement_nu) {
            return false;
        }
        if (CemObjectType::STATIC_OBJECT == cemObject.objectType) {
            return false;
        }
        // If the object was detected in the current cycle.
        if (std::abs(cemObject.lastDetectionTime_s - SimCore.Time) < LSM_GEOML::MIN_FLT_DIVISOR) {
            cemObject.detectionMissesCount = 0U;
            return false;
        }
        // Only consider the ground-truth position of the points because if the CEM effects would be considered,
        // the object polygon might appear to overlap with a sensor FOV even though the object does not,
        // which would result in an undesired removal of that object.
        std::vector<cml::Vec2Df> objectPolygon{};
        std::transform(cemObject.detectedPoints.begin(), cemObject.detectedPoints.end(), std::back_inserter(objectPolygon),
            [](const DetectedPoint& point) {return point.positionGroundTruth_m; });

        for (const auto& sensorFov : mSensorName2Fov) {
            if (sensorFov.second.doOverlap(objectPolygon)) {
                ++(cemObject.detectionMissesCount);
                break;
            }
        }
        if (cemObject.detectionMissesCount * cycleTime_ms * 1E-3 > mConfig.disappearedObjectRemovalTime_s) {
            return true;
        }
        return false;
    }

    tTrafficObj* getTrafficObjByObjId(const unsigned int objId) {
        // for the fragmentation effect, we have objId = (trfId + 1) * SEGMENT_STARTING_INDEX + segmentNumber
        const unsigned trafficId = objId >= SEGMENT_STARTING_INDEX ? objId / SEGMENT_STARTING_INDEX - 1 : objId;
        return Traffic_GetByTrfId(trafficId);
    }

    static LSM_GEOML::Pose getObjectPose(const tTrafficObj& trafficObject)
    {
        const cml::Vec2Df trafficObjPosition{ static_cast<float32_t>(trafficObject.t_0[0]), static_cast<float32_t>(trafficObject.t_0[1]) };
        return LSM_GEOML::Pose{ trafficObjPosition, static_cast<float32_t>(trafficObject.r_zyx[2]) };
    }

    DynamicObjectProperty filldynamicProperty(const unsigned objId, const CemObject& cemObject, const LSM_GEOML::Pose& vehiclePose, const uint64_t cycleTime_ms, std::mt19937& randomEngine) {

        DynamicObjectProperty dynamic_obj_pro;
        const tTrafficObj* trafficObj = getTrafficObjByObjId(objId);
        const LSM_GEOML::CoordinateTransformer2D transformToEgoVehicle(vehiclePose);
        const LSM_GEOML::CoordinateTransformer2D transformToFrTraffic{ getObjectPose(*trafficObj) };

        switch (trafficObj->Cfg.MotionKind) {
        case tMotionKind::MotionKind_4Wheel:
            dynamic_obj_pro.objectClass = VCEM::DynObjClass::CAR;
            break;
        case tMotionKind::MotionKind_2Wheel:
            dynamic_obj_pro.objectClass = VCEM::DynObjClass::BICYCLE;
            break;
        case tMotionKind::MotionKind_Pedestrian:
            dynamic_obj_pro.objectClass = VCEM::DynObjClass::PEDESTRIAN;
            break;
        default:
            dynamic_obj_pro.objectClass = VCEM::DynObjClass::CLASS_UNKNOWN;
            break;
        }
        float32_t cYaw = cos(vehiclePose.Yaw_rad());
        float32_t sYaw = sin(vehiclePose.Yaw_rad());
        float32_t xVel_mps = static_cast<float32_t>(trafficObj->v_0[0]);
        float32_t yVel_mps = static_cast<float32_t>(trafficObj->v_0[1]);
        float32_t xAccel_mps2 = static_cast<float32_t>(trafficObj->a_0[0]);
        float32_t yAccel_mps2 = static_cast<float32_t>(trafficObj->a_0[1]);
        dynamic_obj_pro.velocity = cml::Vec2Df(cYaw * xVel_mps + sYaw * yVel_mps, -sYaw * xVel_mps + cYaw * yVel_mps);
        dynamic_obj_pro.acceleration = cml::Vec2Df(cYaw * xAccel_mps2 + sYaw * yAccel_mps2, -sYaw * xAccel_mps2 + cYaw * yAccel_mps2);
        dynamic_obj_pro.orientation = static_cast<float32_t>(trafficObj->r_zyx[2]) + PI;
        dynamic_obj_pro.yawRate = static_cast<float32_t>(trafficObj->rv_zyx[2]);
        if (dynamic_obj_pro.velocity.x() > 0.1f || dynamic_obj_pro.velocity.y() > 0.1f) {
            dynamic_obj_pro.dynamicProperty = VCEM::DynamicProperty::MOVING;
        }
        else if (dynamic_obj_pro.velocity.x() == 0.0f || dynamic_obj_pro.velocity.y() == 0.0f) {
            dynamic_obj_pro.dynamicProperty = VCEM::DynamicProperty::STOPPED;
        }
        else {
            dynamic_obj_pro.dynamicProperty = VCEM::DynamicProperty::STATIONARY;
        }
        dynamic_obj_pro.state = VCEM::MeasurementObjState::MEASURED; //to do, need more logic to decide the states?
        dynamic_obj_pro.containedInLastSensorUpdate = 16; // 010000 in Binary for CEM active
        dynamic_obj_pro.lastSensorUpdate[CAM] = uint16_t(cemObject.lastDetectionTime_s * 1000);
        dynamic_obj_pro.lifetime = uint32_t(cemObject.lastDetectionTime_s * 1000) - uint32_t(cemObject.firstDetectionCycle * cycleTime_ms);
        LSM_GEOML::AxisAlignedRectangular2D boundingBox{ true };

        if (strstr(trafficObj->Cfg.Info, "- noise")) {
            static std::normal_distribution<float32_t> radNoiseDistribution{ 0.0F, 3.0F * LSM_GEOML::DEG_TO_RAD };
            const float32_t radNoise{ radNoiseDistribution(randomEngine) };

            static std::normal_distribution<float32_t> xNoiseDistribution{ 0.0F, 7.5F * 0.01F };
            const float32_t xNoise_m{ xNoiseDistribution(randomEngine) };

            static std::normal_distribution<float32_t> yNoiseDistribution{ 0.0F, 5.0F * 0.01F };
            const float32_t yNoise_m{ yNoiseDistribution(randomEngine) };

            LSM_GEOML::CoordinateTransformer2D noiseTransform{ };

            for (auto& elem : cemObject.detectedPoints) {
                cml::Vec2Df point{ transformToEgoVehicle.inverseTransform(elem.getPosition()) };

                /// Steepens how fast is the transition from 0.0F to 1.0F. At x = c2, the steepness is c1 / 4.0F. Unit: 1 / m
                const float32_t c1{ 0.75F };
                /// Shifts the inflection point (where the value of sigmoid is 0.5F) away from the ego vehicle. Unit: m
                const float32_t c2{ 10.0F };
                /// Noise dampening constant based on the distance of the point from the origin. The closer the point is, the less noise it gets.
                const float32_t m{ 1.0F / (1.0F + std::exp(- c1 * (point.norm() - c2))) };
                point = noiseTransform.rotateAroundRotCtr(point, { 0.0F, 0.0F }, m * radNoise);
                point.x() += m * xNoise_m;
                point.y() += m * yNoise_m;

                point = transformToFrTraffic.inverseTransform(transformToEgoVehicle.transform(point));
                boundingBox.add(point); // Traffic coordinator
            }
        }
        else {
            for (auto& elem : cemObject.detectedPoints) {
                boundingBox.add(transformToFrTraffic.inverseTransform(elem.getPosition())); // Traffic coordinator
            }
        }

        

        // to global coordinator
        const cml::Vec2Df topLeft_global = transformToFrTraffic.transform(boundingBox.getTopLeft());
        const cml::Vec2Df topRight_global = transformToFrTraffic.transform(boundingBox.getTopRight());
        const cml::Vec2Df botLeft_global = transformToFrTraffic.transform(boundingBox.getBottomRight());
        const cml::Vec2Df botRightglobal = transformToFrTraffic.transform(boundingBox.getBottomLeft());

        // to ego coordinator
        dynamic_obj_pro.points[0] = transformToEgoVehicle.inverseTransform(topLeft_global);
        dynamic_obj_pro.points[1] = transformToEgoVehicle.inverseTransform(topRight_global);
        dynamic_obj_pro.points[2] = transformToEgoVehicle.inverseTransform(botLeft_global);
        dynamic_obj_pro.points[3] = transformToEgoVehicle.inverseTransform(botRightglobal);
        dynamic_obj_pro.referencePoint = {(dynamic_obj_pro.points[0].x() + dynamic_obj_pro.points[1].x() + dynamic_obj_pro.points[2].x() + dynamic_obj_pro.points[3].x())/4,
            (dynamic_obj_pro.points[0].y() + dynamic_obj_pro.points[1].y() + dynamic_obj_pro.points[2].y() + dynamic_obj_pro.points[3].y())/4};

        return dynamic_obj_pro;
    }

    ConvexHull CemSurrogate::createConvexHull(const std::vector<DetectedPoint> &convexHullPoints, const unsigned objId, const CemObject& cemObject,
        const LSM_GEOML::Pose& vehiclePose, const uint64_t cycleTime_ms)
    {
        const tTrafficObj* trafficObj = getTrafficObjByObjId(objId);
        si::StaticObjHeigthType objHeightClass_nu = si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE;
        //the height of the object including the distance from the "Ground" to Object
        float32_t objectHeightFromGround_m = static_cast<float32_t>(trafficObj->Cfg.h + trafficObj->Cfg.zOff);

        // misclassification of high object as curbstone
        if (strstr(trafficObj->Cfg.Info, "misclassify(h,c_t)") || strstr(trafficObj->Cfg.Info, "misclassify(h,c_tb)"))
        {
            //if the distance from the obstacle to sensor is bigger than misclassificationDistanceThreshold_m the traffic object is "misclassified" as CURBSTONE
            if (cemObject.minDistanceToSensor_m > mConfig.misclassificationDistanceThreshold_m)
            {
                objHeightClass_nu = si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE;
            }
            //if the distance from the obstacle to sensor is smaller than misclassificationDistanceThreshold_m the traffic object is classified as HIGH OBSTACLE
            else {
                objHeightClass_nu = si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE;
            }
        }

        // misclassification of curbstone as high object
        else if (strstr(trafficObj->Cfg.Info, "misclassify(c_t,h)") || strstr(trafficObj->Cfg.Info, "misclassify(c_tb,h)"))
        {
            //if the distance from the obstacle to sensor is bigger than misclassificationDistanceThreshold_m the traffic object is "misclassified" as HIGH OBSTACLE
            if (cemObject.minDistanceToSensor_m > mConfig.misclassificationDistanceThreshold_m)
            {
                objHeightClass_nu = si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE;
            }
            //if the distance from the obstacle to sensor is smaller than misclassificationDistanceThreshold_m the traffic object is classified as CURBSTONE
            else {
                objHeightClass_nu = si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE;
            }
        }

        else {
            if (objectHeightFromGround_m > AP_G_MIN_HEIGHT_OBSTACLE_M && objectHeightFromGround_m <= AP_G_MAX_HEIGHT_BODY_TRAVER_M) {
                objHeightClass_nu = si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE;
            }
            else if (objectHeightFromGround_m > AP_G_MAX_HEIGHT_BODY_TRAVER_M  && objectHeightFromGround_m <= AP_G_MAX_HEIGHT_DOOR_OPENABLE_M) {
                objHeightClass_nu = si::StaticObjHeigthType::SO_HI_DOOR_OPENABLE;
            }
        }

        const LSM_GEOML::CoordinateTransformer2D transformToVehicleCS(vehiclePose);
        // Use the perturbed position of the detected points, transformed to the vehicle coordinate system.
        std::vector<cml::Vec2Df> polygon;
        polygon.reserve(convexHullPoints.size());
        for (const auto &point : convexHullPoints) {
            polygon.push_back(transformToVehicleCS.inverseTransform(point.getPosition()));
        }

        const uint64_t objAgeInCycles{ mCycle - cemObject.firstDetectionCycle };
        assert(objAgeInCycles <= static_cast<uint64_t>(std::numeric_limits<uint16_t>::max()));
        //check if name contains wheelstopper
#ifdef VARIANT_PERFORMANCE
        si::StaticObjectClass classification{ si::StaticObjectClass::STAT_OBJ_UNCLASSIFIED_STRUCTURE };
        std::string objInfo{ trafficObj->Cfg.Info };
        std::transform(objInfo.begin(), objInfo.end(), objInfo.begin(),
                       [](unsigned char c) { return (char)std::tolower(c); });
        if (0 == objInfo.compare("wheelstopper"))
        {
            objHeightClass_nu = si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE;
            classification = si::StaticObjectClass::STAT_OBJ_WHEEL_STOPPER;
            //ToDo: add flag to indicate type
        }
#endif
        DynamicObjectProperty dynamic_obj_property;
        CemObjectType objType = CemObjectType::NOT_SET;
        if (cemObject.objectType == DYNAMIC_OBJECT) {
            dynamic_obj_property = filldynamicProperty(objId, cemObject, vehiclePose, cycleTime_ms, mRandomEngine);
            objType = CemObjectType::DYNAMIC_OBJECT;
        }
        else {
            objType = CemObjectType::STATIC_OBJECT;
        }

        return ConvexHull{ std::move(polygon), objId, objHeightClass_nu, static_cast<uint16_t>(objAgeInCycles), objType
#ifdef VARIANT_PERFORMANCE
            , classification
#endif
            , dynamic_obj_property
        };
    }

    //TODO delete this debug function
    void CemSurrogate::saveConvexHulls() {
        const uint64_t timestamp_us = static_cast<uint64_t>(1e6 * SimCore.Time);
        if (timestamp_us % 100000LL == 0LL || timestamp_us % 100000LL == 999LL) { // every 0.1 sec
            std::ofstream hullFile;
            const char *hullFileName = "../Visualization/convex_hulls.csv";
            if (timestamp_us < 200000LL) { // first step
                hullFile.open(hullFileName);
                hullFile << "#timestamp; hull_id; x-groundTruth; y-groundTruth; x-perturbed; y-perturbed" << std::endl;
            }
            else {
                hullFile.open(hullFileName, std::ofstream::out | std::ofstream::app);
            }
            for (const auto &cemObjectEntry : mObjId2CemObjects) {
                for (const auto &detectedPoint : cemObjectEntry.second.detectedPoints) {
                    hullFile << timestamp_us << ";" << cemObjectEntry.first << ";"
                        << detectedPoint.positionGroundTruth_m.x() << ";" << detectedPoint.positionGroundTruth_m.y() << ";"
                        << detectedPoint.getPosition().x() << ";" << detectedPoint.getPosition().y() << std::endl;
                }
            }
            hullFile.close();
        }
    }

    static bool isCircle(const tTrafficObj &trafficObj) {
        return strlen(trafficObj.Cfg.Name) >= 3 && (strncmp("Obs", trafficObj.Cfg.Name, 3) == 0) && strstr(trafficObj.Cfg.Info, "circle");
    }

    std::pair<bool, unsigned int> CemSurrogate::isOnSegment(const tTrafficObj &trafficObj, const DetectedPoint detectedPoint, const TrafficContour2D trafficContours[]) {
        float32_t segmentNumber;
        const cml::Vec2Df trafficObjPosition{ static_cast<float32_t>(trafficObj.t_0[0]), static_cast<float32_t>(trafficObj.t_0[1]) };
        const LSM_GEOML::CoordinateTransformer2D transformToFrTraffic{ LSM_GEOML::Pose{trafficObjPosition, static_cast<float32_t>(trafficObj.r_zyx[2])} };
        // If the object is wider than long, use the y axis as principal axis.
        if (trafficObj.Cfg.w >= trafficObj.Cfg.l) {
            const auto lateralDistanceDetectedPoint = transformToFrTraffic.inverseTransform(detectedPoint.positionGroundTruth_m).y();

            // Calculate how many segments of the SEGMENT + GAP form are from the origin to the detected point.
            // In order to avoid negative points, if the object is mirrored, add to the point half of the object width. If the object is not mirrored, add to the point the lowest y value of the object.
            segmentNumber = trafficContours[trafficObj.Cfg.Id].isMirroringConstruction_nu ?
                static_cast<float32_t>(lateralDistanceDetectedPoint + trafficObj.Cfg.w / 2) / (mConfig.fragmentObjectLength + mConfig.fragmentGapLength) :
                static_cast<float32_t>(lateralDistanceDetectedPoint +
                    abs(*std::min_element(trafficContours[trafficObj.Cfg.Id].points[1], std::end(trafficContours[trafficObj.Cfg.Id].points[1])))) /
                    (mConfig.fragmentObjectLength + mConfig.fragmentGapLength);
        }
        else {
            const auto lateralDistanceDetectedPoint = transformToFrTraffic.inverseTransform(detectedPoint.positionGroundTruth_m).x();
            // In order to avoid negative points, add to the point the smallest x value of the object.
            const auto smallestX = *std::min_element(trafficContours[trafficObj.Cfg.Id].points[0], std::end(trafficContours[trafficObj.Cfg.Id].points[0]));
            segmentNumber = (lateralDistanceDetectedPoint - smallestX) / (mConfig.fragmentObjectLength + mConfig.fragmentGapLength);
        }
        // Extract the decimals and check if they are lower than the weight of the fragment object length in relation to the whole SEGMENT + GAP structure. If it's true, it means that the detected point is on a segment.
        const bool onSegment = (segmentNumber - floor(segmentNumber)) < mConfig.fragmentObjectLength / (mConfig.fragmentGapLength + mConfig.fragmentObjectLength);
        return std::make_pair(onSegment, static_cast<unsigned int>(segmentNumber));
    }

    static LSM_GEOML::Pose getSensorPose(const tFSpaceSensor& freeSpaceSensor)
    {
        const cml::Vec2Df sensorPosition{ static_cast<float32_t>(freeSpaceSensor.BS_Pos_0[0]), static_cast<float32_t>(freeSpaceSensor.BS_Pos_0[1]) };
        return LSM_GEOML::Pose(sensorPosition, static_cast<float32_t>(freeSpaceSensor.rot_zyx[2] + freeSpaceSensor.bs.Fr->r_zyx[2]));
    }

    static bool isDetectionWithinAllowedObjectThickness(const DetectedPoint& detectedPoint, const tTrafficObj& trafficObj, const bool isScanning, const float32_t allowedThickness_m) {
        // The detected object thickness is only limited during scanning and if the allowed thickness is set.
        if ((!isScanning) || (allowedThickness_m <= 0.0F)) {
            return true;
        }
        // transform the detected point to the coordinate system of the traffic object
        const cml::Vec2Df detectedPointLocal{ detectedPoint.isInLocalCoordinates ? detectedPoint.getPosition() :
                        LSM_GEOML::CoordinateTransformer2D::inverseTransform(getObjectPose(trafficObj), detectedPoint.getPosition()) };
        const float32_t angleObjToEgo_rad{ LSM_GEOML::radMod(static_cast<float32_t>(trafficObj.r_zyx[2] - Car.Yaw)) };
        if ((std::abs(angleObjToEgo_rad) > LSM_GEOML::LSM_QUARTER_PI) && (std::abs(angleObjToEgo_rad) < LSM_GEOML::LSM_THREE_QUARTER_PI)) {
            // check longitudinal direction of object since it is perpendicular to the ego vehicle
            return (detectedPointLocal.x() < allowedThickness_m) || ((trafficObj.Cfg.l - detectedPointLocal.x()) < allowedThickness_m);
        }
        else {
            // check lateral direction of object since it is parallel to the ego vehicle
            return (0.5 * trafficObj.Cfg.w - std::abs(detectedPointLocal.y())) < allowedThickness_m;
        }
    }

    std::vector<ConvexHull> CemSurrogate::determineObstacles(const float minObjHeight_m, const bool isScanning, const TrafficContour2D trafficContours[],
        const uint64_t cycleTime_ms, const LSM_GEOML::Pose& vehiclePose)
    {
        struct Key {
            unsigned objId;
            unsigned verticalSegm;
            Key(unsigned objId, unsigned verticalSegm) : objId(objId), verticalSegm(verticalSegm) { }
            bool operator<(const Key &right) const {
                return (objId < right.objId || (objId == right.objId && verticalSegm < right.verticalSegm));
            }
        };
        std::vector<ConvexHull> convexHulls;

        if (SimCore.State == SCState_Simulate && SimCore.Time > 0.0) {
            ++mCycle;
            std::map<Key, std::vector<DetectedPoint>> objId2NewPoints;

            std::vector<int> mainAxisSegments(2);
            bool oddSegmentNumber{ false };
            float32_t detectionDistance{ 0.0f };

            // determine all detected points for each sensor individually
            for (int iSens = 0; iSens < FSpaceSensorCount; ++iSens) {
                // Ignore camera FSpaceSensors if cameras are disabled for the CEM surrogate model
                if (mConfig.disableCameras_nu && strncmp(FSpaceSensor[iSens].bs.Name, "CAM", 3) == 0) {
                    continue;
                }
                // Assert that the free-space sensor data are not older than 1 ms (e.g. one CarMaker cycle delayed)
                assert(std::abs(SimCore.Time - FSpaceSensor[iSens].TimeStamp) < 0.0011);
                // Update the sensor field of views
                mSensorName2Fov[FSpaceSensor[iSens].bs.Name].setSensorPose(getSensorPose(FSpaceSensor[iSens]));

                // determine the main axis segments for each sensor
                // if the number of the horizontal segments is an even number, there are two segments as main axis
                // if the number of the horizontal segments is an odd number, the main axis is the middle segment
                if (mConfig.smallObjectDetectionDelay_nu) {
                    int mainAxisSegment = FSpaceSensor[iSens].nHorSegm / 2;

                    if (FSpaceSensor[iSens].nHorSegm % 2 == 0) {
                        mainAxisSegments[0] = mainAxisSegment - 1;
                        mainAxisSegments[1] = mainAxisSegment;
                        oddSegmentNumber = 0;
                    }

                    else if (FSpaceSensor[iSens].nHorSegm % 2 == 1) {
                        mainAxisSegments[0] = mainAxisSegment;
                        oddSegmentNumber = 1;
                    }
                }

                for (int j = 0; j < FSpaceSensor[iSens].nTotSegm; ++j) {

                    const tSegment &sensorSegment = FSpaceSensor[iSens].Segm[j];
                    const tTrafficObj* trafficObj = Traffic_GetByObjId(sensorSegment.ObjId);
                    if (sensorSegment.ObjId >= 0 && trafficObj && (strcmp(trafficObj->Cfg.Name, "Odo") != 0) // exclude odometry box
                        && !(trafficObj->Cfg.h < minObjHeight_m && trafficObj->Cfg.Name[0] == 'P') // exclude parking boxes
                        && !((trafficObj->Cfg.h < AP_G_MIN_HEIGHT_OBSTACLE_M) && strstr(trafficObj->Cfg.Name, "ODS")) // exclude odSlots
                        // exclude road and parking line marking objects
                        && static_cast<float32_t>(trafficObj->Cfg.h) > AP_G_MIN_HEIGHT_OBSTACLE_M
                        && !(strlen(trafficObj->Cfg.Name) >= 3 && (strncmp("Lim", trafficObj->Cfg.Name, 3) == 0) && strstr(trafficObj->Cfg.Info, "- line"))
                        //exclude the points detected by 4 front-facing and 4 back-facing sensors if the distance between the detected point and the sensor is bigger than a threshold set from CarMaker
                        //this behavior is active only if garageDetectionDistance_m is set from CarMaker and is bigger than 0
                        && !(sensorSegment.ds_p >= mConfig.garageDetectionDistance_m && mConfig.garageDetectionDistance_m > 0.0f
                            && (strncmp(FSpaceSensor[iSens].bs.Name, "USS_R", 5) == 0 || strncmp(FSpaceSensor[iSens].bs.Name, "USS_F", 5) == 0 || strncmp(FSpaceSensor[iSens].bs.Name, "CAM_F", 5) == 0 || strncmp(FSpaceSensor[iSens].bs.Name, "CAM_B", 5) == 0)))
                    {
                        if (mConfig.smallObjectDetectionDelay_nu) {
                            // translate every segment to the first row (keeping the position) of segments in order to store in mainAxisSegments maximum two values
                            // | | | |      | | x |
                            // | | | |  =>  | | | |
                            // | | x |      | | | |
                            const int segmentTranslated = j - (j / FSpaceSensor[iSens].nHorSegm) * FSpaceSensor[iSens].nHorSegm;

                            // Check if the translated segment coincide with the main axis.
                            // If there is no need to differentiate if either the detected point is or is not on the main axis, if the smallObjectDetectionThresholdNotMainAxis_m is not set
                            // to a specific value in Carmaker Minimaneuver, it will take the smallObjectDetectionThreshold_m value
                            if (oddSegmentNumber) {
                                segmentTranslated == mainAxisSegments[0] ? detectionDistance = mConfig.smallObjectDetectionThreshold_m : detectionDistance = mConfig.smallObjectDetectionThresholdNotMainAxis_m;
                            }
                            else {
                                (segmentTranslated == mainAxisSegments[0] || segmentTranslated == mainAxisSegments[1]) ? detectionDistance = mConfig.smallObjectDetectionThreshold_m : detectionDistance = mConfig.smallObjectDetectionThresholdNotMainAxis_m;
                            }
                        }

                        // apply the detection delay only if the detected point is from a curbstone object or a pole object
                        if (mConfig.smallObjectDetectionDelay_nu && ((sensorSegment.ds_p <= detectionDistance && ((trafficObj->Cfg.h <= CURB_OBJECT_HEIGHT_M) || (trafficObj->Cfg.l <= POLE_LENGTH_THRESHOLD_M && trafficObj->Cfg.w <= POLE_WIDTH_THRESHOLD_M)))
                            || (trafficObj->Cfg.h > CURB_OBJECT_HEIGHT_M) && (trafficObj->Cfg.l > POLE_LENGTH_THRESHOLD_M || trafficObj->Cfg.w > POLE_WIDTH_THRESHOLD_M))
                            || !mConfig.smallObjectDetectionDelay_nu)
                        {
                            Key mapKey{ 0U, 0U };
                            std::pair<bool, unsigned int> pointOnSegmentInfo = std::make_pair(false, 0U);
                            const bool isFragmented = strstr(trafficObj->Cfg.Info, "fragmented");
                            const cml::Vec2Df detectedPointPosition{ static_cast<float32_t>(sensorSegment.P_0[0]), static_cast<float32_t>(sensorSegment.P_0[1]) };
                            const SensorType sensorType = strstr(FSpaceSensor[iSens].bs.Name, "CAM") ? SensorType::CAM :
                                (strstr(FSpaceSensor[iSens].bs.Name, "USS") ? SensorType::USS : SensorType::UNKNOWN);
                            const DetectedPoint detectedPoint{ detectedPointPosition, sensorType, sensorSegment.ds_p, sensorSegment.alpha_p };

                            // If there is the "fragmented" keyword in the traffic object info and fragmentGapLength has an assigned value, store in pointOnSegmentInfo if the detected point is on a segment.
                            // If it is on a segment, store the number of that segment.
                            if (isFragmented && mConfig.fragmentGapLength != 0
                                //after fragment merged, no need to add the fragment object anymore.
                                && (mConfig.fragmentMergeTime_s < 0.0 || !mFragMergedflg)) {
                                pointOnSegmentInfo = isOnSegment(*trafficObj, detectedPoint, trafficContours);
                                // Use a different ID for the fragment object than the original object.
                                mapKey = Key{ static_cast<unsigned>((trafficObj->Cfg.Id + 1) * SEGMENT_STARTING_INDEX + pointOnSegmentInfo.second), static_cast<unsigned>(j / FSpaceSensor[iSens].nHorSegm) };
                            }
                            else {
                                mapKey = Key{ static_cast<unsigned>(trafficObj->Cfg.Id), static_cast<unsigned>(j / FSpaceSensor[iSens].nHorSegm) };
                            }

                            if ((isFragmented && pointOnSegmentInfo.first) || !isFragmented || mConfig.fragmentGapLength == 0 || mFragMergedflg) {

                                if (!objId2NewPoints.count(mapKey)) {
                                    objId2NewPoints.insert(std::make_pair(mapKey, std::vector<DetectedPoint>()));
                                }

                                // Add the detected point only if it differs from the last detected point.
                                // This if-condition is necessary since the FSpaceSensor segments sometimes contain identical points, which would cause problems later.
                                if (objId2NewPoints.at(mapKey).empty() ||
                                    static_cast<cml::Vec2Df>(detectedPoint.positionGroundTruth_m - objId2NewPoints.at(mapKey).back().positionGroundTruth_m).norm() > LSM_CML::MIN_FLT_DIVISOR) {
                                    objId2NewPoints.at(mapKey).push_back(detectedPoint);
                                }

                                CemObject& cemObject{ mObjId2CemObjects[mapKey.objId] };
                                // Set objectType and firstDetectionCycle for new CemObjects
                                if (CemObjectType::NOT_SET == cemObject.objectType) {
                                    if (strstr(trafficObj->Cfg.Info, "- stationaryDynamic"))
                                    {
                                        cemObject.objectType = (trfObjIsStatic(trafficObj) && (tObjectKind::ObjectKind_Movable != trafficObj->Cfg.ObjectKind)) ? CemObjectType::STATIC_OBJECT : CemObjectType::DYNAMIC_OBJECT;
                                    }
                                    else
                                    {
                                        cemObject.objectType = (trfObjIsStatic(trafficObj) || (tObjectKind::ObjectKind_Movable != trafficObj->Cfg.ObjectKind)) ? CemObjectType::STATIC_OBJECT : CemObjectType::DYNAMIC_OBJECT;
                                    }
                                    cemObject.firstDetectionCycle = mCycle;
                                }
                                //save the minimum distance between traffic object and ego vehicle
                                if (cemObject.minDistanceToSensor_m < 0.0) {
                                    cemObject.minDistanceToSensor_m = 1000.0;
                                }
                                if (sensorSegment.ds_p < cemObject.minDistanceToSensor_m) {
                                    cemObject.minDistanceToSensor_m = sensorSegment.ds_p;
                                }
                                //update the lastDetectionTime_s for the detected object
                                cemObject.lastDetectionTime_s = SimCore.Time;
                            }
                        }
                    }
                }

                // calculate the normal vector of each detected point
                const cml::Vec2Df sensorPosition{ static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[0]), static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[1]) };
                for (auto &mapElement : objId2NewPoints) {
                    auto &newPointsPerObj = mapElement.second;
                    const auto numPoints = newPointsPerObj.size();
                    if (numPoints > 1) {
                        cml::Vec2Df tangent;
                        for (size_t iPt = 0; iPt < numPoints; ++iPt) {
                            if (iPt == 0) {
                                tangent = newPointsPerObj[iPt + 1].positionGroundTruth_m - newPointsPerObj[iPt].positionGroundTruth_m;
                            }
                            else if (iPt == numPoints - 1) {
                                tangent = newPointsPerObj[iPt].positionGroundTruth_m - newPointsPerObj[iPt - 1].positionGroundTruth_m;
                            }
                            else {
                                tangent = newPointsPerObj[iPt + 1].positionGroundTruth_m - newPointsPerObj[iPt - 1].positionGroundTruth_m;
                            }
                            newPointsPerObj[iPt].normalVector_m = calcNormalVector(newPointsPerObj[iPt].positionGroundTruth_m, tangent, sensorPosition);
                        }
                    }
                    else { // discard single point, since no normal vector can be determined
                        newPointsPerObj.clear();
                    }
                }

                // Check reflection law for points detected by the ultrasound sensors:
                // calculate the incident vector for detected points and remove all points which have an incidence angle grater than 10 degree and SensorType = USS
                for (auto &mapElement : objId2NewPoints) {
                    auto &newPointsPerObj = mapElement.second;
                    for (auto point = newPointsPerObj.begin(); point != newPointsPerObj.end(); ) {
                        const cml::Vec2Df incidentVector{ sensorPosition - point->positionGroundTruth_m };
                        const float32_t incidenceAngle_deg{ abs(LSM_GEOML::convRadToDeg(LSM_GEOML::calcAngleBetweenVectors(incidentVector, point->normalVector_m))) };
                        if ((incidenceAngle_deg > 10.0F) && (point->detectingSensor == VCEM::SensorType::USS)) {
                            point = newPointsPerObj.erase(point);

                        }
                        else
                        {
                            ++point;
                        }
                    }
                }

                // calculate CEM sensor data degradation effects
                for (auto &mapElement : objId2NewPoints) {
                    const tTrafficObj* trafficObj = getTrafficObjByObjId(mapElement.first.objId);
                    const LSM_GEOML::CoordinateTransformer2D transformToFrTraffic{ getObjectPose(*trafficObj) };
                    const cml::Vec2Df lateralToTrafficObj{ -static_cast<float32_t>(sin(trafficObj->r_zyx[2])), static_cast<float32_t>(cos(trafficObj->r_zyx[2])) };
                    const bool wasFlankKinkCorrected = mObjId2CemObjects[mapElement.first.objId].flankKinkCorrectionStarted;
                    for (auto &detectedPoint : mapElement.second) {
                        // calculate offset due to erroneous detection of object corners (i.e. front and back of vehicle object)
                        // method: add offset in normal direction only if the perturbed point will not be at or beyond an object flank
                        const auto lateralDistanceDetectedPoint = mConfig.cornerDetectionError_m > 0.0f ?
                            abs(transformToFrTraffic.inverseTransform(detectedPoint.positionGroundTruth_m + mConfig.cornerDetectionError_m * detectedPoint.normalVector_m).y())
                            : abs(transformToFrTraffic.inverseTransform(detectedPoint.positionGroundTruth_m).y()) * (1.0f + LSM_CML::MIN_FLT_DIVISOR);
                        if (isScanning && lateralDistanceDetectedPoint < static_cast<float32_t>(trafficObj->Cfg.w) / 2.0f && !isCircle(*trafficObj)) {
                            detectedPoint.offsetCornerDetectionError_m = mConfig.cornerDetectionError_m * detectedPoint.normalVector_m;
                        }
                        // Modelling of erroneous US lines which reach into parking slot:
                        // calculate step/kink in the flank of a traffic object if it is tagged by the keywords "flankKinkRight" or "flankKinkLeft"
                        const bool isCenterOfFlank{ abs(transformToFrTraffic.inverseTransform(detectedPoint.positionGroundTruth_m).x() - static_cast<float32_t>(trafficObj->Cfg.l) / 2.0f) < 0.1f };
                        const bool isRightFlank{ transformToFrTraffic.inverseTransform(detectedPoint.positionGroundTruth_m).y() < 0.0f };
                        if (SensorType::USS == detectedPoint.detectingSensor && isCenterOfFlank && !wasFlankKinkCorrected &&
                            (isRightFlank ? strstr(trafficObj->Cfg.Info, "flankKinkRight") : strstr(trafficObj->Cfg.Info, "flankKinkLeft"))) {
                            // apply the flank kink error in the direction of the lateral component of the normal vector
                            detectedPoint.flankKinkErrorAmount = 1.0f;
                            detectedPoint.offsetFlankKinkError_m = mConfig.flankKinkWidth_m *
                                detectedPoint.normalVector_m.scalarProduct(lateralToTrafficObj) * lateralToTrafficObj;
                        }
                        // if the detected point is further than CAMERA_UNCERTAINTY_DISTANCE_THRESHOLD_M from the camera and is on the edge of the sensor FOV, add the cameraUncertainty_m
                        if (isScanning && SensorType::CAM == detectedPoint.detectingSensor && detectedPoint.distanceToSensor_m > CAMERA_UNCERTAINTY_DISTANCE_THRESHOLD_M &&
                            (abs(detectedPoint.azimuthalAngleToSensor_rad) > CAMERA_FOV_EDGE_MIN_ANGLE && abs(detectedPoint.azimuthalAngleToSensor_rad) < CAMERA_FOV_EDGE_MAX_ANGLE)) {
                            detectedPoint.offsetCameraUncertainty_m = mConfig.cameraUncertainty_m * detectedPoint.normalVector_m;
                        }
                    }

                    // Transform detected points of dynamic objects to the object's local coordinate system.
                    // Hereby, they move around with the object if they are transformed back to global coordinates using the updated object pose.
                    if (mConfig.enableDynamicObjectMovement_nu && CemObjectType::DYNAMIC_OBJECT == mObjId2CemObjects[mapElement.first.objId].objectType) {
                        for (auto& detectedPoint : mapElement.second) {
                            detectedPoint.transformToLocalCoordinates(transformToFrTraffic);
                        }
                    }
                }

                // calculate additional points and add them to the detected points
                for (auto &mapElement : objId2NewPoints) {
                    const unsigned int objId = mapElement.first.objId;
                    auto &newPointsPerObj = mapElement.second;
                    addOppositePointsTo(newPointsPerObj);
                    // add new points to the detected points for that objId
                    mObjId2CemObjects[objId].detectedPoints.insert(mObjId2CemObjects[objId].detectedPoints.end(), newPointsPerObj.begin(), newPointsPerObj.end());
                }
                objId2NewPoints.clear();
            } // for iSens

            // evaluate CEM sensor data degradation effects
            const cml::Vec2Df latencyOffset_m = {
                static_cast<float32_t>(Car.ConBdy1.v_1[0]) * mConfig.latencyTime_ms / 1000.0f + mConfig.staticOffsetX_m,
                static_cast<float32_t>(Car.ConBdy1.v_1[1]) * mConfig.latencyTime_ms / 1000.0f + mConfig.staticOffsetY_m };
            const float32_t flankKinkErrorCorrectionStep = (mConfig.flankKinkCorrectionTimeSpan_s > LSM_CML::MIN_FLT_DIVISOR) ?
                std::min(1.0f, cycleTime_ms / (mConfig.flankKinkCorrectionTimeSpan_s*1000.0f)) : 1.0f;
            for (auto &mapElement : mObjId2CemObjects) {
                const auto objId{ mapElement.first };
                CemObject& cemObject{ mapElement.second };
                const tTrafficObj* trafficObj = getTrafficObjByObjId(objId);
                const LSM_GEOML::CoordinateTransformer2D transformToFrTraffic{ getObjectPose(*trafficObj) };
                for (auto &detectedPoint : cemObject.detectedPoints) {
                    // If not in scanning mode, remove the corner detection error if the detected point is close to another accurately detected point.
                    if (!isScanning && detectedPoint.offsetCornerDetectionError_m.norm1() > LSM_CML::MIN_FLT_DIVISOR) {
                        for (const auto &otherPoint : cemObject.detectedPoints) {
                            if (otherPoint.offsetCornerDetectionError_m.norm1() < LSM_CML::MIN_FLT_DIVISOR &&
                                static_cast<cml::Vec2Df>(detectedPoint.positionGroundTruth_m - otherPoint.positionGroundTruth_m).norm() <= MIN_POINT_DISTANCE_M) {
                                detectedPoint.offsetCornerDetectionError_m = cml::Vec2Df(0.0F, 0.0F);
                                break;
                            }
                        }
                    }

                    // Remove the flank kink error when the detected point is sufficiently close to the US sensor
                    // that was defined in the traffic-object description or to one US sensor if no specific sensor was defined.
                    if (detectedPoint.offsetFlankKinkError_m.norm1() > LSM_CML::MIN_FLT_DIVISOR) {
                        for (int iSens = 0; iSens < FSpaceSensorCount; ++iSens) {
                            const bool sensorCanCorrectFlankKink = strstr(FSpaceSensor[iSens].bs.Name, "USS") // is US sensor
                                // the sensor is specified in the traffic-object description or no US sensors have been specified in the traffic-object description
                                && (strstr(trafficObj->Cfg.Info, FSpaceSensor[iSens].bs.Name) || !strstr(trafficObj->Cfg.Info, "USS"));
                            const cml::Vec2Df sensorPosition{ static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[0]), static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[1]) };
                            if (sensorCanCorrectFlankKink && static_cast<cml::Vec2Df>(detectedPoint.getPosition() - sensorPosition).norm() < mConfig.flankKinkCorrectionDistance_m) {
                                detectedPoint.flankKinkErrorAmount = std::max(0.0f, detectedPoint.flankKinkErrorAmount - flankKinkErrorCorrectionStep);
                                cemObject.flankKinkCorrectionStarted = true;
                                break;
                            }
                        }
                    }
                    // Remove the camera uncertainty error if the detected point is close to a point that was detected by ultrasound.
                    if (detectedPoint.offsetCameraUncertainty_m.norm1() > LSM_CML::MIN_FLT_DIVISOR) {
                        for (const auto &otherPoint : cemObject.detectedPoints) {
                            if (SensorType::USS == otherPoint.detectingSensor &&
                                static_cast<cml::Vec2Df>(detectedPoint.positionGroundTruth_m - otherPoint.positionGroundTruth_m).norm() <= MIN_POINT_DISTANCE_M) {
                                detectedPoint.offsetCameraUncertainty_m = cml::Vec2Df(0.0F, 0.0F);
                                break;
                            }
                        }
                    }
                    // Update latency offset only if the object was detected in the current cycle
                    if (std::abs(cemObject.lastDetectionTime_s - SimCore.Time) < LSM_GEOML::MIN_FLT_DIVISOR) {
                        detectedPoint.offsetLatency_m = latencyOffset_m;
                    }

                    detectedPoint.updatePosition(transformToFrTraffic);
                }

                // Limit the detected object thickness during scanning if mConfig.objectThicknessDuringScanning_m > 0.
                for (auto detectedPointIt = cemObject.detectedPoints.begin(); detectedPointIt != cemObject.detectedPoints.end(); ) {
                    if (isDetectionWithinAllowedObjectThickness(*detectedPointIt, *trafficObj, isScanning, mConfig.objectThicknessDuringScanning_m)) {
                        ++detectedPointIt;
                    }
                    else {
                        detectedPointIt = cemObject.detectedPoints.erase(detectedPointIt);
                    }
                }
            }

            //merge the fragments when fragment merge time is up. After merge the flag is set true, this loop will not be gone through again.
            if (mConfig.fragmentMergeTime_s > 0.0 && SimCore.Time >= mConfig.fragmentMergeTime_s && !mFragMergedflg) {
                for (const auto& map_elem : mObjId2CemObjects) {
                    const unsigned obj_id{ map_elem.first };
                    if (obj_id >= SEGMENT_STARTING_INDEX) {
                        const signed merged_obj_id = obj_id / SEGMENT_STARTING_INDEX - 1;
                        const CemObject& cem_object{ map_elem.second };
                        if (mObjId2CemObjects.count(merged_obj_id)) {
                            mObjId2CemObjects[merged_obj_id].detectedPoints.insert(mObjId2CemObjects[merged_obj_id].detectedPoints.end(), cem_object.detectedPoints.begin(), cem_object.detectedPoints.end());
                            mObjId2CemObjects[merged_obj_id].firstDetectionCycle = std::min(mObjId2CemObjects[merged_obj_id].firstDetectionCycle, cem_object.firstDetectionCycle);
                            mObjId2CemObjects[merged_obj_id].lastDetectionTime_s = std::max(mObjId2CemObjects[merged_obj_id].lastDetectionTime_s, cem_object.lastDetectionTime_s);
                            mObjId2CemObjects[merged_obj_id].detectionMissesCount = std::min(mObjId2CemObjects[merged_obj_id].detectionMissesCount, cem_object.detectionMissesCount);
                            mObjId2CemObjects[merged_obj_id].minDistanceToSensor_m = std::min(mObjId2CemObjects[merged_obj_id].minDistanceToSensor_m, cem_object.minDistanceToSensor_m);
                            mObjId2CemObjects[merged_obj_id].flankKinkCorrectionStarted = mObjId2CemObjects[merged_obj_id].flankKinkCorrectionStarted || cem_object.flankKinkCorrectionStarted;
                        }
                        else {
                            mObjId2CemObjects[merged_obj_id] = cem_object;
                        }
                    }
                }
                // remove fragments for CemObject map
                for (auto it = mObjId2CemObjects.begin(); it != mObjId2CemObjects.end();) {
                    if (it->first >= SEGMENT_STARTING_INDEX) {
                        it = mObjId2CemObjects.erase(it);
                    }
                    else {
                        ++it;
                    }
                }
                mFragMergedflg = true;
            }

            // calculate convex hulls from the detected points for all objects
            std::set<unsigned> idsDisappearedObjects;
            for (auto &mapElement : mObjId2CemObjects) {
                const unsigned objId{ mapElement.first };
                CemObject& cemObject{ mapElement.second };
                if (cemObject.detectedPoints.size() < 3) {
                    continue;
                }
                // determine the convex hull of all points for that object
                std::vector<DetectedPoint> convexHullPoints = convex_hull(cemObject.detectedPoints);
                cemObject.detectedPoints = convexHullPoints; // store convex hull as detected points
                reduceNumberOfPoints(convexHullPoints, cemObject.detectedPoints);
                if (hasDisappeared(cemObject, cycleTime_ms))
                {
                    idsDisappearedObjects.insert(objId);
                }
                else if (convexHullPoints.size() > 2)
                {
                    convexHulls.push_back(createConvexHull(convexHullPoints, objId, cemObject, vehiclePose, cycleTime_ms));
                }
            }

            for (const unsigned objId : idsDisappearedObjects)
            {
                mObjId2CemObjects.erase(objId);
            }

            // TODO remove debug writeout
            //saveConvexHulls();
        }
        return convexHulls;
    }

    std::vector<PclDelimiter> CemSurrogate::determineDelimiters(const bool isScanning, const TrafficContour2D trafficContours[],
        const uint64_t cycleTime_ms, const LSM_GEOML::Pose& vehiclePose)
    {
        struct PclLine {
            cml::Vec2Df startPoint{ 0.0F, 0.0F };
            cml::Vec2Df endPoint{ 0.0F, 0.0F };
        };

        std::vector<PclDelimiter> pclDelimiters{};
        if (SimCore.State == SCState_Simulate && SimCore.Time > 0.0 && !(mConfig.disableCameras_nu)) {
            const double offsetX = Car.ConBdy1.v_1[0] * mConfig.latencyTime_ms / 1000.0f + mConfig.staticOffsetX_m;
            const double offsetY = Car.ConBdy1.v_1[1] * mConfig.latencyTime_ms / 1000.0f + mConfig.staticOffsetY_m;
            std::map<unsigned int, PclLine> objId2CompleteLines{};

            //to plot the pmdSurrogate camera images
            if (mConfig.pmdSurrogateEnabled_nu) {
                for (int iSens = 0; iSens < FSpaceSensorCount; ++iSens) {
                    if (strncmp(FSpaceSensor[iSens].bs.Name, "CAM", 3) == 0) {
                        const cml::Vec2Df sensorPosition{ static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[0]), static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[1]) };
                        const LSM_GEOML::Pose cameraPos(sensorPosition, static_cast<float32_t>(FSpaceSensor[iSens].bs.Fr->r_zyx[2]));
                        mPmdSurrogate.createCameraRoiImageMask(cameraPos, FSpaceSensor[iSens].bs.Name);
                    }
                }
            }
            // determine the global position of all parking line marking objects
            for (unsigned int i = 0; i < (unsigned int)Traffic.nObjs; i++) {
                const tTrafficObj* trafficObj = Traffic_GetByTrfId(i);
                if (strlen(trafficObj->Cfg.Name) >= 3 && strncmp("Lim", trafficObj->Cfg.Name, 3) == 0
                    && strstr(trafficObj->Cfg.Info, "- line")) {
                    const double cYaw = cos(trafficObj->r_zyx[2]);
                    const double sYaw = sin(trafficObj->r_zyx[2]);
                    const auto &contourPoints = trafficContours[i].points; // first index: x-, y-coord; second index: point index
                    /*position of start point of the parking space marking*/
                    cml::Vec2Df startPoint{
                        static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][0] - sYaw * contourPoints[1][1] / 2),
                        static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][0] + cYaw * contourPoints[1][1] / 2) };
                    /*position of end point of the parking space marking*/
                    cml::Vec2Df endPoint{
                        static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][2] - sYaw * contourPoints[1][1] / 2),
                        static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][2] + cYaw * contourPoints[1][1] / 2) };

                    objId2CompleteLines.insert(std::make_pair(trafficObj->Cfg.ObjId, PclLine{ startPoint, endPoint }));
                }
            }

            std::set<unsigned> idsOfLinesToCorrectOrientation;
            // update detected line range for all detected line objects
            for (int iSens = 0; iSens < FSpaceSensorCount; ++iSens) {
                //use only the camera FreeSpaceSensors
                if (strncmp(FSpaceSensor[iSens].bs.Name, "CAM", 3) == 0) {
                    const cml::Vec2Df sensorPosition{ static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[0]), static_cast<float32_t>(FSpaceSensor[iSens].BS_Pos_0[1]) };
                    for (int j = 0; j < FSpaceSensor[iSens].nTotSegm; ++j) {
                        const tSegment &sensorSegment = FSpaceSensor[iSens].Segm[j];
                        const tTrafficObj* trafficObj = Traffic_GetByObjId(sensorSegment.ObjId);
                        if (sensorSegment.ObjId >= 0 && trafficObj && objId2CompleteLines.count(sensorSegment.ObjId)) {
                            const PclLine &completeLine = objId2CompleteLines.at(sensorSegment.ObjId);
                            const cml::Vec2Df detectedPoint{ static_cast<float32_t>(sensorSegment.P_0[0] + offsetX), static_cast<float32_t>(sensorSegment.P_0[1] + offsetY) };
                            // project the detected point onto the complete line to determine it's relative position on that line
                            const cml::Vec2Df completeLineVec = completeLine.endPoint - completeLine.startPoint;
                            const float32_t relativeLinePosition = completeLineVec.scalarProduct(detectedPoint - completeLine.startPoint) / completeLineVec.normSq();

                            bool isSegmentDetectedByPmd{ false };
                            if (mConfig.pmdSurrogateEnabled_nu)
                            {
                                //the algorithm will be applied only if is available to the specific camera
                                isSegmentDetectedByPmd = (mPmdSurrogate.getCameraAvailability(FSpaceSensor[iSens].bs.Name)) ?
                                    mPmdSurrogate.isSegmentPointDetected(detectedPoint, FSpaceSensor[iSens].bs.Name) : false;
                            }

                            if (!mConfig.pmdSurrogateEnabled_nu || (mConfig.pmdSurrogateEnabled_nu && isSegmentDetectedByPmd)) {
                                if (relativeLinePosition >= 0.0f && relativeLinePosition <= 1.0f) {
                                    if (mObjId2detectedLine.count(sensorSegment.ObjId)) {
                                        auto &detectedLine = mObjId2detectedLine.at(sensorSegment.ObjId);
                                        if (static_cast<cml::Vec2Df>(detectedPoint - sensorPosition).norm() < mConfig.lineOrientationCorrectionDistance_m && !isScanning) {
                                            // store lines to correct in a set in order to correct them only once per cycle
                                            idsOfLinesToCorrectOrientation.insert(sensorSegment.ObjId);
                                        }
                                        detectedLine.minRange = std::min(detectedLine.minRange, relativeLinePosition);
                                        detectedLine.maxRange = std::max(detectedLine.maxRange, relativeLinePosition);
                                    }
                                    else {
                                        mObjId2detectedLine.insert(std::make_pair(sensorSegment.ObjId, DetectedLine{ relativeLinePosition, relativeLinePosition, 1.0f }));
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // correct the line orientation error
             //if lineOrientationCorrectionTimeSpan_s is not set via CarMaker the correction is made in a single step/cycle
            const float32_t orientationAngleCorrectionStep = (mConfig.lineOrientationCorrectionTimeSpan_s > LSM_CML::MIN_FLT_DIVISOR) ?
                std::min(1.0f, cycleTime_ms / (mConfig.lineOrientationCorrectionTimeSpan_s*1000.0f)) : 1.0f;
            for (const auto id : idsOfLinesToCorrectOrientation) {
                auto &detectedLine = mObjId2detectedLine.at(id);
                if (detectedLine.lineErrorAmount > 0.0f) {
                    detectedLine.lineErrorAmount = std::max(0.0f, detectedLine.lineErrorAmount - orientationAngleCorrectionStep);
                }
            }

            // transform detected line ranges to PCL delimiters in the vehicle coordinate system
            const LSM_GEOML::CoordinateTransformer2D transformToVehicleCS(vehiclePose);
            for (const auto &mapElement : mObjId2detectedLine) {
                const unsigned objId = mapElement.first;
                const auto &detectedLine = mapElement.second;
                const PclLine &completeLine = objId2CompleteLines.at(objId);
                const float32_t cYaw = cos(LSM_GEOML::convDegToRad(mConfig.lineOrientationAngleError_deg*detectedLine.lineErrorAmount));
                const float32_t sYaw = sin(LSM_GEOML::convDegToRad(mConfig.lineOrientationAngleError_deg*detectedLine.lineErrorAmount));
                //calculate the end of the segment based on the line range
                const cml::Vec2Df detectedStartPoint = completeLine.startPoint + detectedLine.minRange * (completeLine.endPoint - completeLine.startPoint);
                const cml::Vec2Df detectedEndPoint = completeLine.startPoint + detectedLine.maxRange * (completeLine.endPoint - completeLine.startPoint);
                //find the coordinates for the middle of line
                const cml::Vec2Df middleOfLine = (detectedEndPoint + detectedStartPoint) / 2.0f;
                //rotate the ends of the segment based on orientation angle error amount, with the rotation center in the middle of the segment
                const cml::Vec2Df rotatedStartPoint = { middleOfLine.x() + (detectedStartPoint.x() - middleOfLine.x())*cYaw - (detectedStartPoint.y() - middleOfLine.y())*sYaw,
                                              middleOfLine.y() + (detectedStartPoint.x() - middleOfLine.x())*sYaw + (detectedStartPoint.y() - middleOfLine.y())*cYaw };
                const cml::Vec2Df rotatedEndPoint = { middleOfLine.x() + (detectedEndPoint.x() - middleOfLine.x())*cYaw - (detectedEndPoint.y() - middleOfLine.y())*sYaw,
                                            middleOfLine.y() + (detectedEndPoint.x() - middleOfLine.x())*sYaw + (detectedEndPoint.y() - middleOfLine.y())*cYaw };
                pclDelimiters.push_back(PclDelimiter{ objId, PclDelimiterType::PCL_SOLID_LINE,
                    transformToVehicleCS.inverseTransform(rotatedStartPoint),
                    transformToVehicleCS.inverseTransform(rotatedEndPoint),
                    1U /*dummy value, first sensor*/ });

            }
        }
        return pclDelimiters;
    }

    std::vector<cml::Vec2Df> sortCounterClockwise(std::vector<cml::Vec2Df> points) {

        // Calculate the center of the polygon.
        cml::Vec2Df center = cml::Vec2Df(0, 0);
        for (auto point : points) {
            center += point;
        }
        center /= static_cast<float>(points.size());

        // Subtract the center of the polygon from each point.
        for (auto& point : points) {
            point -= center;
        }

        // Sort the points using a custom comparator that compares the angles of the points with respect to the center of the polygon.
        std::sort(points.begin(), points.end(), [](cml::Vec2Df a, cml::Vec2Df b) {
            return atan2(a.y(), a.x()) < atan2(b.y(), b.x());
        });

        // Add the center of the polygon back to each point.
        for (auto& point : points) {
            point += center;
        }

        return points;
    }

    static float calculatePolygonArea(const std::vector<cml::Vec2Df>& polygon) {
        float area = 0.0f;
        for (size_t i = 0; i < polygon.size(); ++i) {
            size_t j = (i + 1) % polygon.size();
            area += polygon[i].x() * polygon[j].y() - polygon[j].x() * polygon[i].y();
        }
        area *= 0.5f;
        return std::abs(area);
    }

    // Determine the OD slots based on the traffic contours and the vehicle's pose
    std::vector<ODSlot> CemSurrogate::determineODSlots(const TrafficContour2D trafficContours[], const LSM_GEOML::Pose & vehiclePose, bool const resetODSlotMap, bool const svcAllCameraProc)
    {
        if (resetODSlotMap) {
            mODSlotMap.clear();  // at the moment, only needed when this function is called in the SVC surrogate model context
        }

        // Calculate the X and Y offsets based on the vehicle's motion
        const double offsetX = Car.ConBdy1.v_1[0] * mConfig.latencyTime_ms / 1000.0f + mConfig.staticOffsetX_m;
        const double offsetY = Car.ConBdy1.v_1[1] * mConfig.latencyTime_ms / 1000.0f + mConfig.staticOffsetY_m;

        for (unsigned int i = 0; i < (unsigned int)Traffic.nObjs; i++) {
            const tTrafficObj* trafficObj{ Traffic_GetByTrfId(i) };
            const uint32_t odSlotId{ static_cast<uint32_t>(trafficObj->Cfg.Id) };
            if ((trafficObj->Cfg.h < AP_G_MIN_HEIGHT_OBSTACLE_M) && strstr(trafficObj->Cfg.Name, "ODS") &&
                !mODSlotMap.count(odSlotId)) {  // Only if the OD slot does not already exist in the map.
                ODSlot odSlot;
                odSlot.slotId = odSlotId;
                std::fill(std::begin(odSlot.cameraId), std::end(odSlot.cameraId), 0U); // make sure all cameras have their flags set to false
                // Set the existence probability to the first value from the attributes
                odSlot.existence_probability = static_cast<uint8_t>(std::round(trafficObj->Cfg.attrib[0]));
                // Set the parking scenario confidence based on the description attributions of the attributes
                odSlot.parking_scenario_confidence.angled = static_cast<uint8_t>(std::round(trafficObj->Cfg.attrib[1]));
                odSlot.parking_scenario_confidence.parallel = static_cast<uint8_t>(std::round(trafficObj->Cfg.attrib[2]));
                odSlot.parking_scenario_confidence.perpendicular = static_cast<uint8_t>(std::round(trafficObj->Cfg.attrib[3]));
                const double cYaw = cos(trafficObj->r_zyx[2]);
                const double sYaw = sin(trafficObj->r_zyx[2]);
                const auto &contourPoints = trafficContours[i].points; // first index: x-, y-coord; second index: point index
                // Select the side based on the relative position of the traffic object and the vehicle
                const bool isSlotOnRightSide = (trafficObj->t_0[1] < vehiclePose.Pos().y());
                // Calculate the coordinates of the four corners
                if (isSlotOnRightSide) {
                    //for right Side
                    odSlot.slot_corners[0].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][2] - sYaw * contourPoints[1][2]);
                    odSlot.slot_corners[0].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][2] + cYaw * contourPoints[1][2]);

                    odSlot.slot_corners[1].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][1] - sYaw * contourPoints[1][1]);
                    odSlot.slot_corners[1].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][1] + cYaw * contourPoints[1][1]);

                    odSlot.slot_corners[2].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][0] - sYaw * contourPoints[1][0]);
                    odSlot.slot_corners[2].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][0] + cYaw * contourPoints[1][0]);

                    odSlot.slot_corners[3].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][3] - sYaw * contourPoints[1][3]);
                    odSlot.slot_corners[3].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][3] + cYaw * contourPoints[1][3]);
                }
                else {
                    //for left side
                    odSlot.slot_corners[0].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][0] - sYaw * contourPoints[1][0]);
                    odSlot.slot_corners[0].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][0] + cYaw * contourPoints[1][0]);

                    odSlot.slot_corners[1].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][3] - sYaw * contourPoints[1][3]);
                    odSlot.slot_corners[1].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][3] + cYaw * contourPoints[1][3]);

                    odSlot.slot_corners[2].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][2] - sYaw * contourPoints[1][2]);
                    odSlot.slot_corners[2].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][2] + cYaw * contourPoints[1][2]);

                    odSlot.slot_corners[3].x() = static_cast<float32_t>(trafficObj->t_0[0] + offsetX + cYaw * contourPoints[0][1] - sYaw * contourPoints[1][1]);
                    odSlot.slot_corners[3].y() = static_cast<float32_t>(trafficObj->t_0[1] + offsetY + sYaw * contourPoints[0][1] + cYaw * contourPoints[1][1]);
                }

                std::vector<cml::Vec2Df> corners;
                std::vector<LSM_GEOML::LineSegment2D> lineSegments;
                for (int j = 0; j < 4; j++) {
                    corners.push_back(odSlot.slot_corners[j]);
                    lineSegments.push_back(LSM_GEOML::LineSegment2D(odSlot.slot_corners[j], odSlot.slot_corners[(j + 1) % 4]));
                }
                const float slotArea{ calculatePolygonArea(corners) };// Calculate the area of the slot

                for (int iSens = 0; iSens < FSpaceSensorCount; ++iSens) {
                    // Skip anything other than cameras
                    if (strncmp(FSpaceSensor[iSens].bs.Name, "CAM", 3) != 0) {
                        continue;
                    }

                    // Vector to store the corner points of the slot that are contained in the FOV and also the other points that define the intersection polygon
                    std::vector<cml::Vec2Df> containedPoints;
                    for (int j = 0; j < 4; j++)
                    {
                        if (mSensorName2Fov[FSpaceSensor[iSens].bs.Name].isPointWithin(odSlot.slot_corners[j]))
                        {
                            if (std::find(containedPoints.begin(), containedPoints.end(), odSlot.slot_corners[j]) == containedPoints.end())
                            {
                                containedPoints.push_back(odSlot.slot_corners[j]);// Add the point to the vector if it is within the FOV
                            }
                        }
                    }
                    const size_t number_of_contained_points{ containedPoints.size() };
                    for (const auto& lineSegment : lineSegments) {
                        auto intersects = mSensorName2Fov[FSpaceSensor[iSens].bs.Name].intersectsFovContour(lineSegment);
                        if (intersects.first) {
                            containedPoints.push_back(intersects.second[0]);// Add the intersection point to the vector if it intersects with the FOV
                        }
                    }

                    float quadArea{ 0 };// Area of the intersection polygon
                    if (number_of_contained_points == 0)
                    {
                        quadArea = 0;// If there are no points in the intersection polygon, no overlap
                    }
                    else if (number_of_contained_points == 1)
                    {
                        const cml::Vec2Df fpoint{ containedPoints[1] + containedPoints[2] - containedPoints[0] };// Calculate the fourth point of the parallelogram based on the selected side
                        containedPoints.push_back(fpoint);
                        std::vector<cml::Vec2Df> sortedPoints = sortCounterClockwise(containedPoints);// Sort the corner points of the intersection polygon in a counter-clockwise order
                        quadArea = calculatePolygonArea(sortedPoints);
                    }
                    else if (number_of_contained_points == 2)
                    {
                        std::vector<cml::Vec2Df> sortedPoints = sortCounterClockwise(containedPoints);
                        quadArea = calculatePolygonArea(sortedPoints);
                    }
                    else if (number_of_contained_points == 3)
                    {
                        std::vector<cml::Vec2Df> sortedPoints = sortCounterClockwise(containedPoints);
                        quadArea = calculatePolygonArea(sortedPoints);
                    }
                    else {
                        quadArea = slotArea;
                    }

                    // Calculate the overlap ratio between the quadrilateral (intersection polygon) and the slot
                    const float32_t rapport{ static_cast<float32_t>(quadArea / slotArea) };

                    // If this rapport is higher than the given threshold, it means that the current camera detects the OD slot
                    if (rapport > mConfig.odSlotOverlapThreshold) {
                        if (mODSlotMap.count(odSlot.slotId) == 0U) { // ensure that we insert the OD slot in the map only once
                            mODSlotMap.insert(std::make_pair(odSlot.slotId, odSlot));
                        }

                        if (!svcAllCameraProc) {
                            break; // do not check overlap with other camera sensor FOVs
                        }

                        // if this is reached, it means that we need to mark any camera that detects the current OD Slot
                        // set the camera flag to know which side detected the OD slot; needed only in the SVC surrogate model to provide the correct input to CEM_LSM
                        if (strncmp(FSpaceSensor[iSens].bs.Name, "CAM_F", 5U) == 0) {
                            mODSlotMap.at(odSlot.slotId).cameraId[0U] = true;
                        }
                        else if (strncmp(FSpaceSensor[iSens].bs.Name, "CAM_B", 5U) == 0) {
                            mODSlotMap.at(odSlot.slotId).cameraId[1U] = true;
                        }
                        else if (strncmp(FSpaceSensor[iSens].bs.Name, "CAM_L", 5U) == 0) {
                            mODSlotMap.at(odSlot.slotId).cameraId[2U] = true;
                        }
                        else if (strncmp(FSpaceSensor[iSens].bs.Name, "CAM_R", 5U) == 0) {
                            mODSlotMap.at(odSlot.slotId).cameraId[3U] = true;
                        }
                    }
                }
            }
        }

        std::vector<ODSlot> odSlots;//empty vector to hold the OD slots transformed to vehicle coordinates
        for (auto& odSlot : mODSlotMap) {
            odSlots.push_back(odSlot.second);
        }
        const LSM_GEOML::CoordinateTransformer2D transformToVehicleCS(vehiclePose);
        //Use the coordinate transformer to transform the points to the vehicle coordinate system
        for (int i = 0; i < odSlots.size(); i++)
        {
            for (int j = 0; j < 4; j++) {
                odSlots[i].slot_corners[j] = transformToVehicleCS.inverseTransform(odSlots[i].slot_corners[j]);
            }
        }

        return odSlots;
    }

    void CemSurrogate::resetHistory() {
        mObjId2CemObjects.clear();
        mObjId2detectedLine.clear();
        mODSlotMap.clear();
    }

    void CemSurrogate::init(const CemSurrogateConfig &configuration) {
        mCycle = 0U;
        mConfig = configuration;
        mFragMergedflg = false;
        mRandomEngine.seed();
        // If smallObjectDetectionThresholdNotMainAxis_m is not set in Carmaker Minimaneuver to a specific value,
        // it means that there is no difference of the detection distance threshold when a point is on the main axis and when it is not.
        if (mConfig.smallObjectDetectionThresholdNotMainAxis_m == -1) {
            mConfig.smallObjectDetectionThresholdNotMainAxis_m = mConfig.smallObjectDetectionThreshold_m;
        }

        if (mConfig.pmdSurrogateEnabled_nu)
        {
            mPmdSurrogate.init();
        }

        mSensorName2Fov.clear();
        for (int iSens{ 0 }; iSens < FSpaceSensorCount; ++iSens) {
            // Ignore camera FSpaceSensors if cameras are disabled for the CEM surrogate model
            if (mConfig.disableCameras_nu && strncmp(FSpaceSensor[iSens].bs.Name, "CAM", 3) == 0) {
                continue;
            }
            // Consider sensor mounting height to calculate effective FOV range on the ground.
            const float32_t effectiveRange{ static_cast<float32_t>(std::sqrt(std::pow(FSpaceSensor[iSens].range, 2) - std::pow(FSpaceSensor[iSens].bs.Pos_B[2], 2))) };
            const SensorFov2D sensorFov{ getSensorPose(FSpaceSensor[iSens]), effectiveRange, static_cast<float32_t>(FSpaceSensor[iSens].alpha) };
            mSensorName2Fov[FSpaceSensor[iSens].bs.Name] = sensorFov;
        }
    }

    void CemSurrogate::registerCarMakerDVAs() {
        mPmdSurrogate.registerCarMakerDVAs();
    }

#ifdef USE_ENV_PLOTTER
    void CemSurrogate::convertToCemObjectsForPlotter(const std::vector<ConvexHull>& hulls, MF_Plot::plotterCemObjectList& cemObjects, MF_Plot::DYN_OBJ_LIST_FROM_CEM& cemDynObjects)
    {
        std::vector<ConvexHull> static_hulls;
        std::vector<ConvexHull> dyn_hulls;
        for (auto& elem : hulls) {
            if (elem.objectType == CemObjectType::DYNAMIC_OBJECT) {
                dyn_hulls.push_back(elem);
                const bool isStationaryOrStopped{ (elem.dynamicObjectProperty_nu.dynamicProperty == VCEM::DynamicProperty::STATIONARY)
                                               || (elem.dynamicObjectProperty_nu.dynamicProperty == VCEM::DynamicProperty::STOPPED) };
                if (isStationaryOrStopped){
                    static_hulls.push_back(elem);
                }
            }
            else {
                static_hulls.push_back(elem);
            }
        }
        cemObjects.setSize(static_cast<lsm_geoml::size_type>(static_hulls.size()));
        for (lsm_geoml::size_type objIdx{ 0U }; objIdx < cemObjects.getSize(); ++objIdx) {
            cemObjects[objIdx].points.clear();
            cemObjects[objIdx].id = static_hulls[objIdx].u_id;
            for (const auto& vertex : static_hulls[objIdx].polygon) {
                cemObjects[objIdx].points.append(vertex);
            }
        }
        cemDynObjects.setSize(std::min(static_cast<LSM_GEOML::size_type>(dyn_hulls.size()), static_cast<LSM_GEOML::size_type>(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_DYN_OBJECTS_NU)));
        for (LSM_GEOML::size_type objIdx{ 0U }; objIdx < cemDynObjects.getSize(); ++objIdx) {
            cemDynObjects[objIdx].shape.clear();
            cemDynObjects[objIdx].classificationClass = dyn_hulls[objIdx].dynamicObjectProperty_nu.objectClass; //mapping to the class definition of MF_Plot
            cemDynObjects[objIdx].existProb_perc = 100;
            cemDynObjects[objIdx].refPoint = dyn_hulls[objIdx].dynamicObjectProperty_nu.referencePoint;
            for (const auto& point : dyn_hulls[objIdx].dynamicObjectProperty_nu.points) {
                cemDynObjects[objIdx].shape.append(point);
            }
        }
    }

    void CemSurrogate::convertToCemODSlotsForPlotter(const std::vector<ODSlot>& odSlots, MF_Plot::OD_SLOT_LIST_FROM_CEM& cemODSlots)
    {
        cemODSlots.actualSize = 0;
        for (LSM_GEOML::size_type slotIdx{ 0U }; slotIdx < odSlots.size(); ++slotIdx)
        {
            cemODSlots.array[slotIdx].slotId_nu = static_cast<uint16_t>(odSlots[slotIdx].slotId);
            cemODSlots.array[slotIdx].slotShape_m.actualSize = 0;
            for (int i = 0; i < 4; ++i)
            {
                cemODSlots.array[slotIdx].slotShape_m.array[i].x_dir = odSlots[slotIdx].slot_corners[i].x();
                cemODSlots.array[slotIdx].slotShape_m.array[i].y_dir = odSlots[slotIdx].slot_corners[i].y();
                ++cemODSlots.array[slotIdx].slotShape_m.actualSize;
            }
            cemODSlots.array[slotIdx].existenceProb_perc = odSlots[slotIdx].existence_probability;
            cemODSlots.array[slotIdx].parkingScenarioConfidence_perc.angled_perc = odSlots[slotIdx].parking_scenario_confidence.angled;
            cemODSlots.array[slotIdx].parkingScenarioConfidence_perc.parallel_perc = odSlots[slotIdx].parking_scenario_confidence.parallel;
            cemODSlots.array[slotIdx].parkingScenarioConfidence_perc.perpendicular_perc = odSlots[slotIdx].parking_scenario_confidence.perpendicular;
            ++cemODSlots.actualSize;
        }
    }

    void CemSurrogate::convertToCemLinesForPlotter(const std::vector<PclDelimiter>& pclDelimiters, MF_Plot::plotterCemLineList& cemLines)
    {
        cemLines.setSize(static_cast<lsm_geoml::size_type>(pclDelimiters.size()));
        for (lsm_geoml::size_type idx{ 0U }; idx < cemLines.getSize(); ++idx) {
            cemLines[idx].line.clear();
            cemLines[idx].line.append(pclDelimiters[idx].startPoint);
            cemLines[idx].line.append(pclDelimiters[idx].endPoint);
            cemLines[idx].type = 0U;
            cemLines[idx].probability_perc = 100U;
        }
    }
#endif

} // namespace VCEM