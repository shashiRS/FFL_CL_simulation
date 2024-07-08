#pragma once

#include "MfSilTypes.h"
#include <vector>
#include <geoml/lsm_cml_stl_point.h>  // cml::Vec2Df
#ifdef VARIANT_PERFORMANCE
#include "si/static_object_class.h"
#endif
#include "si/static_obj_heigth_type.h"
#include <geoml/CoordinateTransformer2D.h>
namespace VCEM
{
    constexpr int32_t DYN_OBJECT_NUM_SHAPE_POINTS = 4;
    constexpr int32_t MAX_NUM_SENSORS = 6;

    enum DynObjClass : uint8_t {
        CLASS_UNKNOWN = 0U,
        CAR = 1U,
        PEDESTRIAN = 2U,
        MOTORCYCLE = 3U,
        BICYCLE = 4U
    };

    enum DynamicProperty : uint8_t {
        DYN_PROP_UNKNOWN = 0U,
        MOVING = 1U,
        STATIONARY = 2U,
        STOPPED = 3U
    };

    enum MeasurementObjState : uint8_t {
        MEASURED = 0U,
        PREDICTED = 1U,
        DELETED = 2U,
        INVALID = 3U
    };

    enum CemObjectType { STATIC_OBJECT, DYNAMIC_OBJECT, NOT_SET };

    struct DynamicObjectProperty {
        DynObjClass objectClass;                         // UNKNOWN, CAR, PEDESTRIAN, MOTORCYCLE, BICYCLE
        DynamicProperty dynamicProperty;                 // UNKNOWN, MOVING, STATIONARY, STOPPED
        uint32_t containedInLastSensorUpdate;            // bitfield; USS, CAM, LRR, SRR, SUR, LID
        uint16_t lastSensorUpdate[MAX_NUM_SENSORS];      // time since last sensor update, millis (no cycles)
        uint32_t lifetime;                               // object age in millis (no cycles)
        cml::Vec2Df points[DYN_OBJECT_NUM_SHAPE_POINTS]; // four points of the rectangle
        cml::Vec2Df referencePoint;                      // Relative to ego rear axle middle, all kinematics relative to this point
        MeasurementObjState state{ INVALID };            // MEASURED, PREDICTED, DELETED, INVALID
        float32_t orientation;                           // relative to ego's orientation, rad, [0; 2Pi)
        cml::Vec2Df velocity;                            // x, y, absolute, relative to ego rear axle middle, relative to target middle
        cml::Vec2Df acceleration;                        // x, y, absolute, relative to ego rear axle middle
        float32_t yawRate;                               // rad/s
    };

    struct ConvexHull {
        std::vector<cml::Vec2Df> polygon;             // polygon points of the convex hull
        uint32_t u_id;                                // id of the convex hull
        si::StaticObjHeigthType objHeightClass_nu;    // height class of the object
        uint16_t objAgeInCycles_nu;                   // number of CEM surrogate cycles after the first detection of the object
        CemObjectType objectType{ CemObjectType::NOT_SET }; // Type of the object, i.e. static or dynamic
#ifdef VARIANT_PERFORMANCE
        si::StaticObjectClass classification;         // Classification information of the object
#endif
        DynamicObjectProperty dynamicObjectProperty_nu;         // Dynamic Objects properties
    };

    // sole delimiter type used here. For further delimiter types see cem_DelimiterType_t in <cem_dbm_adapter_if/cem_iodata_pod.h>
    enum PclDelimiterType { PCL_SOLID_LINE };

    struct PclDelimiter {
        uint64_t u_id;                          // unique id of the delimiter
        PclDelimiterType type;                  // type of the delimiter
        cml::Vec2Df startPoint{ 0.0F, 0.0F };   // starting point of the delimiter
        cml::Vec2Df endPoint{ 0.0F, 0.0F };     // end point of the delimiter
        uint64_t contributingSensors;           // list of sensors contributing to fusion
    };

    struct ParkingScenarioConfidence
    {
        uint8_t angled{ 0U };           // 0-100 in % //
        uint8_t parallel{ 0U };         // 0-100 in % //
        uint8_t perpendicular{ 0U };    // 0-100 in % //
    };

    //enum SlotDelimiterType {
    //    UNKNOWN_TYPE_DELIMITER = 0U,
    //    VIRTUAL_DELIMITER = 1U,
    //    PARKING_MARKING_DELIMITER = 2U,
    //    CURBSTONE_DELIMITER = 3U,
    //    STATIC_OBJECT_DELIMITER = 4U,
    //    PARKING_CAR_DELIMITER = 5U
    //};

    struct ODSlot{
        uint32_t                    slotId;
        bool                        cameraId[4U];
        cml::Vec2Df                 slot_corners[4U];
        //SlotDelimiterType           delimiter_type[4U];
        //bool                        corner_occlusion_state[4U]; //TO BE FILLED
        uint8_t                     existence_probability; // 0-100 in % //
        ParkingScenarioConfidence   parking_scenario_confidence;
    };

    struct CemSurrogateConfig {
        float32_t latencyTime_ms = 0.0f;                       // sensor latency time in miliseconds
        float32_t lineOrientationAngleError_deg = 0.0f;        // line orientation angle error added for parking delimiters  
        float32_t lineOrientationCorrectionTimeSpan_s = 0.0f;  // timespan in which the line orientation angle error must be corrected
        float32_t lineOrientationCorrectionDistance_m = 2.0f;  // the distance from detected point to sensor below which the angle error correction must start
        float32_t staticOffsetX_m = 0.0f;                      // static longitudinal offset in meters
        float32_t staticOffsetY_m = 0.0f;                      // static lateral offset in meters
        float32_t cornerDetectionError_m = 0.0f;               // offset added to corner points of objects in the normal direction
        float32_t flankKinkWidth_m = 0.2f;                     // width in normal(lateral) direction of flank kinks used to model erroneous US lines
        // Distance below which a step/kink in the vehicle flank (due to erroneous US lines) will be corrected.
        float32_t flankKinkCorrectionDistance_m = 0.8f;
        //Timespan in which a step/kink in the vehicle flank(due to erroneous US lines) will be corrected.
        float32_t flankKinkCorrectionTimeSpan_s = 0.0f;
        unsigned char smallObjectDetectionDelay_nu = 0; // switch to turn on/off small object detection delay effect

        // threshold for detection-distance depending on object size if smallObjectDetectionThresholdNotMainAxis_m is not set in Minimaneuver
        // else, threshold for detection-distance depending on object size if the detected point is on the main axis
        float32_t smallObjectDetectionThreshold_m = 1.0f;
        float32_t smallObjectDetectionThresholdNotMainAxis_m = -1.0f;   // threshold for detection-distance depending on object size if the detected point is not on the main axis
        // threshold for the distance between traffic object and sensor, above which the object is misclassified
        float32_t misclassificationDistanceThreshold_m = 2.0f;
        float32_t garageDetectionDistance_m = -1.0f;                    // garage-parking-specific detection threshold
        float32_t cameraUncertainty_m = 0.0f;           // camera uncertainty error for the detected point when is further than CAMERA_UNCERTAINTY_DISTANCE_THRESHOLD_M from the camera and is on the edge of the sensor FOV
        float32_t fragmentObjectLength = 0.0f;          // length of the object fragment
        float32_t fragmentGapLength = 0.0f;             // gap length between the fragments
        float32_t fragmentMergeTime_s = -1.0f;          // time threshold to start merging fragments
        uint8_t pmdSurrogateEnabled_nu = 0U;            // enabled/disable the PMD Surrogate algorithm
        uint8_t disableCameras_nu = 0U;                 // disable all camera sensors
        uint8_t enableDynamicObjectMovement_nu{ 0U };   // enable movement of the convex hull for dynamic objects (disables hull trail for moving objects)
        float32_t disappearedObjectRemovalTime_s{ 0.0f }; // Accumulated time over which an object in the sensor FOVs needs not to be detected before it is removed from the object list.
        float32_t odSlotOverlapThreshold{ 0.4f };        // threshold for the overlap of an OD slot with a camera FOV, above which an OD slot will be detected (default 40%)
#ifdef VARIANT_CUS_ONLY
        float32_t objectThicknessDuringScanning_m{ 0.3F }; // Limits the thickness of detected objects during scanning if the parameter is set to a positive value. For variant Entry this parameter is set to a default value which acitvates the feature always.
#else
        float32_t objectThicknessDuringScanning_m{ -1.0F }; // Limits the thickness of detected objects during scanning if the parameter is set to a positive value.
#endif
    };

    enum SensorType { USS, CAM, UNKNOWN };

    struct DetectedPoint {
        cml::Vec2Df positionGroundTruth_m{ 0.0F, 0.0F };        // ground-truth position of the detected point
        cml::Vec2Df normalVector_m{ 0.0F, 0.0F };               // outwards-pointing normal vector of the detected point
        cml::Vec2Df offsetCornerDetectionError_m{ 0.0F, 0.0F }; // position offset due to errorneous detection of object corners
        cml::Vec2Df offsetFlankKinkError_m{ 0.0F, 0.0F };       // position offset due to kink in the flank of an object (often caused by errorneous ultrasound line)
        float32_t flankKinkErrorAmount = 0.0f;                  //
        cml::Vec2Df offsetCameraUncertainty_m{ 0.0F, 0.0F };    // position offset due to inaccurate detection of a point by a camera
        cml::Vec2Df offsetLatency_m{ 0.0F, 0.0F };              // position offset derived from latency time
        cml::Vec2Df perturbedPosition_m{ 0.0F, 0.0F };          // position of the detected point with all offset effects added
        SensorType detectingSensor{ UNKNOWN };      // type of sensor that detected this point
        double distanceToSensor_m;                  // distance of the detected point to the sensor at detection time
        double azimuthalAngleToSensor_rad;          // azimuthal bearing angle of detected point to the main axis of the sensor at detection time
        bool isInLocalCoordinates{ false };         // whether this point is given in local coordinates

        DetectedPoint(const cml::Vec2Df& detectedPositon, const SensorType sensorType, const double distToSensor, const double azimAngleToSensor)
            : positionGroundTruth_m{ detectedPositon }, perturbedPosition_m{ detectedPositon }, detectingSensor{ sensorType },
            distanceToSensor_m{ distToSensor }, azimuthalAngleToSensor_rad{ azimAngleToSensor }{}
        DetectedPoint() = default;

        cml::Vec2Df getPosition() const { return perturbedPosition_m; }

        // Transforms this detected point from global to local coordinates.
        void transformToLocalCoordinates(const LSM_GEOML::CoordinateTransformer2D& localCoordinateTransformer) {
            isInLocalCoordinates = true;
            positionGroundTruth_m = localCoordinateTransformer.inverseTransform(positionGroundTruth_m);
            normalVector_m = localCoordinateTransformer.rotate(normalVector_m, -localCoordinateTransformer.getRefOrientation());
            offsetCornerDetectionError_m = localCoordinateTransformer.rotate(offsetCornerDetectionError_m, -localCoordinateTransformer.getRefOrientation());
            offsetFlankKinkError_m = localCoordinateTransformer.rotate(offsetFlankKinkError_m, -localCoordinateTransformer.getRefOrientation());
            offsetCameraUncertainty_m = localCoordinateTransformer.rotate(offsetCameraUncertainty_m, -localCoordinateTransformer.getRefOrientation());
        }

        // Updates the perturbed position by adding all offsets to the ground-truth position and transforming it to global coordinates.
        void updatePosition(const LSM_GEOML::CoordinateTransformer2D& localCoordinateTransformer) {
            perturbedPosition_m = positionGroundTruth_m + offsetCornerDetectionError_m + flankKinkErrorAmount * offsetFlankKinkError_m
                + offsetCameraUncertainty_m;
            if (isInLocalCoordinates)
            {
                // Transform the point back to the global coordinate system.
                perturbedPosition_m = localCoordinateTransformer.transform(perturbedPosition_m);
            }
            // Do not transform the offsetLatency_m because it is related to the ego vehicle motion and not to the traffic object movement.
            perturbedPosition_m += offsetLatency_m;
        }
    };

    struct CemObject
    {
        CemObjectType objectType{ CemObjectType::NOT_SET }; // Type of the object, i.e. static or dynamic
        std::vector<DetectedPoint> detectedPoints{};        // Convex hull (consisting of detected points)
        uint64_t firstDetectionCycle{ 0U };                 // The cycle number where this object was detected first.
        double lastDetectionTime_s{ 0.0F };                 // Simulation time where this object was detected last
        unsigned detectionMissesCount{ 0U };                // Count how many cycles this object was not detected even though it was presumably in one sensor FOV.
        double minDistanceToSensor_m{ -1.0F };              // Minimum distance between the traffic object and ego vehicle in meters; used for misclassification
        bool flankKinkCorrectionStarted{ false };           // Whether the flank kink error was already corrected for this object.
        //DynamicObjectProperty dynamicProperty_nu;               // Dynamic Objects properties
    };

    struct DetectedLine {
        float32_t minRange = 0.0F;         //minimum relative position
        float32_t maxRange = 0.0F;         //maximum relative position
        float32_t lineErrorAmount = 0.0F; //amount of line orientation angle error
    };

} // namespace VCEM