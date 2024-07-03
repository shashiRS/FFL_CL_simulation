#pragma once

#include <vector>
#include <map>
#include "TestRunWrapper.h" // TrafficContour2D
#include <utility>
#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include <Traffic.h>
#pragma warning( pop )
#include <algorithm>
#include "CemSurrogateTypes.h"
#include "PmdSurrogate.h"
#include "SensorFov2D.h"
#include "ap_common/ap_common_types_consts.h"
#include <random> //For an initial implementation of random noise
#include "MfSilTypes.h"

#ifdef USE_ENV_PLOTTER
#include "mf_plot/MF_PlotterSi.h"
#include "mf_plot/MF_PlotterCem.h"
#endif

namespace VCEM
{
#ifdef VARIANT_CUS_ONLY
    // Maximum number of points for a convex hull that is returned as output from the CEM surrogate model
    static constexpr unsigned MAX_NUM_PTS_HULL{ ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU };
#else
    // Maximum number of points for a convex hull that is returned as output from the CEM surrogate model (SiHigh typically gets up to 20 points from CEM)
    static constexpr unsigned MAX_NUM_PTS_HULL{ 20U };
#endif

    static constexpr float32_t AP_G_MIN_HEIGHT_OBSTACLE_M{ 0.02F }; // In DOORS a value of 0.04 is required. AP_G_MIN_HEIGHT_OBSTACLE_M is only used in SI Surrogate and CEM Surrogate (not in DF).
                                                                    // A value of 0.04 would lead to a lot of curb misclassifications (SO_HI_UNKNOWN instead of SO_HI_WHEEL_TRAVERSABLE) in CM use cases.
                                                                    // In order not to change the height of all curbs in CM uses cases, should be kept at 0.02

    class CemSurrogate {

    public:
        //!
        //! \brief      Detects obstacles and returns them as convex polygons.
        //! \param[in]  minObjHeight_m      Minimal height for an object to be detected.
        //! \param[in]  isScanning          Whether the scanning mode is active.
        //! \param[in]  trafficContours     Array of contours from the CarMaker traffic objects.
        //! \param[in]  cycleTime_ms        The cycle time in miliseconds with which this method is called.
        //! \param[in]  vehiclePose         The (ground-truth) ego vehicle pose in world coordinates.
        //! \returns    Convex hulls, i.e. vector of convex polygons (obstacles) represented as vector of points.
        //!
        std::vector<ConvexHull> determineObstacles(const float minObjHeight_m, const bool isScanning, const TrafficContour2D trafficContours[],
            const uint64_t cycleTime_ms, const LSM_GEOML::Pose& vehiclePose);

        //!
        //! \brief      Detects parking lines and returns them as PCL delimiters.
        //! \param[in]  isScanning           Whether the scanning mode is active.
        //! \param[in]  trafficContours      Array of contours from the CarMaker traffic objects.
        //! \param[in]  cycleTime_ms         The cycle time in miliseconds with which this method is called.
        //! \param[in]  vehiclePose          The (ground-truth) ego vehicle pose in world coordinates.
        //! \returns    PCL delimiters, i.e. vector of delimiters representing detected parking line markings.
        //!
        std::vector<PclDelimiter> determineDelimiters(const bool isScanning, const TrafficContour2D trafficContours[],
            const uint64_t cycleTime_ms, const LSM_GEOML::Pose& vehiclePose);

        //!
        //! \brief      Detects the object-detection parking slots (OD slots) from the traffic object contours
        //! \param[in]  trafficContours      Array of contours from the CarMaker traffic objects.
        //! \param[in]  vehiclePose          The (ground-truth) ego vehicle pose in world coordinates.
        //! \returns    OD slots, i.e. vector of ODSlots representing the parking slots detected by CEM based on CV inputs.
        //!
        std::vector<ODSlot> determineODSlots(const TrafficContour2D trafficContours[], const LSM_GEOML::Pose& vehiclePose, bool const resetODSlotMap = false, bool const svcAllCameraProc = false);

        //!
        //! \brief      Resets the history of detected objects.
        //!
        void resetHistory();

        //!
        //! \brief      Sets the configuration of this CEM surrogate model.
        //!
        void init(const CemSurrogateConfig &configuration);

        void registerCarMakerDVAs();

#ifdef USE_ENV_PLOTTER
        //!
        //! \brief      Converts the ConvexHulls to a plotterCemObjectList.
        //!
        static void convertToCemObjectsForPlotter(const std::vector<ConvexHull>& hulls, MF_Plot::plotterCemObjectList& cemObjects, MF_Plot::DYN_OBJ_LIST_FROM_CEM& cemDynObjects);

        //!
        //! \brief      Converts the ODSlots to a OD-slot list that can be consumed by MF_Plotter.
        //!
        static void convertToCemODSlotsForPlotter(const std::vector<ODSlot>& odSlots, MF_Plot::OD_SLOT_LIST_FROM_CEM& cemODSlots);

        //!
        //! \brief      Converts the PclDelimiters to a plotterCemLineList.
        //!
        static void convertToCemLinesForPlotter(const std::vector<PclDelimiter>& pclDelimiters, MF_Plot::plotterCemLineList& cemLines);
#endif

    private:
        static std::vector<DetectedPoint> convex_hull(std::vector<DetectedPoint> &points);
        static void addOppositePointsTo(std::vector<DetectedPoint> &pointsPerObj);
        static float32_t CemSurrogate::calcTriangleArea(const std::vector<DetectedPoint>& convexHull, const size_t index, const size_t prevInd, const size_t nextInd);
        static void reduceNumberOfPoints(std::vector<DetectedPoint> &convexHull, std::vector<DetectedPoint> &convexHullInternal);
        bool hasDisappeared(CemObject& cemObject, const uint64_t cycleTime_ms);
        ConvexHull createConvexHull(const std::vector<DetectedPoint> &convexHullPoints, const unsigned objId, const CemObject& cemObject,
            const LSM_GEOML::Pose& vehiclePose, const uint64_t cycleTime_ms);
        void saveConvexHulls(); //TODO delete this debug function
        std::pair<bool, unsigned int> isOnSegment(const tTrafficObj &trafficObj, const DetectedPoint detectedPoint, const TrafficContour2D trafficContours[]);

        uint64_t mCycle{ 0U };              // number of cycles the CEM surrogate model is executed
        CemSurrogateConfig mConfig;         // configuration of this CEM surrogate model
        bool mFragMergedflg{ false };       // flag of fragment merged, after fragment merged, the flag is set true
        PmdSurrogate mPmdSurrogate{};       // surrogate model for Park Marker Detection
        // map: sensor name -> sensor field of view
        std::map<std::string, SensorFov2D> mSensorName2Fov;
        // map: object-ID -> Cem Object (consisting of detected points and further properties)
        std::map<unsigned int, CemObject> mObjId2CemObjects;
        // map: object-ID -> DetectedLine(minimum relative line position, maximum relative line position,lineErrorAmount)
        std::map<unsigned int, DetectedLine> mObjId2detectedLine;
        // map: ODSlot-ID -> ODSlot (consisting of slot corners, existence probability and parking-scenario confidences)
        std::map<uint32_t, ODSlot> mODSlotMap;

        std::mt19937 mRandomEngine{}; // explicitly use mt19937 instead of default_random_engine, since the latter is implementation specific
    };

} // namespace VCEM
