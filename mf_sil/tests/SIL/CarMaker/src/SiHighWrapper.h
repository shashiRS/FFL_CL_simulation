#pragma once

#include "CemSurrogate.h"
#include "SICoordinateTransformer.h"
#include <si/coll_env_model_port.h>
#include <si/perception_availability_port.h>
#include <si/ego_motion_port.h>
#include <ap_psm/slot_ctrl_port.h>
#include <lsm_vedodo/lsm_vedodo_generated_types.h>
#include <ap_common/vehicle_params.h>
#include <mf_hmih/user_defined_slot_port.h>
#include <mf_mempark/memory_parking_status_port.h>
#include "mf_mempark/maps_to_meta_maps.h"
#include <mf_memory_parking/mf_memory_parking_interface.h>//For debug data
#include <relocalization/relocalizationModule_interface.h>//For debug data

class SiHighWrapper
{
public:
    static constexpr uint64_t SI_HIGH_CYCLE_TIME_MS = 33U;

    //!
    //! \brief      Initialize the SI algorithm with the configuration.
    //! \param[in]  vehicleParams   Parameters of the ego vehicle
    //!
    void initSiAlgorithm(const ap_common::Vehicle_Params& vehicleParams);

    //!
    //! \brief      Resets the SI algorithm.
    //!
    void resetSiAlgorithm();

    //!
    //! \brief      Main function to run the scene interpretation algorithm.
    //! \param[in]  timestamp_us            The current timestamp in microseconds
    //! \param[in]  slotControlPort
    //! \param[in]  odoEstimationPort
    //! \param[in]  convexHulls             Vector of convex polygons (obstacles) represented as vector of points.
    //! \param[in]  pclDelimiters           Vector of delimiters representing detected parking line markings.
    //! \param[in]  odSlots                 Object detection parking slots (CNN parking slots)
    //! \param[in]  userDefinedSlotPort
    //! \param[in]  targetPosesPort
    //! \param[in]  environmentModelActive  Whether CEM_LSM is part of the simulation chain
    //! \param[out] environmentModel
    //! \param[out] parkingBoxes
    //! \param[out] collisionEnvironmentModel
    //! \param[out] perceptionAvailability
    //! \param[out] egoMotion
    //! \param[out] memoryParkingStatusPort
    //!
    void runSiAlgorithm(const uint64_t &timestamp_us,
        const ap_psm::SlotCtrlPort &slotControlPort,
        const lsm_vedodo::OdoEstimationOutputPort &odoEstimationPort,
        const std::vector<VCEM::ConvexHull> &convexHulls,
        const std::vector<VCEM::PclDelimiter> &pclDelimiters,
        const std::vector<VCEM::ODSlot> &odSlots,
        const mf_hmih::UserDefinedSlotPort &userDefinedSlotPort,
        const ap_tp::TargetPosesPort &targetPosesPort,
        const ap_commonvehsigprovider::OdoGpsPort gpsPort,
        const uint8 environmentModelActive,
        si::ApEnvModelPort &environmentModel,
        si::ApParkingBoxPort &parkingBoxes,
        si::CollEnvModelPort &collisionEnvironmentModel,
        si::PerceptionAvailabilityPort &perceptionAvailability,
        si::EgoMotionPort &egoMotion,
        mf_mempark::MemoryParkingStatusPort& memoryParkingStatusPort
#if defined(USE_ENV_PLOTTER)
        , si::PlotData &plotDataPort
#endif
        );

    //!
    //! \brief      Get coordinate transformer to convert a point from SI (local) coordinates to world coordinates
    //!
    const SICoordinateTransformer& getCoordinateTransformer() {
        return mCoordinateTransformer;
    }

    //! \brief update the SI coordinate transformer by adding the vedodo offset coordinate transformation
    void addVedodoOffsetTransformation(const LSM_GEOML::CoordinateTransformer2D &vedodoOffsetTransformation)
    {
        mCoordinateTransformer.addVedodoOffsetTransformation(vedodoOffsetTransformation);
    }

    //!
    //! \brief      Workaround to get CEM vedodo data from LSM odometry. CEM vedodo data is used for latency compensation.
    //!             In the future CEM will handle it, therefore we can remove vehiclePoseEstAtCemTime from CEM output and also this function.
    //! \param[in]  egoMotionAtCemOutput            Converted egomotion for CEM output
    //! \param[in]  egoMotionData                   Egomotion data from VEDODO
    //!
    void convertLsmVedodoDataToCemEgoData(aupdf::EgoMotionAtCemOutput& egoMotionAtCemOutput, const lsm_vedodo::OdoEstimation& egoMotionData);

#if defined(USE_ENV_PLOTTER) //Visualization stuff
    relocalization::DeveloperData getRelocPlotData();
    mf_mempark::DeveloperData   getDebugData();
#endif

private:
    static void linkStaticAndDynamicObjects( aupdf::SgfOutput& sgfHulls, aupdf::DynamicEnvironment& dynamicEnvironmentInput);
    SICoordinateTransformer mCoordinateTransformer; // coordinate transformer to convert from SI coordinates to world coordinates
    aupdf::EgoMotionAtCemOutput mVehiclePoseEstAtCemTime{};
    mf_mempark::MapsToMetaMaps mMetaMapStorage; // To be used as meta map storage as long as SSF map storage not available in mf_sil
};

