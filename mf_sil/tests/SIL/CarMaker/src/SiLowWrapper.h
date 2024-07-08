#pragma once

#include <si_core/SiInterface.h>
#include "si/coll_env_model_port.h"
#include "CemSurrogateTypes.h"
#include "lsm_vedodo/odo_estimation.h"
#include "SICoordinateTransformer.h"
#include "si/plot_data.h"
#include "com/com_generated_types.h"
#include "us_em/us_em_generated_types.h"
#include "lsm_vedodo/odo_estimation_output_port.h"
#include "si/ego_motion_port.h"
namespace si {

    static constexpr uint64_t SI_LOW_CYCLE_TIME_MS = 40U;

    class SiLowWrapper
    {
    public:
        // @details return the instance of the SiLowWrapper; singleton since SiInterface is a singleton too
        static SiLowWrapper& getInstance();
        SiLowWrapper(SiLowWrapper const&) = delete; // prevent copying the singleton
        SiLowWrapper& operator= (SiLowWrapper const&) = delete;

        //!
        //! \brief      Initialize the SI algorithm with the configuration.
        //! \param[in]  vehicleParams   Parameters of the ego vehicle
        //!
        void init(const ap_common::Vehicle_Params &vehicleParams);

        //!
        //! \brief      Resets the SI algorithm.
        //!
        void reset();

        //!
        //! \brief      Main function to run the SI algorithm.
        //! \param[in]  odoEstimationPort       The OdoEstimationPort from mf_vedodo
        //! \param[in]  slotCtrlPort
        //! \param[in]  collEnvModelPort        The CollEnvModelPort from us_em
        //! \param[in/out]  percAvailPort           The PerceptionAvailabilityPort from us_em
        //! \returns    SiOutput and empty string if successful (string contains error message if not successful)
        //!
        std::pair<SiOutput, std::string> run(
            const lsm_vedodo::OdoEstimationOutputPort &odoEstimationPort,
            const ap_psm::SlotCtrlPort &slotCtrlPort,
            const us_em::UsEnvModelPort& usEnvModelPort,
            const us_em::PerceptionAvailabilityPort& inputPercAvailPort,
            si::PerceptionAvailabilityPort& outputPercAvailPort,
            si::EgoMotionPort& egoMotionPort,
            si::CollEnvModelPort& collEnvModelPort
#if defined(USE_ENV_PLOTTER)
            , si::PlotData &plotDataPort
#endif
        );

        static void fillEgoMotionOutput(const lsm_vedodo::OdoEstimationOutputPort& odoInput, si::EgoMotionPort& egoMotionPort);

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
        //! \brief      Convert ConvexHulls from CemSurrogate to UsEnvModelPort (us_em output to SiLow)
        //! \param[in]  timestamp_us            The current timestamp in microseconds
        //! \param[in]  convexHulls             Vector of convex polygons (obstacles) represented as vector of points.
        //! \param[out] usEnvModelPort          The UsEnvModelPort to update with ConvexHull data.
        //!
        static void convertToUsEnvModelPort(const uint64_t& timestamp_us, const std::vector<VCEM::ConvexHull>& convexHulls,
            us_em::UsEnvModelPort& usEnvModelPort);

       //!
       //! \brief      Update the PerceptionAvailabilityPort (US-sensor and EnvModel status)
       //! \param[out] percAvailPort            The PerceptionAvailabilityPort to update
       //!
       static void update(us_em::PerceptionAvailabilityPort& percAvailPort);

    private:
        SiLowWrapper() = default;  // prevent creation of another instance outside of this class

        SiInput mSiInput{};                       // Data structure used as input for the SI algorithm
        SiOutput mSiOutput{};                     // Data structure used as output for the SI algorithm
        si::ApParkingBoxPort mParkBoxData{};      //  - parking Box output
        si::ApEnvModelPort mPsiEnvModelPort{};    //  - environment model output
        SICoordinateTransformer mCoordinateTransformer{}; // coordinate transformer to convert from SI coordinates to world coordinates
    };

}

