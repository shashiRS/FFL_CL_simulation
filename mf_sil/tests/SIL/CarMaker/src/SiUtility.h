#pragma once

#include "si/si_generated_types.h"            // SI::CollEnvModelPort
#include "ap_common/vehicle_params.h"         // ap_common::Vehicle_Params
#include <si/si_params.h>                     // si::SiParams
#include <geoml/Polygon2D.h>                  // LSM_GEOML::Polygon2D
#include "ap_common/ap_common_types_consts.h" // ap_common::AP_COMMON_TYPES_Consts

#ifndef VARIANT_CUS_ONLY
// mf_memory_parking currently not used in Entry
#include "mf_mempark/mem_park_params.h"
#else
#include "us_em/us_env_model_port.h"
#endif

class SiUtility
{
public:
    struct DelimiterZones
    {
        LSM_GEOML::Polygon2D<4U> curbsideZone;
        LSM_GEOML::Polygon2D<4U> roadsideZone;
        LSM_GEOML::Polygon2D<4U> leftsideZone;
        LSM_GEOML::Polygon2D<4U> rightsideZone;
        LSM_GEOML::Polygon2D<4U> insideZone;
    };

    //! \brief      Get configuration parameters for the scene interpretation component
    //! \param[in]  reload   true: read parameters in again; false: use cached parameter values
    static const si::SiParams& getSiParameters(const bool reload = false);


#ifndef VARIANT_CUS_ONLY
    // mf_memory_parking currently not used in Entry
    //! \brief      Get configuration parameters for the memory parking sub-component
    //! \param[in]  reload   true: read parameters in again; false: use cached parameter values
    static const mf_mempark::MemParkParams& getMemParkParameters(const bool reload = false);
#else

    // sorts the static objects according to their distance to the ego vehicle coordinates origin
    void sortStaticObjectsByDistance(us_em::UsEnvModelPort &collEnvModelPort);

    // Determines and sets the index of the first static/dynamic object outside of the detection zone.
    // Precondition: The static objects need to be sorted according to their distance before this function can be called.
    void setFirstObjOutDetZoneIdx(us_em::UsEnvModelPort &collEnvModelPort);
#endif

    // @details return the singleton instance of the SiUtility
    static SiUtility& getInstance();
    SiUtility(SiUtility const&) = delete; // prevent copying the singleton
    SiUtility& operator= (SiUtility const&) = delete;

    //!
    //! \brief      Initializes the SiUtility with the ego vehicle parametrization
    //! \param[in]  vehicleParams   Parameters of the ego vehicle
    //!
    void init(const ap_common::Vehicle_Params &vehicleParams);

    //!
    //! \brief      Resets all members of the SiUtility.
    //!
    void reset();

    //!
    //! \brief      Registers relevant members of SiUtility for direct variable access.
    //!
    void registerDVAVariables();

    //!
    //! \brief      Update the delimiter zones for the specified parking box.
    //! \param[in]  pbIdx               index of the parking box
    //! \param[in]  delimiterZones      new delimiter zones for the parking box
    //!
    void updateDelimiterZones(const uint8_t pbIdx, const DelimiterZones& delimiterZones);

    //!
    //! \brief      Update the delimiter zones for all parking boxes.
    //! \param[in]  delimiterZones      new delimiter zones from SI for all parking boxes
    //! \param[in]  vehiclePose         ego vehicle pose
    //!
    void updateDelimiterZones(const si::DelimiterZonesSerializable& delimiterZones,
        const LSM_GEOML::Pose& vehiclePose);

    // sorts the static objects according to their distance to the ego vehicle coordinates origin
    void sortStaticObjectsByDistance(si::CollEnvModelPort &collEnvModelPort);

    // Determines and sets the index of the first static/dynamic object outside of the detection zone.
    // Precondition: The static objects need to be sorted according to their distance before this function can be called.
    void setFirstObjOutDetZoneIdx(si::CollEnvModelPort &collEnvModelPort);

private:
    // standard ego vehicle shape with center of rear axle at origin
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU> mEgoVehicleShape_m{};
    LSM_GEOML::Polygon2D<4U> mDetectionZone_m{};  // detection zone for LSCA
    // At least 8 delimiter zones are required for DVA registration so that AUP_Visualization.tclgeo doesn't fail to subscribe to it.
    static constexpr uint8_t maxNumDelimiterZones = std::max(((uint8_t)8U), (uint8_t)ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU);
    std::array<DelimiterZones, maxNumDelimiterZones> mDelimiterZones{};

    SiUtility() = default;  // prevent creation of another instance outside of this class
};
