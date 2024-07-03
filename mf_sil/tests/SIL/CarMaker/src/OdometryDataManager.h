#pragma once

#include <geoml/GeomlTypes.h>                   // Pose
#include "lsm_vedodo/odo_estimation.h"     //OdoEstimationPort

class OdometryDataManager
{
    /**
    * \brief Keep the data required to handle odo overflow.
    */
    struct OdoCorrection_t
    {
        // deltaPos = (reported location) - (corrected location)
        cml::Vec2Df deltaPos{ 0.0F, 0.0F };
        cml::Vec2Df lastOdoPos{ 0.0F, 0.0F };
        LSM_GEOML::Pose absoluteOdoPose{ 0.0F, 0.0F, 0.0F };
    };

    OdoCorrection_t mOdoCorrection{};           // data to manage odometry overflow

    //!
    //! \brief      Identify overflow/underflow of the odometry position, collect the odometry correction
    //!             and calculate the absolute odometry position.
    //! \param[in]  odoPort  The current odometry data from vedodo.
    //!
    void updateAbsoluteOdometryPose(const lsm_vedodo::OdoEstimation& odoPort);

public:

    //!
    //! \brief      Register member variables for public access.
    //!
    void registerDVAVariables();

    //!
    //! \brief      Resets all member data to as-constructed values.
    //!
    void reset();

    //!
    //! Updates all odometry data and catches odometry overflow
    //!
    void update(const lsm_vedodo::OdoEstimation& odoPort);

    //!
    //! \brief      Get the absolute odometry pose that includes the correction of odometry position overflows/underflows.
    //!
    LSM_GEOML::Pose getAbsoluteOdoPose() const
    {
        return mOdoCorrection.absoluteOdoPose;
    }
};

