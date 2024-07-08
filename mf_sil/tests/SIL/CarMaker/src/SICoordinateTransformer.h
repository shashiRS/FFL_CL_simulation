#pragma once

#include "si/reset_origin_result.h"
#include "si/ap_env_model_port.h"
#include "si/ap_parking_box_port.h"
#include "geoml/GeomlTypes.h"
#include <geoml/CoordinateTransformer2D.h>

class SICoordinateTransformer
{
    LSM_GEOML::Pose mSiToWorldTransformation{}; // inverse transformation to convert from SI coordinates to world coordinates
    uint8_t mCoordinateResetCounter{ 0U };      // number of coordinate system resets

public:

    //!
    //! \brief      Get Pose to transform a point from SI (local) coordinates to world coordinates
    //!
    LSM_GEOML::Pose getSiToWorldTransformation() const {
        return mSiToWorldTransformation;
    }

    //!
    //! \brief      Reset the coordinate transformation stored in this transformer
    //!
    void reset();

    //!
    //! \brief      Update the SI coordinate transformation if the SI coordinate origin was reset
    //!
    void updateCoordinateTransformation(const si::ResetOriginResult &resetOriginResult);

    //!
    //! \brief      Update the SI coordinate transformation by adding the vedodo offset transformation
    //!
    void addVedodoOffsetTransformation(const LSM_GEOML::CoordinateTransformer2D &vedodoOffsetTransformation);

    //!
    //! \brief      Transforms the given envModelPort from SI (local) coordinates to global coordinates.
    //!
    void transformToWorldCoordinates(si::ApEnvModelPort &envModelPort) const;

    //!
    //! \brief      Transforms the given parkingBoxPort from SI (local) coordinates to global coordinates.
    //!
    void transformToWorldCoordinates(si::ApParkingBoxPort &parkingBoxPort) const;
};
