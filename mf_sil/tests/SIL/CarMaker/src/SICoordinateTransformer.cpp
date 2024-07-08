#include "SICoordinateTransformer.h"
#include <lsm_geoml/lsm_geoml_generated_types.h>
#include <pod_class_converter.h>

void SICoordinateTransformer::reset()
{
    mSiToWorldTransformation = {};
    mCoordinateResetCounter = 0U;
}

void SICoordinateTransformer::updateCoordinateTransformation(const si::ResetOriginResult &resetOriginResult)
{
    if (resetOriginResult.resetCounter_nu != mCoordinateResetCounter) {
        // transformation to the new SI coordinate system
        const LSM_GEOML::CoordinateTransformer2D transformToNewCS(resetOriginResult.originTransformation);
        // Transform origin of the global (world) coordinate system from the last to the new SI coordinate system.
        mSiToWorldTransformation = transformToNewCS.inverseTransform(mSiToWorldTransformation);
        mCoordinateResetCounter = resetOriginResult.resetCounter_nu;
    }
}

void SICoordinateTransformer::addVedodoOffsetTransformation(const LSM_GEOML::CoordinateTransformer2D &vedodoOffsetTransformation)
{
    mSiToWorldTransformation = vedodoOffsetTransformation.inverseTransform(mSiToWorldTransformation);
}

void SICoordinateTransformer::transformToWorldCoordinates(si::ApEnvModelPort &envModelPort) const {
    const LSM_GEOML::CoordinateTransformer2D transformToWorldCoordinates(mSiToWorldTransformation);
    // envModelPort.dynamicObjects
    for (auto& dynOb : envModelPort.dynamicObjects) {
        if (dynOb.existenceProb_perc > 0u) {
            //TODO: dynOb.objShape.accel_mps2
            //      dynOb.objShape.vel_mps
            dynOb.headingAngle_rad = LSM_GEOML::radMod(dynOb.headingAngle_rad + mSiToWorldTransformation.Yaw_rad());

            // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
            for (lsm_geoml::size_type iPt{ 0U }; iPt < dynOb.objShape_m.actualSize; ++iPt) {
                dynOb.objShape_m.array[iPt] = convert(transformToWorldCoordinates.inverseTransform(dynOb.objShape_m.array[iPt]));
            }
        }
    }

    // envModelPort.staticObjects
    for (auto& statObj : envModelPort.staticObjects) {
        if (statObj.existenceProb_perc > 0U) {
            // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
            for (lsm_geoml::size_type iPt{ 0U }; iPt < statObj.objShape_m.actualSize; ++iPt) {
                statObj.objShape_m.array[iPt] = convert(transformToWorldCoordinates.inverseTransform(statObj.objShape_m.array[iPt]));
            }

        }
    }
#ifndef VARIANT_CUS_ONLY
    // envModelPort.parkingSpaceMarkings
    for (auto& psm : envModelPort.parkingSpaceMarkings) {
        if (psm.existenceProb_perc > 0U) {
            //TODO: psm.covMatrix

            // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
            for (lsm_geoml::size_type iPos{ 0U }; iPos < psm.pos_m.actualSize; ++iPos) {
                psm.pos_m.array[iPos] = convert(transformToWorldCoordinates.inverseTransform(psm.pos_m.array[iPos]));
            }
        }
    }
    // envModelPort.roadDescription
    for (auto& laneBoundary : envModelPort.roadDescription.laneBoundaries) {

        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        for (lsm_geoml::size_type iLS{ 0U }; iLS < laneBoundary.laneShape.actualSize; ++iLS) {
            laneBoundary.laneShape.array[iLS] = convert(transformToWorldCoordinates.inverseTransform(laneBoundary.laneShape.array[iLS]));
        }
    }
#endif
}

void SICoordinateTransformer::transformToWorldCoordinates(si::ApParkingBoxPort &parkingBoxPort) const {
    const LSM_GEOML::CoordinateTransformer2D transformToWorldCoordinates(mSiToWorldTransformation);
    for (uint8 iPB{ 0U }; iPB < parkingBoxPort.numValidParkingBoxes_nu; ++iPB) {

        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        for (lsm_geoml::size_type iCoord{ 0U }; iCoord < parkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.actualSize; ++iCoord) {
            parkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[iCoord] = convert(transformToWorldCoordinates.inverseTransform(parkingBoxPort.parkingBoxes[iPB].slotCoordinates_m.array[iCoord]));
        }

        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        for (uint8 iVL{ 0U }; iVL < parkingBoxPort.parkingBoxes[iPB].numVirtualLines_nu; ++iVL) {
            for (lsm_geoml::size_type iVert{ 0U }; iVert < parkingBoxPort.parkingBoxes[iPB].virtualLines[iVL].virtLineVertices_m.actualSize; ++iVert) {
                parkingBoxPort.parkingBoxes[iPB].virtualLines[iVL].virtLineVertices_m.array[iVert] =
                    convert(transformToWorldCoordinates.inverseTransform(parkingBoxPort.parkingBoxes[iPB].virtualLines[iVL].virtLineVertices_m.array[iVert]));
            }
        }
    }
}
