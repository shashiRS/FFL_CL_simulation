/*****************************************************************************
Description   : Visualization of CEM_LSM outputs (Subscription and deliver to
                IPG Control and MF_Plotter)
Created by    : Zhizhe02 Jia
Co-Author     : Hannes Brauckmann
Company       : Continental Autonomous Mobility Germany GmbH
*****************************************************************************/

#include "string.h"
#include <cstdio>
#include "DataDict.h"
#include "CEM_disp.hpp"

#ifdef USE_ENV_PLOTTER
void CEMSubscribeStaticObjects2Plotter(const aupdf::SgfOutput *sgfOutput_sub, MF_Plot::plotterCemObjectList& cemObjects) {
    cemObjects.clear();
    cemObjects.setSize(sgfOutput_sub->staticObjectsOutput.numberOfObjects);
    for (LSM_GEOML::size_type obj_idx{ 0U }; obj_idx < cemObjects.getSize(); obj_idx++) {
        const aupdf::StaticObject &staticObject = sgfOutput_sub->staticObjectsOutput.objects[obj_idx];
        cemObjects[obj_idx].points.clear();
        cemObjects[obj_idx].id = staticObject.obstacleId;
        for (LSM_GEOML::size_type ver_idx{ 0U }; ver_idx < staticObject.usedVertices; ver_idx++) {
            const LSM_GEOML::size_type vertice_i = staticObject.vertexStartIndex + ver_idx;
            const aupdf::StaticObjectVertex &objectVertex = sgfOutput_sub->staticObjectVerticesOutput.vertices[vertice_i];
            cemObjects[obj_idx].points.append(cml::Vec2Df(objectVertex.x, objectVertex.y));
        }
    }
}

void CEMSubscribeLines2Plotter(const aupdf::PclOutput *pclOutput_disp, MF_Plot::plotterCemLineList& cemLines) {
    cemLines.setSize(static_cast<LSM_GEOML::size_type>(pclOutput_disp->numberOfDelimiters));
    for (LSM_GEOML::size_type idx{ 0U }; idx < cemLines.getSize(); idx++) {
        const aupdf::Delimiter &delimiter_cem = pclOutput_disp->delimiters[idx];
        cemLines[idx].line.clear();
        const cml::Vec2Df start_point{ delimiter_cem.startPointXPosition, delimiter_cem.startPointYPosition };
        const cml::Vec2Df end_point{ delimiter_cem.endPointXPosition, delimiter_cem.endPointYPosition };
        cemLines[idx].line.append(start_point);
        cemLines[idx].line.append(end_point);
        // CEM_DelimiterType_t == 7U -> wheel_stopper (1U for MF_Plot:LineData)
        //cemLines[idx].type = (delimiter_cem.delimiterType == aupdf::DelimiterType::WHEEL_STOPPER_DELIMITER) ? 1U : 0U; //0 -> line, 1 -> WS
        cemLines[idx].probability_perc = static_cast<uint8_t>(std::round(delimiter_cem.confidence * 100.f));
    }
}

void CEMSubscribeDynObj2Plotter(const aupdf::DynamicEnvironment *dyn_evn_sub, MF_Plot::DYN_OBJ_LIST_FROM_CEM& cemDynObjects) {
    cemDynObjects.setSize(std::min(static_cast<LSM_GEOML::size_type>(dyn_evn_sub->numberOfObjects), static_cast<LSM_GEOML::size_type>(ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_DYN_OBJECTS_NU)));
    for (LSM_GEOML::size_type idx{ 0U }; idx < cemDynObjects.getSize(); idx++) {
        cemDynObjects[idx].shape.clear();
        for (const auto& point : dyn_evn_sub->objects[idx].shapePoints.points) {
            cemDynObjects[idx].shape.append({ point.position.x, point.position.y });
        }
        cemDynObjects[idx].refPoint = { dyn_evn_sub->objects[idx].shapePoints.referencePoint.x, dyn_evn_sub->objects[idx].shapePoints.referencePoint.y };
        cemDynObjects[idx].existProb_perc = static_cast<uint8_t>(std::round(dyn_evn_sub->objects[idx].existenceCertainty * 100.f));
        cemDynObjects[idx].classificationClass = dyn_evn_sub->objects[idx].objectClass;
    }
}

void CEMSubscribeODSlots2Plotter(const aupdf::ParkingSlotDetectionOutput *parking_slot_detection, MF_Plot::OD_SLOT_LIST_FROM_CEM & cemODSlots) {
    const uint8 maxODSlotsForPlotter{ (sizeof(cemODSlots.array) / sizeof(cemODSlots.array[0])) };
    cemODSlots.actualSize = std::min(parking_slot_detection->numberOfSlots, maxODSlotsForPlotter);
    for (lsm_geoml::size_type slotIdx{ 0U }; slotIdx < cemODSlots.actualSize; ++slotIdx)
    {
        const aupdf::ParkingSlot& cemParkingSlot{ parking_slot_detection->parkingSlots[slotIdx] };
        cemODSlots.array[slotIdx].slotId_nu = static_cast<uint16_t>(cemParkingSlot.slotId);
        cemODSlots.array[slotIdx].slotShape_m.actualSize = 4U;
        for (int i = 0; i < 4; ++i)
        {
            cemODSlots.array[slotIdx].slotShape_m.array[i].x_dir = cemParkingSlot.slotCorners[i].x;
            cemODSlots.array[slotIdx].slotShape_m.array[i].y_dir = cemParkingSlot.slotCorners[i].y;
        }
        cemODSlots.array[slotIdx].existenceProb_perc = cemParkingSlot.existenceProbability;
        cemODSlots.array[slotIdx].parkingScenarioConfidence_perc.angled_perc = cemParkingSlot.parkingScenarioConfidence.angled;
        cemODSlots.array[slotIdx].parkingScenarioConfidence_perc.parallel_perc = cemParkingSlot.parkingScenarioConfidence.parallel;
        cemODSlots.array[slotIdx].parkingScenarioConfidence_perc.perpendicular_perc = cemParkingSlot.parkingScenarioConfidence.perpendicular;
    }
}
#endif

void registerCemOutputsAsDvaVariables(
    aupdf::DynamicEnvironment *dynamicEnvironment,
    aupdf::EgoMotionAtCemOutput *egoMotionAtCemOutput,
    aupdf::ParkingSlotDetectionOutput *parkingSlotDetectionOutput,
    aupdf::PclOutput *pclOutput,
    aupdf::SgfOutput *sgfOutput,
    aupdf::StopLineOutput *stopLineOutput,
    aupdf::PedestrianCrossingOutput *pedestrianCrossingOutput) {

    char VarName[255];
    uint32_t i[2];

    //CEM_SefOutput
    DDefUInt(NULL, "sgfOutput.uiVersionNumber", "", &sgfOutput->uiVersionNumber, DVA_None); /* uint32 */
    DDefULLong(NULL, "sgfOutput.sSigHeader.uiTimeStamp", "us", &sgfOutput->sSigHeader.uiTimeStamp, DVA_None); /* uint64 */
    DDefUShort(NULL, "sgfOutput.sSigHeader.uiMeasurementCounter", "", &sgfOutput->sSigHeader.uiMeasurementCounter, DVA_None); /* uint16 */
    DDefUShort(NULL, "sgfOutput.sSigHeader.uiCycleCounter", "", &sgfOutput->sSigHeader.uiCycleCounter, DVA_None); /* uint16 */
    DDefUChar(NULL, "sgfOutput.sSigHeader.eSigStatus", "", (unsigned char*)&sgfOutput->sSigHeader.eSigStatus, DVA_None); /* uint8 */
    DDefUChar(NULL, "sgfOutput.staticObjectsOutput.numberOfObjects", "", &sgfOutput->staticObjectsOutput.numberOfObjects, DVA_None); /* uint8 */
    for (i[0] = 0; i[0] < aupdf::Constants::SGF_MAX_OBJECTS; i[0]++) {
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.obstacleId", i[0]);
        DDefUInt(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].obstacleId, DVA_None);/* uint32 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.associatedDynamicObjectId", i[0]);
        DDefUInt(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].associatedDynamicObjectId, DVA_None);/* uint32 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.vertexStartIndex", i[0]);
        DDefUShort(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].vertexStartIndex, DVA_None); /* uint16 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.usedVertices", i[0]);
        DDefUChar(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].usedVertices, DVA_None); /* uint8 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.heightConfidences.wheelTraversable", i[0]);
        DDefFloat(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].heightConfidences.wheelTraversable, DVA_None); /* float32 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.heightConfidences.bodyTraversable", i[0]);
        DDefFloat(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].heightConfidences.bodyTraversable, DVA_None); /* float32 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.heightConfidences.high", i[0]);
        DDefFloat(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].heightConfidences.high, DVA_None); /* float32 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.heightConfidences.hanging", i[0]);
        DDefFloat(NULL, VarName, "", &sgfOutput->staticObjectsOutput.objects[i[0]].heightConfidences.hanging, DVA_None); /* float32 */
        std::sprintf(VarName, "sgfOutput.staticObjectsOutput.objects._%d_.semanticType", i[0]);
        DDefUChar(NULL, VarName, "", (unsigned char*)&sgfOutput->staticObjectsOutput.objects[i[0]].semanticType, DVA_None); /* uint8 */
    }
    DDefUShort(NULL, "sgfOutput.staticObjectVerticesOutput.numberOfVertices", "", &sgfOutput->staticObjectVerticesOutput.numberOfVertices, DVA_None); /* uint16 */
    for (i[0] = 0; i[0] < aupdf::Constants::SGF_MAX_TOTAL_VERTICES; i[0]++) {
        std::sprintf(VarName, "sgfOutput.staticObjectVerticesOutput.vertices._%d_.x", i[0]);
        DDefFloat(NULL, VarName, "m", &sgfOutput->staticObjectVerticesOutput.vertices[i[0]].x, DVA_None);/* float32 */
        std::sprintf(VarName, "sgfOutput.staticObjectVerticesOutput.vertices._%d_.y", i[0]);
        DDefFloat(NULL, VarName, "m", &sgfOutput->staticObjectVerticesOutput.vertices[i[0]].y, DVA_None);/* float32 */
    }

    //CEM_PclOutput
    DDefUInt(NULL, "pclOutput.uiVersionNumber", "", &pclOutput->uiVersionNumber, DVA_None); /* uint32 */
    DDefULLong(NULL, "pclOutput.sSigHeader.uiTimeStamp", "us", &pclOutput->sSigHeader.uiTimeStamp, DVA_None); /* uint64 */
    DDefUShort(NULL, "pclOutput.sSigHeader.uiMeasurementCounter", "", &pclOutput->sSigHeader.uiMeasurementCounter, DVA_None); /* uint16 */
    DDefUShort(NULL, "pclOutput.sSigHeader.uiCycleCounter", "", &pclOutput->sSigHeader.uiCycleCounter, DVA_None); /* uint16 */
    DDefUChar(NULL, "pclOutput.sSigHeader.eSigStatus", "", (unsigned char*)&pclOutput->sSigHeader.eSigStatus, DVA_None); /* uint8 */
    DDefUShort(NULL, "pclOutput.numberOfDelimiters", "", &pclOutput->numberOfDelimiters, DVA_None); /* uint16 */
    for (i[0] = 0; i[0] < aupdf::Constants::PFS_MAX_PCL_DELIMITERS; i[0]++) {
        std::sprintf(VarName, "pclOutput.delimiters._%d_.delimiterId", i[0]);
        DDefULLong(NULL, VarName, "", &pclOutput->delimiters[i[0]].id, DVA_None); /* uint64 */
        std::sprintf(VarName, "pclOutput.delimiters._%d_.startPointXPosition", i[0]);
        DDefFloat(NULL, VarName, "m", &pclOutput->delimiters[i[0]].startPointXPosition, DVA_None); /* float32 */
        std::sprintf(VarName, "pclOutput.delimiters._%d_.startPointYPosition", i[0]);
        DDefFloat(NULL, VarName, "m", &pclOutput->delimiters[i[0]].startPointYPosition, DVA_None); /* float32 */
        std::sprintf(VarName, "pclOutput.delimiters._%d_.endPointXPosition", i[0]);
        DDefFloat(NULL, VarName, "m", &pclOutput->delimiters[i[0]].endPointXPosition, DVA_None); /* float32 */
        std::sprintf(VarName, "pclOutput.delimiters._%d_.endPointYPosition", i[0]);
        DDefFloat(NULL, VarName, "m", &pclOutput->delimiters[i[0]].endPointYPosition, DVA_None); /* float32 */
        std::sprintf(VarName, "pclOutput.delimiters._%d_.confidencePercent", i[0]);
        DDefFloat(NULL, VarName, "", &pclOutput->delimiters[i[0]].confidence, DVA_None); /* float32 */
    }

    //ego_motion_at_cem_time
    DDefUInt(NULL, "egoMotionAtCemOutput.uiVersionNumber", "", &egoMotionAtCemOutput->uiVersionNumber, DVA_None); /*uint32*/
    DDefULLong(NULL, "egoMotionAtCemOutput.sSigHeader.uiTimeStamp", "us", &egoMotionAtCemOutput->sSigHeader.uiTimeStamp, DVA_None); /* uint64 */
    DDefUShort(NULL, "egoMotionAtCemOutput.sSigHeader.uiMeasurementCounter", "", &egoMotionAtCemOutput->sSigHeader.uiMeasurementCounter, DVA_None); /* uint16 */
    DDefUShort(NULL, "egoMotionAtCemOutput.sSigHeader.uiCycleCounter", "", &egoMotionAtCemOutput->sSigHeader.uiCycleCounter, DVA_None); /* uint16 */
    DDefUChar(NULL, "egoMotionAtCemOutput.sSigHeader.eSigStatus", "", (unsigned char*)&egoMotionAtCemOutput->sSigHeader.eSigStatus, DVA_None); /* uint16 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.xPosition_m", "m", &egoMotionAtCemOutput->odoEstimationAtCemTime.xPosition_m, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.yPosition_m", "m", &egoMotionAtCemOutput->odoEstimationAtCemTime.yPosition_m, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.longiVelocity_mps", "m/s", &egoMotionAtCemOutput->odoEstimationAtCemTime.longiVelocity_mps, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.longiAcceleration_mps2", "m/s^2", &egoMotionAtCemOutput->odoEstimationAtCemTime.longiAcceleration_mps2, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.rollAngle_rad", "m/s", &egoMotionAtCemOutput->odoEstimationAtCemTime.rollAngle_rad, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.pitchAngle_rad", "rad", &egoMotionAtCemOutput->odoEstimationAtCemTime.pitchAngle_rad, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.yawAngle_rad", "rad", &egoMotionAtCemOutput->odoEstimationAtCemTime.yawAngle_rad, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.yawRate_radps", "rad/s", &egoMotionAtCemOutput->odoEstimationAtCemTime.yawRate_radps, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.steerAngFrontAxle_rad", "rad", &egoMotionAtCemOutput->odoEstimationAtCemTime.steerAngFrontAxle_rad, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.steerAngRearAxle_rad", "rad", &egoMotionAtCemOutput->odoEstimationAtCemTime.steerAngRearAxle_rad, DVA_None); /* float32 */
    DDefFloat(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.drivenDistance_m", "m", &egoMotionAtCemOutput->odoEstimationAtCemTime.drivenDistance_m, DVA_None); /* float32 */
    DDefUChar(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.motionStatus_nu", "", (unsigned char*)&egoMotionAtCemOutput->odoEstimationAtCemTime.motionStatus_nu, DVA_None); /* uint8 */
    DDefChar(NULL, "egoMotionAtCemOutput.odoEstimationAtCemTime.drivingDirection_nu", "", (char*)&egoMotionAtCemOutput->odoEstimationAtCemTime.drivingDirection_nu, DVA_None); /* sint8 */

    //DynamicEnvironment
    DDefUInt(NULL, "dynamicEnvironment.uiVersionNumber", "", &dynamicEnvironment->uiVersionNumber, DVA_None); /*uint32*/
    DDefULLong(NULL, "dynamicEnvironment.sSigHeader.uiTimeStamp", "us", &dynamicEnvironment->sSigHeader.uiTimeStamp, DVA_None); /* uint64 */
    DDefUShort(NULL, "dynamicEnvironment.sSigHeader.uiMeasurementCounter", "", &dynamicEnvironment->sSigHeader.uiMeasurementCounter, DVA_None); /* uint16 */
    DDefUShort(NULL, "dynamicEnvironment.sSigHeader.uiCycleCounter", "", &dynamicEnvironment->sSigHeader.uiCycleCounter, DVA_None); /* uint16 */
    DDefUChar(NULL, "dynamicEnvironment.sSigHeader.eSigStatus", "", (unsigned char*)&dynamicEnvironment->sSigHeader.eSigStatus, DVA_None); /* uint8 */
    DDefUChar(NULL, "dynamicEnvironment.numberOfObjects", "", &dynamicEnvironment->numberOfObjects, DVA_None); /* uint8 */
    for (i[0] = 0; i[0] < aupdf::Constants::TPF_MAX_DYN_OBJECTS; i[0]++) {
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.id", i[0]);
        DDefUInt(NULL, VarName, "", &dynamicEnvironment->objects[i[0]].id, DVA_None); /* uint32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.objectClass", i[0]);
        DDefUChar(NULL, VarName, "", (unsigned char*)&dynamicEnvironment->objects[i[0]].objectClass, DVA_None); /* uint8 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.classProbability", i[0]);
        DDefFloat(NULL, VarName, "", &dynamicEnvironment->objects[i[0]].classProbability, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.dynamicProperty", i[0]);
        DDefUChar(NULL, VarName, "", (uint8*)&dynamicEnvironment->objects[i[0]].dynamicProperty, DVA_None); /* uint8 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.lifetime", i[0]);
        DDefUInt(NULL, VarName, "ms", &dynamicEnvironment->objects[i[0]].lifetime, DVA_None); /* uint32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.shape.referencePoint.x", i[0]);
        DDefFloat(NULL, VarName, "m", &dynamicEnvironment->objects[i[0]].shapePoints.referencePoint.x, DVA_None);  /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.shape.referencePoint.y", i[0]);
        DDefFloat(NULL, VarName, "m", &dynamicEnvironment->objects[i[0]].shapePoints.referencePoint.y, DVA_None);  /* float32 */
        for (i[1] = 0; i[1] < aupdf::Constants::TPF_NUMBER_OF_SHAPEPOINTS; i[1]++) {
            std::sprintf(VarName, "dynamicEnvironment.objects._%d_.shape.points._%d_.position.x", i[0], i[1]);
            DDefFloat(NULL, VarName, "m", &dynamicEnvironment->objects[i[0]].shapePoints.points[i[1]].position.x, DVA_None); /* float32 */
            std::sprintf(VarName, "dynamicEnvironment.objects._%d_.shape.points._%d_.position.y", i[0], i[1]);
            DDefFloat(NULL, VarName, "m", &dynamicEnvironment->objects[i[0]].shapePoints.points[i[1]].position.y, DVA_None); /* float32 */
            std::sprintf(VarName, "dynamicEnvironment.objects._%d_.shape.points._%d_.varianceX", i[0], i[1]);
            DDefFloat(NULL, VarName, "m^2", &dynamicEnvironment->objects[i[0]].shapePoints.points[i[1]].varianceX, DVA_None); /* float32 */
            std::sprintf(VarName, "dynamicEnvironment.objects._%d_.shape.points._%d_.varianceY", i[0], i[1]);
            DDefFloat(NULL, VarName, "m^2", &dynamicEnvironment->objects[i[0]].shapePoints.points[i[1]].varianceY, DVA_None); /* float32 */
            std::sprintf(VarName, "dynamicEnvironment.objects._%d_.shape.points._%d_.covarianceXY", i[0], i[1]);
            DDefFloat(NULL, VarName, "m^2", &dynamicEnvironment->objects[i[0]].shapePoints.points[i[1]].covarianceXY, DVA_None); /* float32 */
        }
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.state", i[0]);
        DDefUChar(NULL, VarName, "", (uint8*)&dynamicEnvironment->objects[i[0]].state, DVA_None); /* uint8 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.existenceCertainty", i[0]);
        DDefFloat(NULL, VarName, "", &dynamicEnvironment->objects[i[0]].existenceCertainty, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.orientation", i[0]);
        DDefFloat(NULL, VarName, "rad", &dynamicEnvironment->objects[i[0]].orientation, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.orientationStandardDeviation", i[0]);
        DDefFloat(NULL, VarName, "rad", &dynamicEnvironment->objects[i[0]].orientationStandardDeviation, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.velocity.x", i[0]);
        DDefFloat(NULL, VarName, "m/s", &dynamicEnvironment->objects[i[0]].velocity.x, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.velocity.y", i[0]);
        DDefFloat(NULL, VarName, "m/s", &dynamicEnvironment->objects[i[0]].velocity.y, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.velocityStandardDeviation.x", i[0]);
        DDefFloat(NULL, VarName, "m/s", &dynamicEnvironment->objects[i[0]].velocityStandardDeviation.x, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.velocityStandardDeviation.y", i[0]);
        DDefFloat(NULL, VarName, "m/s", &dynamicEnvironment->objects[i[0]].velocityStandardDeviation.y, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.acceleration.x", i[0]);
        DDefFloat(NULL, VarName, "m/s^2", &dynamicEnvironment->objects[i[0]].acceleration.x, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.acceleration.y", i[0]);
        DDefFloat(NULL, VarName, "m/s^2", &dynamicEnvironment->objects[i[0]].acceleration.y, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.accelerationStandardDeviation.x", i[0]);
        DDefFloat(NULL, VarName, "m/s^2", &dynamicEnvironment->objects[i[0]].accelerationStandardDeviation.x, DVA_None);/* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.accelerationStandardDeviation.y", i[0]);
        DDefFloat(NULL, VarName, "m/s^2", &dynamicEnvironment->objects[i[0]].accelerationStandardDeviation.y, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.yawRate", i[0]);
        DDefFloat(NULL, VarName, "rad/s", &dynamicEnvironment->objects[i[0]].yawRate, DVA_None); /* float32 */
        std::sprintf(VarName, "dynamicEnvironment.objects._%d_.yawRateStandardDeviation", i[0]);
        DDefFloat(NULL, VarName, "rad/s", &dynamicEnvironment->objects[i[0]].yawRateStandardDeviation, DVA_None); /* float32 */
    }

    //ParkingSlotDetectionOutput
    DDefUInt(NULL, "parkingSlotDetectionOutput.uiVersionNumber", "", &parkingSlotDetectionOutput->uiVersionNumber, DVA_None); /*uint32*/
    DDefULLong(NULL, "parkingSlotDetectionOutput.sSigHeader.uiTimeStamp", "us", &parkingSlotDetectionOutput->sSigHeader.uiTimeStamp, DVA_None); /* uint64 */
    DDefUShort(NULL, "parkingSlotDetectionOutput.sSigHeader.uiMeasurementCounter", "", &parkingSlotDetectionOutput->sSigHeader.uiMeasurementCounter, DVA_None); /* uint16 */
    DDefUShort(NULL, "parkingSlotDetectionOutput.sSigHeader.uiCycleCounter", "", &parkingSlotDetectionOutput->sSigHeader.uiCycleCounter, DVA_None); /* uint16 */
    DDefUChar(NULL, "parkingSlotDetectionOutput.sSigHeader.eSigStatus", "", (unsigned char*)&parkingSlotDetectionOutput->sSigHeader.eSigStatus, DVA_None); /* uint8 */
    DDefUChar(NULL, "parkingSlotDetectionOutput.numberOfSlots", "", &parkingSlotDetectionOutput->numberOfSlots, DVA_None); /* uint8 */
    for (i[0] = 0; i[0] < aupdf::Constants::PFS_PSD_MAX_PARKING_SLOTS; i[0]++) {
        std::sprintf(VarName, "parkingSlotDetectionOutput.parking_slots._%d_.slotId", i[0]);
        DDefUInt(NULL, VarName, "", &parkingSlotDetectionOutput->parkingSlots[i[0]].slotId, DVA_None); /* uint32 */
        for (i[1] = 0; i[1] < aupdf::Constants::PFS_NUMBER_OF_CORNERS; i[1]++) {
            std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots_%d_.slotCorners._%d_.x", i[0], i[1]);
            DDefFloat(NULL, VarName, "m", &parkingSlotDetectionOutput->parkingSlots[i[0]].slotCorners[i[1]].x, DVA_None); /* float32 */
            std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots_%d_.slotCorners._%d_.y", i[0], i[1]);
            DDefFloat(NULL, VarName, "m", &parkingSlotDetectionOutput->parkingSlots[i[0]].slotCorners[i[1]].y, DVA_None); /* float32 */
            std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots_%d_.cornerOcclusionState._%d_", i[0], i[1]);
            DDefUChar(NULL, VarName, "", (uint8_t*)&parkingSlotDetectionOutput->parkingSlots[i[0]].cornerOcclusionState[i[1]], DVA_None);  /* boolean */
        }
        std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots_%d_.delimiterType", i[0]);
        DDefUChar(NULL, VarName, "", (unsigned char*)&parkingSlotDetectionOutput->parkingSlots[i[0]].type, DVA_None); /* uint8 */
        std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots._%d_.existenceProbability", i[0]);
        DDefFloat(NULL, VarName, "", &parkingSlotDetectionOutput->parkingSlots[i[0]].existenceProbability, DVA_None); /* float32 */
        std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots._%d_.parkingScenarioConfidence.angled", i[0]);
        DDefFloat(NULL, VarName, "", &parkingSlotDetectionOutput->parkingSlots[i[0]].parkingScenarioConfidence.angled, DVA_None); /* float32 */
        std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots._%d_.parkingScenarioConfidence.parallel", i[0]);
        DDefFloat(NULL, VarName, "", &parkingSlotDetectionOutput->parkingSlots[i[0]].parkingScenarioConfidence.parallel, DVA_None); /* float32 */
        std::sprintf(VarName, "parkingSlotDetectionOutput.parkingSlots._%d_.parkingScenarioConfidence.perpendicular", i[0]);
        DDefFloat(NULL, VarName, "", &parkingSlotDetectionOutput->parkingSlots[i[0]].parkingScenarioConfidence.perpendicular, DVA_None); /* float32 */
    }

    //StopLineOutput
    DDefUInt(NULL, "stopLineOutput.uiVersionNumber", "", &stopLineOutput->uiVersionNumber, DVA_None); /*uint32*/
    DDefULLong(NULL, "stopLineOutput.sSigHeader.uiTimeStamp", "us", &stopLineOutput->sSigHeader.uiTimeStamp, DVA_None); /* uint64 */
    DDefUShort(NULL, "stopLineOutput.sSigHeader.uiMeasurementCounter", "", &stopLineOutput->sSigHeader.uiMeasurementCounter, DVA_None); /* uint16 */
    DDefUShort(NULL, "stopLineOutput.sSigHeader.uiCycleCounter", "", &stopLineOutput->sSigHeader.uiCycleCounter, DVA_None); /* uint16 */
    DDefUChar(NULL, "stopLineOutput.sSigHeader.eSigStatus", "", (unsigned char*)&stopLineOutput->sSigHeader.eSigStatus, DVA_None); /* uint8 */
    DDefUChar(NULL, "stopLineOutput.numberOfLines", "", &stopLineOutput->numberOfLines, DVA_None); /* uint8 */
    for (i[0] = 0; i[0] < aupdf::Constants::PFS_STOP_LINES_MAX_ELEMENTS; i[0]++) {
        std::sprintf(VarName, "stopLineOutput.stopLines._%d_.id", i[0]);
        DDefUInt(NULL, VarName, "", &stopLineOutput->stopLines[i[0]].id, DVA_None); /* uint32 */
        std::sprintf(VarName, "stopLineOutput.stopLines._%d_.startPoint.x", i[0]);
        DDefFloat(NULL, VarName, "m", &stopLineOutput->stopLines[i[0]].startPoint.x, DVA_None); /* float32 */
        std::sprintf(VarName, "stopLineOutput.stopLines._%d_.startPoint.y", i[0]);
        DDefFloat(NULL, VarName, "m", &stopLineOutput->stopLines[i[0]].startPoint.y, DVA_None); /* float32 */
        std::sprintf(VarName, "stopLineOutput.stopLines._%d_.endPoint.x", i[0]);
        DDefFloat(NULL, VarName, "m", &stopLineOutput->stopLines[i[0]].endPoint.x, DVA_None); /* float32 */
        std::sprintf(VarName, "stopLineOutput.stopLines._%d_.endPoint.y", i[0]);
        DDefFloat(NULL, VarName, "m", &stopLineOutput->stopLines[i[0]].endPoint.y, DVA_None); /* float32 */
        std::sprintf(VarName, "stopLineOutput.stopLines._%d_.lineConfidence", i[0]);
        DDefFloat(NULL, VarName, "", &stopLineOutput->stopLines[i[0]].lineConfidence, DVA_None); /* float32 */
    }

    //PedestrianCrossingOutput
    DDefUInt(NULL, "pedestrianCrossingOutput.uiVersionNumber", "", &pedestrianCrossingOutput->uiVersionNumber, DVA_None); /*uint32*/
    DDefULLong(NULL, "pedestrianCrossingOutput.sSigHeader.uiTimeStamp", "us", &pedestrianCrossingOutput->sSigHeader.uiTimeStamp, DVA_None); /* uint64 */
    DDefUShort(NULL, "pedestrianCrossingOutput.sSigHeader.uiMeasurementCounter", "", &pedestrianCrossingOutput->sSigHeader.uiMeasurementCounter, DVA_None); /* uint16 */
    DDefUShort(NULL, "pedestrianCrossingOutput.sSigHeader.uiCycleCounter", "", &pedestrianCrossingOutput->sSigHeader.uiCycleCounter, DVA_None); /* uint16 */
    DDefUChar(NULL, "pedestrianCrossingOutput.sSigHeader.eSigStatus", "", (unsigned char*)&pedestrianCrossingOutput->sSigHeader.eSigStatus, DVA_None); /* uint8 */
    DDefUChar(NULL, "pedestrianCrossingOutput.numberOfCrossings", "", &pedestrianCrossingOutput->numberOfCrossings, DVA_None); /* uint8 */
    for (i[0] = 0; i[0] < aupdf::Constants::PFS_PEDESTRIAN_CROSSING_MAX_ELEMENTS; i[0]++) {
        std::sprintf(VarName, "pedestrianCrossingOutput.pedestrianCrossings._%d_.id", i[0]);
        DDefUInt(NULL, VarName, "", &pedestrianCrossingOutput->pedestrianCrossings[i[0]].id, DVA_None); /* uint32 */
        std::sprintf(VarName, "pedestrianCrossingOutput.pedestrianCrossings._%d_.confidence", i[0]);
        DDefFloat(NULL, VarName, "", &pedestrianCrossingOutput->pedestrianCrossings[i[0]].confidence, DVA_None); /* float32 */
        for (i[1] = 0; i[1] < aupdf::Constants::PFS_PEDESTRIAN_CROSSING_BOUNDARY_POINT_NUM; i[1]++) {
            std::sprintf(VarName, "pedestrianCrossingOutput.pedestrianCrossings._%d_.boundaryPoints._%d_.x", i[0], i[1]);
            DDefFloat(NULL, VarName, "m", &pedestrianCrossingOutput->pedestrianCrossings[i[0]].boundaryPoints[i[1]].x, DVA_None); /* float32 */
            std::sprintf(VarName, "pedestrianCrossingOutput.pedestrianCrossings._%d_.boundaryPoints._%d_.y", i[0], i[1]);
            DDefFloat(NULL, VarName, "m", &pedestrianCrossingOutput->pedestrianCrossings[i[0]].boundaryPoints[i[1]].y, DVA_None); /* float32 */
        }
    }
}
