#include "GroundTruthEnvironment.h"

// use ground-truth odometry data from CarMaker
constexpr bool isVedodoActive{ false };
// do not inflate objects to not change the ground-truth object shape
constexpr float inflationLength_m{ 0.0f };

void GroundTruthEnvironment::update(const bool parkingOnLeftSide_nu[],
    const float MIN_OBJ_HEIGHT_M,
    const int pathBeforeActIndex,
    const uint8_t overwritePathBeforeFuncActivation_nu,
    const LSM_GEOML::Pose &pathBeforeFuncActivation0Pose,
    const TrafficContour2D trafficContour2D_t[],
    const bool isFirstCycle,
    const lsm_vedodo::OdoEstimation &odoEstimationCM,
    const lsm_vedodo::OdoEstimationOutputPort &odoEstimationOutputPort)
{
    if (((static_cast<uint64_t>(std::round(SimCore.Time * 1000)) % EnvironmentPerception::SI_HIGH_CYCLE_TIME_MS) == 0)){
        mEnvironmentPerception.updateEnvModelData(isVedodoActive,
            parkingOnLeftSide_nu,
            MIN_OBJ_HEIGHT_M,
            pathBeforeActIndex,
            overwritePathBeforeFuncActivation_nu,
            pathBeforeFuncActivation0Pose,
            trafficContour2D_t,
            inflationLength_m,
            isFirstCycle,
            odoEstimationCM,
            odoEstimationOutputPort,
            mEnvModelPort,
            mEnvModelPortCMOrigin,
            mCollEnvModelPort,
            mParkingBoxPort,
            mParkingBoxPortCMOrigin,
            mPerceptionAvailabilityPort);
    }
    

    const std::vector<VCEM::ODSlot> odSlotsVector{ mCemSurrogate.determineODSlots(trafficContour2D_t, pathBeforeFuncActivation0Pose, false, true) };
    mNumValidODSlots = std::min(static_cast<uint8_t>(odSlotsVector.size()), MAX_OD_SLOTS);

    for (uint8_t i{ 0U }; i < mNumValidODSlots; i++) {
        mODSlots[i] = odSlotsVector[i];
    }
}

void GroundTruthEnvironment::reset()
{
    mEnvModelPort = eco::create_default<si::ApEnvModelPort>();
    mEnvModelPortCMOrigin = eco::create_default<si::ApEnvModelPort>();
    mCollEnvModelPort = eco::create_default<si::CollEnvModelPort>();
    mParkingBoxPort = eco::create_default<si::ApParkingBoxPort>();
    mParkingBoxPortCMOrigin = eco::create_default<si::ApParkingBoxPort>();
    mPerceptionAvailabilityPort = eco::create_default<si::PerceptionAvailabilityPort>();
    mNumValidODSlots = 0U;

    mEnvironmentPerception.resetVariables();
    mEnvironmentPerception.Init(false, false, 0.0f);

    mCemSurrogate.resetHistory();
    VCEM::CemSurrogateConfig vCemConfig{};
    vCemConfig.latencyTime_ms = 0.0f;
    vCemConfig.staticOffsetX_m = 0.0f;
    vCemConfig.staticOffsetY_m = 0.0f;
    mCemSurrogate.init(vCemConfig);
}

void GroundTruthEnvironment::registerCarMakerDVAs()
{
    char VarName[255];
    tDDefault *envMP = DDefaultCreate("GT.envModelPort.");
    DDefULLong(envMP, "sSigHeader.uiTimeStamp", "us", &mEnvModelPortCMOrigin.sSigHeader.uiTimeStamp, DVA_None);
    DDefUChar(envMP, "numberOfStaticObjects_u8", "", &mEnvModelPortCMOrigin.numberOfStaticObjects_u8, DVA_None);
    DDefUChar(envMP, "numberOfDynamicObjects_u8", "", &mEnvModelPortCMOrigin.numberOfDynamicObjects_u8, DVA_None);
    DDefUChar(envMP, "numberOfParkMarkings_u8", "", &mEnvModelPortCMOrigin.numberOfParkMarkings_u8, DVA_None);

    //register static objects
    for (uint16 i = 0U; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_STATIC_OBJ_NU; i++) {
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.existenceProb_perc", i);
        DDefUChar(envMP, VarName, "", &mEnvModelPortCMOrigin.staticObjects[i].existenceProb_perc, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.measurementPrinciple_nu", i);
        DDefUChar(envMP, VarName, "", (uint8_t*)&mEnvModelPortCMOrigin.staticObjects[i].measurementPrinciple_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objAgeInCycles_nu", i);
        DDefUShort(envMP, VarName, "", &mEnvModelPortCMOrigin.staticObjects[i].objAgeInCycles_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objHeightClassConfidence_perc", i);
        DDefUChar(envMP, VarName, "", &mEnvModelPortCMOrigin.staticObjects[i].objHeightClassConfidence_perc, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objHeightClass_nu", i);
        DDefUChar(envMP, VarName, "", (uint8_t*)&mEnvModelPortCMOrigin.staticObjects[i].objHeightClass_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objMeasLastUpdateInCycles_nu", i);
        DDefUShort(envMP, VarName, "", &mEnvModelPortCMOrigin.staticObjects[i].objMeasLastUpdateInCycles_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objTrendLastUpdateInCycles_nu", i);
        DDefUShort(envMP, VarName, "", &mEnvModelPortCMOrigin.staticObjects[i].objTrendLastUpdateInCycles_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objTrend_nu", i);
        DDefUChar(envMP, VarName, "", (uint8_t*)&mEnvModelPortCMOrigin.staticObjects[i].objTrend_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.readFromNVRAM_nu", i);
        DDefUChar(envMP, VarName, "", (uint8_t*)&mEnvModelPortCMOrigin.staticObjects[i].readFromNVRAM_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.refObjClass_nu", i);
        DDefUChar(envMP, VarName, "", (uint8_t*)&mEnvModelPortCMOrigin.staticObjects[i].refObjClass_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.refObjID_nu", i);
        DDefUInt(envMP, VarName, "", &mEnvModelPortCMOrigin.staticObjects[i].refObjID_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objShape_m.actualSize", i);
        DDefUInt(envMP, VarName, "m", &mEnvModelPortCMOrigin.staticObjects[i].objShape_m.actualSize, DVA_None);
        for (uint8 j = 0U; j < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU; j++) {
            snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objShape_m.array._%d_.x_dir", i, j);
            DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.staticObjects[i].objShape_m.array[j].x_dir, DVA_None);
            snprintf(VarName, sizeof(VarName), "staticObjects._%d_.objShape_m.array._%d_.y_dir", i, j);
            DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.staticObjects[i].objShape_m.array[j].y_dir, DVA_None);
        }
    }

    //register dynamic objects
    for (uint8 i = 0U; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_DYN_OBJECTS_NU; i++) {
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.accel_mps2.x_dir", i);
        DDefFloat(envMP, VarName, "m/s^2", &mEnvModelPortCMOrigin.dynamicObjects[i].accel_mps2.x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.accel_mps2.y_dir", i);
        DDefFloat(envMP, VarName, "m/s^2", &mEnvModelPortCMOrigin.dynamicObjects[i].accel_mps2.y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.classConfidence_perc", i);
        DDefUChar(envMP, VarName, "", &mEnvModelPortCMOrigin.dynamicObjects[i].classConfidence_perc, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.existenceProb_perc", i);
        DDefUChar(envMP, VarName, "", &mEnvModelPortCMOrigin.dynamicObjects[i].existenceProb_perc, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.headingAngle_rad", i);
        DDefFloat(envMP, VarName, "rad", &mEnvModelPortCMOrigin.dynamicObjects[i].headingAngle_rad, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.measurementState_nu", i);
        DDefUChar(envMP, VarName, "", (uint8_t*)&mEnvModelPortCMOrigin.dynamicObjects[i].measurementState_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.objClass_nu", i);
        DDefUChar(envMP, VarName, "", (uint8_t*)&mEnvModelPortCMOrigin.dynamicObjects[i].objClass_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.refObjID_nu", i);
        DDefUInt(envMP, VarName, "", &mEnvModelPortCMOrigin.dynamicObjects[i].refObjID_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.vel_mps.x_dir", i);
        DDefFloat(envMP, VarName, "m/s", &mEnvModelPortCMOrigin.dynamicObjects[i].vel_mps.x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.vel_mps.y_dir", i);
        DDefFloat(envMP, VarName, "m/s", &mEnvModelPortCMOrigin.dynamicObjects[i].vel_mps.y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.objShape_m.actualSize", i);
        DDefUInt(envMP, VarName, "", &mEnvModelPortCMOrigin.dynamicObjects[i].objShape_m.actualSize, DVA_None);
        for (uint8 j = 0U; j < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU; j++) {
            snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.objShape_m.array._%d_.x_dir", i, j);
            DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.dynamicObjects[i].objShape_m.array[j].x_dir, DVA_None);
            snprintf(VarName, sizeof(VarName), "dynamicObjects._%d_.objShape_m.array._%d_.y_dir", i, j);
            DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.dynamicObjects[i].objShape_m.array[j].y_dir, DVA_None);
        }
    }

    //register park makers
    for (uint8 i = 0U; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_SPACE_MARKINGS_NU; i++) {
        snprintf(VarName, sizeof(VarName), "parkingSpaceMarkings._%d_.existenceProb_perc", i);
        DDefUChar(envMP, VarName, "", &mEnvModelPortCMOrigin.parkingSpaceMarkings[i].existenceProb_perc, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingSpaceMarkings._%d_.pos_m.actualSize", i);
        DDefUInt(envMP, VarName, "", &mEnvModelPortCMOrigin.parkingSpaceMarkings[i].pos_m.actualSize, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingSpaceMarkings._%d_.pos_m.array._0_.x_dir", i);
        DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.parkingSpaceMarkings[i].pos_m.array[0].x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingSpaceMarkings._%d_.pos_m.array._0_.y_dir", i);
        DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.parkingSpaceMarkings[i].pos_m.array[0].y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingSpaceMarkings._%d_.pos_m.array._1_.x_dir", i);
        DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.parkingSpaceMarkings[i].pos_m.array[1].x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingSpaceMarkings._%d_.pos_m.array._1_.y_dir", i);
        DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.parkingSpaceMarkings[i].pos_m.array[1].y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingSpaceMarkings._%d_.width_m", i);
        DDefFloat(envMP, VarName, "m", &mEnvModelPortCMOrigin.parkingSpaceMarkings[i].width_m, DVA_None);
    }
    
    //register parking boxes
    for (uint8 i = 0U; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU; i++) {
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.parkingBoxID_nu", i);
        DDefUShort(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].parkingBoxID_nu, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.existenceProb_perc", i);
        DDefUChar(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].existenceProb_perc, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.0.x_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[0].x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.0.y_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[0].y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.1.x_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[1].x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.1.y_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[1].y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.2.x_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[2].x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.2.y_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[2].y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.3.x_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[3].x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.3.y_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[3].y_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.4.x_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[4].x_dir, DVA_None);
        snprintf(VarName, sizeof(VarName), "parkingBoxes._%d_.slotCoordinates.4.y_m", i);
        DDefFloat(envMP, VarName, "", &mParkingBoxPortCMOrigin.parkingBoxes[i].slotCoordinates_m.array[4].y_dir, DVA_None);
    }
    DDefaultDelete(envMP);

    //register OD slots
    tDDefault *cemEnv = DDefaultCreate("GT.");
    DDefUChar(cemEnv, "mODSlots.numValidODSlots", "", &mNumValidODSlots, DVA_None);
    for (uint8 i = 0U; i < MAX_OD_SLOTS; i++) {
        snprintf(VarName, sizeof(VarName), "mODSlots._%d_.existenceProb_perc", i);
        DDefUChar(cemEnv, VarName, "", &mODSlots[i].existence_probability, DVA_None);
        snprintf(VarName, sizeof(VarName), "mODSlots._%d_.parking_scenario_confidence.angled", i);
        DDefUChar(cemEnv, VarName, "", &mODSlots[i].parking_scenario_confidence.angled, DVA_None);
        snprintf(VarName, sizeof(VarName), "mODSlots._%d_.parking_scenario_confidence.parallel", i);
        DDefUChar(cemEnv, VarName, "", &mODSlots[i].parking_scenario_confidence.parallel, DVA_None);
        snprintf(VarName, sizeof(VarName), "mODSlots._%d_.parking_scenario_confidence.perpendicular", i);
        DDefUChar(cemEnv, VarName, "", &mODSlots[i].parking_scenario_confidence.perpendicular, DVA_None);
        snprintf(VarName, sizeof(VarName), "mODSlots._%d_.slotId", i);
        DDefUInt(cemEnv, VarName, "", &mODSlots[i].slotId, DVA_None);
        for (uint8 k = 0U; k < 4; k++) {
            snprintf(VarName, sizeof(VarName), "mODSlots._%d_.cameraId[%d]", i, k);
            DDefUChar(cemEnv, VarName, "", reinterpret_cast<uint8*>(&mODSlots[i].cameraId[k]), DVA_None);
        }
        for (uint8 j = 0U; j < 4; j++) {
            snprintf(VarName, sizeof(VarName), "mODSlots._%d_.slot_corners._%d%_.x", i, j);
            DDefFloat(cemEnv, VarName, "m", &mODSlots[i].slot_corners[j].x(), DVA_None);
            snprintf(VarName, sizeof(VarName), "mODSlots._%d_.slot_corners._%d%_.y", i, j);
            DDefFloat(cemEnv, VarName, "m", &mODSlots[i].slot_corners[j].y(), DVA_None);
        }
    }
    DDefaultDelete(cemEnv);
}
