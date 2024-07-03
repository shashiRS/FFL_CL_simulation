#pragma once

#include "EnvironmentPerception.h"
#include "CemSurrogate.h"

static constexpr uint8_t MAX_OD_SLOTS = 12U;

class GroundTruthEnvironment {
public:
    void update(const bool parkingOnLeftSide_nu[],
        const float MIN_OBJ_HEIGHT_M,
        const int pathBeforeActIndex,
        const uint8_t overwritePathBeforeFuncActivation_nu,
        const LSM_GEOML::Pose &pathBeforeFuncActivation0Pose,
        const TrafficContour2D trafficContour2D_t[],
        const bool isFirstCycle,
        const lsm_vedodo::OdoEstimation &odoEstimationCM,
        const lsm_vedodo::OdoEstimationOutputPort &odoEstimationOutputPort);

    void reset();

    void registerCarMakerDVAs();

private:
    // SI surrogate model used to calculate ground-truth data for static/dyn. obj., line markers, parking boxes
    EnvironmentPerception mEnvironmentPerception;
    VCEM::CemSurrogate mCemSurrogate;

    // Ports to be updated by EnvironmentPerception model
    si::ApEnvModelPort mEnvModelPort;
    si::ApEnvModelPort mEnvModelPortCMOrigin;
    si::CollEnvModelPort mCollEnvModelPort;
    si::ApParkingBoxPort mParkingBoxPort;
    si::ApParkingBoxPort mParkingBoxPortCMOrigin;
    si::PerceptionAvailabilityPort mPerceptionAvailabilityPort;
    std::array<VCEM::ODSlot, MAX_OD_SLOTS> mODSlots;

    uint8_t  mNumValidODSlots{ 0U };

};