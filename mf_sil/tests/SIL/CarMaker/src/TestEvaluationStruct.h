#ifndef TEST_EVALUATION_STRUCT_HEADER
#define TEST_EVALUATION_STRUCT_HEADER

#include "MfSilTypes.h"

struct PositionOnTrajectory {
    uint8_t currentTrajectoryIndex;                 //Positioned index of current trajectory element of the planned trajectory input
    bool discontinuityAtCurrentTrajectoryIndex;     //Indicate discontinuity between the current and the next trajectory element of the planned trajectory input
    float32_t intermediateValue_perc;               //Percentage intermediate value between current and next trajectory point (saturated from 0.0 to 1.0)
    float32_t intermediateValue_raw_perc;           //Percentage intermediate value between current and next trajectory point (not saturated; value smaller 0.0 and larger 1.0 possible if outside planned trajectory)
    bool outsideTrajectoryStart_nu;                 //Indicate that position is outside of planned trajectory (at the beginning of the trajectory)
    bool outsideTrajectoryEnd_nu;                   //Indicate that position is outside of planned trajectory (at the end of the trajectory)
};

//Enumeration for maneuver
enum Parking_Maneuver
{
    FORWARD = 1,
    BACKWARD = 2,
    NOT_SPECIFIED = 19
};
struct OptimalTargetPose {
    bool valid;
    LSM_GEOML::Pose pose;
};

struct EvaluationPort {
    uint8_t n_strokes_max_nu;
    float32_t v_max_mps;
    float32_t t_sim_max_s;
    bool scopeBase_nu;
    bool useCase_nu;
    Parking_Maneuver parkingManeuver_nu;
    OptimalTargetPose optimalTargetPose;
    float32_t latMaxDeviation_m;
};
#endif // !TEST_EVALUATION_STRUCT_HEADER


