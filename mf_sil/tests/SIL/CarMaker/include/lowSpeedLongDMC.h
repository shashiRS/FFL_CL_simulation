#ifndef LOW_SPEED_LONG_DMC_H_
#define LOW_SPEED_LONG_DMC_H_
void lowSpeedLongDMC(
    float* BrakeReq_mps2,
    bool* BrakeReqEnableOut,
    float* AccelReq_mps2,
    bool* AccelReqEnableOut,
    bool* standStillHold,
    bool* standStillSecure,
    bool* maneuveringFinished,
    float* velocityRequestIntern,
    const bool AccelReqEnableIn,
    const float VelReq,
    const float DistReq_m,
    const bool HoldReq,
    const bool EmergencyHoldReq,
    const bool SecureReq,
    const float EgoVel,
    const float deltaT_s);
void lowSpeedLongDMCSemiAPBrake(
    float* BrakeReq_mps2,
    bool* BrakeReqEnableOut,
    float* AccelReq_mps2,
    bool* AccelReqEnableOut,
    bool* standStillHold,
    bool* standStillSecure,
    bool* maneuveringFinished,
    float* velocityRequestIntern,
    const bool AccelReqEnableIn,
    const float VelReq,
    const float DistReq_m,
    const bool HoldReq,
    const bool EmergencyHoldReq,
    const bool SecureReq,
    const float EgoVel);
void resetLowSpeedLongDMC();
#endif
