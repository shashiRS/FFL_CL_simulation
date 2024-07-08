#ifndef LOW_SPEED_LATERAL_DMC_H_
#define LOW_SPEED_LATERAL_DMC_H_

void initializeLowSpeedLateralDMC();

void resetLowSpeedLateralDMCDelay();

void lowSpeedLateralDMC(
    float* SteerAngReqInput,
    float* SteerWheelAngleOutput,
    const bool SteerReqEnableIn,
    const float SteerWheelAngleReq,
    const float deltaT_s,
    const float currentSteerWheelAngle);

#endif#pragma once
