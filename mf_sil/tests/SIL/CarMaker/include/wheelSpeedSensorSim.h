/*
 * Simulation of an wheel speed sensor
 * This function is a copy of the simulink model "generic/CAN/Wheel Speed Sensor Signal Processing/Wheel Speed Sensor Simulation" from Kai Bretzigheimer
 * Project: Automated Parking, 2017
 * Contact: Philipp Kunz, philipp.kunz@continental-corporation.com
 */

#include "ap_commonvehsigprovider/wheel_driving_direction.h"
#include <stdint.h>

void wheelSpeedSensorSimCalc(void);

void wheelSpeedSensorSimulationInit(void);

void wheelSpeedSensorSimulation(
    uint16_t& numberOfPulsesFL_nu,
    uint16_t& numberOfPulsesFR_nu,
    uint16_t& numberOfPulsesRL_nu,
    uint16_t& numberOfPulsesRR_nu,
    ap_commonvehsigprovider::WheelDrivingDirection& directionFL_nu,
    ap_commonvehsigprovider::WheelDrivingDirection& directionFR_nu,
    ap_commonvehsigprovider::WheelDrivingDirection& directionRL_nu,
    ap_commonvehsigprovider::WheelDrivingDirection& directionRR_nu,
    const float wheelRotAngFL_rad,
    const float wheelRotAngFR_rad,
    const float wheelRotAngRL_rad,
    const float wheelRotAngRR_rad,
    float latestWheelSpeedFL_radps,
    float latestWheelSpeedFR_radps,
    float latestWheelSpeedRL_radps,
    float latestWheelSpeedRR_radps,
    float& TimeStampDeltaFL_s,
    float& TimeStampDeltaFR_s,
    float& TimeStampDeltaRL_s,
    float& TimeStampDeltaRR_s,
    uint16_t DesicionWhlMdlExt
);