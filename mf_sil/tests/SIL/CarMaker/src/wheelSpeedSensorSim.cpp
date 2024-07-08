#include "wheelSpeedSensorSim.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <random>
const uint16_t gNumberOfTeeth_nu[4] = {43u, 43u, 43u, 43u};
const uint16_t gWheelPulseOverflow{ 1000U }; //Vehicle specific value after this number of pulses is reached it will jump back to 0
const float gDeltaTimeToInit = { 0.5f };  // when time elapsed without a pulse direction will be set to "init"

float gRotationAngles_rad[4]    = {0.0f, 0.0f, 0.0f, 0.0f};
uint16_t gNumberOfPulses_nu[4]  = { 0u, 0u, 0u, 0u };
ap_commonvehsigprovider::WheelDrivingDirection gDirection_nu[4];
float gUnitDelay1_rad[4]        = {0.0f, 0.0f, 0.0f, 0.0f};
uint16_t gUnitDelay2_nu[4]      = {0u, 0u, 0u, 0u};
float gUnitDelay4_rad[4]        = {0.0f, 0.0f, 0.0f, 0.0f};
uint16_t gUnitDelay5_nu[4]      = {0u, 0u, 0u, 0u};
int8_t gUnitDelay6_nu[4]        = {0, 0, 0, 0};
float deltatimestamp_s[4] = { 0.0f, 0.0f, 0.0f, 0.0f },randtimestamp_s[4]= { 0.0f, 0.0f, 0.0f, 0.0f };
float gTimeStampDelta_s[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
float glatestWheelSpeed_radps[4]= { 0.0f, 0.0f, 0.0f, 0.0f };
uint16_t LastgUnitDelay2[4] = { 0u, 0u, 0u, 0u }, randticksstamp_nu[4] = { 0u, 0u, 0u, 0u }, gDesicionWhlMdlExt;
static std::mt19937 gRandomEngine{}; // explicitly use mt19937 instead of default_random_engine, since the latter is implementation specific

void wheelSpeedSensorSimCalc(void)
{
    float afterSubtract_rad         = 0.0f;
    int8_t signAfterSubstract_nu    = 0;
    float radPerPulse_nu            = 0.0f;
    float afterDivide1_nu           = 0.0f;
    uint16_t afterAdd2_nu           = 0;

    for (unsigned int i = 0; i < 4u; i++) {
        LastgUnitDelay2[i] = gUnitDelay2_nu[i];

        // Switch1: Store last angle after pulse occurred
        if (gUnitDelay2_nu[i] != 0u) {
            gUnitDelay1_rad[i] = gUnitDelay4_rad[i];
        }

        // Subtract
        afterSubtract_rad = gRotationAngles_rad[i] - gUnitDelay1_rad[i];

        // Sign
        signAfterSubstract_nu = ((int)(afterSubtract_rad > 0.0f)) - ((int)(afterSubtract_rad < 0.0f));

        // Divide
        radPerPulse_nu = 2.0f * (float)M_PI / ((float)gNumberOfTeeth_nu[i] * 2.0f);

        // Unit Delay4
        gUnitDelay4_rad[i] = gRotationAngles_rad[i] - ((float)signAfterSubstract_nu) * fmodf(fabsf(afterSubtract_rad), radPerPulse_nu);
        // Timestamp correction
        if (glatestWheelSpeed_radps[i]!=0 && gNumberOfPulses_nu[i]>0 && fabsf(afterSubtract_rad)> radPerPulse_nu)
        {
             gTimeStampDelta_s[i]=  (fmodf(fabsf(afterSubtract_rad), radPerPulse_nu)) /fabsf(glatestWheelSpeed_radps[i]);
        }
        else
        {
            gTimeStampDelta_s[i] = 0;
        }
        // Divide1
        afterDivide1_nu = afterSubtract_rad / radPerPulse_nu;  // number of pulses in float and direction

        // Unit Delay2
        gUnitDelay2_nu[i] = (uint16_t)fabsf(afterDivide1_nu);   // number of pulses per step

        // NumberOfPulses
        gNumberOfPulses_nu[i] += (uint16_t)gUnitDelay2_nu[i];

        //Overflow
        gNumberOfPulses_nu[i] = gNumberOfPulses_nu[i] % gWheelPulseOverflow;

        // Add2  pulse per step + pulse since standstill
        afterAdd2_nu = gUnitDelay2_nu[i] + gUnitDelay5_nu[i];

        if (gUnitDelay2_nu[i] == 0) { // number of pulses per step
            deltatimestamp_s[i] += 0.001f;

            // If wanted an empirical time to standstill can be used (see use of gDesicionWhlMdlExt)
            if (deltatimestamp_s[i]==0.001f){
                static std::normal_distribution<double> timeNormalDistribution{ 20.369369369369370, 6.131787154492077 };
                randtimestamp_s[i] = static_cast<float>(timeNormalDistribution(gRandomEngine) * 0.02);
            }
        }
        else {
            deltatimestamp_s[i] = 0.0f;
            // If wanted an empirical number of ticks can be used instead of fixed three ticks (see use of gDesicionWhlMdlExt)
            if (LastgUnitDelay2[i]==0 && glatestWheelSpeed_radps[i]==0)
            {
                static std::normal_distribution<double> tickNormalDistribution{ 2.928571428571428, 3.040270258257190 };
                randticksstamp_nu[i] = uint16_t(tickNormalDistribution(gRandomEngine));
            }
        }

        if ((deltatimestamp_s[i] > randtimestamp_s[i] && gDesicionWhlMdlExt>=2 && glatestWheelSpeed_radps[i] == 0) || (deltatimestamp_s[i] > gDeltaTimeToInit && gDesicionWhlMdlExt < 2)) { // standstill
            gUnitDelay6_nu[i] = 0;
            // Unit Delay5
            gUnitDelay5_nu[i] = 0;
        }
        else if ((afterAdd2_nu >= randticksstamp_nu[i] && gDesicionWhlMdlExt >= 2) || (afterAdd2_nu>=1U && gDesicionWhlMdlExt < 2))  {
            gUnitDelay6_nu[i] = ((int)(afterDivide1_nu > 0.0f)) - ((int)(afterDivide1_nu < 0.0f));  // sign
            gUnitDelay5_nu[i] = afterAdd2_nu;
        }
        else {
            gUnitDelay6_nu[i] = 0;  // sign
            gUnitDelay5_nu[i] += gUnitDelay2_nu[i];
        }
        // Direction
        gDirection_nu[i] = static_cast<ap_commonvehsigprovider::WheelDrivingDirection>(gUnitDelay6_nu[i]);
    }
}

void wheelSpeedSensorSimulationInit()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        gRotationAngles_rad[i] = 0.0f;
        gNumberOfPulses_nu[i]  = 0;
        gUnitDelay1_rad[i]     = 0;
        gUnitDelay2_nu[i]      = 0;
        gUnitDelay4_rad[i]     = 0;
        gUnitDelay5_nu[i]      = 0;
        gUnitDelay6_nu[i]      = 0;
        gTimeStampDelta_s[i]   = 0.0f;
        LastgUnitDelay2[i]     = 0u;
        randticksstamp_nu[i]   = 0u;
        randtimestamp_s[i]     = 0.0f;
        deltatimestamp_s[i]    = 0.0f;
        glatestWheelSpeed_radps[i]   = 0.0f;

    }
    // reset random number generator seed to have reproducible simulation runs
    gRandomEngine.seed();
}

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
)
{
    // Transfer input
    gRotationAngles_rad[0] = wheelRotAngFL_rad;
    gRotationAngles_rad[1] = wheelRotAngFR_rad;
    gRotationAngles_rad[2] = wheelRotAngRL_rad;
    gRotationAngles_rad[3] = wheelRotAngRR_rad;
    glatestWheelSpeed_radps[0] = latestWheelSpeedFL_radps;
    glatestWheelSpeed_radps[1] = latestWheelSpeedFR_radps;
    glatestWheelSpeed_radps[2] = latestWheelSpeedRL_radps;
    glatestWheelSpeed_radps[3] = latestWheelSpeedRR_radps;
    gDesicionWhlMdlExt = DesicionWhlMdlExt;
    // Simulate step of wheel speed sensor
    wheelSpeedSensorSimCalc();
    // Fill output
    numberOfPulsesFL_nu = gNumberOfPulses_nu[0];
    numberOfPulsesFR_nu = gNumberOfPulses_nu[1];
    numberOfPulsesRL_nu = gNumberOfPulses_nu[2];
    numberOfPulsesRR_nu = gNumberOfPulses_nu[3];
    directionFL_nu = gDirection_nu[0];
    directionFR_nu = gDirection_nu[1];
    directionRL_nu = gDirection_nu[2];
    directionRR_nu = gDirection_nu[3];
    TimeStampDeltaFL_s = gTimeStampDelta_s[0];
    TimeStampDeltaFR_s = gTimeStampDelta_s[1];
    TimeStampDeltaRL_s= gTimeStampDelta_s[2];
    TimeStampDeltaRR_s= gTimeStampDelta_s[3];
}
