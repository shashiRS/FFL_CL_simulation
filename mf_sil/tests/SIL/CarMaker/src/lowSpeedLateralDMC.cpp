#define EPS_DEAD_TIME_MS 40
#define EPS_PT1_TIME_S 0.06

float SteerRequestDelay[EPS_DEAD_TIME_MS] = { 0.0f };
int SteerRequestDelayIdx = 0;
float SteerWheelAngleOutputLast = 0.0f;
float SteerWheelAngleDelayedLast = 0.0f;

void initializeLowSpeedLateralDMC() {
    SteerRequestDelay[EPS_DEAD_TIME_MS] = { 0.0f };
    SteerRequestDelayIdx = 0;
    SteerWheelAngleOutputLast = 0.0f;
    SteerWheelAngleDelayedLast = 0.0f;
}

void resetLowSpeedLateralDMCDelay() {
    for (unsigned int i = 0; i < EPS_DEAD_TIME_MS; i++) {
        SteerRequestDelay[i] = 0.0f;
    }
}

void lowSpeedLateralDMC(
    float* SteerAngReqInput,
    float* SteerWheelAngleOutput,
    const bool SteerReqEnableIn,
    const float SteerWheelAngleReq,
    const float deltaT_s,
    const float currentSteerWheelAngle)
{
    float steerWheelAngleDelayed, steerWheelAngleFiltered;

    static bool SteerReqEnableIn_Last;

    //Determine delayed steering
    steerWheelAngleDelayed = SteerRequestDelay[SteerRequestDelayIdx];

    //Determine filtered steering (PT1)
    if (!SteerReqEnableIn_Last && SteerReqEnableIn)
    {
        steerWheelAngleFiltered = steerWheelAngleDelayed;
    }
    else
    {
        steerWheelAngleFiltered = (float)((steerWheelAngleDelayed*deltaT_s + SteerWheelAngleOutputLast * EPS_PT1_TIME_S) / (deltaT_s + EPS_PT1_TIME_S));
    }

    // Set outputs
    (*SteerWheelAngleOutput) = steerWheelAngleFiltered;

    //Hold last values
    SteerWheelAngleDelayedLast = steerWheelAngleDelayed;
    SteerWheelAngleOutputLast = steerWheelAngleFiltered;

    // Set new values for buffers
    (*SteerAngReqInput) = SteerWheelAngleReq;
    if (SteerReqEnableIn) {
        SteerRequestDelay[SteerRequestDelayIdx] = *SteerAngReqInput;
    }
    else {
        SteerRequestDelay[SteerRequestDelayIdx] = currentSteerWheelAngle;
    }

    // Increase buffer counter
    SteerRequestDelayIdx = (SteerRequestDelayIdx + 1) % EPS_DEAD_TIME_MS;

    SteerReqEnableIn_Last = SteerReqEnableIn;

    return;
}
