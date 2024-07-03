/**
    Convertion of CHASE/CarMaker/BrakeSystem/EBS/Ramped_EBS/LowSpeedLongDMC to C
*/

#include <cmath>

#define EBS_DEAD_TIME 1
#define Ramped_EBS_cfg__Delay_EPB 1000
#define Ramped_EBS_cfg__HoldLimit 0.15f
#define Ramped_EBS_cfg__minAccel -1.4f
#define Ramped_EBS_cfg__Kp_v_ctrl 0.234f
#define Ramped_EBS_cfg__Kp_v_ctrl_Semi (0.234f*6.5f)//(0.234f*7.1)
#define MIN_FLT_DIVISOR 0.000001f
#define Ramped_EBS_cfg__maxJerk 3.0f
#define Ramped_EBS_cfg__minJerk -1.0f
#define Ramped_EBS_cfg__maxAccel 2.0f


float EgoVelDelay[EBS_DEAD_TIME] = { 0.0f };
bool FirstCallFlag = false;
float InitialSpeed = 0;
float InitialDistance = 0;
int EgoVelDelayIdx = 0;
float SecureReqDelay[Ramped_EBS_cfg__Delay_EPB] = { 0.0f };
int SecureReqDelayIdx = 0;
float HoldReqDelay[EBS_DEAD_TIME] = { 0.0f };
int HoldReqDelayIdx = 0;
int EmergencyStopDelay[EBS_DEAD_TIME] = { 0 };
int EmergencyStopDelayIdx = 0;

const float lookUpData[12] = { 0.0f, 2.0f, 3.6f, 5.0f, 6.2f, 7.4f, 8.1f, 9.0f, 9.4f, 9.8f, 10.0f, 10.0f };
const float lookUpBreakpoints[12] = { 0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 15.0f };

float lastAccelReqRateLim = 0.0f;

void resetLowSpeedLongDMC(){
    for(unsigned int i = 0; i < EBS_DEAD_TIME; i++){
        EgoVelDelay[i]          = 0.0f;
        HoldReqDelay[i]         = 0.0f;
        EmergencyStopDelay[i]   = 0;
    }
    EgoVelDelayIdx        = 0;
    HoldReqDelayIdx       = 0;
    EmergencyStopDelayIdx = 0;
    for(unsigned int i = 0; i < Ramped_EBS_cfg__Delay_EPB; i++){
        SecureReqDelay[i] = 0.0f;
    }
    SecureReqDelayIdx = 0;
    lastAccelReqRateLim = 0.0f;
}

float distToVelLookup(float dist) {
    float vel = 0.0f;
    int foundThreshold = 0;
    int i;
    for (i = 0; i < 12; i++) {
        if (lookUpBreakpoints[i] > dist) {
            foundThreshold = 1;
            break;
        }
    }
    if (foundThreshold && i == 0) {
        vel = lookUpData[0];
    }
    else if (foundThreshold) {
        // define line ( y = a * x + b) for linear interpolation by using points at i-1 and i from lookup
        float y1 = lookUpData[i - 1];
        float y2 = lookUpData[i];
        float x1 = lookUpBreakpoints[i - 1];
        float x2 = lookUpBreakpoints[i];
        float divisorForA = x2 - x1;
        if ((divisorForA < MIN_FLT_DIVISOR) && (divisorForA > -MIN_FLT_DIVISOR)) {
            return 0.0f;
        }
        float a = (y2 - y1) / divisorForA;
        float b = y1 - a * x1;

        // Get output value
        vel = a * dist + b;
    }
    else {
        vel = lookUpData[11];
    }

    return vel;
}

float rateLimiter(float input, float lastOutput, float deltaT_s, float risingSlewRate, float fallingSlewRate) {
    float output = 0.0f;
    if (deltaT_s < MIN_FLT_DIVISOR && deltaT_s > -MIN_FLT_DIVISOR) {
        output = lastOutput;
    }
    else {
        float rate = (input - lastOutput) / deltaT_s;
        if (rate > risingSlewRate) {
            output = deltaT_s * risingSlewRate + lastOutput;
        }
        else if (rate < fallingSlewRate) {
            output = deltaT_s * fallingSlewRate + lastOutput;
        }
        else {
            output = input;
        }
    }
    
    return output;
}

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
    const float deltaT_s)
{
    // Set booleans
    bool distReqHoldBool = (DistReq_m < Ramped_EBS_cfg__HoldLimit);
    bool standStillHoldBool = (EgoVelDelay[EgoVelDelayIdx] < Ramped_EBS_cfg__HoldLimit);
    bool brakeEnableBool = (bool)((int)SecureReqDelay[SecureReqDelayIdx] || (int)HoldReqDelay[HoldReqDelayIdx] || EmergencyStopDelay[EmergencyStopDelayIdx]);
    
    // Calculate acceleration
    float AccelReqOut = 0.0f;
    bool AccelReqEnable = AccelReqEnableIn;
    float velReqMin = 0.0f;
    if (brakeEnableBool) {
        AccelReqOut = 0.0f;
        AccelReqEnable = false;
    }
    else {
        float velFromDist = distToVelLookup(DistReq_m);
        velReqMin = velFromDist;
        if (VelReq * 3.6f < velFromDist) {
            velReqMin = VelReq * 3.6f;
        }
        float velDiff = velReqMin - fabsf(EgoVel) * 3.6f;
        float accelReq = velDiff * Ramped_EBS_cfg__Kp_v_ctrl;
        accelReq = rateLimiter(accelReq, lastAccelReqRateLim, deltaT_s, Ramped_EBS_cfg__maxJerk, Ramped_EBS_cfg__minJerk);
        lastAccelReqRateLim = accelReq;
        if (accelReq > Ramped_EBS_cfg__maxAccel) {
            accelReq = Ramped_EBS_cfg__maxAccel;
        }
        else if (accelReq < Ramped_EBS_cfg__minAccel) {
            accelReq = Ramped_EBS_cfg__minAccel;
        }
        AccelReqOut = accelReq * AccelReqEnableIn;
    }

    // Set outputs
    (*BrakeReq_mps2) = -Ramped_EBS_cfg__minAccel * (float)brakeEnableBool;
    (*BrakeReqEnableOut) = brakeEnableBool;
    (*AccelReq_mps2) = AccelReqOut;
    (*AccelReqEnableOut) = AccelReqEnable;
    (*standStillHold) = (bool)(standStillHoldBool);
    (*standStillSecure) = (SecureReqDelay[SecureReqDelayIdx] > 0.0f );
    (*maneuveringFinished) = (bool)(standStillHoldBool && distReqHoldBool);
    (*velocityRequestIntern) = fmaxf(0.0f,velReqMin/3.6f);

    // Set new values for buffers
    SecureReqDelay[SecureReqDelayIdx] = SecureReq;
    EgoVelDelay[EgoVelDelayIdx] = fabsf(EgoVel);
    HoldReqDelay[HoldReqDelayIdx] = HoldReq;
    EmergencyStopDelay[EmergencyStopDelayIdx] = (VelReq*3.6f < 0.01f && DistReq_m < 0.01f) || EmergencyHoldReq;

    // Increase buffer counter
    SecureReqDelayIdx = (SecureReqDelayIdx + 1) % Ramped_EBS_cfg__Delay_EPB;
    EgoVelDelayIdx = (SecureReqDelayIdx + 1) % EBS_DEAD_TIME;
    HoldReqDelayIdx = (HoldReqDelayIdx + 1) % EBS_DEAD_TIME;
    EmergencyStopDelayIdx = (EmergencyStopDelayIdx + 1) % EBS_DEAD_TIME;
    
    return;
}

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
    const float EgoVel)
{
    // Set booleans
    bool distReqHoldBool = (DistReq_m < Ramped_EBS_cfg__HoldLimit);
    bool standStillHoldBool = (EgoVelDelay[EgoVelDelayIdx] < Ramped_EBS_cfg__HoldLimit);
    bool brakeEnableBool = (bool)((int)SecureReqDelay[SecureReqDelayIdx] || (int)HoldReqDelay[HoldReqDelayIdx] || EmergencyStopDelay[EmergencyStopDelayIdx]);
    float velFromDist;
    float velDiff;
    // Calculate acceleration
    float AccelReqOut = 0.0f;
    bool AccelReqEnable = AccelReqEnableIn;
    float velReqMin = 0.0f;
    if (brakeEnableBool) {
        AccelReqOut = 0.0f;
        AccelReqEnable = false;
    }
    else {
        if (FirstCallFlag == false)
        {
            InitialSpeed = fabsf(EgoVel);
            InitialDistance = DistReq_m;
            FirstCallFlag = true;
        }
        // straight line equation for (x1 , y1) - initial speed / distance to stop and (x2 , y2) 0 and respectively 0
        velFromDist = 3.6f * ((DistReq_m - InitialDistance)*(0 - InitialSpeed) / (0 - InitialDistance) + InitialSpeed);
        if (velFromDist < 0) {
            velFromDist = 0;
        }
        velReqMin = velFromDist;
        if (VelReq * 3.6f < velFromDist) {
            velReqMin = VelReq * 3.6f;
        }
        velDiff = velReqMin - fabsf(EgoVel) * 3.6f;
        float accelReq = velDiff * Ramped_EBS_cfg__Kp_v_ctrl_Semi;
        // to check: rate limiter could be added here
        AccelReqOut = accelReq * AccelReqEnableIn;
    }

    // Set outputs
    (*BrakeReq_mps2) = -Ramped_EBS_cfg__minAccel * (float)brakeEnableBool;
    (*BrakeReqEnableOut) = brakeEnableBool;
    (*AccelReq_mps2) = AccelReqOut;
    (*AccelReqEnableOut) = AccelReqEnable;
    (*standStillHold) = (bool)(standStillHoldBool);
    (*standStillSecure) = (SecureReqDelay[SecureReqDelayIdx] > 0.0f);
    (*maneuveringFinished) = (bool)(standStillHoldBool && distReqHoldBool);
    (*velocityRequestIntern) = fmaxf(0.0f, velReqMin / 3.6f);

    // Set new values for buffers
    SecureReqDelay[SecureReqDelayIdx] = SecureReq;
    EgoVelDelay[EgoVelDelayIdx] = fabsf(EgoVel);
    HoldReqDelay[HoldReqDelayIdx] = HoldReq;
    EmergencyStopDelay[EmergencyStopDelayIdx] = (VelReq*3.6f < 0.01f && DistReq_m < 0.01f) || EmergencyHoldReq;

    // Increase buffer counter
    SecureReqDelayIdx = (SecureReqDelayIdx + 1) % Ramped_EBS_cfg__Delay_EPB;
    EgoVelDelayIdx = (SecureReqDelayIdx + 1) % EBS_DEAD_TIME;
    HoldReqDelayIdx = (HoldReqDelayIdx + 1) % EBS_DEAD_TIME;
    EmergencyStopDelayIdx = (EmergencyStopDelayIdx + 1) % EBS_DEAD_TIME;

    return;
}
