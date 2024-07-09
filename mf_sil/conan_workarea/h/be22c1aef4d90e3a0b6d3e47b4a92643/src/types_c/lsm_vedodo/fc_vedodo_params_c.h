//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef LSM_VEDODO_FC_VEDODO_PARAMS_C_H_
#define LSM_VEDODO_FC_VEDODO_PARAMS_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// parameters of VEDODO function
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///Odometry calling update time rate.
    float32 LSM_O_SAMPLE_TIME_S;
    ///Time for standstill detection (motionStatus_nu changes from 1 to 0 if no wheel ticks are measured for the defined time
    float32 LSM_O_MSE_MOTION_STATUS_TIMER_S;
    ///Maximum steering wheel angle velocity for yaw rate offset estimation
    float32 LSM_O_YOC_MAX_STRWHL_VEL_RADPS;
    ///Define maximum wheel counter value (e.g. Passat B8: 1000).
    uint16 LSM_O_MAX_WHEEL_PULSE_VALUE_NU;
    ///Limitation of x position (min: -LSM_O_MAX_POS_X_VALUE_M max: LSM_O_MAX_POS_X_VALUE_M)
    float32 LSM_O_MAX_POS_X_VALUE_M;
    ///Limitation of y position (min: -LSM_O_MAX_POS_Y_VALUE_M max: LSM_O_MAX_POS_Y_VALUE_M)
    float32 LSM_O_MAX_POS_Y_VALUE_M;
    ///Limitation of heading angle (min: -LSM_O_MAX_POS_PHI_VALUE_RAD max: LSM_O_MAX_POS_PHI_VALUE_RAD)
    float32 LSM_O_MAX_POS_PHI_VALUE_RAD;
    ///Filter steer wheel angle: 0 = Filter ON 1 = Filter OFF
    uint8 LSM_O_USE_STEER_WHEEL_ANG_FILT;
    ///Determine influence of steering on yaw angle at standstill (using calculated Motion Status) 0 = Only use yaw rate input 1 = Use steering wheel angle in standstill situation to adapt yaw angle change
    uint8 LSM_O_USE_YAW_STANDSTILL_CORR;
    ///Select source of driving direction: 0 = Use driving direction from gear 1 = Use driving direction from rear axis 2 = Use driving direction from front axis 3 = USE_DRIVING_DIRECTION_FROM_FOUR_WHEELS, 4 = Use driving direction from accelerometer 5 = Use driving direction from last pulse 6 = Use driving direction from encoder, accelerometer and yawrate-steerangle
    uint8 LSM_O_DRIVING_DIRECTION_METHOD;
    ///Latency of input signals. Zero turns the latency compensation of
    float32 LSM_O_SIGNAL_LATENCY_S;
    ///This is a threshold for the deviation between the rear wheel speeds which is used as an indication of moving straight forward. The more this value increased the more tolerance we have for indicating straight motion.
    float32 LSM_O_WS_STRAIGHT_THRESH_MPS;
    ///Correction factor for gyro yaw rate linear error.
    float32 LSM_O_GYRO_LINEAR_ERR_FACTOR;
    ///Additional signal latency of yaw rate sensor
    float32 LSM_O_YAW_RATE_SIGNAL_LATENCY_S;
    ///Vehicle wheelbase. "copied from vehicle params"
    float32 LSM_O_WHEELBASE_M;
    ///Number of teeth used to measure wheel pulses. "copied from vehicle params"
    uint8 LSM_O_WHEEL_NUMBER_OF_TEETH_NU;
    ///Circumference of front wheels. "copied from vehicle params"
    float32 LSM_O_TYRE_CIRCUMFERENCE_FR_M;
    ///Circumference of rear wheels. "copied from vehicle params"
    float32 LSM_O_TYRE_CIRCUMFERENCE_RE_M;
    ///Track at front axle. "copied from vehicle params"
    float32 LSM_O_TRACK_FRONT_M;
    ///Track at rear axle. "copied from vehicle params"
    float32 LSM_O_TRACK_REAR_M;
    ///Influence of front steering angle on yaw angle (vehicle at standstill). "copied from vehicle params"
    float32 LSM_O_STEER_ANG_TO_YAW_ANG_NU;
    ///3nd deg polynomial coefficients of characteristic curve steergain "virtual" center wheel. "copied from vehicle params"
    float32 LSM_O_STEER_POLY_CTR_WHL_RAD[4];
    ///tire circumference value used by the brake system to compute the wheel velocity from angular to translation value
    float32 LSM_O_TYRE_CIRCUMFERENCE_IN_ESC;
    ///Rear Axle to center of rotation option 1
    float32 LSM_O_REAR_AXLE_TO_COR_1_M;
    ///Rear Axle to center of rotation option 2
    float32 LSM_O_REAR_AXLE_TO_COR_2_M;
    ///Rear Axle to center of rotation option 3
    float32 LSM_O_REAR_AXLE_TO_COR_3_M;
    ///Side slip angle gradient (unit:rad*s^2/m)
    float32 LSM_O_SIDE_SLIP_ANGLE_GRADIENT_RADS2PM;
    ///Imu mounting position translation from center to rear along x,y,z
    float32 LSM_O_IMU_MOUNTING_POS_TRANSLATION_M[3];
    ///Imu mounting position rotation around the vehicle coordinate system axis x,y,z
    float32 LSM_O_IMU_MOUNTING_POS_ROTATION_RAD[3];
} LSM_VEDODO_FC_VEDODO_Params;

inline LSM_VEDODO_FC_VEDODO_Params create_LSM_VEDODO_FC_VEDODO_Params(void)
{
  LSM_VEDODO_FC_VEDODO_Params m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // LSM_VEDODO_FC_VEDODO_PARAMS_C_H_
