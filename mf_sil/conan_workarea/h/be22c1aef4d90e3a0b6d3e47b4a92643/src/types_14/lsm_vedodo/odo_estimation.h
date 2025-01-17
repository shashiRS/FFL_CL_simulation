// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef LSM_VEDODO_ODO_ESTIMATION_H_
#define LSM_VEDODO_ODO_ESTIMATION_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "lsm_vedodo/motion_state.h"
#include "lsm_vedodo/direction.h"


namespace lsm_vedodo
{

  /// Result of odometry estimation.
  struct OdoEstimation
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@unit{m/s}
    ///@range{-100,100}
    ///Value for the current vehicle velocity; positive: driving forward, negative: driving backward
    float32 longiVelocity_mps{};
    ///@unit{m/s}
    ///@range{-100,100}
    ///Value for the current vehicle lateral velocity
    float32 lateralVelocity_mps{};
    ///@unit{m/s2}
    ///@range{-12,12}
    ///Value for the current vehicle calibrated and slope compensated longitudinal acceleration; forward direction positive: accelerating, negative: decelerating.
    float32 longiAcceleration_mps2{};
    ///@unit{m/s2}
    ///@range{-12,12}
    ///Value for the current vehicle calibrated and slope compensated lateral acceleration; forward direction positive: left curve, negative: right curve.
    ///Vertical Acceleration to be zero on horizontal plane.
    float32 lateralAcceleration_mps2{};
    ///@unit{m/s2}
    ///@range{-12,12}
    ///Value for the current vehicle calibrated and slope compensated vertical acceleration; positive: when driving uphill, negative: when driving downhill.
    float32 verticalAcceleration_mps2{};
    ///@unit{m}
    ///@range{-COORDINATE_OVERFLOW_POSITION_M,+COORDINATE_OVERFLOW_POSITION_M}
    ///current x position of the middle of the rear axle in frozen/absolute coordinates.
    float32 xPosition_m{};
    ///@unit{m}
    ///@range{-COORDINATE_OVERFLOW_POSITION_M,+COORDINATE_OVERFLOW_POSITION_M}
    ///Current y position of the middle of the rear axle in frozen/absolute coordinates.
    float32 yPosition_m{};
    ///@unit{m/s}
    ///@range{-100,100}
    ///Value for the current vehicle velocitiy in the local coordinate system; positive: vehicle is moving towards greater position in x direction, negative towards smaller in x direction.
    float32 xVelocity_mps{};
    ///@unit{m/s}
    ///@range{-100,100}
    ///Value for the current vehicle velocitiy in the local coordinate system; positive: vehicle is moving towards greater position in y direction, negative towards smaller in y direction.
    float32 yVelocity_mps{};
    ///@unit{m2}
    ///@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M}
    ///Uncertainty of x position estimation
    float32 xPositionVar_m2{};
    ///@unit{m2}
    ///@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M}
    ///Uncertainty of y position estimation
    float32 yPositionVar_m2{};
    ///@unit{m2}
    ///@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M}
    ///Correlation of x-y position estimation.
    float32 xyPositionVar_m2{};
    ///@unit{m2}
    ///@range{0,COORDINATE_OVERFLOW_POSITION_M * COORDINATE_OVERFLOW_POSITION_M}
    ///Correlation of y-x position estimation.
    float32 yxPositionVar_m2{};
    ///@unit{rad}
    ///@range{-PI_RAD,+PI_RAD}
    ///current vehicle roll angle in absolute coordinates.
    float32 rollAngle_rad{};
    ///@unit{rad}
    ///@range{-PI_RAD,+PI_RAD}
    ///current vehicle pitch angle in absolute coordinates.
    float32 pitchAngle_rad{};
    ///@unit{rad}
    ///@range{-COORDINATE_OVERFLOW_YAW_ANGLE_RAD,+COORDINATE_OVERFLOW_YAW_ANGLE_RAD}
    ///Current vehicle yaw angle in absolute coordinates.
    float32 yawAngle_rad{};
    ///@unit{rad2}
    ///@range{0,COORDINATE_OVERFLOW_YAW_ANGLE_RAD * COORDINATE_OVERFLOW_YAW_ANGLE_RAD}
    ///Uncertainty of roll angle Estimation
    float32 rollAngleVar_rad2{};
    ///@unit{rad2}
    ///@range{0,COORDINATE_OVERFLOW_YAW_ANGLE_RAD * COORDINATE_OVERFLOW_YAW_ANGLE_RAD}
    ///Uncertainty of Pitch Angle Estimation
    float32 pitchAngleVar_rad2{};
    ///@unit{rad2}
    ///@range{0,COORDINATE_OVERFLOW_YAW_ANGLE_RAD * COORDINATE_OVERFLOW_YAW_ANGLE_RAD}
    ///Uncertainty of yaw angle estimation
    float32 yawAngleVar_rad2{};
    ///@unit{rad/s}
    ///@range{-PI_RAD,+PI_RAD}
    ///current vehicle roll rate in absolute coordinates.
    float32 rollRate_radps{};
    ///@unit{rad/s}
    ///@range{-PI_RAD,+PI_RAD}
    ///current vehicle pitch rate in absolute coordinates.
    float32 pitchRate_radps{};
    ///@unit{rad/s}
    ///@range{-PI_RAD,+PI_RAD}
    ///Current vehicle yaw rate
    float32 yawRate_radps{};
    ///@unit{rad}
    ///@range{-0.8,0.8}
    ///virtual centre steer angle at front axle.
    float32 steerAngFrontAxle_rad{};
    ///@unit{rad}
    ///@range{-0.8,0.8}
    ///virtual centre steer angle at rear axle.
    float32 steerAngRearAxle_rad{};
    ///@unit{rad}
    ///Wheel Angle Front Right
    float32 wheelAngleFR_rad{};
    ///@unit{rad}
    ///Wheel Angle Front Left
    float32 wheelAngleFL_rad{};
    ///@unit{rad}
    ///Side Slip Angle.
    float32 sideSlipAngle_rad{};
    ///@unit{rad}
    ///Suspension Pitch angle.
    float32 suspPitch_rad{};
    ///@unit{rad}
    ///Suspension Roll angle.
    float32 suspRoll_rad{};
    ///@unit{m}
    ///Suspension Height.
    float32 suspHeight_m{};
    ///@unit{m}
    ///@range{TBD,TBD}
    ///The accumulation of the absolute vehicle displacements.
    float32 drivenDistance_m{};
    ///@range{0,1}
    ///determined motion state
    MotionState motionStatus_nu{};
    ///@range{-1,1}
    ///The vehicle current driving direction.
    Direction drivingDirection_nu{};
  };

} // namespace lsm_vedodo

#endif // LSM_VEDODO_ODO_ESTIMATION_H_
