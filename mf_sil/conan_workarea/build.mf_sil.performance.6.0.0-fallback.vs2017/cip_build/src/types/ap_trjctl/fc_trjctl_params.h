// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef AP_TRJCTL_FC_TRJCTL_PARAMS_H_
#define AP_TRJCTL_FC_TRJCTL_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_trjctl/fc_trjctl_vehicle_params.h"
#include "ap_trjctl/fc_trjctl_sys_func_params.h"
#include "eco/memset.h"


namespace ap_trjctl
{

  struct FC_TRJCTL_Params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///SEMI_AP: Limit for passed time without any movement. If this value is reached the current stroke will be finished.
    float32 AP_C_MANEUV_FINISHED_TIME_DRV_S;
    ///FULL_AP: Define distance limit for intern calculation of "maneuveringFinished_nu".
    ///Remark: - Value needs to be aligned with LoDMC threshold to start maneuvering. This value needs to be higher than all LoDMC thresholds (e.g. normal/curb scenario).
    float32 AP_C_MANEUV_FINISHED_LIMIT_M;
    ///SEMI_AP: Lower velocity threshold for reset of maneuvering finished timer.
    float32 AP_C_MANEUV_V_THRESH_RESET_MPS;
    ///SEMI_AP: Lower distance threshold for reset of maneuvering finished timer.
    float32 AP_C_MANEUV_D_THRESH_RESET_M;
    ///FULL_AP: Limit for passed time without any movement. If this value is reached the current stroke will be finished.
    float32 AP_C_MANEUV_FINISHED_TIME_S;
    ///Number of control gains. (Describes the number of elements in velocity depending control gains.)
    uint8 AP_C_PC_NUM_CONTROL_GAINS_NU;
    ///Velocity vector for velocity depending control gains.
    float32 AP_C_PC_GAIN_VELVEC_MPS[6];
    ///Velocity depending control gains for yaw deviation.
    float32 AP_C_PC_GAIN_YAW_DEVIATION_NU[6];
    ///Velocity depending control gains for lateral deviation.
    float32 AP_C_PC_GAIN_LAT_DEVIATION_NU[6];
    ///Maximum allowed absolute deviation from calculated trajectory. Trajectory control stops car and requests replanning of trajectory inputs if this value is reached.
    float32 AP_C_PC_FAIL_MAX_LAT_ERROR_M;
    ///Maximum allowed absolute yaw angle (orientation) error from calculated trajectory. Trajectory control stops car and requests replanning of trajectory input if this value is reached.
    float32 AP_C_PC_FAIL_MAX_YAW_ERROR_RAD;
    ///Filter coefficients for steer angle request output.
    float32 AP_C_PC_STEER_ANGLE_T_FILT_S;
    ///Filter coefficients for calculated yaw deviation.
    float32 AP_C_PI_YAW_DEVIATION_T_FILT_S;
    ///Factor for steer angle filter coefficient. Used for low speed maneuvering to compensate discrete steps in
    ///odometry estimation. Between velocity 0 and AP_C_PC_FILT_FAC_VEL_LIMIT_MPS the resulting filter factor will be interpolated between this value and 1.
    float32 AP_C_PC_FILT_FACTOR_MAX_NU;
    ///Upper velocity limit for filter factor. (see AP_C_PC_FILT_FACTOR_MAX_NU)
    float32 AP_C_PC_FILT_FAC_VEL_LIMIT_MPS;
    ///Threshold for parallel parking in. If the current distance to stop of the last stroke is smaller than this value the end maneuver for parking in parallel will be activated. (end maneuver is orientation correction)
    float32 AP_C_PC_ORIENT_CTRL_END_DIST_M;
    ///Threshold for parallel parking in. If the initial distance to stop of the last stroke is smaller than this value the end maneuver for parking in parallel will be activated. (end maneuver is orientation correction)
    float32 AP_C_PC_ORIENT_CTRL_INIT_DIST_M;
    ///Interval limits (positive and negative) to reach end of first steering. (at steering wheel)
    float32 AP_C_PC_FIRST_STEER_ACCUR_RAD;
    ///Maximum steer angle velocity for ramping steer angle to desired value for first steering (at wheels)
    float32 AP_C_PC_FIRST_STEER_VEL_RADPS;
    ///Maximum steer angle acceleration for ramping steer angle to desired value for first steering (at wheels)
    float32 AP_C_PC_FIRST_STEER_ACC_RADPS2;
    ///Factor to manipulate preview length (factor*v*T_Filter)
    float32 AP_C_PC_CURV_PREVIEW_FACTOR_NU;
    ///Minimum preview length to cover drive off situations (velocity changes from zero to non-zero value).
    float32 AP_C_PC_CURV_PREVIEW_MIN_M;
    ///Limit for passed time with secured vehicle standstill until information will be forwarded to status port
    float32 AP_C_SECURE_FINISHED_TIME_S;
    ///Activation/Deactivation of steer intervention detection.
    boolean AP_C_STEER_INTERV_ACTIVE_NU;
    ///Filter coefficient for steering intervention by driver.
    float32 AP_C_STEER_INTERV_FILT_TIME_S;
    ///Schmitt-Trigger rising threshold for steering intervention by driver.
    float32 AP_C_STEER_INTERV_RISE_NU;
    ///Schmitt-Trigger falling threshold for steering intervention by driver.
    float32 AP_C_STEER_INTERV_FALL_NU;
    ///Factor for steer angle filter coefficient. Used for path control to compensate discontinuities in trajectory. Factor will be multiplied if discontinuity at current trajectory point occurs.
    float32 AP_C_PC_FILT_FAC_TRAJ_STEP_NU;
    ///Minimum time interval for active control. Maneuvering finished flag will be accepted when this time was passed.
    float32 AP_C_ACTIVE_CONTROL_MIN_TIME_S;
    ///Flag to activate or deactivate the ramp up feature for limitation of velocity limit request. In case of rising unsteadiness in velocity limit request during maneuvering the acceleration will be limited by a ramped velocity limit.
    boolean AP_C_VL_RAMP_UP_VEL_NU;
    ///Maximum acceleration used to ramp up the velocity limit request.
    float32 AP_C_VL_VEL_RAMP_LIMIT_MPS2;
    ///Flag to activate or deactivate the ramp up feature for limitation of distance to stop. In case of rising unsteadiness in distance to stop request during maneuvering the acceleration will be limited by a ramped distance to stop based on current ego velocity and AP_C_MIN_PARKING_VEL_MPS.
    boolean AP_C_VL_RAMP_UP_DIST_NU;
    ///Minimum parking velocity.
    float32 AP_C_MIN_PARKING_VEL_MPS;
    ///Minimum steer angle velocity for ramping steer angle to desired value for first steering (at wheels)
    float32 AP_C_PC_MIN_STEER_VEL_RADPS;
    ///Temp: Boolean to set test data in driving resistance interface.
    boolean AP_C_DRV_RESIST_FAKE_DATA_NU;
    ///Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    uint8 AP_C_DRV_RESIST_FL_TYPE_NU;
    ///Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    float32 AP_C_DRV_RESIST_FL_RED_DIST_M;
    ///Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    uint8 AP_C_DRV_RESIST_RL_TYPE_NU;
    ///Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    float32 AP_C_DRV_RESIST_RL_RED_DIST_M;
    ///Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    uint8 AP_C_DRV_RESIST_RR_TYPE_NU;
    ///Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    float32 AP_C_DRV_RESIST_RR_RED_DIST_M;
    ///Temp: Type of driving resistance send to LoDMC. (00: NONE, 01: FALLING_LOW, 02: FALLING_MEDIUM, 03: FALLING_HIGH, 04: RISING_LOW, 05: RISING_MEDIUM, 06: RISING_HIGH, 07: WHEEL_STOPPER)
    uint8 AP_C_DRV_RESIST_FR_TYPE_NU;
    ///Temp: Reduce distance to stop for driving resistance information sent to LoDMC by this parameter. (only for first stroke) (e.g. if distance to stop of first stroke is 5 meter and distance to this curb is 4 meter set value to 1m)
    float32 AP_C_DRV_RESIST_FR_RED_DIST_M;
    ///Threshold to maximum steer angle to use a reduced steer angle rate. This will reduce slope of ramp into steer angle limitation. (Warning: This might influence the control performance.)
    float32 AP_C_STEER_SATURATE_THRESH_RAD;
    ///Maximum steering angle velocity (at wheels) in case of steer angle request in saturation threshold.  (Warning: This might influence the control performance.)
    float32 AP_C_STEER_SATURATE_RATE_RADPS;
    ///Maximum steer angle acceleration for increasing steer angle velocity (at wheels)
    float32 AP_C_PC_MAX_STEER_ACC_RADPS2;
    ///Factor to reduce rate limit in path control output. Ensures overall rate limit and acceleration limitation.
    float32 AP_C_PC_RATE_LIMIT_FACTOR_NU;
    ///Add this value to AP_C_PC_CURV_PREVIEW_FACTOR_NU to compensate curvature step in trajectory. (factor*v*T_Filter)
    float32 AP_C_PC_CURV_PREV_FACTOR_ADD_NU;
    ///Flag to activate the reduced preview for curvature. If activated a check will be performed whether an curvature step is in planned trajectory. If yes, the extended preview length will be used. If no curvature step was detected the preview length will be reduced.
    boolean AP_C_PC_CURV_PREV_REDUCED_NU;
    ///Maximum allowed longitudinal overshoot from calculated trajectory. Trajectory control demands replanning in Trajectory Planning if value is reached. Trajectory control demands emergency hold if value is reached.
    float32 AP_C_FAIL_MAX_LONG_OVERSHOOT_M;
    ///Factor that is used to compensate tire contact area deformation during lateral control mode LACTRL_COMF_ANGLE_ADJUSTMENT. (e.g. if standstill steering starts at -400deg and should steer to 0deg the difference 400deg is multiplied with this factor to steer to a
    ///positive value to avoid that the tire contact area deformation causes and a value smaller than 0deg when the steering system is deactivated)
    float32 AP_C_COMP_TIRE_DEF_FACTOR_NU;
    ///Additional hysteresis distance threshold to keep maneuvering finished indicated. (e.g. to cover odometry drift)
    float32 AP_C_MANEUV_FINISHED_HYST_M;
    ///Minimum length in case of distance control (distance, distance-velocity, trajectory) to start/request maneuvering
    float32 AP_C_MIN_DIST_REQ_M;
    ///Activate feature to wait for contact (e.g. in case of a wheel stopper detected). Force an overshoot of the stop point and wait for contact to finish the control.
    boolean AP_C_FEAT_WAIT_FOR_CONTACT_NU;
    ///Overshoot length for wait for contact feature.
    float32 AP_C_WFC_OVERSHOOT_LENGTH_M;
    ///Distance threshold for wait for contact feature distance overshoot. (Below this threshold the distance to stop send to LoDMC will be manipulated; AP_C_WFC_OVERSHOOT_LENGTH_M will be added)
    float32 AP_C_WFC_OVERSHOOT_DIST_THRES_M;
    ///Distance threshold for wait for contact feature VDY based detection of wheelstopper. (If the smallest distance to any wheel stopper is smaller than this threshold the VDY based wheel stopper detection will be activated.)
    float32 AP_C_WFC_VDY_DIST_THRES_M;
    ///Distance threshold for wait for contact feature VDY based detection of wheelstopper. (To ensure no false positive detection during drive off the feature is deactivated at the beginning of a stroke.)
    float32 AP_C_WFC_VDY_DRIVE_OFF_THRES_M;
    ///Activate feature to reduce target velocity in case of a close wheel stopper information.
    boolean AP_C_FEAT_WS_VEL_REDUCED_NU;
    ///Distance threshold for reduced target velocity in case of a close wheelstopper. (below this distance threshold of the minimum distance to any wheel stopper the velocity limit will be reduced)
    float32 AP_C_WFC_WS_VEL_DIST_THRESH_M;
    ///Velocity limit for the reduced target velocity in case of a close wheelstopper.
    float32 AP_C_WFC_WS_VEL_LIMIT_MPS;
    ///HACK: Allow that for last stroke mf_control internally sets a wheel stopper information and distance equal to the distance to stop.
    boolean AP_C_HACK_WS_LAST_STROKE_NU;
    ///Longitudinal distance threshold between front wheels and wheel stopper to deactivate comfortable standstill steering (e.g. after parking maneuver).
    float32 AP_C_NO_COMF_STEER_WS_THRES_M;
    ///Distance threshold to detect drive off was realized by underlaid control. If the passed distance gets above this limit the drive off is interpreted as done.
    float32 AP_C_DRIVE_OFF_DISTANCE_THRES_M;
    ///Velocity threshold to detect drive off was realized by underlaid control. If the passed velocity gets above this limit the drive off is interpreted as done.
    float32 AP_C_DRIVE_OFF_VELO_THRES_MPS;
    ///Handshake wait time to detect failure in handshake . If the wait time gets above this limit it should be reported as failure.
    float32 AP_C_HANDSHAKE_WAIT_THRES_TIME_S;
    ///Preview time of velocity request to compensate delay in underlaid longitudinal control chain.
    float32 AP_C_PC_VELO_PREVIEW_TIME_S;
    ///Threshold to detect that the ego position is in a critical distance outside the planned path (before first path point).
    float32 AP_C_LEAVING_PATH_BEFORE_M;
    ///Threshold to detect that the ego position is in a critical distance outside the planned path (behind last path point).
    float32 AP_C_LEAVING_PATH_BEHIND_M;
    ///Nested struct for vehicle params section
    FC_TRJCTL_Vehicle_Params vehicleParams;
    ///Nested struct for sys func params section
    FC_TRJCTL_Sys_Func_Params sysFuncParams;
  };

  inline ::ap_trjctl::FC_TRJCTL_Params createFC_TRJCTL_Params()
  {
    FC_TRJCTL_Params m;
    (void)::eco::memset(&m, 0U, sizeof(FC_TRJCTL_Params));
    m.sSigHeader = ::eco::createSignalHeader();
    m.vehicleParams = createFC_TRJCTL_Vehicle_Params();
    m.sysFuncParams = createFC_TRJCTL_Sys_Func_Params();
    return m;
  }

} // namespace ap_trjctl

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_trjctl::FC_TRJCTL_Params create_default()
  {
      return ::ap_trjctl::createFC_TRJCTL_Params();
  }
}


#endif // AP_TRJCTL_FC_TRJCTL_PARAMS_H_
