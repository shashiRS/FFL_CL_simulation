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

#ifndef MF_LSCA_CONFIG_BRAKE_T_C_H_
#define MF_LSCA_CONFIG_BRAKE_T_C_H_

#include "mf_lsca/brake_set_t_c.h"
#include "Platform_Types.h"
#include "cml/vec2_df_pod_c.h"
#include "eco/memset_c.h"

/// Struct that contains all relevant braking function parameter data
typedef struct
{
    ///Brake model related parameters
    MF_LSCA_brakeSet_t brakeModel;
    uint32 bodyAdjustmentActualSize_nu;
    ///Each body point can be adjusted for the brake function
    CML_Vec2Df_POD bodyAdjustment[16];
    uint32 protectedMirrorShapeLeftActualSize_nu;
    ///Per-point adjustment of the left mirror
    CML_Vec2Df_POD protectedMirrorShapeLeft[4];
    uint32 protectedMirrorShapeRightActualSize_nu;
    ///Per-point adjustment of the right mirror
    CML_Vec2Df_POD protectedMirrorShapeRight[4];
    ///Enlargement of ego shape to the front
    float32 enlargementFront_m;
    ///Enlargement of ego shape to the rear
    float32 enlargementRear_m;
    ///Enlargement of ego shape to the side
    float32 enlargementSide_m;
    ///Time that the warning shall occur before the braking at constant speed
    float32 warnTime_s;
    ///Minimum speed that needs to be exceeded for function activation - forwards
    float32 lowerActivationSpeedForwards_ms;
    ///Maximum speed that must not be exceeded for function activation - forwards
    float32 upperActivationSpeedForwards_ms;
    ///Minimum speed that needs to be exceeded for function to not become deactivated - forwards
    float32 lowerDeactivationSpeedForwards_ms;
    ///Minimum speed that must notbe exceeded for function to not become deactivated - forwards
    float32 upperDeactivationSpeedForwards_ms;
    ///Minimum (unsigned consideration) speed that needs to be exceeded for function activation - backwards
    float32 lowerActivationSpeedBackwards_ms;
    ///Maximum (unsigned consideration) speed that must not be exceeded for function activation - backwards
    float32 upperActivationSpeedBackwards_ms;
    ///Minimum (unsigned consideration) speed that needs to be exceeded for function to not become deactivated - backwards
    float32 lowerDeactivationSpeedBackwards_ms;
    ///Minimum (unsigned consideration) speed that must not be exceeded for function to not become deactivated - backwards
    float32 upperDeactivationSpeedBackwards_ms;
    ///Maximum distance that is ignored after override command
    float32 maximumOverrideDistance_m;
    ///Maximum distance that is ignored after override command
    float32 autoContinueOverrideDistance_m;
    ///Safety margin in seconds (safety distance along trajectory = current speed * this parameter)
    float32 marginDelay_s;
    ///Time that the vehicle must stay in standstill until it allows to proceed/releases the brake
    float32 standStillTime_s;
    ///Every object behind this point is ignored if a trailer is attached
    float32 ignoreTrailerX_m;
    ///Maximum allowed speed for driving over wheel traversanble objects - e.g. curbstone
    float32 rimProtectionSpeed_mps;
    ///Defines under what angle objects are considered as obstacles
    float32 rimProtectionAngle_deg;
    ///Minimum number of collision triggering before an EBA flag is propagated to the output
    uint16 minTriggerCountStaticBrake_nu;
    ///Minimum required height confidence to trust the classification
    uint8 minHeightConfidence_perc;
    ///Minimum required class confidence to trust the classification
    uint8 minClassConfidence_perc;
    ///Minimum required static detection probability for the function to react on an object
    uint8 minStaticObjectProbability_perc;
    ///Switch to determine if the body shall be protected against high objects
    boolean protectBody_nu;
    ///Switch to determine if the wheels shall be protected against body traversable objects
    boolean protectWheel_nu;
    ///Switch to determine if the rim shall be protected against speed-profile-dependent-wheel-traversable-objects
    boolean protectRim_nu;
    ///Switch to determine if the mirrors shall be protected against high objects
    boolean protectMirror_nu;
    ///Switch to determine if the hitch shall be protected against high objects
    boolean protectHitch_nu;
    ///Switch to determine if there shall be used a comfort braking in manual mode (AP mode is always EBA)
    boolean comfortInManual_nu;
    ///Switch to determine if an automatic continue or a continue screen shall be used in manual mode
    boolean autoContinue_nu;
    ///Warn before braking at high objects that are on collision course
    boolean warnHigh_nu;
    ///Warn before braking at low objects that are on collision course
    boolean warnLow_nu;
    ///Disable the final brake triggering for low objects but don"t disable the calculation process - used for warning only
    boolean enableBrakeLow_nu;
    ///Disable the final brake triggering for high objects but don"t disable the calculation process - used for warning only
    boolean enableBrakeHigh_nu;
    ///Use a complete driving tube calculation for collision detection (instead of just the AAB of the driving tube)
    boolean drivingTubeEnabled_nu;
    ///Disable braking function if a door is openedisable the final brake triggering for high objects but not disable the calculation process - used for warning only
    boolean checkDoors_nu;
    ///Disable braking function if the driver"s seatbelt is not fastened
    boolean checkDriverSeatbelt_nu;
    ///Disable braking function if the trunk is opened
    boolean checkTrunk_nu;
    ///If trunk is open keep braking function active in front of the car, only disable if moving backwards
    boolean forwardBrakeEnabledIfTrunkOpen_nu;
    ///Check if the engine hood is open or not
    boolean checkHood_nu;
    ///Check if stepping on the gas pedal deactivates LSCA
    boolean checkPedalOverrideGas_nu;
    ///Check if stepping on the brake pedal deactivates LSCA
    boolean checkPedalOverrideBrake_nu;
    ///Check if the drivier is in the vehicle
    boolean checkDriverSeatOccupied_nu;
} MF_LSCA_configBrake_t;

inline MF_LSCA_configBrake_t create_MF_LSCA_configBrake_t(void)
{
  MF_LSCA_configBrake_t m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.brakeModel = create_MF_LSCA_brakeSet_t();
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.bodyAdjustment) / sizeof(m.bodyAdjustment[0])); ++i)
    {
      m.bodyAdjustment[i] = create_CML_Vec2Df_POD();
    }
  }
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.protectedMirrorShapeLeft) / sizeof(m.protectedMirrorShapeLeft[0])); ++i)
    {
      m.protectedMirrorShapeLeft[i] = create_CML_Vec2Df_POD();
    }
  }
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.protectedMirrorShapeRight) / sizeof(m.protectedMirrorShapeRight[0])); ++i)
    {
      m.protectedMirrorShapeRight[i] = create_CML_Vec2Df_POD();
    }
  }
  return m;
}

#endif // MF_LSCA_CONFIG_BRAKE_T_C_H_