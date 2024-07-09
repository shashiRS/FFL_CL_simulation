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

#ifndef AP_TP_PLANNED_GEOMETRIC_PATH_OUTPUT_H_
#define AP_TP_PLANNED_GEOMETRIC_PATH_OUTPUT_H_

#include "Platform_Types.h"


namespace ap_tp
{

  /// 
  struct PlannedGeometricPathOutput
  {
    ///@unit{nu}
    ///start pose of the segment @min: 0 @max: 0 @unit: nu
    float32 startPose[3]{};
    ///@unit{nu}
    ///end pose of the segment @min: 0 @max: 0 @unit: nu
    float32 endPose[3]{};
    ///@unit{m}
    ///number of valid positions in the planned path @min: 0 @max: inf @unit: m
    float32 turnRadius_m{};
    ///@unit{m}
    ///number of valid positions in the planned path @min: 0 @max: inf @unit: m
    float32 turnRadiusSecond_m{};
    ///@range{0,3}
    ///@unit{---}
    ///number of valid positions in the planned path @min: 0 @max: 3 @unit: ---
    uint8 drvDir{};
    ///@range{0,6}
    ///@unit{---}
    ///number of valid positions in the planned path @min: 0 @max: 6 @unit: ---
    uint8 steerDir{};
    ///@range{0,2.778}
    ///@unit{m/s}
    ///number of valid positions in the planned path @min: 0 @max: AP_G_MAX_AVG_V_MPS = 10kph = 2.778 m/s @unit: m/s
    float32 longVel_mps{};
    ///@range{0,20}
    ///@unit{m}
    ///number of valid positions in the planned path @min: 0 @max: AP_G_MAX_LENGHT_STROKE_M = 20 @unit: m
    float32 length_m{};
    ///@range{-500,500}
    ///@unit{m}
    ///number of valid positions in the planned path @min: -500 @max: 500 @unit: m
    float32 rotationCenter_m[2]{};
    ///@range{-500,500}
    ///@unit{m}
    ///number of valid positions in the planned path @min: -500 @max: 500 @unit: m
    float32 rotationCenterSecond_m[2]{};
    ///@range{0,26}
    ///@unit{---}
    ///number of valid positions in the planned path @min: 0 @max: 26 @unit: ---
    uint8 planPhase{};
  };

} // namespace ap_tp

#endif // AP_TP_PLANNED_GEOMETRIC_PATH_OUTPUT_H_
