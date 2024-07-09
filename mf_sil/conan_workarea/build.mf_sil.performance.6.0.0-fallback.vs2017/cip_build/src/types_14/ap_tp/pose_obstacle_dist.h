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

#ifndef AP_TP_POSE_OBSTACLE_DIST_H_
#define AP_TP_POSE_OBSTACLE_DIST_H_

#include "Platform_Types.h"


namespace ap_tp
{

  /// 
  struct PoseObstacleDist
  {
    ///@range{0,10}
    ///@unit{m}
    ///Distance to the closest obstacle perpendicular to the vehicle front side. @min: 0 @max: 10.0 @unit: m
    float32 frontObstDist_m{};
    ///@unit{boolean}
    ///@range{0,1}
    ///Flag if the distance to front side obstacle is valid. @min: 0 @max: 1 @unit: boolean
    boolean frontObstDistValid{};
    ///@range{0,10}
    ///@unit{m}
    ///Distance to the closest obstacle perpendicular to the vehicle rear side. @min: 0 @max: 10.0 @unit: m
    float32 rearObstDist_m{};
    ///@unit{boolean}
    ///@range{0,1}
    ///Flag if the distance to rear side obstacle is valid. @min: 0 @max: 1 @unit: boolean
    boolean rearObstDistValid{};
    ///@range{0,10}
    ///@unit{m}
    ///Distance to the closest obstacle perpendicular to the vehicle left side. @min: 0 @max: 10.0 @unit: m
    float32 leftObstDist_m{};
    ///@unit{boolean}
    ///@range{0,1}
    ///Flag if the distance to left side obstacle is valid. @min: 0 @max: 1 @unit: boolean
    boolean leftObstDistValid{};
    ///@range{0,10}
    ///@unit{m}
    ///Distance to the closest obstacle perpendicular to the vehicle right side. @min: 0 @max: 10.0 @unit: m
    float32 rightObstDist_m{};
    ///@unit{boolean}
    ///@range{0,1}
    ///Flag if the distance to right side obstacle is valid. @min: 0 @max: 1 @unit: boolean
    boolean rightObstDistValid{};
  };

} // namespace ap_tp

#endif // AP_TP_POSE_OBSTACLE_DIST_H_
