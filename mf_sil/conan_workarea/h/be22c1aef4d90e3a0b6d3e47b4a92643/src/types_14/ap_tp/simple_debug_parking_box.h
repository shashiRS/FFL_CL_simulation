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

#ifndef AP_TP_SIMPLE_DEBUG_PARKING_BOX_H_
#define AP_TP_SIMPLE_DEBUG_PARKING_BOX_H_

#include "Platform_Types.h"


namespace ap_tp
{

  /// 
  struct SimpleDebugParkingBox
  {
    ///@range{0,255}
    ///@unit{Count}
    ///Number of valid points in parking box @min: 0 @max: 255 @unit: Count
    uint8 numValidPoints_nu{};
    ///@unit{m}
    ///Parking Box vertex: x-Position @min: 0 @max: 0 @unit: m
    float32 posX_m[4]{};
    ///@unit{m}
    ///Parking Box vertex: y-Position @min: 0 @max: 0 @unit: m
    float32 posY_m[4]{};
  };

} // namespace ap_tp

#endif // AP_TP_SIMPLE_DEBUG_PARKING_BOX_H_
