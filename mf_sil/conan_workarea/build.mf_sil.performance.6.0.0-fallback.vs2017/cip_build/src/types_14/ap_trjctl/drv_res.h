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

#ifndef AP_TRJCTL_DRV_RES_H_
#define AP_TRJCTL_DRV_RES_H_

#include "Platform_Types.h"
#include "ap_trjctl/drv_res_type.h"


namespace ap_trjctl
{

  /// Wheel specific driving resistance to consider for example crossing curbstone and wheel stopper scenarios
  struct DrvRes
  {
    ///@unit{m}
    ///@range{0,10}
    ///Distance to the wheel individual driving resistance based on the intended movement of the rear axle center.
    float32 distance_m{};
    ///@range{0,8}
    ///Type of the driving resistance.
    DrvResType type_nu{};
  };

} // namespace ap_trjctl

#endif // AP_TRJCTL_DRV_RES_H_