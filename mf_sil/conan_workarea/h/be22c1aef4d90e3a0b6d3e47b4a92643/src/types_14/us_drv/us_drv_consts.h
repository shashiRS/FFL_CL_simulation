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

#ifndef US_DRV_US_DRV_CONSTS_H_
#define US_DRV_US_DRV_CONSTS_H_

#include "Platform_Types.h"


namespace us_drv
{

  struct US_DRV_Consts
  {
    static constexpr uint8 US_DRV_MAX_NUM_ASIC_COMMAND_WORDS = 64U;
    static constexpr uint8 US_DRV_MAX_NUM_SENSORS = 18U;
    static constexpr uint8 US_DRV_MAX_NUM_STOCHASTIC_CODES = 8U;
    static constexpr uint16 US_DRV_MAX_NUM_DETECTIONS = 1800U;
    static constexpr uint8 US_DRV_MAX_NUM_SIGNAL_PATHS = 32U;
    static constexpr uint32 US_DRV_MAX_NUM_SAMPLES = 4096U;
    static constexpr uint8 US_DRV_MAX_NUM_ASICS = 2U;
    static constexpr uint8 US_DRV_MAX_NUM_DSI_CHANNELS = 2U;
  };

} // namespace us_drv

#endif // US_DRV_US_DRV_CONSTS_H_
