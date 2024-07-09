// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\enum.h.template!

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

#ifndef AP_LODMC_LO_DMCSYSTEM_STATE_H_
#define AP_LODMC_LO_DMCSYSTEM_STATE_H_

#include "Platform_Types.h"

namespace ap_lodmc
{
  ///System status and availability of the longitudinal dynamic motion control and the underlaid actuators.
  enum class LoDMCSystemState : uint8
  {
      LODMC_NOT_AVAILABLE = 0U,
      LODMC_INITIALISATION = 1U,
      LSCA_SUPPORT_AVAILABLE = 2U,
      LSCA_MSP_SUPPORT_AVAILABLE = 3U,
      LSCA_MSP_APA_SUPPORT_AVAILABLE = 4U,
      LSCA_MSP_APA_REMOTE_SUPPORT_AVAILABLE = 5U,
      LSCA_MSP_APA_REMOTE_SUPPORT_AVAILABLE_REMOTE_READY = 6U,
      LODMC_CTRL_ACTIVE = 7U,
      LODMC_CANCEL_BY_DRIVER = 8U,
      LODMC_ERROR = 9U,
  };
} // namespace ap_lodmc
#endif // AP_LODMC_LO_DMCSYSTEM_STATE_H_