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

#ifndef AP_TRJCTL_LADMC_HANDSHAKE_FAILED_STATUS_H_
#define AP_TRJCTL_LADMC_HANDSHAKE_FAILED_STATUS_H_

#include "Platform_Types.h"

namespace ap_trjctl
{
  ///Information as to handshake failure.
  enum class LadmcHandshakeFailedStatus : uint8
  {
      LADMC_NO_HANDSHAKE_FAILURE = 0U,
      LADMC_LSCA_HANDSHAKE_FAILURE = 1U,
      LADMC_AUP_HANDSHAKE_FAILURE = 2U,
      LADMC_REMOTE_HANDSHAKE_FAILURE = 3U,
  };
} // namespace ap_trjctl
#endif // AP_TRJCTL_LADMC_HANDSHAKE_FAILED_STATUS_H_
