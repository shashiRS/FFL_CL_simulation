//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\enum.h.template!

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

#ifndef AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_C_H_
#define AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_C_H_

#include "Platform_Types.h"

///Information as to handshake failure.
typedef uint8 AP_TRJCTL_LodmcHandshakeFailedStatus;

#define AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_LODMC_NO_HANDSHAKE_FAILURE 0U
#define AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_LODMC_LSCA_HANDSHAKE_FAILURE 1U
#define AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_LODMC_MSP_HANDSHAKE_FAILURE 2U
#define AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_LODMC_AUP_HANDSHAKE_FAILURE 3U
#define AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_LODMC_REMOTE_HANDSHAKE_FAILURE 4U


#endif // AP_TRJCTL_LODMC_HANDSHAKE_FAILED_STATUS_C_H_