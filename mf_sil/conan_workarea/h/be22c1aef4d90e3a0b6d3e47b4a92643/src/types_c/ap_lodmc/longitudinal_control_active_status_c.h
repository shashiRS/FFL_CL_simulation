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

#ifndef AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_C_H_
#define AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_C_H_

#include "Platform_Types.h"

///LoDMC Acknowledgement signal whether requested control type is granted and active
typedef uint8 AP_LODMC_longitudinalControlActiveStatus;

#define AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_LODMC_NO_ACTIVE_HANDSHAKE 0U
#define AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_LODMC_AUP_HANDSHAKE_ACTIVE 1U
#define AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_LODMC_LSCA_HANDSHAKE_ACTIVE 2U
#define AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_LODMC_REMOTE_PARKING_HANDSHAKE_ACTIVE 3U
#define AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_LODMC_MSP_HANDSHAKE_ACTIVE 4U


#endif // AP_LODMC_LONGITUDINAL_CONTROL_ACTIVE_STATUS_C_H_
