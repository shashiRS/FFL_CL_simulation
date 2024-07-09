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

#ifndef AP_COMMONVEHSIGPROVIDER_ODO_EXT_CTRL_PORT_C_H_
#define AP_COMMONVEHSIGPROVIDER_ODO_EXT_CTRL_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// None
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///Signal in order to reset the pose estimation
    boolean resetPoseEstimation_nu;
} AP_COMMONVEHSIGPROVIDER_OdoExtCtrlPort;

inline AP_COMMONVEHSIGPROVIDER_OdoExtCtrlPort create_AP_COMMONVEHSIGPROVIDER_OdoExtCtrlPort(void)
{
  AP_COMMONVEHSIGPROVIDER_OdoExtCtrlPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_COMMONVEHSIGPROVIDER_ODO_EXT_CTRL_PORT_C_H_
