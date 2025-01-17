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

#ifndef MF_LSCA_LSCA_STEER_PORT_C_H_
#define MF_LSCA_LSCA_STEER_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "mf_lsca/lsca_steer_mode_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///Requested torque, if torque is requested
    float32 requestTorque_Nm;
    ///Requested front steering angle, if angle is requested
    float32 requestAngleFront_rad;
    ///Requested rear steering angle, if angle is requested
    float32 requestAngleRear_rad;
    ///The current request mode
    MF_LSCA_LSCA_STEER_MODE requestMode;
} MF_LSCA_LscaSteerPort;

inline MF_LSCA_LscaSteerPort create_MF_LSCA_LscaSteerPort(void)
{
  MF_LSCA_LscaSteerPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // MF_LSCA_LSCA_STEER_PORT_C_H_
