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

#ifndef AP_COMMONVEHSIGPROVIDER_MOTION_STATE_PORT_C_H_
#define AP_COMMONVEHSIGPROVIDER_MOTION_STATE_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_commonvehsigprovider/vehicle_driving_direction_c.h"
#include "eco/memset_c.h"

/// Measurements desribing the current motion state
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{m/s}
    ///@range{0,104.854}
    ///reference vehicle velocity ESC
    float32 vRefESC_mps;
    ///@range{-1,2}
    ///vehicle driving direction
    AP_COMMONVEHSIGPROVIDER_VehicleDrivingDirection vehicleDrivingDirection_nu;
    ///Qualifier-Bit vehicle direction
    boolean vRefESC_QF_nu;
    ///Vehicle standstill information
    boolean vehicleStandstill_nu;
} AP_COMMONVEHSIGPROVIDER_MotionStatePort;

inline AP_COMMONVEHSIGPROVIDER_MotionStatePort create_AP_COMMONVEHSIGPROVIDER_MotionStatePort(void)
{
  AP_COMMONVEHSIGPROVIDER_MotionStatePort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_COMMONVEHSIGPROVIDER_MOTION_STATE_PORT_C_H_