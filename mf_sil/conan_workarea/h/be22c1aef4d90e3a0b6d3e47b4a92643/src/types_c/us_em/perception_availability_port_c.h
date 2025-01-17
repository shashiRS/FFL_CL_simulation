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

#ifndef US_EM_PERCEPTION_AVAILABILITY_PORT_C_H_
#define US_EM_PERCEPTION_AVAILABILITY_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "us_em/availability_status_c.h"
#include "eco/memset_c.h"

/// Availability states of the sensors and EM
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@range{0,5}
    ///Availability state of the USS sensors
    US_EM_AvailabilityStatus statusUSSensors_nu[12];
    ///@range{0,5}
    ///Availability state of the environment model
    US_EM_AvailabilityStatus statusEnvModel_nu;
} US_EM_PerceptionAvailabilityPort;

inline US_EM_PerceptionAvailabilityPort create_US_EM_PerceptionAvailabilityPort(void)
{
  US_EM_PerceptionAvailabilityPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // US_EM_PERCEPTION_AVAILABILITY_PORT_C_H_
