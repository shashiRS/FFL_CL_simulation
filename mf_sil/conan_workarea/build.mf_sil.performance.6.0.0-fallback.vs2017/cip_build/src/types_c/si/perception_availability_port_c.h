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

#ifndef SI_PERCEPTION_AVAILABILITY_PORT_C_H_
#define SI_PERCEPTION_AVAILABILITY_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "si/availability_status_c.h"
#include "eco/memset_c.h"

/// availability states of the sensors and EM
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@range{0,5}
    ///availability state of the USS sensors (enum values tbd)
    SI_AvailabilityStatus statusUSSensors_nu[12];
    ///@range{0,5}
    ///availability state of the env model (enum values tbd)
    SI_AvailabilityStatus statusEnvModel_nu;
} SI_PerceptionAvailabilityPort;

inline SI_PerceptionAvailabilityPort create_SI_PerceptionAvailabilityPort(void)
{
  SI_PerceptionAvailabilityPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // SI_PERCEPTION_AVAILABILITY_PORT_C_H_
