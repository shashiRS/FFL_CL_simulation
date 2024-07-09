// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef SI_PERCEPTION_AVAILABILITY_PORT_H_
#define SI_PERCEPTION_AVAILABILITY_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "si/availability_status.h"
#include "eco/memset.h"


namespace si
{

  /// availability states of the sensors and EM
  struct PerceptionAvailabilityPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///@range{0,5}
    ///availability state of the USS sensors (enum values tbd)
    AvailabilityStatus statusUSSensors_nu[12];
    ///@range{0,5}
    ///availability state of the env model (enum values tbd)
    AvailabilityStatus statusEnvModel_nu;
  };

  inline ::si::PerceptionAvailabilityPort createPerceptionAvailabilityPort()
  {
    PerceptionAvailabilityPort m;
    (void)::eco::memset(&m, 0U, sizeof(PerceptionAvailabilityPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::PerceptionAvailabilityPort create_default()
  {
      return ::si::createPerceptionAvailabilityPort();
  }
}


#endif // SI_PERCEPTION_AVAILABILITY_PORT_H_