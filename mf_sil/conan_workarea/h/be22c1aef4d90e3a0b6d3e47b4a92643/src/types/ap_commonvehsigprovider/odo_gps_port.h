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

#ifndef AP_COMMONVEHSIGPROVIDER_ODO_GPS_PORT_H_
#define AP_COMMONVEHSIGPROVIDER_ODO_GPS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_commonvehsigprovider/gpsdata.h"
#include "eco/memset.h"


namespace ap_commonvehsigprovider
{

  /// None
  struct OdoGpsPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///@unit{nu}
    ///Data specifications of the GPS System.
    GPSData gpsData;
  };

  inline ::ap_commonvehsigprovider::OdoGpsPort createOdoGpsPort()
  {
    OdoGpsPort m;
    (void)::eco::memset(&m, 0U, sizeof(OdoGpsPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.gpsData = createGPSData();
    return m;
  }

} // namespace ap_commonvehsigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_commonvehsigprovider::OdoGpsPort create_default()
  {
      return ::ap_commonvehsigprovider::createOdoGpsPort();
  }
}


#endif // AP_COMMONVEHSIGPROVIDER_ODO_GPS_PORT_H_
