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

#ifndef VC_USER_EVENT_DATA_PORT_H_
#define VC_USER_EVENT_DATA_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace vc
{

  struct UserEventDataPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    uint16 deltaPolarAngle_u16;
    uint16 deltaAzimuthAngle_u16;
    uint8 GestureFinger_nu_u8;
    uint8 gestureCounter;
    uint16 deltaZoom;
    boolean isSequence;
    float32 clickEventX_px;
    float32 clickEventY_px;
  };

  inline ::vc::UserEventDataPort createUserEventDataPort()
  {
    UserEventDataPort m;
    (void)::eco::memset(&m, 0U, sizeof(UserEventDataPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace vc

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::vc::UserEventDataPort create_default()
  {
      return ::vc::createUserEventDataPort();
  }
}


#endif // VC_USER_EVENT_DATA_PORT_H_