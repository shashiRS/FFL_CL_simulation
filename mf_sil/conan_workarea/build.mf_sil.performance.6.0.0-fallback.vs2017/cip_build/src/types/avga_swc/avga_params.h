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

#ifndef AVGA_SWC_AVGA_PARAMS_H_
#define AVGA_SWC_AVGA_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace avga_swc
{

  struct AVGA_params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    uint8 AP_M_DUMMY_FOR_PDO;
  };

  inline ::avga_swc::AVGA_params createAVGA_params()
  {
    AVGA_params m;
    (void)::eco::memset(&m, 0U, sizeof(AVGA_params));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace avga_swc

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::avga_swc::AVGA_params create_default()
  {
      return ::avga_swc::createAVGA_params();
  }
}


#endif // AVGA_SWC_AVGA_PARAMS_H_
