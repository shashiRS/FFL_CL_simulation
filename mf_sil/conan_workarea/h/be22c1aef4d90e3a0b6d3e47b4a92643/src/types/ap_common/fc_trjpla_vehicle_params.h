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

#ifndef AP_COMMON_FC_TRJPLA_VEHICLE_PARAMS_H_
#define AP_COMMON_FC_TRJPLA_VEHICLE_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_common
{

  struct FC_TRJPLA_Vehicle_Params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///Minimum safety distance which shall be applied to vehicle shape by inflation
    float32 AP_V_MIN_SAFETY_DIST_M;
    ///Maximum robustness distance which shall be applied to vehicle shape by inflation
    float32 AP_V_MAX_ROBUST_DIST_M;
  };

  inline ::ap_common::FC_TRJPLA_Vehicle_Params createFC_TRJPLA_Vehicle_Params()
  {
    FC_TRJPLA_Vehicle_Params m;
    (void)::eco::memset(&m, 0U, sizeof(FC_TRJPLA_Vehicle_Params));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace ap_common

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_common::FC_TRJPLA_Vehicle_Params create_default()
  {
      return ::ap_common::createFC_TRJPLA_Vehicle_Params();
  }
}


#endif // AP_COMMON_FC_TRJPLA_VEHICLE_PARAMS_H_