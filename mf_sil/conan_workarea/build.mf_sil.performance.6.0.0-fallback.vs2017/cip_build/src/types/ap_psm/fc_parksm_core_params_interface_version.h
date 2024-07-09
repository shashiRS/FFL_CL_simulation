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

#ifndef AP_PSM_FC_PARKSM_CORE_PARAMS_INTERFACE_VERSION_H_
#define AP_PSM_FC_PARKSM_CORE_PARAMS_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_psm
{

  struct FC_PARKSM_Core_Params_InterfaceVersion
  {
    enum { FC_PARKSM_Core_Params_VERSION = 1U};
  };

  inline ::ap_psm::FC_PARKSM_Core_Params_InterfaceVersion createFC_PARKSM_Core_Params_InterfaceVersion()
  {
    FC_PARKSM_Core_Params_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(FC_PARKSM_Core_Params_InterfaceVersion));
    return m;
  }

} // namespace ap_psm

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_psm::FC_PARKSM_Core_Params_InterfaceVersion create_default()
  {
      return ::ap_psm::createFC_PARKSM_Core_Params_InterfaceVersion();
  }
}


#endif // AP_PSM_FC_PARKSM_CORE_PARAMS_INTERFACE_VERSION_H_
