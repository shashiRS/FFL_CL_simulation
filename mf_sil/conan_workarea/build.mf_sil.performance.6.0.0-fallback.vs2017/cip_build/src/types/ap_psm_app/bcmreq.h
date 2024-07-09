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

#ifndef AP_PSM_APP_BCMREQ_H_
#define AP_PSM_APP_BCMREQ_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_psm_app
{

  struct BCMReq
  {
    ///Direction indicator on left side is requested
    boolean dirIndLeftReq_nu;
    ///Direction indicator on right side is requested
    boolean dirIndRightReq_nu;
    ///Hazard warning is active
    boolean hazardWarning_nu;
    ///Special environmental lights for surround view requested
    boolean environmentLightReq_nu;
    ///Low beam light requested
    boolean lowBeamLightReq_nu;
    ///Back-up light requested
    boolean backUpLightReq_nu;
    ///Request to fold mirrors
    boolean foldMirrors_nu;
    ///Request to unfold mirrors
    boolean unFoldMirrors_nu;
    ///Block activation of convertible top
    boolean blockConvTopActivation_nu;
  };

  inline ::ap_psm_app::BCMReq createBCMReq()
  {
    BCMReq m;
    (void)::eco::memset(&m, 0U, sizeof(BCMReq));
    return m;
  }

} // namespace ap_psm_app

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_psm_app::BCMReq create_default()
  {
      return ::ap_psm_app::createBCMReq();
  }
}


#endif // AP_PSM_APP_BCMREQ_H_