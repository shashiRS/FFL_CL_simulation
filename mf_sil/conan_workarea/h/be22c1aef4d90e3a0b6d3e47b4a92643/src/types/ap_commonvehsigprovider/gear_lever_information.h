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

#ifndef AP_COMMONVEHSIGPROVIDER_GEAR_LEVER_INFORMATION_H_
#define AP_COMMONVEHSIGPROVIDER_GEAR_LEVER_INFORMATION_H_

#include "ap_commonvehsigprovider/gear.h"
#include "eco/memset.h"


namespace ap_commonvehsigprovider
{

  /// None
  struct GearLeverInformation
  {
    ///@range{0,15}
    ///Current position of gear lever.
    Gear gearLeverPositionCur_nu;
  };

  inline ::ap_commonvehsigprovider::GearLeverInformation createGearLeverInformation()
  {
    GearLeverInformation m;
    (void)::eco::memset(&m, 0U, sizeof(GearLeverInformation));
    return m;
  }

} // namespace ap_commonvehsigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_commonvehsigprovider::GearLeverInformation create_default()
  {
      return ::ap_commonvehsigprovider::createGearLeverInformation();
  }
}


#endif // AP_COMMONVEHSIGPROVIDER_GEAR_LEVER_INFORMATION_H_
