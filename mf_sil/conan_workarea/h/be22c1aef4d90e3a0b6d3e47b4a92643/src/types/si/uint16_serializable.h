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

#ifndef SI_UINT16_SERIALIZABLE_H_
#define SI_UINT16_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Unsigned 16bit integer serializable.
  struct Uint16Serializable
  {
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_STATIC_OBJ_NU}
    ///@unit{nu}
    ///@brief Describes how many unsigned 16bit integers were already added to the array.
    ::lsm_geoml::size_type actualSize;
    ///@brief Array of unsigned 16bit integers.
    uint16 array[32];
  };

  inline ::si::Uint16Serializable createUint16Serializable()
  {
    Uint16Serializable m;
    (void)::eco::memset(&m, 0U, sizeof(Uint16Serializable));
    m.actualSize = 0U;
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::Uint16Serializable create_default()
  {
      return ::si::createUint16Serializable();
  }
}


#endif // SI_UINT16_SERIALIZABLE_H_
