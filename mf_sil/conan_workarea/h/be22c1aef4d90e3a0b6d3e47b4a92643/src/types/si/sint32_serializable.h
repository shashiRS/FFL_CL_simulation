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

#ifndef SI_SINT32_SERIALIZABLE_H_
#define SI_SINT32_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Signed 32bit integer serializable.
  struct Sint32Serializable
  {
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU}
    ///@unit{nu}
    ///@brief Describes how many signed 32bit integers were already added to the array.
    ::lsm_geoml::size_type actualSize;
    ///@brief Array of signed 32bit integers.
    sint32 array[6];
  };

  inline ::si::Sint32Serializable createSint32Serializable()
  {
    Sint32Serializable m;
    (void)::eco::memset(&m, 0U, sizeof(Sint32Serializable));
    m.actualSize = 0U;
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::Sint32Serializable create_default()
  {
      return ::si::createSint32Serializable();
  }
}


#endif // SI_SINT32_SERIALIZABLE_H_