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

#ifndef SI_DELIMITER_ZONES_SERIALIZABLE_H_
#define SI_DELIMITER_ZONES_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "si/delimiter_zones.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Optimization result serializable.
  struct DelimiterZonesSerializable
  {
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU}
    ///@unit{nu}
    ///@brief Describes how many optimization results were already added to the array.
    ::lsm_geoml::size_type actualSize;
    ///@brief Array of optimization results.
    DelimiterZones array[6];
  };

  inline ::si::DelimiterZonesSerializable createDelimiterZonesSerializable()
  {
    DelimiterZonesSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(DelimiterZonesSerializable));
    m.actualSize = 0U;
    {
      const uint64 arraysize = (sizeof(m.array) / sizeof(m.array[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.array[i] = createDelimiterZones();
      }
    }
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::DelimiterZonesSerializable create_default()
  {
      return ::si::createDelimiterZonesSerializable();
  }
}


#endif // SI_DELIMITER_ZONES_SERIALIZABLE_H_
