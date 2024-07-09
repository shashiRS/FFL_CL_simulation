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

#ifndef SI_VIRTUAL_LINE_INFO_H_
#define SI_VIRTUAL_LINE_INFO_H_

#include "lsm_geoml/size_type.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Aggregating information relevant for virtual lines in one structure.
  struct VirtualLineInfo
  {
    ///@range{0,65535}
    ///@unit{nu}
    ///@brief Describes to which parking slot does the virtual line belong.
    ::lsm_geoml::size_type parkingSlotId_nu;
    ///@range{0,65535}
    ///@unit{nu}
    ///@brief Describes to which delimiter does the virtual line belong.
    ::lsm_geoml::size_type delimiterId_nu;
    ///@range{0,65535}
    ///@unit{nu}
    ///@brief Describes to which static structure does the virtual line belong.
    ::lsm_geoml::size_type staticStructureId_nu;
    ///@range{0,65535}
    ///@unit{nu}
    ///@brief Defines the ID of the virtual line.
    ::lsm_geoml::size_type virtualLineId_nu;
    ///@range{0,65535}
    ///@unit{nu}
    ///@brief Defines the ID of the virtual line"s starting vertex.
    ::lsm_geoml::size_type startVertex_nu;
    ///@range{0,65535}
    ///@unit{nu}
    ///@brief Defines the ID of the virtual line"s starting vertex.
    ::lsm_geoml::size_type endVertex_nu;
  };

  inline ::si::VirtualLineInfo createVirtualLineInfo()
  {
    VirtualLineInfo m;
    (void)::eco::memset(&m, 0U, sizeof(VirtualLineInfo));
    m.parkingSlotId_nu = 65535U;
    m.delimiterId_nu = 65535U;
    m.staticStructureId_nu = 65535U;
    m.virtualLineId_nu = 65535U;
    m.startVertex_nu = 65535U;
    m.endVertex_nu = 65535U;
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::VirtualLineInfo create_default()
  {
      return ::si::createVirtualLineInfo();
  }
}


#endif // SI_VIRTUAL_LINE_INFO_H_
