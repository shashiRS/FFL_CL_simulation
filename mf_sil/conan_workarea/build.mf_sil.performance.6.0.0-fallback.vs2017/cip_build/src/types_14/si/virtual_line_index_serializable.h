// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef SI_VIRTUAL_LINE_INDEX_SERIALIZABLE_H_
#define SI_VIRTUAL_LINE_INDEX_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"


namespace si
{

  /// @brief Array of virtual line indices.
  struct VirtualLineIndexSerializable
  {
    ///@range{0,16}
    ///@unit{nu}
    ///@brief Describes how many virtual line indices were already added to the array.
    ::lsm_geoml::size_type actualSize{0U};
    ///@brief Array containing the virtual line indices.
    ::lsm_geoml::size_type array[16]{};
  };

} // namespace si

#endif // SI_VIRTUAL_LINE_INDEX_SERIALIZABLE_H_
