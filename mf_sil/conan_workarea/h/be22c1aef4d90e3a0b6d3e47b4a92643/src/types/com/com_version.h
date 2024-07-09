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

#ifndef COM_COM_VERSION_H_
#define COM_COM_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace com
{

  /// ! \brief Version information struct
  ///   usage for global version information
  struct ComVersion
  {
    uint8 major;
    uint8 minor;
    uint16 patch;
  };

  inline ::com::ComVersion createComVersion()
  {
    ComVersion m;
    (void)::eco::memset(&m, 0U, sizeof(ComVersion));
    return m;
  }

} // namespace com

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::com::ComVersion create_default()
  {
      return ::com::createComVersion();
  }
}


#endif // COM_COM_VERSION_H_
