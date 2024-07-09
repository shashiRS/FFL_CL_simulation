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

#ifndef SI_TRIANGLE_SERIALIZABLE_INTERFACE_VERSION_H_
#define SI_TRIANGLE_SERIALIZABLE_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace si
{

  struct TriangleSerializable_InterfaceVersion
  {
    enum { TriangleSerializable_VERSION = 1U};
  };

  inline ::si::TriangleSerializable_InterfaceVersion createTriangleSerializable_InterfaceVersion()
  {
    TriangleSerializable_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(TriangleSerializable_InterfaceVersion));
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::TriangleSerializable_InterfaceVersion create_default()
  {
      return ::si::createTriangleSerializable_InterfaceVersion();
  }
}


#endif // SI_TRIANGLE_SERIALIZABLE_INTERFACE_VERSION_H_