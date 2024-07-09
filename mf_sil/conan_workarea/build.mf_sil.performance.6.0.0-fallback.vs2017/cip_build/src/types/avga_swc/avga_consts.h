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

#ifndef AVGA_SWC_AVGA_CONSTS_H_
#define AVGA_SWC_AVGA_CONSTS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace avga_swc
{

  struct AVGA_consts
  {
    enum { DUMMY_CONST = 1U};
  };

  inline ::avga_swc::AVGA_consts createAVGA_consts()
  {
    AVGA_consts m;
    (void)::eco::memset(&m, 0U, sizeof(AVGA_consts));
    return m;
  }

} // namespace avga_swc

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::avga_swc::AVGA_consts create_default()
  {
      return ::avga_swc::createAVGA_consts();
  }
}


#endif // AVGA_SWC_AVGA_CONSTS_H_