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

#ifndef PDCP_PDCP_CONSTS_H_
#define PDCP_PDCP_CONSTS_H_

#include "Platform_Types.h"


namespace pdcp
{

  struct PDCP_Consts
  {
    static constexpr uint8 MAX_NUM_SECTORS_PER_SIDE = 4U;
    static constexpr uint8 NUM_MTS_DEBUG_FREESPACE_PDCP = 10U;
  };

} // namespace pdcp

#endif // PDCP_PDCP_CONSTS_H_
