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

#ifndef US_EM_US_EM_POSITION_H_
#define US_EM_US_EM_POSITION_H_

#include "Platform_Types.h"


namespace us_em
{

  struct UsEmPosition
  {
    float32 xPosition_m{};
    float32 yPosition_m{};
  };

} // namespace us_em

#endif // US_EM_US_EM_POSITION_H_
