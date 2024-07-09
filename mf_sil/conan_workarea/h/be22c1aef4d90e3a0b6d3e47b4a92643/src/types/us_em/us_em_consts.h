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

#ifndef US_EM_US_EM_CONSTS_H_
#define US_EM_US_EM_CONSTS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_em
{

  struct US_EM_Consts
  {
    enum { US_EM_MAX_NUM_MAP_POINTS = 500U};
    enum { US_EM_MAX_NUM_STATIC_OBJ_PTS = 10U};
    enum { US_EM_MAX_NUM_STATIC_OBJ = 32U};
    enum { US_EM_MAX_NUM_DYN_OBJ_PTS = 4U};
    enum { US_EM_MAX_NUM_DYN_OBJ = 1U};
    enum { US_EM_MAX_NUM_OF_SENSORS = 12U};
  };

  inline ::us_em::US_EM_Consts createUS_EM_Consts()
  {
    US_EM_Consts m;
    (void)::eco::memset(&m, 0U, sizeof(US_EM_Consts));
    return m;
  }

} // namespace us_em

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_em::US_EM_Consts create_default()
  {
      return ::us_em::createUS_EM_Consts();
  }
}


#endif // US_EM_US_EM_CONSTS_H_