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

#ifndef LSM_VEDODO_LAT_ACC_T_H_
#define LSM_VEDODO_LAT_ACC_T_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace lsm_vedodo
{

  struct LatAcc_t
  {
    float32 ZeroAccel;
    uint32 CalStatus;
  };

  inline ::lsm_vedodo::LatAcc_t createLatAcc_t()
  {
    LatAcc_t m;
    (void)::eco::memset(&m, 0U, sizeof(LatAcc_t));
    return m;
  }

} // namespace lsm_vedodo

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::lsm_vedodo::LatAcc_t create_default()
  {
      return ::lsm_vedodo::createLatAcc_t();
  }
}


#endif // LSM_VEDODO_LAT_ACC_T_H_
