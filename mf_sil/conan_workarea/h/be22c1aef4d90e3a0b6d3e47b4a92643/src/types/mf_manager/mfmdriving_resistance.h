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

#ifndef MF_MANAGER_MFMDRIVING_RESISTANCE_H_
#define MF_MANAGER_MFMDRIVING_RESISTANCE_H_

#include "Platform_Types.h"
#include "mf_manager/mfmdriving_resistance_type.h"
#include "eco/memset.h"


namespace mf_manager
{

  struct MFMDrivingResistance
  {
    float32 distance_m;
    MFMDrivingResistanceType type_nu;
  };

  inline ::mf_manager::MFMDrivingResistance createMFMDrivingResistance()
  {
    MFMDrivingResistance m;
    (void)::eco::memset(&m, 0U, sizeof(MFMDrivingResistance));
    return m;
  }

} // namespace mf_manager

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_manager::MFMDrivingResistance create_default()
  {
      return ::mf_manager::createMFMDrivingResistance();
  }
}


#endif // MF_MANAGER_MFMDRIVING_RESISTANCE_H_
