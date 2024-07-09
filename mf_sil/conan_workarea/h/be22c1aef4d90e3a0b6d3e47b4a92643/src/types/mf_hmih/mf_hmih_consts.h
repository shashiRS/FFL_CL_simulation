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

#ifndef MF_HMIH_MF_HMIH_CONSTS_H_
#define MF_HMIH_MF_HMIH_CONSTS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_hmih
{

  /// MHMIH consts
  struct MF_HMIH_Consts
  {
    enum { AP_H_MAX_NUM_SLOTS_SIDE_NU = 4U};
    enum { AP_H_MAX_NUM_PLACEHOLDER_NU = 9U};
    enum { AP_H_MAX_NUM_PARKING_POSES_NU = 20U};
    enum { AP_H_MAX_NUM_VISIBILITY_TAGS_NU = 35U};
    enum { MAX_NUM_SECTORS_PER_SIDE_HMIH = 4U};
    enum { NUM_MTS_DEBUG_FREESPACE_HMIH = 10U};
  };

  inline ::mf_hmih::MF_HMIH_Consts createMF_HMIH_Consts()
  {
    MF_HMIH_Consts m;
    (void)::eco::memset(&m, 0U, sizeof(MF_HMIH_Consts));
    return m;
  }

} // namespace mf_hmih

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_hmih::MF_HMIH_Consts create_default()
  {
      return ::mf_hmih::createMF_HMIH_Consts();
  }
}


#endif // MF_HMIH_MF_HMIH_CONSTS_H_
