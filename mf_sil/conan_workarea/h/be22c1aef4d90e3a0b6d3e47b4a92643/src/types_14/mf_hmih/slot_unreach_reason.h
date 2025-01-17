// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\enum.h.template!

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

#ifndef MF_HMIH_SLOT_UNREACH_REASON_H_
#define MF_HMIH_SLOT_UNREACH_REASON_H_

#include "Platform_Types.h"

namespace mf_hmih
{
  ///Keeps information about why a slot is not reachable
  enum class SlotUnreachReason : uint8
  {
      NO_UNREACHABLE_REASON = 0U,
      SLOT_TOO_NARROW = 1U,
      SLOT_TOO_SHORT = 2U,
      SLOT_EXCEEDS_MAX_PB = 3U,
      SLOT_WHEEL_COLLISION = 4U,
      SLOT_HIGH_OBJECT_COLLISION = 5U,
      UNKNOWN_REASON = 6U,
      NO_PATH_FOUND = 7U,
  };
} // namespace mf_hmih
#endif // MF_HMIH_SLOT_UNREACH_REASON_H_
