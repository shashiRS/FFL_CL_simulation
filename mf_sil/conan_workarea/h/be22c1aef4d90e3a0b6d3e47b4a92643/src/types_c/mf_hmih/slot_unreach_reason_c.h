//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\enum.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef MF_HMIH_SLOT_UNREACH_REASON_C_H_
#define MF_HMIH_SLOT_UNREACH_REASON_C_H_

#include "Platform_Types.h"

///Keeps information about why a slot is not reachable
typedef uint8 MF_HMIH_SlotUnreachReason;

#define MF_HMIH_SLOT_UNREACH_REASON_NO_UNREACHABLE_REASON 0U
#define MF_HMIH_SLOT_UNREACH_REASON_SLOT_TOO_NARROW 1U
#define MF_HMIH_SLOT_UNREACH_REASON_SLOT_TOO_SHORT 2U
#define MF_HMIH_SLOT_UNREACH_REASON_SLOT_EXCEEDS_MAX_PB 3U
#define MF_HMIH_SLOT_UNREACH_REASON_SLOT_WHEEL_COLLISION 4U
#define MF_HMIH_SLOT_UNREACH_REASON_SLOT_HIGH_OBJECT_COLLISION 5U
#define MF_HMIH_SLOT_UNREACH_REASON_UNKNOWN_REASON 6U
#define MF_HMIH_SLOT_UNREACH_REASON_NO_PATH_FOUND 7U


#endif // MF_HMIH_SLOT_UNREACH_REASON_C_H_