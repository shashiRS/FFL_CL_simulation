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

#ifndef MF_MEMPARK_MPSTATUS_H_
#define MF_MEMPARK_MPSTATUS_H_

#include "Platform_Types.h"
#include "mf_mempark/memorized_slot.h"
#include "mf_mempark/memorized_parking_status.h"
#include "mf_mempark/training_status.h"
#include "mf_mempark/localization_status.h"
#include "mf_mempark/user_update_request_status.h"


namespace mf_mempark
{

  /// The elements for Memory Parking Status
  struct MPStatus
  {
    uint8 numStoredMemoryParkingSlots_nu{};
    MemorizedSlot memorizedParkingSlots[10]{};
    MemorizedParkingStatus memoryParkingState{};
    TrainingStatus trainingStatus{};
    LocalizationStatus localizationStatus{};
    UserUpdateRequestStatus userUpdateRequestStatus{};
  };

} // namespace mf_mempark

#endif // MF_MEMPARK_MPSTATUS_H_