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

#ifndef MF_MEMPARK_TRAINING_STATUS_H_
#define MF_MEMPARK_TRAINING_STATUS_H_

#include "Platform_Types.h"

namespace mf_mempark
{
  ///Current status of the training phase whether memory parking is
  ///reasdy to save recorded data and reason for failure if it
  ///cannot save data.
  enum class TrainingStatus : uint8
  {
      TRAIN_STATUS_INIT = 0U,
      TRAIN_STATUS_ACTIVE_NOT_READY_TO_SAVE = 1U,
      TRAIN_STATUS_ACTIVE_READY_TO_SAVE = 2U,
      TRAIN_STATUS_FAILED_MAX_LIMIT_EXCEEDED = 3U,
      TRAIN_STATUS_SAVE_FAILED_TIMEOUT = 4U,
      TRAIN_STATUS_SAVE_FAILED_NO_SPACE = 5U,
  };
} // namespace mf_mempark
#endif // MF_MEMPARK_TRAINING_STATUS_H_
