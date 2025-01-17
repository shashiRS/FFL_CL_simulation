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

#ifndef SI_OBJ_MEASUREMENT_STATE_H_
#define SI_OBJ_MEASUREMENT_STATE_H_

#include "Platform_Types.h"

namespace si
{
  ///Measurement state of the dynamic object
  enum class ObjMeasurementState : uint8
  {
      MEAS_STATE_NEW = 0U,
      MEAS_STATE_MEASURED = 1U,
      MEAS_STATE_PREDICTED = 2U,
      MEAS_STATE_DELETED = 3U,
      MAX_NUM_MEASUREMENT_STATES = 4U,
  };
} // namespace si
#endif // SI_OBJ_MEASUREMENT_STATE_H_
