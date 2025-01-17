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

#ifndef CFG_MGR_US_SENSOR_DEGRADATION_TYPE_C_H_
#define CFG_MGR_US_SENSOR_DEGRADATION_TYPE_C_H_

#include "Platform_Types.h"

typedef uint8 CFG_MGR_UsSensorDegradationType;

///DEGRADATION_MODE_NO_DEGRADATION data or struct is not valid
#define CFG_MGR_US_SENSOR_DEGRADATION_TYPE_DEGRADATION_MODE_NO_DEGRADATION 0U
///DEGRADATION_MODE_GROUP_DEGRADATION data is valid for other usage
#define CFG_MGR_US_SENSOR_DEGRADATION_TYPE_DEGRADATION_MODE_GROUP_DEGRADATION 1U
///DEGRADATION_MODE_SENSOR_DEGRADATION data is invalid or out of range, usage with care
#define CFG_MGR_US_SENSOR_DEGRADATION_TYPE_DEGRADATION_MODE_SENSOR_DEGRADATION 2U


#endif // CFG_MGR_US_SENSOR_DEGRADATION_TYPE_C_H_
