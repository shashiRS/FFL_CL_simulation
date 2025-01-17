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

#ifndef US_EM_OBJECT_TREND_C_H_
#define US_EM_OBJECT_TREND_C_H_

#include "Platform_Types.h"

///Trend information about the object.
typedef uint8 US_EM_ObjectTrend;

#define US_EM_OBJECT_TREND_OBJ_TRND_UNKNOWN 0U
#define US_EM_OBJECT_TREND_OBJ_TRND_APPROACHING 1U
#define US_EM_OBJECT_TREND_OBJ_TRND_MOVING_AWAY 2U
#define US_EM_OBJECT_TREND_OBJ_TRND_FIXED_DIST 3U
#define US_EM_OBJECT_TREND_OBJ_TRND_MAX_NUM 4U


#endif // US_EM_OBJECT_TREND_C_H_
