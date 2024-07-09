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

#ifndef MF_HMIH_AVGTYPE_C_H_
#define MF_HMIH_AVGTYPE_C_H_

#include "Platform_Types.h"

///Shows the simplified situation during Automated Vehicle Guidance to enable HMI to inform driver of maneuver.
typedef uint8 MF_HMIH_AVGType;

#define MF_HMIH_AVGTYPE_NO_AVG_TYPE 0U
#define MF_HMIH_AVGTYPE_PARK_IN_PARALLEL_LEFT 1U
#define MF_HMIH_AVGTYPE_PARK_IN_PARALLEL_RIGHT 2U
#define MF_HMIH_AVGTYPE_PARK_IN_PERPENDICULAR_LEFT_FWD 3U
#define MF_HMIH_AVGTYPE_PARK_IN_PERPENDICULAR_LEFT_BWD 4U
#define MF_HMIH_AVGTYPE_PARK_IN_PERPENDICULAR_RIGHT_FWD 5U
#define MF_HMIH_AVGTYPE_PARK_IN_PERPENDICULAR_RIGHT_BWD 6U
#define MF_HMIH_AVGTYPE_PARK_OUT_PARALLEL_LEFT 7U
#define MF_HMIH_AVGTYPE_PARK_OUT_PARALLEL_RIGHT 8U
#define MF_HMIH_AVGTYPE_PARK_OUT_PERPENDICULAR_LEFT_FWD 9U
#define MF_HMIH_AVGTYPE_PARK_OUT_PERPENDICULAR_LEFT_BWD 10U
#define MF_HMIH_AVGTYPE_PARK_OUT_PERPENDICULAR_RIGHT_FWD 11U
#define MF_HMIH_AVGTYPE_PARK_OUT_PERPENDICULAR_RIGHT_BWD 12U
#define MF_HMIH_AVGTYPE_REM_MAN_FWD 13U
#define MF_HMIH_AVGTYPE_REM_MAN_BWD 14U
#define MF_HMIH_AVGTYPE_GP_GENREAL 15U


#endif // MF_HMIH_AVGTYPE_C_H_