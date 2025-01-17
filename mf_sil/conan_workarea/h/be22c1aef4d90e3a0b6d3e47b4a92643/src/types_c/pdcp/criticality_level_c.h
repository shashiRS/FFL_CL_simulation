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

#ifndef PDCP_CRITICALITY_LEVEL_C_H_
#define PDCP_CRITICALITY_LEVEL_C_H_

#include "Platform_Types.h"

///Criticality level for this sector (the higher the criticality, the closer the obstacle is to the car).
typedef uint8 PDCP_CriticalityLevel;

#define PDCP_CRITICALITY_LEVEL_PDC_CRIT_LVL_OUTSIDE 0U
#define PDCP_CRITICALITY_LEVEL_PDC_CRIT_LVL_GREEN 1U
#define PDCP_CRITICALITY_LEVEL_PDC_CRIT_LVL_YELLOW 2U
#define PDCP_CRITICALITY_LEVEL_PDC_CRIT_LVL_RED 3U


#endif // PDCP_CRITICALITY_LEVEL_C_H_
