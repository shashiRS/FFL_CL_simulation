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

#ifndef SI_PARKING_SCENARIO_TYPES_C_H_
#define SI_PARKING_SCENARIO_TYPES_C_H_

#include "Platform_Types.h"

///type of parking sccenario associated with this parking box
typedef uint8 SI_ParkingScenarioTypes;

#define SI_PARKING_SCENARIO_TYPES_PARALLEL_PARKING 0U
#define SI_PARKING_SCENARIO_TYPES_PERPENDICULAR_PARKING 1U
#define SI_PARKING_SCENARIO_TYPES_ANGLED_PARKING_OPENING_TOWARDS_BACK 2U
#define SI_PARKING_SCENARIO_TYPES_ANGLED_PARKING_OPENING_TOWARDS_FRONT 3U
#define SI_PARKING_SCENARIO_TYPES_GARAGE_PARKING 4U
#define SI_PARKING_SCENARIO_TYPES_DIRECT_PARKING 5U
#define SI_PARKING_SCENARIO_TYPES_EXTERNAL_TAPOS_PARALLEL 6U
#define SI_PARKING_SCENARIO_TYPES_EXTERNAL_TAPOS_PERPENDICULAR 7U
#define SI_PARKING_SCENARIO_TYPES_EXTERNAL_TAPOS_PARALLEL_OUT 8U
#define SI_PARKING_SCENARIO_TYPES_EXTERNAL_TAPOS_PERPENDICULAR_OUT 9U
#define SI_PARKING_SCENARIO_TYPES_MAX_NUM_PARKING_SCENARIO_TYPES 10U


#endif // SI_PARKING_SCENARIO_TYPES_C_H_