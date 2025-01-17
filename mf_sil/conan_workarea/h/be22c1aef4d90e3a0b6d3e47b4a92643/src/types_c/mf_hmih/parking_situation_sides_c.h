//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

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

#ifndef MF_HMIH_PARKING_SITUATION_SIDES_C_H_
#define MF_HMIH_PARKING_SITUATION_SIDES_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Situation on left side of parking vehicle
typedef struct
{
    ///Ego vehicle rear axle position, relative to the parking slots. 0 = rear axle is one slot width ahead of the top most slot. Increment 1 means half a slot width further down.
    uint8 egoRelativePos_nu;
    ///Perpendicular parking spaces on left side of vehicle
    boolean perpendicularParkingSpaces_nu;
    ///Parallel parking spaces on left side of vehicle
    boolean parallelParkingSpaces_nu;
    ///Angled parking spaces on left side of vehicle in orientation for forwards parking in
    boolean angledStandardSpaces_nu;
    ///Angled parking spaces on left side of vehicle in orientation for backwards parking in
    boolean angledReverseSpaces_nu;
    ///Street on left side of vehicle
    boolean street_nu;
    ///If perpendicular and parallel parking is possible on this side (note: One situation will be preselected).
    boolean uncertainSituation_nu;
    ///Parking out "parking spaces" (target direction) on the left side of vehicle in a parallel parking situation.
    boolean parallelParkingOut_nu;
    ///Especially for parking out or if no parking spaces are available
    boolean notAvailable_nu;
    ///Reserved for additional situations
    boolean placeholder_nu[9];
} MF_HMIH_ParkingSituationSides;

inline MF_HMIH_ParkingSituationSides create_MF_HMIH_ParkingSituationSides(void)
{
  MF_HMIH_ParkingSituationSides m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_HMIH_PARKING_SITUATION_SIDES_C_H_
