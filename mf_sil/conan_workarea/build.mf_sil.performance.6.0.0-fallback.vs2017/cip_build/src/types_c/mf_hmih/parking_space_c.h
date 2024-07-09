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

#ifndef MF_HMIH_PARKING_SPACE_C_H_
#define MF_HMIH_PARKING_SPACE_C_H_

#include "Platform_Types.h"
#include "mf_hmih/poss_orientation_c.h"
#include "mf_hmih/selected_orientation_c.h"
#include "mf_hmih/possible_direction_c.h"
#include "mf_hmih/selected_direction_c.h"
#include "eco/memset_c.h"

/// Status of the left parking spaces
typedef struct
{
    ///Parking space [1,...,4] scanned
    boolean scanned_nu[4];
    ///Parking space [1,...,4] free
    boolean free_nu[4];
    ///Selected parking space for parking maneuver
    boolean selected_nu[4];
    ///@range{0,1}
    ///Indicates wether orientation of parking space is certain or not.
    MF_HMIH_PossOrientation possOrientation_nu[4];
    ///@range{0,3}
    ///If orientation is uncertain, the user can choose the final orientation by a button. The result of the user selection is indicated in this signal. If selection of orientation is not possible, the signal value indicates the only possible parking orientation.
    MF_HMIH_SelectedOrientation selectedOrientation_nu[4];
    ///@range{0,1}
    ///Indicates wether it is possible to select direction of vehicle in parking space (BOTH_DIRECTIONS) or not.
    MF_HMIH_PossibleDirection possDirection_nu[4];
    ///@range{0,1}
    ///If selection of direction is possible, the user can choose the direction of the vehicle in the parking space. The result of the user selection is indicated in this signal. If selection of direction is not possible, the signal value indicates the only possible parking direction.
    MF_HMIH_SelectedDirection selectedDirection_nu[4];
    ///@range{0,255}
    ///ID of corresponding target pose.
    uint8 poseID_nu[4];
    ///@range{-3.14159265359,3.14159265359}
    ///Yaw angle for the redetected memorized pose(currently only one pose can be redetected).
    ///It is filled only in case a memorized pose is redetected on that side and used by the HMI to correctly display the rotation angle of the pose.
    float32 memorizedPoseYaw_rad;
} MF_HMIH_ParkingSpace;

inline MF_HMIH_ParkingSpace create_MF_HMIH_ParkingSpace(void)
{
  MF_HMIH_ParkingSpace m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_HMIH_PARKING_SPACE_C_H_
