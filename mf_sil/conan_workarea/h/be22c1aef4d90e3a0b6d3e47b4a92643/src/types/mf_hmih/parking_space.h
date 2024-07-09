// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef MF_HMIH_PARKING_SPACE_H_
#define MF_HMIH_PARKING_SPACE_H_

#include "Platform_Types.h"
#include "mf_hmih/poss_orientation.h"
#include "mf_hmih/selected_orientation.h"
#include "mf_hmih/possible_direction.h"
#include "mf_hmih/selected_direction.h"
#include "eco/memset.h"


namespace mf_hmih
{

  /// Status of the left parking spaces
  struct ParkingSpace
  {
    ///Parking space [1,...,4] scanned
    boolean scanned_nu[4];
    ///Parking space [1,...,4] free
    boolean free_nu[4];
    ///Selected parking space for parking maneuver
    boolean selected_nu[4];
    ///@range{0,1}
    ///Indicates wether orientation of parking space is certain or not.
    PossOrientation possOrientation_nu[4];
    ///@range{0,3}
    ///If orientation is uncertain, the user can choose the final orientation by a button. The result of the user selection is indicated in this signal. If selection of orientation is not possible, the signal value indicates the only possible parking orientation.
    SelectedOrientation selectedOrientation_nu[4];
    ///@range{0,1}
    ///Indicates wether it is possible to select direction of vehicle in parking space (BOTH_DIRECTIONS) or not.
    PossibleDirection possDirection_nu[4];
    ///@range{0,1}
    ///If selection of direction is possible, the user can choose the direction of the vehicle in the parking space. The result of the user selection is indicated in this signal. If selection of direction is not possible, the signal value indicates the only possible parking direction.
    SelectedDirection selectedDirection_nu[4];
    ///@range{0,255}
    ///ID of corresponding target pose.
    uint8 poseID_nu[4];
    ///@range{-3.14159265359,3.14159265359}
    ///Yaw angle for the redetected memorized pose(currently only one pose can be redetected).
    ///It is filled only in case a memorized pose is redetected on that side and used by the HMI to correctly display the rotation angle of the pose.
    float32 memorizedPoseYaw_rad;
  };

  inline ::mf_hmih::ParkingSpace createParkingSpace()
  {
    ParkingSpace m;
    (void)::eco::memset(&m, 0U, sizeof(ParkingSpace));
    return m;
  }

} // namespace mf_hmih

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_hmih::ParkingSpace create_default()
  {
      return ::mf_hmih::createParkingSpace();
  }
}


#endif // MF_HMIH_PARKING_SPACE_H_