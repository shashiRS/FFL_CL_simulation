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

#ifndef MF_HMIH_PARKING_TARGET_POSE_H_
#define MF_HMIH_PARKING_TARGET_POSE_H_

#include "Platform_Types.h"
#include "lsm_geoml/pose_pod.h"
#include "eco/memset.h"


namespace mf_hmih
{

  /// Target pose information to be displayed by Visu
  struct ParkingTargetPose
  {
    ///ID of target pose
    uint8 id_nu;
    ///Parking slot free
    boolean isFree_nu;
    ///Parking slot was scanned
    boolean isScanned_nu;
    ///Parking slot selected
    boolean isSelected_nu;
    ///Parking slot switchable
    boolean isSwitchable_nu;
    ///A pose defined by an x and y coordinate as well as a yaw angle
    ::lsm_geoml::Pose_POD pose_nu;
  };

  inline ::mf_hmih::ParkingTargetPose createParkingTargetPose()
  {
    ParkingTargetPose m;
    (void)::eco::memset(&m, 0U, sizeof(ParkingTargetPose));
    m.id_nu = 255U;
    m.pose_nu = ::lsm_geoml::createPose_POD();
    return m;
  }

} // namespace mf_hmih

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_hmih::ParkingTargetPose create_default()
  {
      return ::mf_hmih::createParkingTargetPose();
  }
}


#endif // MF_HMIH_PARKING_TARGET_POSE_H_