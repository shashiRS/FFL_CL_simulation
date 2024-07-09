// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

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

#ifndef AP_VEHSTATESIGPROVIDER_BELT_BUCKLE_STATUS_H_
#define AP_VEHSTATESIGPROVIDER_BELT_BUCKLE_STATUS_H_

#include "ap_vehstatesigprovider/belt_buckle.h"


namespace ap_vehstatesigprovider
{

  /// Status of the belt buckle for each seat
  struct BeltBuckleStatus
  {
    ///@range{0,3}
    ///Passenger"s front seat belt buckle status
    BeltBuckle frontPsgr_nu{};
    ///@range{0,3}
    ///Driver"s seat belt buckle status
    BeltBuckle driver_nu{};
    ///@range{0,3}
    ///Seat belt buckle status in backrow on the right (row 2 or row 3)
    BeltBuckle backrowRight_nu{};
    ///@range{0,3}
    ///Seat belt buckle status in bacjrow on the left (row 2 or row 3)
    BeltBuckle backrowLeft_nu{};
  };

} // namespace ap_vehstatesigprovider

#endif // AP_VEHSTATESIGPROVIDER_BELT_BUCKLE_STATUS_H_
