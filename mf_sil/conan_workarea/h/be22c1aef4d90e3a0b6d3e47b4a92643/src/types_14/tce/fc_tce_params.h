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

#ifndef TCE_FC_TCE_PARAMS_H_
#define TCE_FC_TCE_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"


namespace tce
{

  struct FC_TCE_Params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///GPS synchronization buffer time delay
    uint64 TCE_SYNC_BUFFER_DELAY_US{};
    ///Vehicle wheel base (distance between center of rear axle and front axle)
    float32 TCE_WHEELBASE_M{};
    ///Vehicle configured tire circumference for front left wheel
    float32 TCE_TYRE_CIRCUMFERENCE_FL_M{};
    ///Vehicle configured tire circumference for front right wheel
    float32 TCE_TYRE_CIRCUMFERENCE_FR_M{};
    ///Vehicle configured tire circumference for rear left wheel
    float32 TCE_TYRE_CIRCUMFERENCE_RL_M{};
    ///Vehicle configured tire circumference for rear right wheel
    float32 TCE_TYRE_CIRCUMFERENCE_RR_M{};
    ///Vehicle track width, distance between center of wheels on the front axle.
    float32 TCE_TRACK_FRONT_M{};
    ///Vehicle track width, distance between center of wheels on the rear axle.
    float32 TCE_TRACK_REAR_M{};
    ///lower limitation of tyre circumference
    float32 TCE_MIN_TYRE_CIRCUMFERENCE_M{};
    ///upper limitation of tyre circumference
    float32 TCE_MAX_TYRE_CIRCUMFERENCE_M{};
    ///generic factor to tune the speed dependancy of the circumference
    float32 TCE_SPEED_DEPENDANCY{};
    ///Number of teeth in the wheel pulse/tick encoder
    uint8 TCE_WHEEL_NUMBER_OF_TEETH_NU{};
  };

} // namespace tce

#endif // TCE_FC_TCE_PARAMS_H_
