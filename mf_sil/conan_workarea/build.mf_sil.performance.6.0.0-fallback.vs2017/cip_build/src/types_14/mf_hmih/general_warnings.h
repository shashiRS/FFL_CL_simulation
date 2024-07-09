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

#ifndef MF_HMIH_GENERAL_WARNINGS_H_
#define MF_HMIH_GENERAL_WARNINGS_H_

#include "mf_hmih/day_time.h"
#include "Platform_Types.h"


namespace mf_hmih
{

  /// General warning and attention information which can be displayed on the HMI.
  struct GeneralWarnings
  {
    ///@range{0,2}
    ///Estimated daytime recognition. Can be used to inform the driver about low-light situation.
    DayTime visualDaytimeEstimation_nu{::mf_hmih::DayTime::DT_NIGHT};
    ///Parking situation is a narrow slot. Driver should consider to use remote parking.
    boolean narrowParkingSpaceWarning_nu{0};
    ///Vehicles next to the parking in slot are non stationairy. Driver attention should be raised.
    boolean parkingSpaceObstaclesAreMoving_nu{0};
    ///Warn the driver that the camera system might be visually impaired.
    ///Shall be based on the information from PerceptionAvailabilityPort and similar sources.
    boolean visualSensorSystemWarning_nu{0};
    ///Warn the driver that the USS system might be impaired.
    ///Shall be based on the information from PerceptionAvailabilityPort and similar sources.
    boolean ultrasoundSensorSystemWarning_nu{0};
  };

} // namespace mf_hmih

#endif // MF_HMIH_GENERAL_WARNINGS_H_
