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

#ifndef US_DRV_US_DRV_ENVELOPE_SIGNAL_PATH_H_
#define US_DRV_US_DRV_ENVELOPE_SIGNAL_PATH_H_

#include "Platform_Types.h"


namespace us_drv
{

  struct UsDrvEnvelopeSignalPath
  {
    ///receiving logical sensor id
    uint8 rxSensorId{};
    ///transmitting logical sensor id
    uint8 txSensorId{};
    ///signal path id
    uint8 signalPathId{};
    ///index of first sample in sample data array
    uint32 startSample{};
    ///count of samples in sample data array
    uint32 numSamples{};
    ///timestamp when the first sample was captured
    uint64 timestamp_us{};
  };

} // namespace us_drv

#endif // US_DRV_US_DRV_ENVELOPE_SIGNAL_PATH_H_
