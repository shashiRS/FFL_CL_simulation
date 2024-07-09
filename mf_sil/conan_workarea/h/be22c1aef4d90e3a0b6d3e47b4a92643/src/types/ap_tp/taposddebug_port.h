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

#ifndef AP_TP_TAPOSDDEBUG_PORT_H_
#define AP_TP_TAPOSDDEBUG_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_tp/parking_box_debug_info.h"
#include "ap_tp/pose_box_dataset.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_tp
{

  /// 
  struct TAPOSDDebugPort
  {
    ///@unit{eco.AlgoInterfaceVersionNumber}
    ///
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ///@unit{eco.SignalHeader}
    ///
    ::eco::SignalHeader sSigHeader;
    ///@unit{nu}
    ///Debug information of generation of target poses for backwards parking (Index relates to input data parking box index) @min: 0 @max: 0 @unit: nu
    ParkingBoxDebugInfo pbDebugBackwards[6];
    ///@unit{nu}
    ///Debug information of generation of target poses for forwards parking (Index relates to input data parking box index) @min: 0 @max: 0 @unit: nu
    ParkingBoxDebugInfo pbDebugForwards[6];
    ///@unit{nu}
    ///Linking of all Target Poses to their Parking Boxes @min: 0 @max: 0 @unit: nu
    PoseBoxDataset poseBoxDataset;
    ///@range{0,100}
    ///@unit{m}
    ///(only filled at end of last stroke) Lateral distance between vehicle pose and target pose @min: 0 @max: 100 @unit: m
    float32 latDistToTarget_m;
    ///@range{0,100}
    ///@unit{m}
    ///(only filled at end of last stroke) Longitudinal distance between vehicle pose and target pose @min: 0 @max: 100 @unit: m
    float32 longDistToTarget_m;
    ///@range{-3.14,3.14}
    ///@unit{Radian}
    ///(only filled at end of last stroke) Difference in yaw angle between vehicle pose and target pose @min: -3.14 @max: 3.14 @unit: Radian
    float32 yawDiffToTarget_rad;
    ///@unit{nu}
    ///freespace for MTS debug values @min: 0 @max: 0 @unit: nu
    sint32 debugInt[10];
    ///@unit{nu}
    ///freespace for MTS debug values @min: 0 @max: 0 @unit: nu
    float32 debugFloat[10];
  };

  inline ::ap_tp::TAPOSDDebugPort createTAPOSDDebugPort()
  {
    TAPOSDDebugPort m;
    (void)::eco::memset(&m, 0U, sizeof(TAPOSDDebugPort));
    m.sSigHeader = ::eco::createSignalHeader();
    {
      const uint64 arraysize = (sizeof(m.pbDebugBackwards) / sizeof(m.pbDebugBackwards[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.pbDebugBackwards[i] = createParkingBoxDebugInfo();
      }
    }
    {
      const uint64 arraysize = (sizeof(m.pbDebugForwards) / sizeof(m.pbDebugForwards[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.pbDebugForwards[i] = createParkingBoxDebugInfo();
      }
    }
    m.poseBoxDataset = createPoseBoxDataset();
    return m;
  }

} // namespace ap_tp

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_tp::TAPOSDDebugPort create_default()
  {
      return ::ap_tp::createTAPOSDDebugPort();
  }
}


#endif // AP_TP_TAPOSDDEBUG_PORT_H_