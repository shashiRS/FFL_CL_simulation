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

#ifndef AP_TP_TRAJ_PLAN_VISU_PORT_H_
#define AP_TP_TRAJ_PLAN_VISU_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_tp/planned_geometric_path_output.h"
#include "ap_tp/driving_resistance.h"
#include "eco/memset.h"


namespace ap_tp
{

  /// 
  struct TrajPlanVisuPort
  {
    ///@unit{eco.AlgoInterfaceVersionNumber}
    ///
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ///@unit{eco.SignalHeader}
    ///
    ::eco::SignalHeader sSigHeader;
    ///@range{0,1023}
    ///@unit{---}
    ///index of current pose along the planned path @min: 0 @max: 1023 @unit: ---
    uint16 currentPoseIdx_nu;
    ///@range{0,1023}
    ///@unit{---}
    ///number of valid positions in the planned path @min: 0 @max: 1023 @unit: ---
    uint16 numValidPoses_nu;
    ///@range{0,255}
    ///@unit{---}
    ///number of valid positions in the planned path @min: 0 @max: 255 @unit: ---
    uint8 numValidSegments;
    ///@range{-1000,1000}
    ///@unit{m}
    ///x coordinates of the whole path @min: -1000 @max: 1000 @unit: m
    float32 plannedPathXPos_m[1000];
    ///@range{-1000,1000}
    ///@unit{m}
    ///y coordinates of the whole path @min: -1000 @max: 1000 @unit: m
    float32 plannedPathYPos_m[1000];
    ///@unit{nu}
    ///None @min: 0 @max: 0 @unit: nu
    PlannedGeometricPathOutput plannedGeometricPath[25];
    ///Driving resistance distance and type information per wheel.
    DrivingResistance drivingResistance[4];
  };

  inline ::ap_tp::TrajPlanVisuPort createTrajPlanVisuPort()
  {
    TrajPlanVisuPort m;
    (void)::eco::memset(&m, 0U, sizeof(TrajPlanVisuPort));
    m.sSigHeader = ::eco::createSignalHeader();
    {
      const uint64 arraysize = (sizeof(m.plannedGeometricPath) / sizeof(m.plannedGeometricPath[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.plannedGeometricPath[i] = createPlannedGeometricPathOutput();
      }
    }
    {
      const uint64 arraysize = (sizeof(m.drivingResistance) / sizeof(m.drivingResistance[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.drivingResistance[i] = createDrivingResistance();
      }
    }
    return m;
  }

} // namespace ap_tp

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_tp::TrajPlanVisuPort create_default()
  {
      return ::ap_tp::createTrajPlanVisuPort();
  }
}


#endif // AP_TP_TRAJ_PLAN_VISU_PORT_H_
