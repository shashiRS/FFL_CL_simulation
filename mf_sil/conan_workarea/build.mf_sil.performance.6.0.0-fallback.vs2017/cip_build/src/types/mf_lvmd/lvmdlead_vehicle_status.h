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

#ifndef MF_LVMD_LVMDLEAD_VEHICLE_STATUS_H_
#define MF_LVMD_LVMDLEAD_VEHICLE_STATUS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_lvmd
{

  /// lvmd lead Vehicle status
  struct LVMDLeadVehicleStatus
  {
    ///@unit{s}
    ///@range{0,1000}
    ///Lead Vehicle Standstill Time
    float32 leadvehicle_standstilltime;
    ///@unit{m}
    ///@range{0,10}
    ///Lead Vehicle Forward Distance
    float32 leadvehicle_forward_distance;
    ///Boolean to indicate whether Lead Vehicle is in ROI or Not
    boolean leadvehicle_valid;
    ///@unit{m}
    ///@range{0,15}
    ///Lead Vehicle Driven Distance from Standstill Condition
    float32 leadvehicle_driven_distance;
    ///Lead Vehicle Object Reference Id
    uint16 leadvehicle_object_id;
  };

  inline ::mf_lvmd::LVMDLeadVehicleStatus createLVMDLeadVehicleStatus()
  {
    LVMDLeadVehicleStatus m;
    (void)::eco::memset(&m, 0U, sizeof(LVMDLeadVehicleStatus));
    return m;
  }

} // namespace mf_lvmd

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lvmd::LVMDLeadVehicleStatus create_default()
  {
      return ::mf_lvmd::createLVMDLeadVehicleStatus();
  }
}


#endif // MF_LVMD_LVMDLEAD_VEHICLE_STATUS_H_
