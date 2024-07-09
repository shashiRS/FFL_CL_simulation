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

#ifndef MF_LVMD_LVMD_STATUS_PORT_H_
#define MF_LVMD_LVMD_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_lvmd/lvmdsystem_status.h"
#include "mf_lvmd/lvmdwarning_trigger.h"
#include "mf_lvmd/lvmdwarning_status.h"
#include "mf_lvmd/lvmdlead_vehicle_status.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_lvmd
{

  struct LvmdStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///lvmd status of the system
    LVMDSystemStatus lvmdSystemStatus_nu;
    ///lvmd warning trigger
    LVMDWarningTrigger lvmdWarningTrigger_nu;
    ///lvmd warning status
    LVMDWarningStatus lvmdWarningStatus_nu;
    ///lvmd lead vehicle status
    LVMDLeadVehicleStatus lvmdLeadVehicleStatus_nu;
    ///Number of Dynamic Objects in ROI
    uint8 numVehiclesinROI_nu;
  };

  inline ::mf_lvmd::LvmdStatusPort createLvmdStatusPort()
  {
    LvmdStatusPort m;
    (void)::eco::memset(&m, 0U, sizeof(LvmdStatusPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.lvmdLeadVehicleStatus_nu = createLVMDLeadVehicleStatus();
    return m;
  }

} // namespace mf_lvmd

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lvmd::LvmdStatusPort create_default()
  {
      return ::mf_lvmd::createLvmdStatusPort();
  }
}


#endif // MF_LVMD_LVMD_STATUS_PORT_H_