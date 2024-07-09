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

#ifndef AP_PSM_SLOT_CTRL_PORT_H_
#define AP_PSM_SLOT_CTRL_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_psm/reset_origin_request_port.h"
#include "ap_psm/planning_ctrl_commands.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_psm
{

  /// Slot control data.
  struct SlotCtrlPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///@unit{nu}
    ///Reset request data in order to request the map reset from SI.
    ResetOriginRequestPort resetOriginRequestPort;
    PlanningCtrlCommands planningCtrlCommands;
    ///ID of the currently selected parking box.
    uint16 selectedParkingBoxId_nu;
    ///@unit{nu}
    ///Flag indicating if we want to store the current map or not
    boolean storeMemorizedParkingData;
    ///@unit{nu}
    ///Flag indicating if we want to delete the current map or not
    boolean deleteMemorizedParkingData;
    ///ID of the currently selected map
    uint16 currentSelectedMemParkSlotId;
  };

  inline ::ap_psm::SlotCtrlPort createSlotCtrlPort()
  {
    SlotCtrlPort m;
    (void)::eco::memset(&m, 0U, sizeof(SlotCtrlPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.resetOriginRequestPort = createResetOriginRequestPort();
    m.planningCtrlCommands = createPlanningCtrlCommands();
    return m;
  }

} // namespace ap_psm

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_psm::SlotCtrlPort create_default()
  {
      return ::ap_psm::createSlotCtrlPort();
  }
}


#endif // AP_PSM_SLOT_CTRL_PORT_H_
