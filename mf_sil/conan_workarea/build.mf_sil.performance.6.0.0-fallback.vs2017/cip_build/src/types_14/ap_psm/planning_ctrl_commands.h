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

#ifndef AP_PSM_PLANNING_CTRL_COMMANDS_H_
#define AP_PSM_PLANNING_CTRL_COMMANDS_H_

#include "Platform_Types.h"
#include "ap_psm/apstate.h"
#include "ap_psm/applanning_specification.h"
#include "ap_psm/rmstate.h"
#include "ap_psm/gpstate.h"
#include "ap_psm/mpstate.h"
#include "ap_psm/tpstate.h"
#include "ap_psm/rastate.h"


namespace ap_psm
{

  /// From Parking State Machine or Planning and Control State Machine.
  /// This is in fact not a port! It was a port earlier but now is just a substructure in SlotCtrlPort!
  /// No interface version number signal needed!
  struct PlanningCtrlCommands
  {
    ///@range{0,255}
    ///Unique identifier of selected target pose. If no pose is selected, ID is 255.
    uint8 apChosenTargetPoseId_nu{255U};
    ///@range{0,128}
    ///State of the Autonumous Parking.
    APState apState{::ap_psm::APState::AP_INACTIVE};
    ///@range{0,4}
    ///Additional information to consider for the planning of the requested parking maneuver.
    APPlanningSpecification apPlanningSpecification{::ap_psm::APPlanningSpecification::APPS_INVALID};
    ///@range{0,128}
    ///State of the Remote Parking.
    RMState rmState{};
    ///@range{0,128}
    ///State of the Garage Parking.
    GPState gpState{::ap_psm::GPState::GP_INACTIVE};
    ///@range{0,128}
    ///State of the Memory Parking.
    MPState mpState{};
    ///@range{0,128}
    ///State of the Trained Parking.
    TPState tpState{};
    ///@range{0,128}
    ///State of the Reverse Assist.
    RAState raState{};
  };

} // namespace ap_psm

#endif // AP_PSM_PLANNING_CTRL_COMMANDS_H_