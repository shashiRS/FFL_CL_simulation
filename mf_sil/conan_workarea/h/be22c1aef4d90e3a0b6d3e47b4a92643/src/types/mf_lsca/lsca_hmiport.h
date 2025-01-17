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

#ifndef MF_LSCA_LSCA_HMIPORT_H_
#define MF_LSCA_LSCA_HMIPORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "mf_lsca/lsca_warning_status.h"
#include "mf_lsca/lsca_tube_marking.h"
#include "eco/memset.h"


namespace mf_lsca
{

  struct LscaHMIPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///Indicate to HMI that LSCA requests an information screen for the driver
    boolean activateBrakeInterventionScreen_nu;
    ///Demand HMI confirmation after LSCA braking to release brake in standstill
    boolean enforceHMIConfirmation_nu;
    ///On what part of the ego body will the collision be
    LSCA_WARNING_STATUS warningBody;
    ///What wheel is affected by a collision with a low object
    LSCA_WARNING_STATUS warningWheel;
    ///Where is the object relative to the ego vehicle
    LSCA_WARNING_STATUS warningObject;
    ///Indicate what side of the driving tube shall be marked as critical in HMI
    LSCA_TUBE_MARKING warningTube;
    ///ID of most critical object
    uint16 criticalObjectBrakeID_nu;
    ///RCTRA test module output - left
    boolean RctraAlertLeft_nu;
    ///RCTRA test module output - right
    boolean RctraAlertRight_nu;
    ///Door protection module output - front left door
    boolean DoorProtFL_nu;
    ///Door protection module output - front right door
    boolean DoorProtFR_nu;
    ///Door protection module output - rear left door
    boolean DoorProtBL_nu;
    ///Door protection module output - rear right door
    boolean DoorProtBR_nu;
    ///Door protection module output - trunk
    boolean DoorProtTrunk_nu;
  };

  inline ::mf_lsca::LscaHMIPort createLscaHMIPort()
  {
    LscaHMIPort m;
    (void)::eco::memset(&m, 0U, sizeof(LscaHMIPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace mf_lsca

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lsca::LscaHMIPort create_default()
  {
      return ::mf_lsca::createLscaHMIPort();
  }
}


#endif // MF_LSCA_LSCA_HMIPORT_H_
