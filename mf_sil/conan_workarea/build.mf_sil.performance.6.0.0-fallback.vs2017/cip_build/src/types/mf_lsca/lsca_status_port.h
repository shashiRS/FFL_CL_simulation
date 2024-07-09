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

#ifndef MF_LSCA_LSCA_STATUS_PORT_H_
#define MF_LSCA_LSCA_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_lsca/lsca_state.h"
#include "mf_lsca/lsca_mode.h"
#include "eco/memset.h"


namespace mf_lsca
{

  struct LscaStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///Braking state machine state
    LSCA_STATE brakingModuleState_nu;
    ///Door protection state machine state
    LSCA_STATE doorProtectionModuleState_nu;
    ///Rear cross traffic assist state machine state
    LSCA_STATE rctaModuleState_nu;
    ///Pedal misapplication protection state machine state
    LSCA_STATE PmpModuleState_nu;
    ///Steering (Steering Resistance) state machine state
    LSCA_STATE steeringResistanceModuleState_nu;
    ///Steering (Steering Proposal) state machine state
    LSCA_STATE steeringProposalModuleState_nu;
    ///General LSCA state machine state
    LSCA_MODE lscaOverallMode_nu;
  };

  inline ::mf_lsca::LscaStatusPort createLscaStatusPort()
  {
    LscaStatusPort m;
    (void)::eco::memset(&m, 0U, sizeof(LscaStatusPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace mf_lsca

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lsca::LscaStatusPort create_default()
  {
      return ::mf_lsca::createLscaStatusPort();
  }
}


#endif // MF_LSCA_LSCA_STATUS_PORT_H_
