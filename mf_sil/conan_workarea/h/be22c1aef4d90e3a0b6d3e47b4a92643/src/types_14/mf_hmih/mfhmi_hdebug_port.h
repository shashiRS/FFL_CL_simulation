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

#ifndef MF_HMIH_MFHMI_HDEBUG_PORT_H_
#define MF_HMIH_MFHMI_HDEBUG_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "mf_hmih/hmistate.h"


namespace mf_hmih
{

  /// Debug port for HMIH.
  struct MFHmiHDebugPort
  {
    ///Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ///Common header for all structured data types (e.g. timestamp)
    ::eco::SignalHeader sSigHeader{};
    ///@unit{nu}
    ///freespace for MTS debug values
    sint32 debugInt[10]{};
    ///@unit{nu}
    ///freespace for MTS debug values
    float32 debugFloat[10]{};
    ///State of the  hmi communication state machine
    HMIState stateVarHMI_nu{};
  };

} // namespace mf_hmih

#endif // MF_HMIH_MFHMI_HDEBUG_PORT_H_
