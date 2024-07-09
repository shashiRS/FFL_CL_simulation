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

#ifndef MF_LSCA_LSCA_DEBUG_DATA_PORT_H_
#define MF_LSCA_LSCA_DEBUG_DATA_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_lsca/lsca_static_brake_debug_data_serializable.h"
#include "mf_lsca/lsca_dynamic_brake_debug_data_serializable.h"
#include "mf_lsca/lsca_general_debug_data_serializable.h"


namespace mf_lsca
{

  struct LscaDebugDataPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///Debug data regarding static breaking
    LscaStaticBrakeDebugDataSerializable staticBraking{};
    ///Debug data regarding dynamic breaking
    LscaDynamicBrakeDebugDataSerializable dynamicBraking{};
    ///Debug data regarding general information
    LscaGeneralDebugDataSerializable generalInfo{};
  };

} // namespace mf_lsca

#endif // MF_LSCA_LSCA_DEBUG_DATA_PORT_H_