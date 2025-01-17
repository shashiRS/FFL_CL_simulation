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

#ifndef ECO_COMPONENT_RESPONSE_H_
#define ECO_COMPONENT_RESPONSE_H_

#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/comp_op_state.h"


namespace eco
{

  struct ComponentResponse
  {
    ///Returned SignalHeader Common header for all structure.
    SignalHeader sigHeader{};
    ///Returned error code by the component
    uint64 errorCode{};
    ///Returned functional error codes (range check results) by the component
    uint64 functionalErrorCode[4]{};
    CompOpState opState{};
  };

} // namespace eco

#endif // ECO_COMPONENT_RESPONSE_H_
