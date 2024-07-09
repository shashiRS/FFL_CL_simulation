// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\enum.h.template!

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

#ifndef ECO_COMP_OP_STATE_H_
#define ECO_COMP_OP_STATE_H_

#include "Platform_Types.h"

namespace eco
{
  ///Represents the operation state of components updated by
  ///the framework
  ///through shell with return provided by the components.
  ///values: enum { OPS_INIT=0,OPS_SCHEDULED=1,OPS_RUNNING=2,OPS_DONE=3
  ///,OPS_CANCELED=4,OPS_UNDEFINED=255,}
  ///@range{0,255}
  enum class CompOpState : uint8
  {
      OPS_INIT = 0U,
      OPS_SCHEDULED = 1U,
      OPS_RUNNING = 2U,
      OPS_DONE = 3U,
      OPS_CANCELED = 4U,
      OPS_UNDEFINED = 255U,
  };
} // namespace eco
#endif // ECO_COMP_OP_STATE_H_