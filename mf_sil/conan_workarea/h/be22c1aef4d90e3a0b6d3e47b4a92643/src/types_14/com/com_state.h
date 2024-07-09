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

#ifndef COM_COM_STATE_H_
#define COM_COM_STATE_H_

#include "Platform_Types.h"

namespace com
{
  ///! \brief return value enum for getStateInfo()  for all modules
  enum class ComState : uint8
  {
      ///!< COMSTATE_UNDEFINED
      COMSTATE_UNDEFINED = 0U,
      ///!< COMSTATE_OFF
      COMSTATE_OFF = 1U,
      ///!< COMSTATE_INITIALISING
      COMSTATE_INITIALISING = 2U,
      ///!< COMSTATE_READY
      COMSTATE_READY = 3U,
      ///!< COMSTATE_RUNNING
      COMSTATE_RUNNING = 4U,
      ///!< Filesafe state
      COMSTATE_FAILSAFE = 5U,
      ///!< COMSTATE_DEINITIALISING
      COMSTATE_DEINITIALISING = 6U,
  };
} // namespace com
#endif // COM_COM_STATE_H_