//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\enum.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef COM_COM_STATE_C_H_
#define COM_COM_STATE_C_H_

#include "Platform_Types.h"

///! \brief return value enum for getStateInfo()  for all modules
typedef uint8 COM_ComState;

///!< COMSTATE_UNDEFINED
#define COM_COM_STATE_COMSTATE_UNDEFINED 0U
///!< COMSTATE_OFF
#define COM_COM_STATE_COMSTATE_OFF 1U
///!< COMSTATE_INITIALISING
#define COM_COM_STATE_COMSTATE_INITIALISING 2U
///!< COMSTATE_READY
#define COM_COM_STATE_COMSTATE_READY 3U
///!< COMSTATE_RUNNING
#define COM_COM_STATE_COMSTATE_RUNNING 4U
///!< Filesafe state
#define COM_COM_STATE_COMSTATE_FAILSAFE 5U
///!< COMSTATE_DEINITIALISING
#define COM_COM_STATE_COMSTATE_DEINITIALISING 6U


#endif // COM_COM_STATE_C_H_
