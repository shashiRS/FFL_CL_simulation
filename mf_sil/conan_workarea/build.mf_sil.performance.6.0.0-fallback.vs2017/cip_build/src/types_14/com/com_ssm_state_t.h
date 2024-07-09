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

#ifndef COM_COM_SSM_STATE_T_H_
#define COM_COM_SSM_STATE_T_H_

#include "Platform_Types.h"

namespace com
{
  ///! \brief global System state Machine states
  enum class ComSSM_State_t : uint8
  {
      ///!< Init state default
      COM_SSM_SYS_INIT = 0U,
      ///!< normal running state
      COM_SSM_SYS_RUN = 1U,
      ///!< run with limited function
      COM_SSM_SYS_RUN_LIMITED = 2U,
      ///!< failsave, no fuction run, only support save output data
      COM_SSM_SYS_FAILSAFE = 3U,
      ///!< shutdown no functionality switch to other states only after restart
      COM_SSM_SYS_SHUTDOWN = 4U,
      COM_SSM_SD_OFF = 0U,
      COM_SSM_SD_RESTART = 1U,
      COM_SSM_SD_GET_PERS_DATA = 2U,
  };
} // namespace com
#endif // COM_COM_SSM_STATE_T_H_
