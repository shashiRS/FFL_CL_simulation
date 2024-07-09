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

#ifndef ECO_COMPONENT_EXECUTION_MODE_H_
#define ECO_COMPONENT_EXECUTION_MODE_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace eco
{

  /// The execution mode of the components. It contains the configuration and sub-configuration id.
  struct ComponentExecutionMode
  {
    ///The component execution configuration id.
    uint8 configurationID;
    ///The component execution sub-configuration id.
    uint8 subConfigurationID;
  };

  inline ::eco::ComponentExecutionMode createComponentExecutionMode()
  {
    ComponentExecutionMode m;
    (void)::eco::memset(&m, 0U, sizeof(ComponentExecutionMode));
    return m;
  }

} // namespace eco

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::eco::ComponentExecutionMode create_default()
  {
      return ::eco::createComponentExecutionMode();
  }
}


#endif // ECO_COMPONENT_EXECUTION_MODE_H_
