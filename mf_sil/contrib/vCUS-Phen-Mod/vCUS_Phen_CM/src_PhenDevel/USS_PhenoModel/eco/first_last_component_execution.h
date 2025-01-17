// Attention, this file is generated by Cobolt from template: C:\_repos\mf_sil\dbg\eco\eco.generic\codegen\templates\types\struct.h.template!

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

#ifndef ECO_FIRST_LAST_COMPONENT_EXECUTION_H_
#define ECO_FIRST_LAST_COMPONENT_EXECUTION_H_

#include "eco/op_mode.h"
#include "eco/memset.h"


namespace eco
{

  /// Represents the operation mode of the first and last execution of the component within a cycle.
  /// If the operation mode for first and last execution are equal the component is triggered only once.
  /// The data is undefined if both operation modes are set to "OPM_UNDEF".
  struct FirstLastComponentExecution
  {
    OpMode firstExecutionOpMode;
    OpMode lastExecutionOpMode;
  };

  inline ::eco::FirstLastComponentExecution createFirstLastComponentExecution()
  {
    FirstLastComponentExecution m;
    (void)::eco::memset(&m, 0U, sizeof(FirstLastComponentExecution));
    return m;
  }

} // namespace eco

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::eco::FirstLastComponentExecution create_default()
  {
      return ::eco::createFirstLastComponentExecution();
  }
}


#endif // ECO_FIRST_LAST_COMPONENT_EXECUTION_H_
