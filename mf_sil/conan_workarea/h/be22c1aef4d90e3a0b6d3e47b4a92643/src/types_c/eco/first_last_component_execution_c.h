//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

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

#ifndef ECO_FIRST_LAST_COMPONENT_EXECUTION_C_H_
#define ECO_FIRST_LAST_COMPONENT_EXECUTION_C_H_

#include "eco/op_mode_c.h"
#include "eco/memset_c.h"

/// Represents the operation mode of the first and last execution of the component within a cycle.
/// If the operation mode for first and last execution are equal the component is triggered only once.
/// The data is undefined if both operation modes are set to "OPM_UNDEF".
typedef struct
{
    ECO_OpMode firstExecutionOpMode;
    ECO_OpMode lastExecutionOpMode;
} ECO_FirstLastComponentExecution;

inline ECO_FirstLastComponentExecution create_ECO_FirstLastComponentExecution(void)
{
  ECO_FirstLastComponentExecution m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // ECO_FIRST_LAST_COMPONENT_EXECUTION_C_H_
