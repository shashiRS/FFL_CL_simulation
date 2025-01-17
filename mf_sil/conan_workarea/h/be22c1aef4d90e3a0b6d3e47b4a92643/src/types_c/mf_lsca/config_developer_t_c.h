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

#ifndef MF_LSCA_CONFIG_DEVELOPER_T_C_H_
#define MF_LSCA_CONFIG_DEVELOPER_T_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Struct that contains all relevant develop parameter data
typedef struct
{
    ///Fake Input switch for static object
    boolean useFakeInputStatic_nu;
    ///Fake Input switch for dynamic object
    boolean useFakeInputDynamic_nu;
    ///Can be used for quick testing of whatever
    boolean genericDeveloperSwitch_nu;
    ///Determine from what level on, messages shall be active
    uint8 messageLevel_nu;
    ///Determine for what component, messages shall be active
    uint8 messageComponent_nu;
    ///Select from differnt FakeEmScenarios
    uint8 fakeEmScenario_nu;
} MF_LSCA_configDeveloper_t;

inline MF_LSCA_configDeveloper_t create_MF_LSCA_configDeveloper_t(void)
{
  MF_LSCA_configDeveloper_t m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_LSCA_CONFIG_DEVELOPER_T_C_H_
