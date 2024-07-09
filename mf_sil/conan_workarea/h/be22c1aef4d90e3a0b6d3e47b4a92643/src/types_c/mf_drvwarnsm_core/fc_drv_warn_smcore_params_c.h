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

#ifndef MF_DRVWARNSM_CORE_FC_DRV_WARN_SMCORE_PARAMS_C_H_
#define MF_DRVWARNSM_CORE_FC_DRV_WARN_SMCORE_PARAMS_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///Maximum allowed cruising speed. Below this threshold  the processing component does the calculations
    float32 DWF_C_CRUISING_SPEED_OFF_MPS;
    ///Coding parameter to enable WHP function
    boolean DWF_C_WHP_ENABLED_NU;
    ///Coding parameter to enable PDW function
    boolean DWF_C_PDW_ENABLED_NU;
    ///The hysteresis to be applied to the speed threshold when switching to off state
    float32 DWF_C_SPEED_HYSTERESIS_MPS;
} MF_DRVWARNSM_CORE_FC_DrvWarnSMCore_Params;

inline MF_DRVWARNSM_CORE_FC_DrvWarnSMCore_Params create_MF_DRVWARNSM_CORE_FC_DrvWarnSMCore_Params(void)
{
  MF_DRVWARNSM_CORE_FC_DrvWarnSMCore_Params m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // MF_DRVWARNSM_CORE_FC_DRV_WARN_SMCORE_PARAMS_C_H_