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

#ifndef AP_COMMON_FC_PARKSM_SYS_FUNC_PARAMS_C_H_
#define AP_COMMON_FC_PARKSM_SYS_FUNC_PARAMS_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    float32 AP_G_ESC_TIME_THRESH_S;
    float32 AP_G_MAX_REM_DIST_M;
    float32 AP_G_MAX_IDLE_TIME_S;
    float32 AP_G_ROLLED_DIST_IN_THRESH_M;
    float32 AP_G_ROLLED_DIST_OUT_THRESH_M;
    float32 AP_G_V_SCANNING_THRESH_MPS;
    float32 AP_G_V_START_AVG_THRESH_MPS;
    float32 AP_G_EBA_TIME_THRESH_S;
    float32 AP_G_RCTA_TIME_THRESH_S;
    float32 AP_G_TCS_TIME_THRESH_S;
    float32 AP_G_ABS_TIME_THRESH_S;
    float32 AP_G_EBD_TIME_THRESH_S;
    float32 AP_G_THROTTLE_THRESH_PERC;
    float32 AP_G_MAX_HANDOVER_TIME_S;
    float32 AP_G_STEERING_TORQUE_THRESH_NM;
    float32 AP_G_MAX_RM_DRIVE_DIST_M;
    float32 AP_G_PERC_HMI_DIST_BAR_M;
    float32 AP_G_ROLLED_DIST_RA_THRESH_M;
} AP_COMMON_FC_PARKSM_Sys_Func_Params;

inline AP_COMMON_FC_PARKSM_Sys_Func_Params create_AP_COMMON_FC_PARKSM_Sys_Func_Params(void)
{
  AP_COMMON_FC_PARKSM_Sys_Func_Params m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_COMMON_FC_PARKSM_SYS_FUNC_PARAMS_C_H_