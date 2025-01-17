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

#ifndef AP_COMMON_FC_PARKSM_SYS_FUNC_PARAMS_H_
#define AP_COMMON_FC_PARKSM_SYS_FUNC_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_common
{

  struct FC_PARKSM_Sys_Func_Params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
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
  };

  inline ::ap_common::FC_PARKSM_Sys_Func_Params createFC_PARKSM_Sys_Func_Params()
  {
    FC_PARKSM_Sys_Func_Params m;
    (void)::eco::memset(&m, 0U, sizeof(FC_PARKSM_Sys_Func_Params));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace ap_common

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_common::FC_PARKSM_Sys_Func_Params create_default()
  {
      return ::ap_common::createFC_PARKSM_Sys_Func_Params();
  }
}


#endif // AP_COMMON_FC_PARKSM_SYS_FUNC_PARAMS_H_
