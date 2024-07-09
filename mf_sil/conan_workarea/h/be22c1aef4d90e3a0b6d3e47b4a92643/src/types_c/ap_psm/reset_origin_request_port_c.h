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

#ifndef AP_PSM_RESET_ORIGIN_REQUEST_PORT_C_H_
#define AP_PSM_RESET_ORIGIN_REQUEST_PORT_C_H_

#include "lsm_geoml/pose_pod_c.h"
#include "ap_psm/reset_origin_type_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Reset request data in order to request the map reset from SI.
typedef struct
{
    ///@unit{nu}
    ///Relative to current origin, will be disregarded unless resetOrigin_nu equals RRT_RESET_CUSTOM. Contains target pose coordinates.
    LSM_GEOML_Pose_POD transformation;
    ///Type of the reset request.
    AP_PSM_ResetOriginType resetOrigin_nu;
    ///Will start from zero and increase by one on every reset request.
    uint8 resetCounter_nu;
} AP_PSM_ResetOriginRequestPort;

inline AP_PSM_ResetOriginRequestPort create_AP_PSM_ResetOriginRequestPort(void)
{
  AP_PSM_ResetOriginRequestPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.transformation = create_LSM_GEOML_Pose_POD();
  return m;
}

#endif // AP_PSM_RESET_ORIGIN_REQUEST_PORT_C_H_
