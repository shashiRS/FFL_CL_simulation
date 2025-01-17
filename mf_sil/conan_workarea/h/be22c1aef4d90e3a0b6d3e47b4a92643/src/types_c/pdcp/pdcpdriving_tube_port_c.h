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

#ifndef PDCP_PDCPDRIVING_TUBE_PORT_C_H_
#define PDCP_PDCPDRIVING_TUBE_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "cml/vec2_df_pod_c.h"
#include "pdcp/drv_tube_display_c.h"
#include "eco/memset_c.h"

/// Signals from PDC Processing containing the driving tube information
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{m}
    ///Radius of the driving tube on the front
    float32 frontRadius_m;
    ///@unit{m}
    ///Radius of the driving tube on the rear
    float32 rearRadius_m;
    ///Center of the turning circle
    CML_Vec2Df_POD turningCircleCenter_nu;
    ///@range{0,2}
    ///Information of where the driving tube should be displayed
    PDCP_DrvTubeDisplay drvTubeDisplay_nu;
    ///Indicates if the driving tube is perfectly straight (radiuses should be ignored)
    boolean straightDrvTube_nu;
} PDCP_PDCPDrivingTubePort;

inline PDCP_PDCPDrivingTubePort create_PDCP_PDCPDrivingTubePort(void)
{
  PDCP_PDCPDrivingTubePort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.turningCircleCenter_nu = create_CML_Vec2Df_POD();
  m.drvTubeDisplay_nu = PDCP_DRV_TUBE_DISPLAY_PDC_DRV_TUBE_NONE;
  return m;
}

#endif // PDCP_PDCPDRIVING_TUBE_PORT_C_H_
