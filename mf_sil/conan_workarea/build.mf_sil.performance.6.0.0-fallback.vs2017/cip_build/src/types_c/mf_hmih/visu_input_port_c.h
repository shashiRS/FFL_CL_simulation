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

#ifndef MF_HMIH_VISU_INPUT_PORT_C_H_
#define MF_HMIH_VISU_INPUT_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "mf_hmih/visu_hmidata_c.h"
#include "ap_hmitoap/screen_types_c.h"
#include "ap_tp/traj_plan_visu_port_c.h"
#include "mf_hmih/parking_target_poses_c.h"
#include "eco/memset_c.h"

/// Keep information for Visu: e.g.trajectory planner, target poses, distance to stop.
typedef struct
{
    ///Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ///Common header for all structured data types (e.g. timestamp)
    ECO_SignalHeader sSigHeader;
    ///Value for the remaining distance to stop the vehicle.
    float32 distanceToStopReq_m;
    ///HMI data for Visu
    MF_HMIH_VisuHMIData visuHMIData;
    ///Description which stream should be renderd
    AP_HMITOAP_ScreenTypes HmiOutUserActScreenReq_u8;
    ///Trajectory planner data for Visu
    AP_TP_TrajPlanVisuPort trajPlanVisuPort_nu;
    ///Target poses information for Visu
    MF_HMIH_ParkingTargetPoses parkingPosesVisu;
    ///Indicates if the slot was selected by the user or if the selected slot was preselected by HMIHandler.
    boolean driverSelection_nu;
} MF_HMIH_VisuInputPort;

inline MF_HMIH_VisuInputPort create_MF_HMIH_VisuInputPort(void)
{
  MF_HMIH_VisuInputPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.visuHMIData = create_MF_HMIH_VisuHMIData();
  m.trajPlanVisuPort_nu = create_AP_TP_TrajPlanVisuPort();
  m.parkingPosesVisu = create_MF_HMIH_ParkingTargetPoses();
  return m;
}

#endif // MF_HMIH_VISU_INPUT_PORT_C_H_
