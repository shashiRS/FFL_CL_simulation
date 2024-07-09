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

#ifndef AVGA_SWC_AVGA_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_C_H_
#define AVGA_SWC_AVGA_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_C_H_

#include "avga_swc/automated_vehicle_guidance_request_type_c.h"
#include "eco/memset_c.h"

typedef struct
{
    AVGA_SWC_AutomatedVehicleGuidanceRequestType automatedVehicleGuidanceRequest;
} AVGA_SWC_AVGA_AutomatedVehicleGuidanceRequest;

inline AVGA_SWC_AVGA_AutomatedVehicleGuidanceRequest create_AVGA_SWC_AVGA_AutomatedVehicleGuidanceRequest(void)
{
  AVGA_SWC_AVGA_AutomatedVehicleGuidanceRequest m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // AVGA_SWC_AVGA_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_C_H_
