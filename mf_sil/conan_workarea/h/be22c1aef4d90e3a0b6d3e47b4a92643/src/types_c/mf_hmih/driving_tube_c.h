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

#ifndef MF_HMIH_DRIVING_TUBE_C_H_
#define MF_HMIH_DRIVING_TUBE_C_H_

#include "Platform_Types.h"
#include "pdcp/drv_tube_display_c.h"
#include "mf_hmih/drv_tube_direction_c.h"
#include "eco/memset_c.h"

/// Driving tube
typedef struct
{
    ///@unit{cm}
    ///Radius of the driving tube on the front.  0..65533: valid radius values 65534: infinite (straight driving tube should be displayed) 65535: unknown (driving tube cannot be determined)
    uint16 frontRadius_cm;
    ///@unit{cm}
    ///Radius of the driving tube on the rear.  0..65533: valid radius values 65534: infinite (straight driving tube should be displayed) 65535: unknown (driving tube cannot be determined)
    uint16 rearRadius_cm;
    ///@range{0,2}
    ///Side where the driving tube should be displayed
    PDCP_DrvTubeDisplay drvTubeDisplay_nu;
    ///@range{0,1}
    ///Driving tube direction
    MF_HMIH_DrvTubeDirection drvTubeDirection_nu;
} MF_HMIH_DrivingTube;

inline MF_HMIH_DrivingTube create_MF_HMIH_DrivingTube(void)
{
  MF_HMIH_DrivingTube m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_HMIH_DRIVING_TUBE_C_H_
