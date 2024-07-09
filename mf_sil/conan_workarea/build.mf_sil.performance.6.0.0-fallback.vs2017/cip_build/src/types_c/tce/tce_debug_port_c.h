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

#ifndef TCE_TCE_DEBUG_PORT_C_H_
#define TCE_TCE_DEBUG_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// None
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ///timestamp_us
    ECO_SignalHeader sSigHeader;
    ///@unit{nu}
    ///freespace for MTS debug values
    sint32 debugInt[10];
    ///@unit{nu}
    ///freespace for MTS debug values
    float32 debugFloat[10];
    ///@unit{m}
    ///internal learned circumference for front right tire.
    float32 tireCircInternalFL_m;
    ///@unit{m}
    ///internal learned circumference for rear left tire.
    float32 tireCircInternalFR_m;
    ///@unit{m}
    ///internal learned circumference for rear right tire.
    float32 tireCircInternalRL_m;
    ///@unit{m}
    ///internal learned circumference for front left tire.
    float32 tireCircInternalRR_m;
    ///@unit{m}
    ///calculated vehicle displacement from the GPS measurements.
    float32 gpsDisplacements_m;
    ///@unit{m}
    ///vehicle driven distance calculated by wheel sensor displacements.
    float32 vehDrivenDistanceWhl_m;
    ///@unit{m}
    ///vehicle driven distance calculated by GPS displacements.
    float32 vehDrivenDistanceGPS_m;
    ///@unit{rad}
    ///summation of the average of rear wheel angular rotations
    float32 sumWheelRotations_rad;
    ///@unit{m}
    ///earth radius calculated from the ellipsoid model and latitude angle measured by GPS
    float32 earthEllipsoidRadius_m;
    ///@unit{bool}
    ///flag indicating reception of new GPS measurement.
    boolean gpsUpdateFlag;
} TCE_TceDebugPort;

inline TCE_TceDebugPort create_TCE_TceDebugPort(void)
{
  TCE_TceDebugPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // TCE_TCE_DEBUG_PORT_C_H_
