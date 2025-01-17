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

#ifndef US_PROCESSING_US_PROCESSING_REFLECTION_TRACKING_CFG_C_H_
#define US_PROCESSING_US_PROCESSING_REFLECTION_TRACKING_CFG_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    float32 fovDeviation;
    float32 measurementDistanceUncertainty;
    uint64 halfLifeTime;
    float32 halfLifeTimeMultiplicator;
    float32 unbindTrackerDirSigmaIncrease;
    float32 singleTrackerRelaxDirSigma;
    float32 singleTrackerRelaxRange;
    float32 dualTrackerRelaxDirSigma;
    float32 tripleTrackerRelaxDirSigma;
    float32 convexSaturationMaximumIncrement;
    float32 convexSaturationIncrementLimit;
    float32 convexSaturationIncrementMultiplicator;
    float32 rftrOutputFilterRadiusAngleSigma;
    float32 rftrOutputFilterConvexSaturationLimit;
    float32 multiplicityFractionAcceptance;
    float32 multiplicityMaxRadius;
    float32 multiplicityMaxDistanceSinceLastUpdate;
    float32 measurementUncertaintyModifierMax;
    float32 measurementUncertaintyMultplier;
    uint8 stableTrackerMeasurementNumber;
} US_PROCESSING_UsProcessingReflectionTrackingCfg;

inline US_PROCESSING_UsProcessingReflectionTrackingCfg create_US_PROCESSING_UsProcessingReflectionTrackingCfg(void)
{
  US_PROCESSING_UsProcessingReflectionTrackingCfg m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // US_PROCESSING_US_PROCESSING_REFLECTION_TRACKING_CFG_C_H_
