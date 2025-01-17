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

#ifndef SI_RESET_ORIGIN_RESULT_C_H_
#define SI_RESET_ORIGIN_RESULT_C_H_

#include "lsm_geoml/pose_pod_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Information about the previously used coordinate system.
typedef struct
{
    ///@unit{no unit}
    ///Transformation from old to new coordinate system.
    LSM_GEOML_Pose_POD originTransformation;
    ///@range{0,255}
    ///reset counter increasing by one on every reset of the coordinate system origin
    uint8 resetCounter_nu;
} SI_ResetOriginResult;

inline SI_ResetOriginResult create_SI_ResetOriginResult(void)
{
  SI_ResetOriginResult m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.originTransformation = create_LSM_GEOML_Pose_POD();
  return m;
}

#endif // SI_RESET_ORIGIN_RESULT_C_H_
