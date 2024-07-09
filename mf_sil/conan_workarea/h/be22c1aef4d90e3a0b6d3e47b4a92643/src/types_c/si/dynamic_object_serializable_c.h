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

#ifndef SI_DYNAMIC_OBJECT_SERIALIZABLE_C_H_
#define SI_DYNAMIC_OBJECT_SERIALIZABLE_C_H_

#include "Platform_Types.h"
#include "si/dynamic_obj_shape_serializable_c.h"
#include "cml/vec2_df_pod_c.h"
#include "si/obj_measurement_state_c.h"
#include "eco/memset_c.h"

/// dynamic objects
typedef struct
{
    ///@unit{Percent}
    ///@range{0,100}
    ///Existence probability of the dynamic object
    uint8 existenceProb_perc;
    SI_DynamicObjShapeSerializable objShape_m;
    ///@unit{m / s}
    ///velocity vector of the object
    CML_Vec2Df_POD vel_mps;
    ///@unit{m / s^2}
    CML_Vec2Df_POD accel_mps2;
    ///@unit{Radian}
    ///@range{-3.14159265359,3.14159265359}
    float32 headingAngle_rad;
    ///@range{0,4}
    ///Measurement state of the dynamic object
    SI_ObjMeasurementState measurementState_nu;
    uint16 refObjID_nu;
} SI_DynamicObjectSerializable;

inline SI_DynamicObjectSerializable create_SI_DynamicObjectSerializable(void)
{
  SI_DynamicObjectSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.objShape_m = create_SI_DynamicObjShapeSerializable();
  m.vel_mps = create_CML_Vec2Df_POD();
  m.accel_mps2 = create_CML_Vec2Df_POD();
  m.refObjID_nu = 255U;
  return m;
}

#endif // SI_DYNAMIC_OBJECT_SERIALIZABLE_C_H_