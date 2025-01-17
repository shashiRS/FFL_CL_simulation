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

#ifndef SI_DYNAMIC_OBJECT_SERIALIZABLE_H_
#define SI_DYNAMIC_OBJECT_SERIALIZABLE_H_

#include "Platform_Types.h"
#include "si/dynamic_obj_shape_serializable.h"
#include "cml/vec2_df_pod.h"
#include "si/obj_measurement_state.h"
#include "eco/memset.h"


namespace si
{

  /// dynamic objects
  struct DynamicObjectSerializable
  {
    ///@unit{Percent}
    ///@range{0,100}
    ///Existence probability of the dynamic object
    uint8 existenceProb_perc;
    DynamicObjShapeSerializable objShape_m;
    ///@unit{m / s}
    ///velocity vector of the object
    ::cml::Vec2Df_POD vel_mps;
    ///@unit{m / s^2}
    ::cml::Vec2Df_POD accel_mps2;
    ///@unit{Radian}
    ///@range{-3.14159265359,3.14159265359}
    float32 headingAngle_rad;
    ///@range{0,4}
    ///Measurement state of the dynamic object
    ObjMeasurementState measurementState_nu;
    uint16 refObjID_nu;
  };

  inline ::si::DynamicObjectSerializable createDynamicObjectSerializable()
  {
    DynamicObjectSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(DynamicObjectSerializable));
    m.objShape_m = createDynamicObjShapeSerializable();
    m.vel_mps = ::cml::createVec2Df_POD();
    m.accel_mps2 = ::cml::createVec2Df_POD();
    m.refObjID_nu = 255U;
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::DynamicObjectSerializable create_default()
  {
      return ::si::createDynamicObjectSerializable();
  }
}


#endif // SI_DYNAMIC_OBJECT_SERIALIZABLE_H_
