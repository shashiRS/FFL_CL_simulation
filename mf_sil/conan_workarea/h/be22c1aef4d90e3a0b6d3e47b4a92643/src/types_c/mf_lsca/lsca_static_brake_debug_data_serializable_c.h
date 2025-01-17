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

#ifndef MF_LSCA_LSCA_STATIC_BRAKE_DEBUG_DATA_SERIALIZABLE_C_H_
#define MF_LSCA_LSCA_STATIC_BRAKE_DEBUG_DATA_SERIALIZABLE_C_H_

#include "mf_lsca/lsca_static_object_list_brake_serializable_c.h"
#include "mf_lsca/lsca_ego_body_shape_serializable_c.h"
#include "mf_lsca/lsca_ego_wheel_shape_serializable_c.h"
#include "mf_lsca/lsca_ego_hitch_shape_serializable_c.h"
#include "mf_lsca/lsca_ego_mirror_shape_serializable_c.h"
#include "Platform_Types.h"
#include "cml/vec2_df_pod_c.h"
#include "mf_lsca/lsca_ego_static_roi_shape_serializable_c.h"
#include "eco/memset_c.h"

/// Debug data regarding static breaking
typedef struct
{
    ///The objects that LSCA detected inside its Region of Interest
    MF_LSCA_LscaStaticObjectListBrakeSerializable objectsInRoi;
    ///Polygon that contains all coordinates for the body shape description
    MF_LSCA_LscaEgoBodyShapeSerializable bodyShape;
    ///Polygon that contains all coordinates for the front left wheel shape description
    MF_LSCA_LscaEgoWheelShapeSerializable wheelFrontLeftShape;
    ///Polygon that contains all coordinates for the front right wheel shape description
    MF_LSCA_LscaEgoWheelShapeSerializable wheelFrontRightShape;
    ///Polygon that contains all coordinates for the rear left wheel shape description
    MF_LSCA_LscaEgoWheelShapeSerializable wheelRearLeftShape;
    ///Polygon that contains all coordinates for the rear right wheel shape description
    MF_LSCA_LscaEgoWheelShapeSerializable wheelRearRightShape;
    ///Polygon that contains all coordinates for the hitch shape description
    MF_LSCA_LscaEgoHitchShapeSerializable hitchShape;
    ///Polygon that contains all coordinates for the left mirror shape description
    MF_LSCA_LscaEgoMirrorShapeSerializable mirrorLeftShape;
    ///Polygon that contains all coordinates for the right mirror shape description
    MF_LSCA_LscaEgoMirrorShapeSerializable mirrorRightShape;
    ///Signed rotation until vehicle standstill
    float32 rotationAngleToBrake;
    ///Center or rotation for rotationAngleToBrake
    CML_Vec2Df_POD icr;
    ///Region of interest for static braking
    MF_LSCA_LscaEgoStaticRoiShapeSerializable roi;
} MF_LSCA_LscaStaticBrakeDebugDataSerializable;

inline MF_LSCA_LscaStaticBrakeDebugDataSerializable create_MF_LSCA_LscaStaticBrakeDebugDataSerializable(void)
{
  MF_LSCA_LscaStaticBrakeDebugDataSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.objectsInRoi = create_MF_LSCA_LscaStaticObjectListBrakeSerializable();
  m.bodyShape = create_MF_LSCA_LscaEgoBodyShapeSerializable();
  m.wheelFrontLeftShape = create_MF_LSCA_LscaEgoWheelShapeSerializable();
  m.wheelFrontRightShape = create_MF_LSCA_LscaEgoWheelShapeSerializable();
  m.wheelRearLeftShape = create_MF_LSCA_LscaEgoWheelShapeSerializable();
  m.wheelRearRightShape = create_MF_LSCA_LscaEgoWheelShapeSerializable();
  m.hitchShape = create_MF_LSCA_LscaEgoHitchShapeSerializable();
  m.mirrorLeftShape = create_MF_LSCA_LscaEgoMirrorShapeSerializable();
  m.mirrorRightShape = create_MF_LSCA_LscaEgoMirrorShapeSerializable();
  m.icr = create_CML_Vec2Df_POD();
  m.roi = create_MF_LSCA_LscaEgoStaticRoiShapeSerializable();
  return m;
}

#endif // MF_LSCA_LSCA_STATIC_BRAKE_DEBUG_DATA_SERIALIZABLE_C_H_
