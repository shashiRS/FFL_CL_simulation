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

#ifndef MF_LSCA_LSCA_DYNAMIC_BRAKE_DEBUG_DATA_SERIALIZABLE_H_
#define MF_LSCA_LSCA_DYNAMIC_BRAKE_DEBUG_DATA_SERIALIZABLE_H_

#include "mf_lsca/lsca_dynamic_object_list_brake_serializable.h"
#include "mf_lsca/lsca_dynamic_object_prediction_list_brake_serializable.h"
#include "mf_lsca/lsca_ego_dynamic_roi_shape_serializable.h"
#include "mf_lsca/lsca_ego_simple_body_shape_serializable.h"
#include "eco/memset.h"


namespace mf_lsca
{

  /// Debug data regarding dynamic breaking
  struct LscaDynamicBrakeDebugDataSerializable
  {
    ///The current positions of the surrounding dynamic objects
    LscaDynamicObjectListBrakeSerializable currentPositions;
    ///The end positions of the surrounding dynamic objects (predicted positions after time-to-stop seconds)
    LscaDynamicObjectListBrakeSerializable endPositions;
    ///The collision moment positions of the surrounding dynamic objects (predicted positions in the last cycle before impact, if impact is imminent)
    LscaDynamicObjectListBrakeSerializable collisionMomentPositions;
    ///Polygons describing the trajectory of the surrounding dynamic objects until ego standstill (assuming ego starts braking right now)
    LscaDynamicObjectPredictionListBrakeSerializable objectRois;
    ///Region of interest for dynamic braking
    LscaEgoDynamicRoiShapeSerializable roi;
    ///Ego body shape (4 point approximation)
    LscaEgoSimpleBodyShapeSerializable bodyShape;
  };

  inline ::mf_lsca::LscaDynamicBrakeDebugDataSerializable createLscaDynamicBrakeDebugDataSerializable()
  {
    LscaDynamicBrakeDebugDataSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(LscaDynamicBrakeDebugDataSerializable));
    m.currentPositions = createLscaDynamicObjectListBrakeSerializable();
    m.endPositions = createLscaDynamicObjectListBrakeSerializable();
    m.collisionMomentPositions = createLscaDynamicObjectListBrakeSerializable();
    m.objectRois = createLscaDynamicObjectPredictionListBrakeSerializable();
    m.roi = createLscaEgoDynamicRoiShapeSerializable();
    m.bodyShape = createLscaEgoSimpleBodyShapeSerializable();
    return m;
  }

} // namespace mf_lsca

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lsca::LscaDynamicBrakeDebugDataSerializable create_default()
  {
      return ::mf_lsca::createLscaDynamicBrakeDebugDataSerializable();
  }
}


#endif // MF_LSCA_LSCA_DYNAMIC_BRAKE_DEBUG_DATA_SERIALIZABLE_H_
