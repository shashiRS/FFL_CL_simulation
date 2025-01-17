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

#ifndef SI_STATIC_OBJECT_SERIALIZABLE_H_
#define SI_STATIC_OBJECT_SERIALIZABLE_H_

#include "Platform_Types.h"
#include "si/object_trend.h"
#include "si/static_obj_shape_serializable.h"
#include "si/static_obj_heigth_type.h"
#include "eco/memset.h"


namespace si
{

  /// structures comprising the static environment of the vehicle
  struct StaticObjectSerializable
  {
    ///@range{0,65535}
    ///Unique identifier of the object of which this structure is part (a physical object maybe split into multiple structures to comply to the convexity requirement)
    uint16 refObjID_nu;
    ///@unit{Percent}
    ///@range{0,100}
    ///Existence probability of the structure
    uint8 existenceProb_perc;
    ///timestamp of the first corresponding detection
    uint16 objAgeInCycles_nu;
    ///timestamp of the last time, the object has been updated
    uint16 objMeasLastUpdateInCycles_nu;
    ///timestamp of the last time, the object-trend has been updated
    uint16 objTrendLastUpdateInCycles_nu;
    ///@range{0,255}
    ///Trend information about the object.
    ObjectTrend objTrend_nu;
    ///true if structure has only been read from non volatile RAM and not (yet) been confirmed by new measurements
    boolean readFromNVRAM_nu;
    ///@unit{m}
    ///Positions of the vertices
    StaticObjShapeSerializable objShape_m;
    ///@range{0,5}
    ///height calss of the shape
    StaticObjHeigthType objHeightClass_nu;
    ///@unit{Percent}
    ///@range{0,100}
    ///confidence in the height classification
    uint8 objHeightClassConfidence_perc;
  };

  inline ::si::StaticObjectSerializable createStaticObjectSerializable()
  {
    StaticObjectSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(StaticObjectSerializable));
    m.refObjID_nu = 65535U;
    m.objShape_m = createStaticObjShapeSerializable();
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::StaticObjectSerializable create_default()
  {
      return ::si::createStaticObjectSerializable();
  }
}


#endif // SI_STATIC_OBJECT_SERIALIZABLE_H_
