// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef US_EM_STATIC_OBJECT_SERIALIZABLE_H_
#define US_EM_STATIC_OBJECT_SERIALIZABLE_H_

#include "Platform_Types.h"
#include "us_em/object_trend.h"
#include "us_em/static_obj_shape_serializable.h"
#include "us_em/static_obj_height_type.h"


namespace us_em
{

  /// Structures comprising the static environment of the vehicle
  struct StaticObjectSerializable
  {
    ///@range{0,65535}
    ///@unit{nu}
    ///Unique identifier of the object of which this structure is part (a physical object maybe split into multiple structures to comply to the convexity requirement)
    uint16 refObjID_nu{65535U};
    ///@unit{Percent}
    ///@range{0,100}
    ///Existence probability of the structure
    uint8 existenceProb_perc{};
    ///@range{0,65535}
    ///@unit{nu}
    ///Algo cycles since the first corresponding detection
    uint16 objAgeInCycles_nu{};
    ///@range{0,65535}
    ///@unit{nu}
    ///Algo cycles since of the last object update
    uint16 objMeasLastUpdateInCycles_nu{};
    ///@range{0,65535}
    ///@unit{nu}
    ///Algo cycles since last cycle where the object-trend has been updated
    uint16 objTrendLastUpdateInCycles_nu{};
    ///@range{0,4}
    ///@unit{nu}
    ///Movement trend of object
    ObjectTrend objTrend_nu{};
    ///@range{0,1}
    ///@unit{nu}
    ///True if structure has only been read from non volatile RAM and not (yet) been confirmed by new measurements
    boolean readFromNVRAM_nu{};
    ///@unit{m}
    ///Positions of the object polygon vertices
    StaticObjShapeSerializable objShape_m{};
    ///@range{0,5}
    ///@unit{nu}
    ///Height class of the object
    StaticObjHeightType objHeightClass_nu{};
    ///@unit{Percent}
    ///@range{0,100}
    ///Confidence in the object height classification
    uint8 objHeightClassConfidence_perc{};
  };

} // namespace us_em

#endif // US_EM_STATIC_OBJECT_SERIALIZABLE_H_