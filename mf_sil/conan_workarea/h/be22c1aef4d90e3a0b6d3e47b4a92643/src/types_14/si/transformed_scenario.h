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

#ifndef SI_TRANSFORMED_SCENARIO_H_
#define SI_TRANSFORMED_SCENARIO_H_

#include "si/quadrilateral_serializable.h"
#include "si/object_shape_serializable_array.h"


namespace si
{

  /// @brief Aggregate structure containing relevant information about the transformed parking slots.
  struct TransformedScenario
  {
    ///@brief Parking slot before transformation.
    QuadrilateralSerializable parkingSlotBefore{};
    ///@brief Enlarged parking slot.
    QuadrilateralSerializable parkingSlotEnlarged{};
    ///@brief Parking slot after transformation.
    QuadrilateralSerializable parkingSlotAfter{};
    ///@brief Array of object related to a parking slot.
    ObjectShapeSerializableArray objects{};
  };

} // namespace si

#endif // SI_TRANSFORMED_SCENARIO_H_
