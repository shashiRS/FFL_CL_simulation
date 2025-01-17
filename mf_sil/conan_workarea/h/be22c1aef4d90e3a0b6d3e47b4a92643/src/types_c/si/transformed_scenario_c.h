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

#ifndef SI_TRANSFORMED_SCENARIO_C_H_
#define SI_TRANSFORMED_SCENARIO_C_H_

#include "si/quadrilateral_serializable_c.h"
#include "si/object_shape_serializable_array_c.h"
#include "eco/memset_c.h"

/// @brief Aggregate structure containing relevant information about the transformed parking slots.
typedef struct
{
    ///@brief Parking slot before transformation.
    SI_QuadrilateralSerializable parkingSlotBefore;
    ///@brief Enlarged parking slot.
    SI_QuadrilateralSerializable parkingSlotEnlarged;
    ///@brief Parking slot after transformation.
    SI_QuadrilateralSerializable parkingSlotAfter;
    ///@brief Array of object related to a parking slot.
    SI_ObjectShapeSerializableArray objects;
} SI_TransformedScenario;

inline SI_TransformedScenario create_SI_TransformedScenario(void)
{
  SI_TransformedScenario m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.parkingSlotBefore = create_SI_QuadrilateralSerializable();
  m.parkingSlotEnlarged = create_SI_QuadrilateralSerializable();
  m.parkingSlotAfter = create_SI_QuadrilateralSerializable();
  m.objects = create_SI_ObjectShapeSerializableArray();
  return m;
}

#endif // SI_TRANSFORMED_SCENARIO_C_H_
