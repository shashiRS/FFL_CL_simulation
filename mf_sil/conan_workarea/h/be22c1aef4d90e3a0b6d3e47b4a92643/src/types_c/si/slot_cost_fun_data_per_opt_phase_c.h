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

#ifndef SI_SLOT_COST_FUN_DATA_PER_OPT_PHASE_C_H_
#define SI_SLOT_COST_FUN_DATA_PER_OPT_PHASE_C_H_

#include "si/quadrilateral_serializable_c.h"
#include "si/virtual_line_serializable_array_c.h"
#include "si/virtual_line_index_serializable_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ///@brief Rectangular left side RoI.
    SI_QuadrilateralSerializable roiLeft;
    ///@brief Rectangular right side RoI.
    SI_QuadrilateralSerializable roiRight;
    ///@brief RoI for curb alignment.
    SI_QuadrilateralSerializable roiCurb;
    ///@brief Array of virtual lines.
    SI_VirtualLineSerializableArray virtualLines;
    ///@brief Index array for the best virtual lines.
    SI_VirtualLineIndexSerializable bestVLIndices;
    ///@range{0,90}
    ///@unit{°}
    ///@brief Rotation angle between the parking slot and the curb virtual line.
    float32 curbAlignment_deg;
    ///@range{-90° * SI_params.weightCurbOrientation,0}
    ///@unit{nu}
    ///@brief Alignment performance.
    float32 curbAlignmentPerf_nu;
    ///@range{0,3.4028237e+38}
    ///@unit{m}
    ///@brief Distance to the closest object from the front left corner of the parking slot.
    float32 closestFL_m;
    ///@range{0,3.4028237e+38}
    ///@unit{m}
    ///@brief Distance to the closest object from the front right corner of the parking slot.
    float32 closestFR_m;
    ///@range{-3.4028237e+38,3.4028237e+38}
    ///@unit{nu}
    ///@brief Pull performance. Currently only high objects at opening are targeted.
    float32 pullPerformFront_nu;
    ///@range{0,3.4028237e+38}
    ///@unit{m}
    ///@brief Size of the parking slot opening.
    float32 opening_m;
    ///@range{1.175494e-38,3.4028237e+38}
    ///@unit{nu}
    ///@brief The cost function that can be adjusted to control the slot rotation optimization.
    float32 openingPerf_nu;
    ///@range{-90,90}
    ///@unit{°}
    ///@brief Side alignment error in degrees. Rotation angle between the side virtual line and the road normal vector. 0° means perfect alignment while +/-90° means total misalignment or no side object to align to.
    float32 sideAlignmentError_deg;
    ///@range{-0.5F * π * SI_params.sideAlignmentWeight_mpr,1.5F * π * SI_params.sideAlignmentWeight_mpr}
    ///@unit{m}
    ///@brief Side alignment performance.
    float32 sideAlignmentPerf_m;
    ///@range{-90,90}
    ///@unit{°}
    ///@brief Wing alignment error in degrees. Rotation angle between the wing virtual line and the road side edge. 0° means perfect alignment while +/-90° means total misalignment or no wing object to align to.
    float32 wingAlignmentError_deg;
    ///@range{-0.5F * π * SI_params.wingAlignmentWeight_mpr,1.5F * π * SI_params.wingAlignmentWeight_mpr}
    ///@unit{m}
    ///@brief Wing alignment performance.
    float32 wingAlignmentPerf_m;
    ///@range{-3.4028237e+38,3.4028237e+38}
    ///@unit{m}
    ///@brief Combined performance based on all cost function components.
    float32 overallPerf_m;
} SI_SlotCostFunDataPerOptPhase;

inline SI_SlotCostFunDataPerOptPhase create_SI_SlotCostFunDataPerOptPhase(void)
{
  SI_SlotCostFunDataPerOptPhase m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.roiLeft = create_SI_QuadrilateralSerializable();
  m.roiRight = create_SI_QuadrilateralSerializable();
  m.roiCurb = create_SI_QuadrilateralSerializable();
  m.virtualLines = create_SI_VirtualLineSerializableArray();
  m.bestVLIndices = create_SI_VirtualLineIndexSerializable();
  m.curbAlignment_deg = 0.0F;
  m.curbAlignmentPerf_nu = 0.0F;
  m.closestFL_m = 0.0F;
  m.closestFR_m = 0.0F;
  m.pullPerformFront_nu = 0.0F;
  m.opening_m = 0.0F;
  m.openingPerf_nu = 0.0F;
  m.sideAlignmentError_deg = 0.0F;
  m.sideAlignmentPerf_m = 0.0F;
  m.wingAlignmentError_deg = 0.0F;
  m.wingAlignmentPerf_m = 0.0F;
  m.overallPerf_m = 0.0F;
  return m;
}

#endif // SI_SLOT_COST_FUN_DATA_PER_OPT_PHASE_C_H_
