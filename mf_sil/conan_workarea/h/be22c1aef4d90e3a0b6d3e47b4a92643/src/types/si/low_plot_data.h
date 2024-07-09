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

#ifndef SI_LOW_PLOT_DATA_H_
#define SI_LOW_PLOT_DATA_H_

#include "si/quadrilateral_serializable_array.h"
#include "si/triangle_serializable_array.h"
#include "si/ego_shape_polygon_serializable.h"
#include "si/transformed_scenario_serializable.h"
#include "si/quadrilateral_serializable.h"
#include "si/slot_cost_fun_data_per_slot_serializable.h"
#include "eco/memset.h"


namespace si
{

  /// @brief SI Low Plot Data contains the debug data needed for plotting low features.
  struct LowPlotData
  {
    ///@brief Array of left object RoI. If object overlaps this, the slot is NOT shrunk (we assume that the area was sensed correctly).
    QuadrilateralSerializableArray shrinkingObjectLeftRoi;
    ///@brief Array of right object RoI. If object overlaps this, the slot is NOT shrunk (we assume that the area was sensed correctly).
    QuadrilateralSerializableArray shrinkingObjectRightRoi;
    ///@brief Array of left ego RoI. If the ego overlaps that area, the slot is NOT shrunk (we assume that the ego is close enough to sense the relevant area correctly).
    TriangleSerializableArray shrinkingEgoLeftRoi;
    ///@brief Array of right ego RoI. If the ego overlaps that area, the slot is NOT shrunk (we assume that the ego is close enough to sense the relevant area correctly).
    TriangleSerializableArray shrinkingEgoRightRoi;
    ///@brief Array of slots before postprocessing.
    QuadrilateralSerializableArray slotsBeforePostprocessing;
    ///@brief Array of slots after postprocessing.
    QuadrilateralSerializableArray slotsAfterPostprocessing;
    ///@brief Array of per-slot sub-em RoIs.
    QuadrilateralSerializableArray perSlotSubEmRoi;
    ///@brief Array of current maneuvering RoIs.
    QuadrilateralSerializableArray perSlotAboveSubEmRoi;
    ///@brief Array of plot per-slot follow-em RoIs (used for slot entrance ROIs because follow-em ROI doesn"t exist any more).
    QuadrilateralSerializableArray perSlotFollowROI;
    ///@brief Array of per-slot oncoming-em RoIs.
    QuadrilateralSerializableArray perSlotOncomingROI;
    ///@brief Array of vertices corresponding to the inflated shape of the ego vehicle.
    EgoShapePolygonSerializable inflatedEgoShape;
    ///@brief Array of aggregate information related to the different steps of optimization.
    TransformedScenarioSerializable slot1InSlotCoordinates;
    ///@brief Dynamic object classification RoI.
    QuadrilateralSerializable dynObjClassifierROI;
    ///@brief Dynamic object classification RoI for overhead beams.
    QuadrilateralSerializable dynObjClassifierROI2;
    ///@brief Array of slot cost function data for slots.
    SlotCostFunDataPerSlotSerializable slotCostFunctionData;
  };

  inline ::si::LowPlotData createLowPlotData()
  {
    LowPlotData m;
    (void)::eco::memset(&m, 0U, sizeof(LowPlotData));
    m.shrinkingObjectLeftRoi = createQuadrilateralSerializableArray();
    m.shrinkingObjectRightRoi = createQuadrilateralSerializableArray();
    m.shrinkingEgoLeftRoi = createTriangleSerializableArray();
    m.shrinkingEgoRightRoi = createTriangleSerializableArray();
    m.slotsBeforePostprocessing = createQuadrilateralSerializableArray();
    m.slotsAfterPostprocessing = createQuadrilateralSerializableArray();
    m.perSlotSubEmRoi = createQuadrilateralSerializableArray();
    m.perSlotAboveSubEmRoi = createQuadrilateralSerializableArray();
    m.perSlotFollowROI = createQuadrilateralSerializableArray();
    m.perSlotOncomingROI = createQuadrilateralSerializableArray();
    m.inflatedEgoShape = createEgoShapePolygonSerializable();
    m.slot1InSlotCoordinates = createTransformedScenarioSerializable();
    m.dynObjClassifierROI = createQuadrilateralSerializable();
    m.dynObjClassifierROI2 = createQuadrilateralSerializable();
    m.slotCostFunctionData = createSlotCostFunDataPerSlotSerializable();
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::LowPlotData create_default()
  {
      return ::si::createLowPlotData();
  }
}


#endif // SI_LOW_PLOT_DATA_H_
