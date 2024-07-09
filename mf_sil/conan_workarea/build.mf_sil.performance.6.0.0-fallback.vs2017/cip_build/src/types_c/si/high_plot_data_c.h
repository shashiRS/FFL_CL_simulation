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

#ifndef SI_HIGH_PLOT_DATA_C_H_
#define SI_HIGH_PLOT_DATA_C_H_

#include "si/delimiter_zones_serializable_c.h"
#include "si/slot_surrounding_lines_serializable_c.h"
#include "si/virtual_line_info_serializable_c.h"
#include "si/triangle_expansion_serializable_c.h"
#include "si/quadrilateral_expansion_serializable_c.h"
#include "si/quadrilateral_serializable_array_c.h"
#include "si/high_cluster_zone_serializable_c.h"
#include "si/cnn_based_parking_slot_serializable_c.h"
#include "eco/memset_c.h"

/// @brief SI High Plot Data contains the debug data needed for plotting high features.
typedef struct
{
    ///@brief Array of aggregated delimiter zones.
    SI_DelimiterZonesSerializable delimiterZones;
    ///@brief Array of lines surrounding slots.
    SI_SlotSurroundingLinesSerializable surroundingLinesPerSlot;
    ///@brief Array of virtual line information.
    SI_VirtualLineInfoSerializable virtualLinesInfo;
    ///@brief Array containing the triangular side expansion RoIs.
    SI_TriangleExpansionSerializable triangleExpansionRois;
    ///@brief Array containing the rectangular side expansion RoIs.
    SI_QuadrilateralExpansionSerializable sideExpansionRois;
    ///@brief Array of blow up rectangles, which describe how far CNN slots may be blown up.
    SI_QuadrilateralSerializableArray cnnBlowUpLimits;
    ///@brief Array of blow up rectangles, which describe how far slots may be blown up.
    SI_QuadrilateralSerializableArray slotBlowUpLimits;
    ///@brief Array of rectangular slots.
    SI_QuadrilateralSerializableArray rectSlots;
    ///@brief Array of cluster zones.
    SI_HighClusterZoneSerializable clusterZones;
    ///@brief Array of CNN based parking slots.
    SI_CnnBasedParkingSlotSerializable cnnBasedParkingSlots;
} SI_HighPlotData;

inline SI_HighPlotData create_SI_HighPlotData(void)
{
  SI_HighPlotData m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.delimiterZones = create_SI_DelimiterZonesSerializable();
  m.surroundingLinesPerSlot = create_SI_SlotSurroundingLinesSerializable();
  m.virtualLinesInfo = create_SI_VirtualLineInfoSerializable();
  m.triangleExpansionRois = create_SI_TriangleExpansionSerializable();
  m.sideExpansionRois = create_SI_QuadrilateralExpansionSerializable();
  m.cnnBlowUpLimits = create_SI_QuadrilateralSerializableArray();
  m.slotBlowUpLimits = create_SI_QuadrilateralSerializableArray();
  m.rectSlots = create_SI_QuadrilateralSerializableArray();
  m.clusterZones = create_SI_HighClusterZoneSerializable();
  m.cnnBasedParkingSlots = create_SI_CnnBasedParkingSlotSerializable();
  return m;
}

#endif // SI_HIGH_PLOT_DATA_C_H_
