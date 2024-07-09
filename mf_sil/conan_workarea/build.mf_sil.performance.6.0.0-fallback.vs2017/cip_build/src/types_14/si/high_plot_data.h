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

#ifndef SI_HIGH_PLOT_DATA_H_
#define SI_HIGH_PLOT_DATA_H_

#include "si/delimiter_zones_serializable.h"
#include "si/slot_surrounding_lines_serializable.h"
#include "si/virtual_line_info_serializable.h"
#include "si/triangle_expansion_serializable.h"
#include "si/quadrilateral_expansion_serializable.h"
#include "si/quadrilateral_serializable_array.h"
#include "si/high_cluster_zone_serializable.h"
#include "si/cnn_based_parking_slot_serializable.h"


namespace si
{

  /// @brief SI High Plot Data contains the debug data needed for plotting high features.
  struct HighPlotData
  {
    ///@brief Array of aggregated delimiter zones.
    DelimiterZonesSerializable delimiterZones{};
    ///@brief Array of lines surrounding slots.
    SlotSurroundingLinesSerializable surroundingLinesPerSlot{};
    ///@brief Array of virtual line information.
    VirtualLineInfoSerializable virtualLinesInfo{};
    ///@brief Array containing the triangular side expansion RoIs.
    TriangleExpansionSerializable triangleExpansionRois{};
    ///@brief Array containing the rectangular side expansion RoIs.
    QuadrilateralExpansionSerializable sideExpansionRois{};
    ///@brief Array of blow up rectangles, which describe how far CNN slots may be blown up.
    QuadrilateralSerializableArray cnnBlowUpLimits{};
    ///@brief Array of blow up rectangles, which describe how far slots may be blown up.
    QuadrilateralSerializableArray slotBlowUpLimits{};
    ///@brief Array of rectangular slots.
    QuadrilateralSerializableArray rectSlots{};
    ///@brief Array of cluster zones.
    HighClusterZoneSerializable clusterZones{};
    ///@brief Array of CNN based parking slots.
    CnnBasedParkingSlotSerializable cnnBasedParkingSlots{};
  };

} // namespace si

#endif // SI_HIGH_PLOT_DATA_H_
