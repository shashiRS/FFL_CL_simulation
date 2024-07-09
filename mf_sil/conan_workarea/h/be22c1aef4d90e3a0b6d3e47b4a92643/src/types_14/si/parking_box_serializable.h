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

#ifndef SI_PARKING_BOX_SERIALIZABLE_H_
#define SI_PARKING_BOX_SERIALIZABLE_H_

#include "Platform_Types.h"
#include "si/slot_coordinates_m_serializable.h"
#include "si/parking_scenario_types.h"
#include "si/parking_scenario_side_types.h"
#include "si/parking_box_delimiter.h"
#include "si/virtual_line_serializable.h"
#include "cml/vec2_df_pod.h"


namespace si
{

  /// description of potential parking boxes
  struct ParkingBoxSerializable
  {
    ///@range{0,65535}
    ///ID of the parking box
    uint16 parkingBoxID_nu{65535U};
    ///@unit{m}
    ///vertices describing the parking box polygon
    SlotCoordinates_mSerializable slotCoordinates_m{};
    ///@unit{Percent}
    ///@range{0,100}
    ///existence probability of the parking box
    uint8 existenceProb_perc{};
    ///@range{0,10}
    ///type of parking sccenario associated with this parking box
    ParkingScenarioTypes parkingScenario_nu{};
    ///@range{0,3}
    ///marks which side of the road is the parking scenario
    ParkingScenarioSideTypes parkingScenarioSide_nu{};
    ///@unit{nu}
    ///delimiter of a parking box
    ParkingBoxDelimiter delimiters[8]{};
    ///@range{0,255}
    ///number of valid entries in delimiter list
    uint8 numValidDelimiters_nu{};
    ///@range{0,255}
    ///number of virtual lines for this parking box
    uint8 numVirtualLines_nu{};
    ///@unit{nu}
    ///virtual line constructed to facilitate a smooth orientation alignment of the parked vehicle
    VirtualLineSerializable virtualLines[4]{};
    uint16 groupID_nu{65535U};
    uint8 priority_nu{255U};
    ///the edge of the parking box on the road side
    ::cml::Vec2Df_POD parkingBoxRoadSideEdge[2]{};
    ///boolean showing if parkingBoxRoadSideEdge value had been filled with valid data
    boolean hasValueParkingBoxRoadSideEdge{};
    ///the road side edge of the CNN parking box
    ::cml::Vec2Df_POD cnnBoxRoadSideEdge[2]{};
    ///boolean showing if cnnBoxRoadSideEdge value had been filled with valid data
    boolean hasValueCnnBoxRoadSideEdge{};
  };

} // namespace si

#endif // SI_PARKING_BOX_SERIALIZABLE_H_
