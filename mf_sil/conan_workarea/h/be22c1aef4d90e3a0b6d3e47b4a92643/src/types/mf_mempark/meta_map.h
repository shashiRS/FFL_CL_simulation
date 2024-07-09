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

#ifndef MF_MEMPARK_META_MAP_H_
#define MF_MEMPARK_META_MAP_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "mf_mempark/parking_slot.h"
#include "mf_mempark/parking_trajectory.h"
#include "eco/memset.h"


namespace mf_mempark
{

  /// This structure represents a MetaMap which contains arrays of Parking Slots and Parking Trajectories
  struct MetaMap
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    uint8 correspondingMapID;
    uint8 numValidParkingSlots;
    ParkingSlot parkingSlots[1];
    uint8 numValidTrajectories;
    ParkingTrajectory parkingTrajectories[1];
  };

  inline ::mf_mempark::MetaMap createMetaMap()
  {
    MetaMap m;
    (void)::eco::memset(&m, 0U, sizeof(MetaMap));
    m.sSigHeader = ::eco::createSignalHeader();
    {
      const uint64 arraysize = (sizeof(m.parkingSlots) / sizeof(m.parkingSlots[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.parkingSlots[i] = createParkingSlot();
      }
    }
    {
      const uint64 arraysize = (sizeof(m.parkingTrajectories) / sizeof(m.parkingTrajectories[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.parkingTrajectories[i] = createParkingTrajectory();
      }
    }
    return m;
  }

} // namespace mf_mempark

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_mempark::MetaMap create_default()
  {
      return ::mf_mempark::createMetaMap();
  }
}


#endif // MF_MEMPARK_META_MAP_H_
