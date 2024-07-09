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

#ifndef MF_MEMPARK_MAPS_TO_META_MAPS_C_H_
#define MF_MEMPARK_MAPS_TO_META_MAPS_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "mf_mempark/map_idto_meta_map_c.h"
#include "eco/memset_c.h"

/// This structure represents a collection of map IDs each associated with its corresponding MetaMap.
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    uint8 numValidMaps;
    MF_MEMPARK_MapIDToMetaMap mapsMetaMaps[10];
} MF_MEMPARK_MapsToMetaMaps;

inline MF_MEMPARK_MapsToMetaMaps create_MF_MEMPARK_MapsToMetaMaps(void)
{
  MF_MEMPARK_MapsToMetaMaps m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.mapsMetaMaps) / sizeof(m.mapsMetaMaps[0])); ++i)
    {
      m.mapsMetaMaps[i] = create_MF_MEMPARK_MapIDToMetaMap();
    }
  }
  return m;
}

#endif // MF_MEMPARK_MAPS_TO_META_MAPS_C_H_
