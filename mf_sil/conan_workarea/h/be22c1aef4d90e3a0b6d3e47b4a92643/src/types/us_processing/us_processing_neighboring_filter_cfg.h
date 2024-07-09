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

#ifndef US_PROCESSING_US_PROCESSING_NEIGHBORING_FILTER_CFG_H_
#define US_PROCESSING_US_PROCESSING_NEIGHBORING_FILTER_CFG_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_processing
{

  struct UsProcessingNeighboringFilterCfg
  {
    float32 keep_distance;
    float32 neighbor_region_threshold;
    uint16 keep_cycles;
    uint16 keep_buffer_length;
    uint16 neighbor_count_threshold;
    uint16 neighbor_count_threshold_only;
    uint8 neighbor_paths_threshold;
    boolean use_absolute_distance;
    boolean weight_neighbors_by_dir_variance;
  };

  inline ::us_processing::UsProcessingNeighboringFilterCfg createUsProcessingNeighboringFilterCfg()
  {
    UsProcessingNeighboringFilterCfg m;
    (void)::eco::memset(&m, 0U, sizeof(UsProcessingNeighboringFilterCfg));
    return m;
  }

} // namespace us_processing

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_processing::UsProcessingNeighboringFilterCfg create_default()
  {
      return ::us_processing::createUsProcessingNeighboringFilterCfg();
  }
}


#endif // US_PROCESSING_US_PROCESSING_NEIGHBORING_FILTER_CFG_H_
