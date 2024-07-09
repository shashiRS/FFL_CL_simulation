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

#ifndef US_EM_US_EM_NEIGHBORING_FILTER_CFG_INTERN_H_
#define US_EM_US_EM_NEIGHBORING_FILTER_CFG_INTERN_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_em
{

  struct UsEmNeighboringFilterCfgIntern
  {
    ///maximum distance away from the car to keep the points  = 5.0f
    float32 keep_distance;
    ///parameters to define the region around the point where we look for neighbors = 0.18f
    float32 neighbor_region_threshold;
    ///maximum number of cycles after which the points will be deleted
    uint16 keep_cycles;
    ///the maximum length of the object tracker buffer
    uint16 keep_buffer_length;
    ///filter threshold based on the count of neighbors
    uint16 neighbor_count_threshold;
    ///filter threshold based on the count of neighbors used only without the paths
    uint16 neighbor_count_threshold_only;
    ///filter threshold based on the number of pathes
    uint8 neighbor_paths_threshold;
    ///use abs(x)+abs(y) as distance and not sqrt(x^2+y^2) = false
    boolean use_absolute_distance;
    ///weight the each neighbor by its own variance of the direction = true
    boolean weight_neighbors_by_dir_variance;
  };

  inline ::us_em::UsEmNeighboringFilterCfgIntern createUsEmNeighboringFilterCfgIntern()
  {
    UsEmNeighboringFilterCfgIntern m;
    (void)::eco::memset(&m, 0U, sizeof(UsEmNeighboringFilterCfgIntern));
    return m;
  }

} // namespace us_em

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_em::UsEmNeighboringFilterCfgIntern create_default()
  {
      return ::us_em::createUsEmNeighboringFilterCfgIntern();
  }
}


#endif // US_EM_US_EM_NEIGHBORING_FILTER_CFG_INTERN_H_
