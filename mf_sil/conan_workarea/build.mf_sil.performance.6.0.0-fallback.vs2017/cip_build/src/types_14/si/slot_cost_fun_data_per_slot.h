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

#ifndef SI_SLOT_COST_FUN_DATA_PER_SLOT_H_
#define SI_SLOT_COST_FUN_DATA_PER_SLOT_H_

#include "lsm_geoml/size_type.h"
#include "si/slot_cost_fun_data_per_opt_phase.h"


namespace si
{

  /// @brief Array of slot cost function data for every optimization phase.
  struct SlotCostFunDataPerSlot
  {
    ///@range{0,3}
    ///@unit{nu}
    ///@brief Describes how many slot cost function data were already added to the array.
    ::lsm_geoml::size_type actualSize{0U};
    ///@brief Array containing the slot cost function data.
    SlotCostFunDataPerOptPhase array[3]{};
  };

} // namespace si

#endif // SI_SLOT_COST_FUN_DATA_PER_SLOT_H_
