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

#ifndef SI_PLOT_DATA_C_H_
#define SI_PLOT_DATA_C_H_

#include "si/core_plot_data_c.h"
#include "si/low_plot_data_c.h"
#include "si/high_plot_data_c.h"
#include "eco/memset_c.h"

/// @brief SI Plot Data contains the aggregate of all the debug data needed for plotting.
typedef struct
{
    ///@brief Data structure containing debug data related to core features.
    SI_CorePlotData core;
    ///@brief Data structure containing debug data related to low features.
    SI_LowPlotData low;
    ///@brief Data structure containing debug data related to high features.
    SI_HighPlotData high;
} SI_PlotData;

inline SI_PlotData create_SI_PlotData(void)
{
  SI_PlotData m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.core = create_SI_CorePlotData();
  m.low = create_SI_LowPlotData();
  m.high = create_SI_HighPlotData();
  return m;
}

#endif // SI_PLOT_DATA_C_H_