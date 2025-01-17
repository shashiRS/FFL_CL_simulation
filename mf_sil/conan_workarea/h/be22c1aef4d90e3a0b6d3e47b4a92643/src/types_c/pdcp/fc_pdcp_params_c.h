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

#ifndef PDCP_FC_PDCP_PARAMS_C_H_
#define PDCP_FC_PDCP_PARAMS_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// PDCP Parameters
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///The x coordinates of the inner sectors contour
    float32 PDC_P_SECTOR_INNER_COORDS_X_M[16];
    ///The y coordinates of the inner sectors contour
    float32 PDC_P_SECTOR_INNER_COORDS_Y_M[16];
    ///The x coordinates of the outer sectors contour
    float32 PDC_P_SECTOR_OUTER_COORDS_X_M[16];
    ///The y coordinates of the outer sectors contour
    float32 PDC_P_SECTOR_OUTER_COORDS_Y_M[16];
    ///Sector length
    float32 PDC_P_SECTOR_LENGTH_M;
    ///Percentage from slice length, based on which the hysteresis length is computed
    float32 PDC_P_DIST_HYST_PERCENTAGE_NU;
    ///Number of sectors for the front side of the car
    uint8 PDC_P_NUM_FRONT_SECTORS_NU;
    ///Number of sectors for the right side of the car
    uint8 PDC_P_NUM_RIGHT_SECTORS_NU;
    ///Number of sectors for the rear side of the car
    uint8 PDC_P_NUM_REAR_SECTORS_NU;
    ///Number of sectors for the left side of the car
    uint8 PDC_P_NUM_LEFT_SECTORS_NU;
    ///Number of criticality levels
    uint8 PDC_P_NUM_CRITICALITY_LEVELS_NU;
    ///Number of slices in each level
    uint8 PDC_P_NUM_SLICES_PER_LEVEL_NU;
    ///Minimum confidence required for low obstacle validation by the PDCP component
    uint8 PDC_P_LOW_OBSTACLE_MIN_CONFIDENCE_NU;
    ///The minimum existance probability for static obstacles
    uint8 PDC_P_MIN_EXISTANCE_PROB_STATIC_OBJ_NU;
    ///The minimum existance probability for dynamic obstacles
    uint8 PDC_P_MIN_EXISTANCE_PROB_DYNAMIC_OBJ_NU;
    ///Flag for switching on/off warning for low obstacles
    boolean PDC_P_WARN_ON_LOW_OBSTACLE_NU;
    ///Flag for switching on/off filtering of obstacles based on their positions relative to an internal PDCP Region of Interest
    boolean PDC_P_ACTIVATE_INTERNAL_ROI_NU;
} PDCP_FC_PDCP_Params;

inline PDCP_FC_PDCP_Params create_PDCP_FC_PDCP_Params(void)
{
  PDCP_FC_PDCP_Params m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // PDCP_FC_PDCP_PARAMS_C_H_
