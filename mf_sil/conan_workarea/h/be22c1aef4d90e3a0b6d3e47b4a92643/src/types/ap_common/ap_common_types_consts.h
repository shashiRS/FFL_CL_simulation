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

#ifndef AP_COMMON_AP_COMMON_TYPES_CONSTS_H_
#define AP_COMMON_AP_COMMON_TYPES_CONSTS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_common
{

  struct AP_COMMON_TYPES_Consts
  {
    enum { AP_G_MAX_NUM_STATIC_OBJ_NU = 32U};
    enum { AP_G_MAX_NUM_SENSOR_TYPES_NU = 1U};
    enum { AP_G_MAX_NUM_PTS_STATIC_POLY_NU = 10U};
    enum { AP_G_MAX_NUM_DYN_OBJECTS_NU = 4U};
    enum { AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU = 4U};
    enum { AP_G_MAX_NUM_P_SPACE_MARKINGS_NU = 50U};
    enum { AP_G_NUM_POINTS_PER_PARKING_LINE_NU = 2U};
    enum { AP_G_MAX_NUM_POSES_PREV_PATH_NU = 40U};
    enum { AP_G_MAX_NUM_LANE_BOUNDARIES_NU = 3U};
    enum { AP_G_MAX_NUM_LANE_BOUND_PTS_NU = 20U};
    enum { AP_G_MAX_NUM_LANES_NU = 2U};
    enum { AP_G_MAX_NUM_US_SENSORS_NU = 12U};
    enum { AP_G_MAX_NUM_PARKING_BOXES_NU = 6U};
    enum { AP_G_MAX_NUM_EXTERNAL_POSES_NU = 4U};
    enum { AP_G_MAX_NUM_SV_CAMS_NU = 4U};
    enum { AP_G_MAX_NUM_P_BOX_VERTICES_NU = 4U};
    enum { AP_G_MAX_NUM_P_BOX_DELIMITERS_NU = 8U};
    enum { AP_G_MAX_NUM_VIRTUAL_LINES_NU = 4U};
    enum { AP_G_NUM_TAPOS_INFLATED_OBJ_NU = 8U};
    enum { AP_G_MAX_NUM_TARGET_POSES_NU = 8U};
    enum { COLL_G_MAX_NUM_DYN_OBJECTS_NU = 1U};
    enum { COLL_G_MAX_NUM_STATIC_OBJ_NU = 32U};
    enum { AP_G_MAX_NUM_PTS_IN_VIRTUAL_LINE_NU = 2U};
    enum { AP_V_MIRROR_SHAPE_MAX_SIZE_NU = 4U};
    enum { AP_V_HITCH_SHAPE_MAX_SIZE_NU = 4U};
    enum { AP_V_WHEEL_SHAPE_MAX_SIZE_NU = 6U};
    enum { AP_V_VEHICLE_SHAPE_MAX_SIZE_NU = 16U};
    enum { AP_V_NUM_WHEELS_NU = 4U};
  };

  inline ::ap_common::AP_COMMON_TYPES_Consts createAP_COMMON_TYPES_Consts()
  {
    AP_COMMON_TYPES_Consts m;
    (void)::eco::memset(&m, 0U, sizeof(AP_COMMON_TYPES_Consts));
    return m;
  }

} // namespace ap_common

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_common::AP_COMMON_TYPES_Consts create_default()
  {
      return ::ap_common::createAP_COMMON_TYPES_Consts();
  }
}


#endif // AP_COMMON_AP_COMMON_TYPES_CONSTS_H_
