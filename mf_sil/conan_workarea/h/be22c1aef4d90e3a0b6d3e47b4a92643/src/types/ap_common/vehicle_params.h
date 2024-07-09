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

#ifndef AP_COMMON_VEHICLE_PARAMS_H_
#define AP_COMMON_VEHICLE_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_common
{

  struct Vehicle_Params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///Steer ratio (steer wheel angle to effective wheel angle).
    float32 AP_V_STEER_RATIO_NU;
    ///Steer ratio look up table; values for steer wheel angle
    float32 AP_V_STEER_LOOKUP_ST_WHL_RAD[10];
    ///Steer ratio look up table; values for inner wheel angle
    float32 AP_V_STEER_LOOKUP_IN_WHL_RAD[10];
    ///Steer ratio look up table; values for outer wheel angle
    float32 AP_V_STEER_LOOKUP_OUT_WHL_RAD[10];
    ///Steer ratio look up table; values for effective steer angle
    float32 AP_V_STEER_LOOKUP_CTR_WHL_RAD[10];
    ///Vehicle wheelbase
    float32 AP_V_WHEELBASE_M;
    ///Rear overhang of vehicle.
    float32 AP_V_OVERHANG_REAR_M;
    ///Length of vehicle.
    float32 AP_V_LENGTH_M;
    ///Width of vehicle.
    float32 AP_V_WIDTH_M;
    ///Number of teeth used to measure wheel pulses.
    uint8 AP_V_WHEEL_NUMBER_OF_TEETH_NU;
    ///Circumference of front wheels.
    float32 AP_V_TYRE_CIRCUMFERENCE_FRONT_M;
    ///Circumference of rear wheels.
    float32 AP_V_TYRE_CIRCUMFERENCE_REAR_M;
    ///Number of valid vertices of the standard ego vehicle shape polygon
    uint8 AP_V_NUM_STANDARD_SHAPE_PTS;
    ///x-Values of the standard ego vehicle shape polygon. Center of rear axle is at origin.
    float32 AP_V_STANDARD_SHAPE_X_M[16];
    ///y-Values of the standard ego vehicle shape polygon. Center of rear axle is at origin.
    float32 AP_V_STANDARD_SHAPE_Y_M[16];
    ///Maximal number of vertices of the ego vehicle bounding box
    uint8 AP_V_NUM_BOUNDING_PTS;
    ///x-Values of the ego vehicle bounding box
    float32 AP_V_BOUNDINGBOX_X_M[10];
    ///y-Values of the ego vehicle bounding box
    float32 AP_V_BOUNDINGBOX_Y_M[10];
    ///Track at front axle.
    float32 AP_V_TRACK_FRONT_M;
    ///Track at rear axle.
    float32 AP_V_TRACK_REAR_M;
    ///Actual number of mirror vertices
    uint8 AP_V_MIRROR_SHAPE_SIZE_NU;
    ///Actual number of hitch vertices
    uint8 AP_V_HITCH_SHAPE_SIZE_NU;
    ///Actual number of wheel vertices
    uint8 AP_V_WHEEL_SHAPE_SIZE_NU;
    ///x-values of the left mirror shape
    float32 AP_V_LEFT_MIRROR_SHAPE_X_M[4];
    ///y-values of the left mirror shape
    float32 AP_V_LEFT_MIRROR_SHAPE_Y_M[4];
    ///x-values of the right mirror shape
    float32 AP_V_RIGHT_MIRROR_SHAPE_X_M[4];
    ///y-values of the right mirror shape
    float32 AP_V_RIGHT_MIRROR_SHAPE_Y_M[4];
    ///x-values of the trailer hitch shape
    float32 AP_V_HITCH_SHAPE_X_M[4];
    ///y-values of the trailer hitch shape
    float32 AP_V_HITCH_SHAPE_Y_M[4];
    ///x-values of the left front wheel shape
    float32 AP_V_FL_WHEEL_SHAPE_X_M[6];
    ///y-values of the left front wheel shape
    float32 AP_V_FL_WHEEL_SHAPE_Y_M[6];
    ///x-values of the left rear wheel shape
    float32 AP_V_RL_WHEEL_SHAPE_X_M[6];
    ///y-values of the left rear wheel shape
    float32 AP_V_RL_WHEEL_SHAPE_Y_M[6];
    ///x-values of the right rear wheel shape
    float32 AP_V_RR_WHEEL_SHAPE_X_M[6];
    ///y-values of the right rear wheel shape
    float32 AP_V_RR_WHEEL_SHAPE_Y_M[6];
    ///x-values of the right front wheel shape
    float32 AP_V_FR_WHEEL_SHAPE_X_M[6];
    ///y-values of the right front wheel shape
    float32 AP_V_FR_WHEEL_SHAPE_Y_M[6];
    ///Maximum steering angle velocity (at wheels). (based on safery analysis)
    float32 AP_V_MAX_STEER_ANG_VEL_RADPS;
    ///Maximum comfortable steering angle velocity (at wheels).
    float32 AP_V_COMF_STEER_ANG_VEL_RADPS;
    ///Influence of front steering angle on yaw angle (vehicle at standstill)
    float32 AP_V_STEER_ANG_TO_YAW_ANG_NU;
    ///3nd deg polynomial coefficients of characteristic curve steergain outer wheel
    float32 AP_V_STEER_POLY_OUT_WHL_RAD[4];
    ///3nd deg polynomial coefficients of characteristic curve steergain "virtual" center wheel
    float32 AP_V_STEER_POLY_CTR_WHL_RAD[4];
    ///3nd deg polynomial coefficients of characteristic curve steergain inner wheel
    float32 AP_V_STEER_POLY_IN_WHL_RAD[4];
    ///Maximum distance for ego vehicle shape part inflation (max of all situations)
    float32 AP_V_MAX_INFL_DIST_M;
  };

  inline ::ap_common::Vehicle_Params createVehicle_Params()
  {
    Vehicle_Params m;
    (void)::eco::memset(&m, 0U, sizeof(Vehicle_Params));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace ap_common

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_common::Vehicle_Params create_default()
  {
      return ::ap_common::createVehicle_Params();
  }
}


#endif // AP_COMMON_VEHICLE_PARAMS_H_
