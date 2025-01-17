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

#ifndef AP_TP_FC_TAPOSD_PARAMS_H_
#define AP_TP_FC_TAPOSD_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_tp/pos_def_approach.h"


namespace ap_tp
{

  struct FC_TAPOSD_Params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///Maximal deviation between old target position and new target position in lateral direction that still allows sending the old target pose.
    float32 AP_T_MAX_DEV_UPDATE_LAT_PAR_M{};
    ///Maximal deviation between old target position and new target position in longitudinal direction that still allows sending the old target pose.
    float32 AP_T_MAX_DEV_UPDATE_LONG_PAR_M{};
    ///Maximal deviation between old target angle and new target angle that still allows sending the old target pose.
    float32 AP_T_MAX_DEV_UPD_YAW_PAR_RAD{};
    ///Minimal static object existence percentage to consider object in the calculation of the target pose.
    uint8 AP_T_MIN_OBJ_EXIST_PERC{};
    ///Approach of positioning the ego vehicle in the parking box. 0 = ATTACH_EGO_TO_EDGE; 1 = CENTERING; 2 = ROAD_SIDE_CENTERING; 3 = SHORT_SIDE_CENTERING
    PosDefApproach AP_T_POS_DEF_APPROACH{};
    ///X-movement of parking out pose for parallel parking slot relative to current ego vehicle position (in ego coordinates).
    float32 AP_T_POUT_PAR_POSE_X_M{};
    ///Y-movement of parking out pose for parallel parking slot relative to current ego vehicle position  (in ego coordinates). The sign of this parameter will be flipped according to side of parking out.
    float32 AP_T_POUT_PAR_POSE_Y_M{};
    ///Rotation of parking out pose for parallel parking slot relative to current ego vehicle rotation (in ego coordinates).
    float32 AP_T_POUT_PAR_POSE_YAW_RAD{};
    ///Right movement of parking out pose for perpendicular parking slot relative to road side edge.
    ///Direction is equal to vector from parking box point 1 to point 0 (on road side edge from right edge to left edge).
    ///The sign of this parameter will be flipped according to direction of parking out.
    float32 AP_T_POUT_PERP_POSE_RIGHT_M{};
    ///Gap between parking box and vehicle in target pose for perpendicular parking out with target side left. Y-direction is orthogonal to road side edge pointing into the street.
    float32 AP_T_POUT_PERP_POSE_Y_LEFT_M{};
    ///Gap between parking box and vehicle in target pose for perpendicular parking out with target side right. Y-direction is orthogonal to road side edge pointing into the street.
    float32 AP_T_POUT_PERP_POSE_Y_RIGHT_M{};
    ///Same as AP_T_POUT_PERP_POSE_Y_RIGHT_M but for backward perp. parking out as it is not possible for the planner to hold such  small values.
    float32 AP_T_POUT_PERP_POSE_Y_R_BWD_M{};
    ///Allowed lateral deviation from target axis on parking out perpendicular
    float32 AP_T_POUT_PERP_DEV_LAT_M{};
    ///Allowed yaw angle deviation of ego vehicle relative to target axis on parking out perpendicular
    float32 AP_T_POUT_PERP_DEV_YAW_RAD{};
    ///Safety margin for straight driving corridor used to check if target pose is reached for parallel parking out
    float32 AP_T_POUT_PAR_SAFETY_DIST_M{};
    ///Area around vehicle center to exclude for definition of parking out pose based on driven path (x-length)
    float32 AP_T_LEN_IRRELEVANT_AREA_X_M{};
    ///Area around vehicle center to exclude for definition of parking out pose based on driven path (y-length)
    float32 AP_T_LEN_IRRELEVANT_AREA_Y_M{};
    ///Demo 2 relative movement of pose
    float32 AP_T_PIN_PAR_POSE_X_M{};
    ///Demo 2 relative movement of pose
    float32 AP_T_PIN_PAR_POSE_Y_M{};
    ///Demo 2 relative movement of pose
    float32 AP_T_PIN_PAR_POSE_YAW_RAD{};
    ///Demo 2 relative movement of pose
    float32 AP_T_PIN_PERP_POSE_X_M{};
    ///Demo 2 relative movement of pose
    float32 AP_T_PIN_PERP_POSE_Y_M{};
    ///Demo 2 relative movement of pose
    float32 AP_T_PIN_PERP_POSE_YAW_RAD{};
    ///Minimal static object existence percentage to consider object in the calculation of the target pose.
    uint8 AP_T_MIN_PBOX_EXIST_PERC{};
    ///Defines if it is allowed to update the Target Pose during last stroke
    boolean AP_T_UPDATE_POSE_LAST_STROKE_NU{};
    ///Maximal deviation between old target position and new target position in lateral direction that still allows sending the old target pose.
    float32 AP_T_MAX_DEV_UPDATE_LAT_PERP_M{};
    ///Maximal deviation between old target position and new target position in longitudinal direction that still allows sending the old target pose.
    float32 AP_T_MAX_DEV_UPDATE_LONG_PERP_M{};
    ///Maximal deviation between old target angle and new target angle that still allows sending the old target pose.
    float32 AP_T_MAX_DEV_UPD_YAW_PERP_RAD{};
    ///For a parallel parking slot update only the longitudinal position of the target pose while in AVG mode. (TODO set back to zero after demo)
    boolean AP_T_UPDATE_LONG_ONLY_PAR_NU{};
    ///Left movement of parking out pose for perpendicular parking slot relative to road side edge. Direction is equal to vector from parking box
    ///point 1 to point 0 (on road side edge from right edge to left edge). The sign of this parameter will be flipped according to direction of parking out.
    float32 AP_T_POUT_PERP_POSE_LEFT_M{};
    ///Tolerance for intersections between 1D projection line and comfort or maximal box for maxComfPositioning approach
    float32 AP_T_MAXCOMFPOS_INT_TOL_M{};
    ///Parameter, limitting the maximum allowed lateral deviation for planner.
    float32 AP_T_MAX_LATERAL_DEVIATION_M{};
    ///In case that in PoseDefinerParkingIn::targetYawAngleDefinition no delimiter for orientating the ego vehicle is found, this parametrs determines whether
    ///the orientation shall be determined according the the long side orientation of the parking box (TRUE) or according to the orientation of the ego vehicle
    ///when the parking box has been detected first time (FALSE)
    boolean AP_T_YAW_ANG_PBOX_ORI_NU{};
    ///For the park out corridor for perpendicular parking out: Distance between the front of the ego vehicle to the Front Side of the park out corridor.
    float32 AP_T_POUT_PERP_COR_OFFS_FRONT_M{};
    ///For the park out corridor for perpendicular parking out: In case of positive value: Reduces the distance between rear axle and Rear Side. In case of
    ///negative value: Increases the distance between rear axle and Rear Side.
    float32 AP_T_POUT_PERP_COR_OFFS_REAR_M{};
    ///For the park out corridor for perpendicular parking out:  Gap between the lateral vehicle border and the "Slot Side"
    float32 AP_T_POUT_PERP_COR_OFFS_SLOTS_M{};
    ///For the park out corridor for perpendicular parking out: Gap between the lateral vehicle border and the "Opposite Side"
    float32 AP_T_POUT_PERP_COR_OFFS_OPPOS_M{};
    ///Parameter, limitting the maximum allowed lateral deviation for planner.
    float32 AP_T_MIN_LATERAL_DEVIATION_M{};
    ///Maximal angular deviation of parking space marking compared Parking Box orientation
    float32 AP_T_MAX_MARKING_ANG_DEV_RAD{};
    ///Minimum length for a marking to be considered
    float32 AP_T_MIN_LENGTH_MARKING_M{};
    ///FOR HACK: in case the target pose does not fit in the max box, extend the parking box by this length to the road side edge
    float32 AP_T_EXTENT_MAX_BOX_ROADSIDE_M{};
    ///max. distance between a delimiter and the input parking box, to consider this object as relevant delimiter
    float32 AP_T_MAX_DELIM_DIST_TO_BOX_M{};
    ///Garage Parking: Circular area around the target pose that will freeze the target pose if ego vehicle"s rear-axle is inside.
    float32 AP_T_GP_POSE_FREEZE_RADIUS_M{};
    ///Garage Parking: Pose update during garage parking if lateral deviation between new and old pose is higher than this threshold.
    float32 AP_T_MAX_DEV_UPDATE_LAT_GP_M{};
    ///Garage Parking: Pose update during garage parking if longitudinal deviation between new and old pose is higher than this threshold.
    float32 AP_T_MAX_DEV_UPDATE_LONG_GP_M{};
    ///Garage Parking: Pose update during garage parking if angular deviation between new and old pose is higher than this threshold.
    float32 AP_T_MAX_DEV_UPD_YAW_GP_RAD{};
    ///Garage Parking: Distance of ego vehilce to garage in final parking pose on leaving the garage.
    float32 AP_T_GP_OUT_GAP_ENTRANCE_M{};
    ///Garage Parking: Distance of ego vehilce to garage in final parking pose on leaving the garage.
    float32 AP_T_GP_OUT_REACHED_M{};
    ///Garage Parking: Allowed overlap over maxBox to successfully finish maneuver. Set to 1 to "deactivate" check for being inside the maximum box.
    float32 AP_T_GP_REACHED_DEV_BOX_M{};
    ///Garage Parking: Allowed longitudinal deviation towards garage door to successfully finish parking in maneuver
    float32 AP_T_GP_REACHED_DEV_LONG_M{};
    ///Garage Parking: If this parameter is set to true, only center in the maximum box for entering the garage. This is disregarding any comfort values.
    boolean AP_T_GP_INSIDE_LAT_CENTER_ONLY{};
    ///Garage Parking: If the vehicle dived into the garage that only the length defined by this parameter is still out on the street, switch to pose mode
    ///even though no delimiter was detected at the back wall (curb side edge)
    float32 AP_T_AXIS_MIN_OUTSIDE_GARAGE_M{};
    ///Defines the actual number of poses stored in the pose history that are used to calculate the average target pose. Note that this parameter has to be <= AP_T_MAX_LENGTH_POSE_HISTORY
    uint8 AP_T_ACTUAL_LENGTH_POSE_HISTORY{};
    ///Maximal angular deviation of virtual lines of objects compared Parking Box orientation
    float32 AP_T_MAX_VIRTUALLINE_ANG_DEV_RAD{};
    ///Category weight for orientation calculation cost function for short side parking markings.
    float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_PARKMARKING_NU{};
    ///Category weight for orientation calculation cost function for long side parking markings.
    float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_PARKMARKING_NU{};
    ///Category weight for orientation calculation cost function for short side curbs.
    float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_CURB_NU{};
    ///Category weight for orientation calculation cost function for long side curbs.
    float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_CURB_NU{};
    ///Category weight for orientation calculation cost function for short side virtual lines.
    float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_VIRTUALLINE_NU{};
    ///Category weight for orientation calculation cost function for long side virtual lines.
    float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_VIRTUALLINE_NU{};
    ///Category weight for orientation calculation cost function for short side lane boundaries.
    float32 AP_T_ORI_CATEG_WEIGHT_SHORTSIDE_LANEBOUND_NU{};
    ///Category weight for orientation calculation cost function for long side lane boundaries.
    float32 AP_T_ORI_CATEG_WEIGHT_LONGSIDE_LANEBOUND_NU{};
    ///Category weight for orientation calculation cost function for fallback orientation.
    float32 AP_T_ORI_CATEG_WEIGHT_FALLBACK_NU{};
    ///Category score for orientation calculation cost function for fallback orientation.
    float32 AP_T_ORI_CATEG_SCORE_FALLBACK_M{};
    ///How far a curb side edge is allowed to go to the road side when adapting to inside-RELATED_OTHERWISE-delimiter. Given as a multitude of the original distance of Curb- and Road-Side.
    float32 AP_T_DEL_REL_OTHERW_CURB_ASSIGNMENT_RATIO_NU{};
    ///Treshold for the min angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for parallel and perpendicular use cases.
    float32 AP_T_MIN_ANG_TRESH_CMF_BOX_LIMIT_PA_PER_DEG{};
    ///Treshold for the max angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for parallel and perpendicular use cases.
    float32 AP_T_MAX_ANG_TRESH_CMF_BOX_LIMIT_PA_PER_DEG{};
    ///Treshold for the min angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for angled use cases.
    float32 AP_T_MIN_ANG_TRESH_CMF_BOX_LIMIT_ANGLED_DEG{};
    ///Treshold for the max angle between comfort box edge and estimated road side, allowing to limit the comfort box by the estimated road side for angled use cases
    float32 AP_T_MAX_ANG_TRESH_CMF_BOX_LIMIT_ANGLED_DEG{};
    ///Distance between the wheels of the ego vehicle to the wheel stopper. Remark: In case that the wheel stopper is not perpendicular to the ego vehicle, this is the minimum distance.
    float32 AP_T_WHEEL_DIST_TO_WHEELSTOPPER_M{};
    ///Enabling an urgend TAPOSD update for perpendicular parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LONG_PERP_M
    ///is smaller than the relative difference to the yaw angle of the new calculated pose. The higher the factor, the less likely an urgent update.
    float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PERP_LONG_NU{};
    ///Enabling an urgend TAPOSD update for perpendicular parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LAT_PERP_M is smaller than the relative difference to the lateral
    ///position of the new calculated pose. The higher the factor, the less likely an urgent update.
    float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PERP_LAT_NU{};
    ///Enabling an urgend TAPOSD update for perpendicular parking slots, if this factor multiplied with AP_T_MAX_DEV_UPD_YAW_PERP_RAD is smaller than the relative difference to the longitudinal
    ///position of the new calculated pose. The higher the factor, the less likely an urgent update.
    float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PERP_ANG_NU{};
    ///Enabling an urgend TAPOSD update for parallel parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LONG_PAR_M is smaller than the relative difference to the yaw angle of
    ///the new calculated pose. The higher the factor, the less likely an urgent update.
    float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PAR_LONG_NU{};
    ///Enabling an urgend TAPOSD update for parallel parking slots, if this factor multiplied with AP_T_MAX_DEV_UPDATE_LAT_PAR_M is smaller than the relative difference to the lateral position
    ///of the new calculated pose. The higher the factor, the less likely an urgent update.
    float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PAR_LAT_NU{};
    ///Enabling an urgend TAPOSD update for parallel parking slots, if this factor multiplied with AP_T_MAX_DEV_UPD_YAW_PAR_RAD is smaller than the relative difference to the longitudinal position
    ///of the new calculated pose. The higher the factor, the less likely an urgent update.
    float32 AP_T_URGENT_UPDATE_DEV_FACTOR_PAR_ANG_NU{};
    ///Reflects the maximum allowed distance of a curb delimiter to the (estimated) roadside of a parallel parking box.
    float32 AP_T_MAX_CURB_DEL_DIST_TO_ROADSIDE_PAR_M{};
    ///For perpendicular parking limit max allowed shift in road direction to avoid a collision with obstacles, which potentially block the slot to right or left side.
    float32 AP_T_POUT_PERP_MAX_PULL_OUT_DIST_IN_ROAD_DIR_M{};
  };

} // namespace ap_tp

#endif // AP_TP_FC_TAPOSD_PARAMS_H_
