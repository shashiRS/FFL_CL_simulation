#include "enumproperty.h"

#include <TrajectoryPlanner.h>
#include <models/targetposereachableareamodel.h>

EnumPropertyDescription::EnumPropertyDescription(const QMap<int, QString> &names)
    : nameMap(names)
{

}

static EnumPropertyDescription ap_tp_PoseFailReason_Description(
    QMap<int, QString> {
        { (int)ap_tp::PoseFailReason::TAPOSD_PFR_NONE, "TAPOSD_PFR_NONE"},
        { (int)ap_tp::PoseFailReason::TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW, "TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW" },
        { (int)ap_tp::PoseFailReason::TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT, "TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT" },
        { (int)ap_tp::PoseFailReason::TAPOSD_PFR_MAXBOX_EXCEEDED, "TAPOSD_PFR_MAXBOX_EXCEEDED" },
        { (int)ap_tp::PoseFailReason::TAPOSD_PFR_WHEEL_COLLISION, "TAPOSD_PFR_WHEEL_COLLISION" },
        { (int)ap_tp::PoseFailReason::TAPOSD_PFR_HIGH_OBJECT_COLLISION, "TAPOSD_PFR_HIGH_OBJECT_COLLISION" },
        { (int)ap_tp::PoseFailReason::TAPOSD_PFR_UNKNOWN, "TAPOSD_PFR_UNKNOWN" },
        { (int)ap_tp::PoseFailReason::MAX_NUM_POSE_FAIL_TYPES, "MAX_NUM_POSE_FAIL_TYPES" },
});


template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_tp::PoseFailReason>()
{
    return &ap_tp_PoseFailReason_Description;
}

static EnumPropertyDescription ap_tp_PoseType_Description(
QMap<int, QString> {
            {(int)ap_tp::PoseType::T_PARALLEL_PARKING, "PARALLEL_PARKING"},
            {(int)ap_tp::PoseType::T_PERP_PARKING_FWD, "PERP_PARKING_FWD"},
            {(int)ap_tp::PoseType::T_PERP_PARKING_BWD, "PERP_PARKING_BWD"},
            {(int)ap_tp::PoseType::T_ANGLED_PARKING_STANDARD, "ANGLED_PARKING_STANDARD"},
            {(int)ap_tp::PoseType::T_ANGLED_PARKING_REVERSE, "ANGLED_PARKING_REVERSE"},
            {(int)ap_tp::PoseType::T_REM_MAN_FWD, "REM_MAN_FWD"},
            {(int)ap_tp::PoseType::T_REM_MAN_BWD, "REM_MAN_BWD"},
            {(int)ap_tp::PoseType::T_PERP_PARKING_OUT_FWD, "PERP_PARKING_OUT_FWD"},
            {(int)ap_tp::PoseType::T_PERP_PARKING_OUT_BWD, "PERP_PARKING_OUT_BWD"},
            {(int)ap_tp::PoseType::T_PAR_PARKING_OUT, "PAR_PARKING_OUT"},
            {(int)ap_tp::PoseType::T_ANGLED_PARKING_STANDARD_OUT, "T_ANGLED_PARKING_STANDARD_OUT" },
            {(int)ap_tp::PoseType::T_ANGLED_PARKING_REVERSE_OUT, "T_ANGLED_PARKING_REVERSE_OUT" },
            {(int)ap_tp::PoseType::T_UNDO, "UNDO"},
            {(int)ap_tp::PoseType::T_UNDEFINED, "UNDEFINED"},
            {(int)ap_tp::PoseType::T_GP_FWD, "GP_FWD"},
            {(int)ap_tp::PoseType::T_GP_BWD, "GP_BWD"},
            {(int)ap_tp::PoseType::T_GP_OUT_FWD, "GP_OUT_FWD"},
            {(int)ap_tp::PoseType::T_GP_OUT_BWD, "GP_OUT_BWD"},
            {(int)ap_tp::PoseType::T_GP_FWD_AXIS, "GP_FWD_AXIS"},
            {(int)ap_tp::PoseType::T_GP_BWD_AXIS, "GP_BWD_AXIS"},
            {(int)ap_tp::PoseType::T_GP_OUT_FWD_AXIS, "GP_OUT_FWD_AXIS"},
            {(int)ap_tp::PoseType::T_GP_OUT_BWD_AXIS, "GP_OUT_BWD_AXIS"},
        });


template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_tp::PoseType>()
{
    return &ap_tp_PoseType_Description;
}

static EnumPropertyDescription ap_tp_TargetSide_Description(
QMap<int, QString> {
            {(int)ap_tp::TargetSide::TS_RIGHT_SIDE, "RIGHT_SIDE"},
            {(int)ap_tp::TargetSide::TS_LEFT_SIDE, "LEFT_SIDE"},
            {(int)ap_tp::TargetSide::TS_IN_FRONT_RIGHT, "IN_FRONT_RIGHT"},
            {(int)ap_tp::TargetSide::TS_IN_FRONT_CENTER, "IN_FRONT_CENTER"},
            {(int)ap_tp::TargetSide::TS_IN_FRONT_LEFT, "IN_FRONT_LEFT"},
            {(int)ap_tp::TargetSide::TS_IN_REAR_RIGHT, "IN_FRONT_RIGHT"},
            {(int)ap_tp::TargetSide::TS_IN_REAR_CENTER, "IN_FRONT_CENTER"},
            {(int)ap_tp::TargetSide::TS_IN_REAR_LEFT, "IN_FRONT_LEFT"},
            {(int)ap_tp::TargetSide::TS_UNDEFINED_SIDE, "UNDEFINED_SIDE"},
        });

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_tp::TargetSide>()
{
    return &ap_tp_TargetSide_Description;
}

static EnumPropertyDescription ap_common_SteeringDirection_Description(
    QMap<int, QString> {
        {(int)ap_common::SteeringDirection::CIRCLE_LEFT, "CIRCLE_LEFT" },
        {(int)ap_common::SteeringDirection::CIRCLE_RIGHT, "CIRCLE_RIGHT" },
        {(int)ap_common::SteeringDirection::STRAIGHT, "STRAIGHT" },
        {(int)ap_common::SteeringDirection::CLOTHOID_LEFT_BACKEND, "CLOTHOID_LEFT_BACKEND" },
        {(int)ap_common::SteeringDirection::CLOTHOID_LEFT_FRONTEND, "CLOTHOID_LEFT_FRONTEND" },
        {(int)ap_common::SteeringDirection::CLOTHOID_RIGHT_BACKEND, "CLOTHOID_RIGHT_BACKEND" },
        {(int)ap_common::SteeringDirection::CLOTHOID_RIGHT_FRONTEND, "CLOTHOID_RIGHT_FRONTEND" },
});

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_common::SteeringDirection>()
{
    return &ap_common_SteeringDirection_Description;
}

static EnumPropertyDescription ap_common_DrivingDirection_Description(
    QMap<int, QString> {
        {(int)ap_common::DrivingDirection::DIRECTION_UNKNOWN, "DIRECTION_UNKNOWN" },
        {(int)ap_common::DrivingDirection::DRIVING_BACKWARDS, "DRIVING_BACKWARDS" },
        {(int)ap_common::DrivingDirection::DRIVING_FORWARDS, "DRIVING_FORWARDS" },
        {(int)ap_common::DrivingDirection::STANDSTILL, "STANDSTILL" },
});

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_common::DrivingDirection>()
{
    return &ap_common_DrivingDirection_Description;
}

static EnumPropertyDescription ap_tp_ReplanTrigger_Description(
    QMap<int, QString> {
        {(int)ap_tp::ReplanTrigger::NO_TRIGGER_SET, "NO_TRIGGER_SET" },
        {(int)ap_tp::ReplanTrigger::MANEUVER_START, "MANEUVER_START" },
        {(int)ap_tp::ReplanTrigger::TAPOS_UPDATE_DYNAMIC, "TAPOS_UPDATE_DYNAMIC" },
        {(int)ap_tp::ReplanTrigger::TAPOS_UPDATE_STATIC, "TAPOS_UPDATE_STATIC" },
        {(int)ap_tp::ReplanTrigger::TAPOS_NOT_REACHED, "TAPOS_NOT_REACHED" },
        {(int)ap_tp::ReplanTrigger::STROKE_NOT_FULLY_COMPLETED, "STROKE_NOT_FULLY_COMPLETED" },
        {(int)ap_tp::ReplanTrigger::UPCOMING_STROKE_IN_COLLISION, "UPCOMING_STROKE_IN_COLLISION" },
        {(int)ap_tp::ReplanTrigger::LAT_CTRL_FAILED, "LAT_CTRL_FAILED" },
        {(int)ap_tp::ReplanTrigger::LAT_PATH_CTRL_FAILED, "LAT_PATH_CTRL_FAILED" },
        {(int)ap_tp::ReplanTrigger::LONG_CTRL_FAILED, "LONG_CTRL_FAILED" },
});

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_tp::ReplanTrigger>()
{
    return &ap_tp_ReplanTrigger_Description;
}

static EnumPropertyDescription ap_tp_PoseReachableStatus_Description(
	QMap<int, QString> {
		{(int)ap_tp::PoseReachableStatus::TP_NOT_VALID, "TP_NOT_VALID"},
		{(int)ap_tp::PoseReachableStatus::TP_NOT_REACHABLE, "TP_NOT_REACHABLE" },
		{(int)ap_tp::PoseReachableStatus::TP_FULLY_REACHABLE, "TP_FULLY_REACHABLE" },
		{(int)ap_tp::PoseReachableStatus::TP_SAFE_ZONE_REACHABLE, "TP_SAFE_ZONE_REACHABLE" },
		{(int)ap_tp::PoseReachableStatus::TP_MANUAL_FWD_REACHABLE, "TP_MANUAL_FWD_REACHABLE" },
		{(int)ap_tp::PoseReachableStatus::TP_MANUAL_BWD_REACHABLE, "TP_MANUAL_BWD_REACHABLE" },
		{(int)ap_tp::PoseReachableStatus::MAX_NUM_POSE_REACHABLE_STATUS, "MAX_NUM_POSE_REACHABLE_STATUS" },
});

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_tp::PoseReachableStatus>()
{
	return &ap_tp_PoseReachableStatus_Description;
}

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<si::StaticObjHeigthType>()
{
    static EnumPropertyDescription description(
    QMap<int, QString> {
                    {(int)si::StaticObjHeigthType::SO_HI_UNKNOWN, "HI_UNKNOWN"},
                    {(int)si::StaticObjHeigthType::SO_HI_WHEEL_TRAVERSABLE, "WHEEL_TRAVERSABLE"},
                    {(int)si::StaticObjHeigthType::SO_HI_BODY_TRAVERSABLE, "BODY_TRAVERSABLE"},
                    {(int)si::StaticObjHeigthType::SO_HI_DOOR_OPENABLE, "DOOR_OPENABLE"},
                    {(int)si::StaticObjHeigthType::SO_HI_HIGH_OBSTACLE, "HIGH_OBSTACLE"},
                    {(int)si::StaticObjHeigthType::SO_HI_HANGING_OBJECT, "HANGING_OBJECT"},
                    {(int)si::StaticObjHeigthType::SO_HI_LOWER_BUMPER_HEIGHT, "SO_HI_LOWER_BUMPER_HEIGHT" },
                });
    return &description;
}

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<ap_tp::TrajPlanState>()
{
    static EnumPropertyDescription description(
    QMap<int, QString> {
                    {(int)ap_tp::TrajPlanState::INIT, "INIT"},
                    {(int)ap_tp::TrajPlanState::IDLE, "IDLE"},
                    {(int)ap_tp::TrajPlanState::AP_SCANNING, "AP_SCANNING"},
                    {(int)ap_tp::TrajPlanState::AP_FOLLOW_PATH, "AP_FOLLOW_PATH"},
                    {(int)ap_tp::TrajPlanState::AP_FOLLOW_UNDO_PATH, "AP_FOLLOW_UNDO_PATH"},
                    {(int)ap_tp::TrajPlanState::AP_INITIAL_PLAN_PATH, "AP_INITIAL_PLAN_PATH"},
                    {(int)ap_tp::TrajPlanState::AP_STATIC_REPLAN_PATH, "AP_STATIC_REPLAN_PATH" },
                    {(int)ap_tp::TrajPlanState::AP_DYNAMIC_REPLAN_PATH, "AP_DYNAMIC_REPLAN_PATH" },
                    {(int)ap_tp::TrajPlanState::AP_PLAN_UNDO, "AP_PLAN_UNDO"},
                    {(int)ap_tp::TrajPlanState::AP_PLANNING_FAILED, "AP_PLANNING_FAILED"},
                    {(int)ap_tp::TrajPlanState::AP_REACHED_WAITING_FOR_FINISH, "AP_REACHED_WAITING_FOR_FINISH" },
                    {(int)ap_tp::TrajPlanState::RM_SCANNING, "RM_SCANNING"},
                    {(int)ap_tp::TrajPlanState::RM_PLAN_AND_FOLLOW_PATH, "RM_PLAN_AND_FOLLOW_PATH"},
                    {(int)ap_tp::TrajPlanState::RM_PLANNING_FAILED, "RM_PLANNING_FAILED"},
                    {(int)ap_tp::TrajPlanState::GP_SCANNING, "GP_SCANNING" },
                    {(int)ap_tp::TrajPlanState::GP_PLAN_AND_FOLLOW_PATH, "GP_PLAN_AND_FOLLOW_PATH" },
                    {(int)ap_tp::TrajPlanState::GP_PLANNING_FAILED, "GP_PLANNING_FAILED" },
                    {(int)ap_tp::TrajPlanState::RA_FOLLOW_PATH, "RA_FOLLOW_PATH" },
                    {(int)ap_tp::TrajPlanState::RA_REACHED_WAITING_FOR_FINISH, "RA_REACHED_WAITING_FOR_FINISH" },
                });
    return &description;
}

#ifndef ULTRASONIC_ONLY
template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<si::DelimiterTypes>()
{
    static EnumPropertyDescription description(
    QMap<int, QString> {
                    {(int)si::DelimiterTypes::STATIC_STRUCTURE, "STATIC_STRUCTURE"},
                    {(int)si::DelimiterTypes::PARKING_SPACE_MARKING, "PARKING_SPACE_MARKING"},
                    {(int)si::DelimiterTypes::LANE_BOUNDARY, "LANE_BOUNDARY"},
                });
    return &description;
}
#endif

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<si::RelativeLocationToParkingBox>()
{
    static EnumPropertyDescription description(
    QMap<int, QString> {
                    {(int)si::RelativeLocationToParkingBox::UNDEFINED_EDGE, "UNDEFINED_EDGE" },
                    {(int)si::RelativeLocationToParkingBox::ROAD_SIDE_EDGE, "ROAD_SIDE_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::RIGHT_EDGE, "RIGHT_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::CURB_SIDE_EDGE, "CURB_SIDE_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::LEFT_EDGE, "LEFT_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::INSIDE_ROAD_SIDE_EDGE, "INSIDE_ROAD_SIDE_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::INSIDE_RIGHT_EDGE, "INSIDE_RIGHT_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::INSIDE_CURB_SIDE_EDGE, "INSIDE_CURB_SIDE_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::INSIDE_LEFT_EDGE, "INSIDE_LEFT_EDGE"},
                    {(int)si::RelativeLocationToParkingBox::RELATED_OTHERWISE, "RELATED_OTHERWISE" },
                    {(int)si::RelativeLocationToParkingBox::MAX_NUM_PARKING_BOX_EDGE_TYPES, "MAX_NUM_PARKING_BOX_EDGE_TYPES" },
                });
    return &description;
}

template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<si::ParkingScenarioTypes>()
{
    static EnumPropertyDescription description(
    QMap<int, QString> {
                    {(int)si::ParkingScenarioTypes::PARALLEL_PARKING, "PARALLEL_PARKING"},
                    {(int)si::ParkingScenarioTypes::PERPENDICULAR_PARKING, "PERPENDICULAR_PARKING"},
                    {(int)si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_BACK, "ANGLED_PARKING_OPENING_TOWARDS_BACK"},
                    {(int)si::ParkingScenarioTypes::ANGLED_PARKING_OPENING_TOWARDS_FRONT, "ANGLED_PARKING_OPENING_TOWARDS_FRONT" },
                    {(int)si::ParkingScenarioTypes::GARAGE_PARKING, "GARAGE_PARKING"},
                    {(int)si::ParkingScenarioTypes::DIRECT_PARKING, "DIRECT_PARKING"},
                    {(int)si::ParkingScenarioTypes::EXTERNAL_TAPOS_PARALLEL, "EXTERNAL_TAPOS_PARALLEL" },
                    {(int)si::ParkingScenarioTypes::EXTERNAL_TAPOS_PERPENDICULAR, "EXTERNAL_TAPOS_PERPENDICULAR" },
                    {(int)si::ParkingScenarioTypes::EXTERNAL_TAPOS_PARALLEL_OUT, "EXTERNAL_TAPOS_PARALLEL_OUT" },
                    {(int)si::ParkingScenarioTypes::EXTERNAL_TAPOS_PERPENDICULAR_OUT, "EXTERNAL_TAPOS_PERPENDICULAR_OUT" },
                    {(int)si::ParkingScenarioTypes::MAX_NUM_PARKING_SCENARIO_TYPES, "MAX_NUM_PARKING_SCENARIO_TYPES"},
                });
    return &description;
}

#ifndef ULTRASONIC_ONLY
template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<si::ParkingLineType>()
{
    static EnumPropertyDescription description(
    QMap<int, QString> {
                    {static_cast<int>(si::ParkingLineType::PLT_RED), "PLT_RED"},
                    {static_cast<int>(si::ParkingLineType::PLT_BLUE), "PLT_BLUE"},
                    {static_cast<int>(si::ParkingLineType::PLT_GREEN), "PLT_GREEN"},
                    {static_cast<int>(si::ParkingLineType::PLT_WHITE), "PLT_WHITE"},
                    {static_cast<int>(si::ParkingLineType::PLT_YELLOW), "PLT_YELLOW"},
                    {static_cast<int>(si::ParkingLineType::PLT_UNKNOWN), "PLT_UNKNOWN"},
                });
    return &description;
}
#endif

#ifndef ULTRASONIC_ONLY
template<>
const EnumPropertyDescription *EnumPropertyDescription::getForType<si::StaticObjectClass>()
{
    static EnumPropertyDescription description(
    QMap<int, QString> {
                    {static_cast<int>(si::StaticObjectClass::STAT_OBJ_CURB), "STAT_OBJ_CURB"},
                    {static_cast<int>(si::StaticObjectClass::STAT_OBJ_POLE), "STAT_OBJ_POLE"},
                    {static_cast<int>(si::StaticObjectClass::STAT_OBJ_VEHICLE), "STAT_OBJ_VEHICLE"},
                    {static_cast<int>(si::StaticObjectClass::STAT_OBJ_WHEEL_STOPPER), "STAT_OBJ_WHEEL_STOPPER"},
                    {static_cast<int>(si::StaticObjectClass::STAT_OBJ_UNCLASSIFIED_STRUCTURE), "STAT_OBJ_UNCLASSIFIED_STRUCTURE"},
                });
    return &description;
}
#endif
