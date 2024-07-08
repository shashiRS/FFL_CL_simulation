#pragma once

#include "geoml/Polygon2D.h"
#include "ap_common/ap_common_types_consts.h"
#include "si/slot_coordinates_m_serializable.h"
#include "si/static_obj_shape_serializable.h"
#include "si/dynamic_obj_shape_serializable.h"
#include "us_em/static_obj_shape_serializable.h"
#include "us_em/dynamic_obj_shape_serializable.h"

template<LSM_CML::boolean_t EXT_MEM>
inline cml::Vec2Df_POD convert(const cml::Vec2Df_uniMem<EXT_MEM>& vec2Df)
{
    return cml::Vec2Df_POD{ vec2Df.x(), vec2Df.y() };
}

template<LSM_CML::boolean_t EXT_MEM>
inline lsm_geoml::Pose_POD convert(const LSM_GEOML::Pose_uniMem<EXT_MEM>& pose)
{
    return lsm_geoml::Pose_POD{ pose.Pos().x(), pose.Pos().y(), pose.Yaw_rad() };
}

// TODO: convert these three array copy functions to template functions

inline LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU> convert(const si::SlotCoordinates_mSerializable& slotCoordinates_m)
{
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU> tempSlotCoordinates;
    for (lsm_geoml::size_type i{ 0U }; i < slotCoordinates_m.actualSize; ++i) {
        tempSlotCoordinates.append(slotCoordinates_m.array[i]);
    }
    return tempSlotCoordinates;
}

inline LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> convert(const si::StaticObjShapeSerializable& objShape_m)
{
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempObjShape;
    for (lsm_geoml::size_type i{ 0U }; i < objShape_m.actualSize; ++i) {
        tempObjShape.append(objShape_m.array[i]);
    }
    return tempObjShape;
}

inline LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU> convert(const si::DynamicObjShapeSerializable& objShape_m)
{
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU> tempObjShape;
    for (lsm_geoml::size_type i{ 0U }; i < objShape_m.actualSize; ++i) {
        tempObjShape.append(objShape_m.array[i]);
    }
    return tempObjShape;
}


inline LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> convert(const us_em::StaticObjShapeSerializable& objShape_m)
{
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempObjShape;
    for (lsm_geoml::size_type i{ 0U }; i < objShape_m.actualSize; ++i) {
        tempObjShape.append(objShape_m.array[i]);
    }
    return tempObjShape;
}

inline LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU> convert(const us_em::DynamicObjShapeSerializable& objShape_m)
{
    LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU> tempObjShape;
    for (lsm_geoml::size_type i{ 0U }; i < objShape_m.actualSize; ++i) {
        tempObjShape.append(objShape_m.array[i]);
    }
    return tempObjShape;
}