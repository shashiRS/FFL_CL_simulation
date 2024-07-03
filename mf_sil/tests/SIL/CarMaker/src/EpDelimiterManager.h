#pragma once

#include "geoml/AxisAlignedRectangular2D.h"
#include "ap_common/ap_common_types_consts.h"
#include "si/ap_parking_box_port.h"
#include "MfSilTypes.h"

// adopted from si_core component SiHighMiniEmManager.h
// see https://github-am.geo.conti.de/ADAS/si_core/blob/release/8.0.28/src/component/include/SiHighMiniEmManager.h

class EpDelimiterManager
{
    // Type alias for Polygon2D with 4 vertices
    using Quadrangle2D = LSM_GEOML::Polygon2D<4U>;

    struct DelimiterZone
    {
        Quadrangle2D zone; // agreement: first two coordinates must be the long outer side
        LSM_GEOML::AxisAlignedRectangular2D rect;
        cml::Vec2Df projectionAxis{}; //Normal of sum of sideEdges of zone
        std::pair<float32_t, float32_t> minMaxProjection{}; //MinMaxProjection of zone vertices onto projectionAxis
        float32_t scoreFactor{}; //Used to "move" projection from projectionAxis to outerAxis
        bool useVanishingPoint{}; //Only used for getScoreHighComplexity
        cml::Vec2Df vanishingPoint{}; //Only used for getScoreHighComplexity
    };

    struct DelimiterZoneSet
    {
        DelimiterZone left{};
        DelimiterZone right{};
        DelimiterZone road{};
        DelimiterZone curb{};
        DelimiterZone inside{};
        LSM_GEOML::AxisAlignedRectangular2D all{};
    };

    struct DelimiterZoneCalcInfo
    {
        const cml::Vec2Df* roadLeft{ nullptr };
        const cml::Vec2Df* roadRight{ nullptr };
        const cml::Vec2Df* curbRight{ nullptr };
        const cml::Vec2Df* curbLeft{ nullptr };

        cml::Vec2Df shiftToLeftRoadSide{ 0.0F, 0.0F };
        cml::Vec2Df shiftToLeftCurbSide{ 0.0F, 0.0F };
        cml::Vec2Df shiftToRoadLeftSide{ 0.0F, 0.0F };
        cml::Vec2Df shiftToRoadRightSide{ 0.0F, 0.0F };

        float32_t roadLeftFactor{ 1.0F };
        float32_t roadRightFactor{ 1.0F };
        float32_t curbRightFactor{ 1.0F };
        float32_t curbLeftFactor{ 1.0F };
    };
    std::array<DelimiterZoneSet, ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PARKING_BOXES_NU> mDelimiterZoneSets{};

    template<uint8_t objectPolySize>
    float32_t getScoreLowComplexity(const DelimiterZone& delimiterZone, const LSM_GEOML::Polygon2D<objectPolySize>& objectPolygon) const;
    template<uint8_t objectPolySize>
    float32_t getScoreHighComplexity(const DelimiterZone& delimiterZone, const LSM_GEOML::Polygon2D<objectPolySize>& objectPolygon) const;

    static DelimiterZoneCalcInfo getDelimiterZoneCalcInfo(const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU>& slotCoordinates_m);
    DelimiterZone createCurbsideZone(const DelimiterZoneCalcInfo& data) const;
    DelimiterZone createRoadsideZone(const DelimiterZoneCalcInfo& data) const;
    void createLeftAndRightZone(const DelimiterZoneCalcInfo& data, DelimiterZone& inoutLeft, DelimiterZone& inoutRight) const;
    DelimiterZone createInsideZone(const DelimiterZoneCalcInfo& data) const;
    void setDelimiterZoneParameters(DelimiterZone& delimiterZone) const;

public:
    //!
    //! \brief      Determine the delimiter type of the traffic object in relation to the specified parking box.
    //! \param[in]  pbIdx                   index of the parking box
    //! \param[in]  objectPolygon           shape polygon of the traffic object to check
    //! \param[in]  trafficObjDescription   desciption of the CarMaker traffic object to check
    //! \param[in]  parkingScenario_nu      the parking scenario type of the parking box
    //! \returns    the parking box delimiter type (if the object is not a delimiter function return UNDEFINED_EDGE)
    //!
    template<uint8_t objectPolySize>
    si::RelativeLocationToParkingBox determineDelimiterType(const uint8_t pbIdx, const LSM_GEOML::Polygon2D<objectPolySize>& objectPolygon,
        const char* trafficObjDescription, const si::ParkingScenarioTypes parkingScenario_nu);

    //!
    //! \brief      Update the delimiter zones for the specified parking box.
    //! \param[in]  pbIdx               index of the parking box
    //! \param[in]  parkingBoxShape     polygon of the parking box slot coordinates
    //!
    void updateDelimiterZones(const uint8_t pbIdx, const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU>& parkingBoxShape);
};

