#include "SiUtility.h"
#include <algorithm>
#include <memory> // std::unique_ptr
#include <Log.h> // CarMaker logging
#include "DataDict.h"
#include "pod_class_converter.h"
#include <geoml/CoordinateTransformer2D.h> // LSM_GEOML::CoordinateTransformer2D
#include "MfSilTypes.h"

#ifdef VARIANT_CUS_ONLY
// For the CUS-only variant, the parameters are hard coded in the following header and not read from a configuration file
#include <si_core/SI_Params_default_set.h>
#else
#pragma warning( push )
#pragma warning ( disable: 4996 ) // disable deprecation warnings
#include <si_core/SiParamReader.h>
#include <mf_memory_parking/MemParkParamReader.h>
#pragma warning( pop )
#ifdef CIP_BRICKS_BUILD
    static constexpr auto SI_CONFIG_FILE = "../../../conf/package/data/si_core/SI_Config.json";
    static constexpr auto SI_CONFIG_DIFF_FOR_SIM_FILE = "../../../conf/package/data/si_core/SI_Config_Diff.json";
    static constexpr auto MEMPARK_CONFIG_FILE = "../../../conf/package/data/mf_memory_parking/MemPark_Config.json";
#else
    static constexpr auto SI_CONFIG_FILE = "../../../../si_core/src/platform/data/AP_Sim/SI_Config.json";
    static constexpr auto SI_CONFIG_DIFF_FOR_SIM_FILE = "../../../../si_core/src/platform/data/AP_Sim/SI_Config_Diff.json";
    static constexpr auto MEMPARK_CONFIG_FILE = "../../../../mf_memory_parking/src/platform/data/AP_Sim/MemPark_Config.json";
#endif
#endif

const si::SiParams& SiUtility::getSiParameters(const bool reload) {
    static std::unique_ptr<si::SiParams> mSiParams;
#ifdef VARIANT_CUS_ONLY
    reload; /*silence unreferenced warning*/
    mSiParams = std::make_unique<si::SiParams>();
    initSiParams(mSiParams.get());
#else
    if (!mSiParams || reload) {
        mSiParams = std::make_unique<si::SiParams>();
        const bool siConfigOk = si::loadConfiguration(*mSiParams, SI_CONFIG_FILE, SI_CONFIG_DIFF_FOR_SIM_FILE);

        if (!siConfigOk) {
            LogErrStr(EC_General, "SiUtility::getSiParameters() - error loading SI_Config.json");
        }
    }
#endif
    return *mSiParams;
}

#ifndef VARIANT_CUS_ONLY
const mf_mempark::MemParkParams& SiUtility::getMemParkParameters(const bool reload) {
    static std::unique_ptr<mf_mempark::MemParkParams> mMemParkParams;
    if (!mMemParkParams || reload) {
        mMemParkParams = std::make_unique<mf_mempark::MemParkParams>();
        const bool memParkConfigOk = mf_mempark::loadConfiguration(*mMemParkParams, MEMPARK_CONFIG_FILE);
        if (!memParkConfigOk) {
            LogErrStr(EC_General, "SiUtility::getMemParkParameters() - error loading MemPark_Config.json");
        }
    }
    return *mMemParkParams;
}
#else
void SiUtility::sortStaticObjectsByDistance(us_em::UsEnvModelPort &collEnvModelPort)
{
    // Sort staticObjects array into two independent blocks in the following order:
    // - Block A: All objects inside of the detection zone, sorted by their distance to the ego-vehicle shape
    // - Block B: All objects outside of the detection zone, sorted by their distance to the ego-vehicle shape
    std::sort(collEnvModelPort.staticObjects,
        &collEnvModelPort.staticObjects[collEnvModelPort.numberOfStaticObjects_u8],
        [this](const us_em::StaticObjectSerializable &obj1, const us_em::StaticObjectSerializable &obj2) {

        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempObj1{ convert(obj1.objShape_m) };
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempObj2{ convert(obj2.objShape_m) };

        const bool isObj1InsideDetectionZone = tempObj1.doPolygonsOverlap(mDetectionZone_m);
        const bool isObj2InsideDetectionZone = tempObj2.doPolygonsOverlap(mDetectionZone_m);
        if (isObj1InsideDetectionZone == isObj2InsideDetectionZone) {
            // Objects of the same block are sorted by increasing distance.
            return tempObj1.calcDistance(mEgoVehicleShape_m) < tempObj2.calcDistance(mEgoVehicleShape_m);
        }
        // Take objects within detection zone first.
        return isObj1InsideDetectionZone > isObj2InsideDetectionZone;
    });
}

void SiUtility::setFirstObjOutDetZoneIdx(us_em::UsEnvModelPort &collEnvModelPort)
{
    // static object
    collEnvModelPort.firstStatObjOutDetZoneIdx_u8 = collEnvModelPort.numberOfStaticObjects_u8; // if no object outside zone
    for (uint8 uiIdx = 0u; uiIdx < collEnvModelPort.numberOfStaticObjects_u8; ++uiIdx) {
        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempStaticObj1{
            convert(collEnvModelPort.staticObjects[uiIdx].objShape_m) };
        if (!tempStaticObj1.doPolygonsOverlap(mDetectionZone_m)) {
            collEnvModelPort.firstStatObjOutDetZoneIdx_u8 = uiIdx;
            break;
        }
    }

    // dynamic object
    collEnvModelPort.firstDynObjOutDetZoneIdx_u8 = collEnvModelPort.numberOfDynamicObjects_u8; // if no object outside zone
    for (uint8 uiIdx = 0u; uiIdx < collEnvModelPort.numberOfDynamicObjects_u8; ++uiIdx) {
        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU> tempDynObj1{
        convert(collEnvModelPort.dynamicObjects[uiIdx].objShape_m) };
        if (!tempDynObj1.doPolygonsOverlap(mDetectionZone_m)) {
            collEnvModelPort.firstStatObjOutDetZoneIdx_u8 = uiIdx;
            break;
        }
    }

}
#endif

SiUtility& SiUtility::getInstance() {
    static SiUtility instance{};
    return instance;
}

void SiUtility::init(const ap_common::Vehicle_Params &vehicleParams) {
    assert(vehicleParams.AP_V_NUM_STANDARD_SHAPE_PTS <= ap_common::AP_COMMON_TYPES_Consts::AP_V_VEHICLE_SHAPE_MAX_SIZE_NU);
    mEgoVehicleShape_m.clear();
    for (int iPt = 0; iPt < vehicleParams.AP_V_NUM_STANDARD_SHAPE_PTS; iPt++) {
        mEgoVehicleShape_m.append({ vehicleParams.AP_V_STANDARD_SHAPE_X_M[iPt],
            vehicleParams.AP_V_STANDARD_SHAPE_Y_M[iPt] });
    }

    // Determine detection zone
    constexpr float32_t detectionZoneLongituDistance_m{ 5.0f };
    constexpr float32_t detectionZoneLateralDistance_m{ 2.55f };
    const auto minmaxX_m = std::minmax_element(std::begin(vehicleParams.AP_V_STANDARD_SHAPE_X_M),
        std::end(vehicleParams.AP_V_STANDARD_SHAPE_X_M));
    const auto minmaxY_m = std::minmax_element(std::begin(vehicleParams.AP_V_STANDARD_SHAPE_Y_M),
        std::end(vehicleParams.AP_V_STANDARD_SHAPE_Y_M));
    mDetectionZone_m.clear();
    mDetectionZone_m.append({ *minmaxX_m.second + detectionZoneLongituDistance_m, *minmaxY_m.first - detectionZoneLateralDistance_m });   // front right
    mDetectionZone_m.append({ *minmaxX_m.second + detectionZoneLongituDistance_m, *minmaxY_m.second + detectionZoneLateralDistance_m });  // front left
    mDetectionZone_m.append({ *minmaxX_m.first - detectionZoneLongituDistance_m, *minmaxY_m.second + detectionZoneLateralDistance_m });   // rear left
    mDetectionZone_m.append({ *minmaxX_m.first - detectionZoneLongituDistance_m, *minmaxY_m.first - detectionZoneLateralDistance_m });    // rear right
}

void SiUtility::reset()
{
    mEgoVehicleShape_m.clear();
    mDetectionZone_m.clear();
    mDelimiterZones.fill(DelimiterZones());
}

void SiUtility::registerDVAVariables()
{
    //Add signal for plot DelimiterZones
    tDDefault *dfDelZone = DDefaultCreate("AP.delimiterZone.");
    for (unsigned int j = 0U; j < maxNumDelimiterZones; ++j)
    {
        mDelimiterZones[j].curbsideZone.setSize(mDelimiterZones[j].curbsideZone.getMaxSize());
        mDelimiterZones[j].roadsideZone.setSize(mDelimiterZones[j].roadsideZone.getMaxSize());
        mDelimiterZones[j].leftsideZone.setSize(mDelimiterZones[j].leftsideZone.getMaxSize());
        mDelimiterZones[j].rightsideZone.setSize(mDelimiterZones[j].rightsideZone.getMaxSize());
        mDelimiterZones[j].insideZone.setSize(mDelimiterZones[j].insideZone.getMaxSize());

        for (uint32_t i = 0U; i < ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_P_BOX_VERTICES_NU; i++) {
            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.curbsideZone_%d.x", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].curbsideZone[i].x(), DVA_None);
            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.curbsideZone_%d.y", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].curbsideZone[i].y(), DVA_None);

            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.roadsideZone_%d.x", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].roadsideZone[i].x(), DVA_None);
            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.roadsideZone_%d.y", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].roadsideZone[i].y(), DVA_None);

            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.leftsideZone_%d.x", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].leftsideZone[i].x(), DVA_None);
            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.leftsideZone_%d.y", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].leftsideZone[i].y(), DVA_None);

            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.rightsideZone_%d.x", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].rightsideZone[i].x(), DVA_None);
            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.rightsideZone_%d.y", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].rightsideZone[i].y(), DVA_None);

            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.insideZone_%d.x", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].insideZone[i].x(), DVA_None);
            DDefPrefix(dfDelZone, "AP.delimiterZone_%d.insideZone_%d.y", j, i);
            DDefFloat(dfDelZone, "", "", &mDelimiterZones[j].insideZone[i].y(), DVA_None);
        }
        mDelimiterZones[j].curbsideZone.setSize(0U);
        mDelimiterZones[j].roadsideZone.setSize(0U);
        mDelimiterZones[j].leftsideZone.setSize(0U);
        mDelimiterZones[j].rightsideZone.setSize(0U);
        mDelimiterZones[j].insideZone.setSize(0U);
    }
}

void SiUtility::updateDelimiterZones(const uint8_t pbIdx, const DelimiterZones & delimiterZones)
{
    mDelimiterZones[pbIdx] = delimiterZones;
}

void SiUtility::updateDelimiterZones(const si::DelimiterZonesSerializable& delimiterZones,
    const LSM_GEOML::Pose& vehiclePose)
{
    std::fill(mDelimiterZones.begin() + delimiterZones.actualSize, mDelimiterZones.end(), DelimiterZones());
    const LSM_GEOML::CoordinateTransformer2D vehicleCoordinateTransformer{ vehiclePose };
    const lsm_geoml::size_type numVertices { mDelimiterZones.front().curbsideZone.getMaxSize() };
    for (lsm_geoml::size_type iPB{ 0U }; iPB < delimiterZones.actualSize; ++iPB) {
        mDelimiterZones[iPB].curbsideZone.setSize(numVertices);
        mDelimiterZones[iPB].roadsideZone.setSize(numVertices);
        mDelimiterZones[iPB].leftsideZone.setSize(numVertices);
        mDelimiterZones[iPB].rightsideZone.setSize(numVertices);
        mDelimiterZones[iPB].insideZone.setSize(numVertices);
        for (lsm_geoml::size_type iV{ 0U }; iV < numVertices; ++iV) {
            mDelimiterZones[iPB].curbsideZone[iV]  = vehicleCoordinateTransformer.transform(delimiterZones.array[iPB].curbZone.array[iV]);
            mDelimiterZones[iPB].roadsideZone[iV]  = vehicleCoordinateTransformer.transform(delimiterZones.array[iPB].roadZone.array[iV]);
            mDelimiterZones[iPB].leftsideZone[iV]  = vehicleCoordinateTransformer.transform(delimiterZones.array[iPB].leftZone.array[iV]);
            mDelimiterZones[iPB].rightsideZone[iV] = vehicleCoordinateTransformer.transform(delimiterZones.array[iPB].rightZone.array[iV]);
            mDelimiterZones[iPB].insideZone[iV]    = vehicleCoordinateTransformer.transform(delimiterZones.array[iPB].insideZone.array[iV]);
        }
    }
}

void SiUtility::sortStaticObjectsByDistance(si::CollEnvModelPort &collEnvModelPort)
{
    // Sort staticObjects array into two independent blocks in the following order:
    // - Block A: All objects inside of the detection zone, sorted by their distance to the ego-vehicle shape
    // - Block B: All objects outside of the detection zone, sorted by their distance to the ego-vehicle shape
    std::sort(collEnvModelPort.staticObjects,
        &collEnvModelPort.staticObjects[collEnvModelPort.numberOfStaticObjects_u8],
        [this](const si::StaticObjectSerializable &obj1, const si::StaticObjectSerializable &obj2) {

        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempObj1{ convert(obj1.objShape_m) };
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempObj2{ convert(obj2.objShape_m) };

        const bool isObj1InsideDetectionZone = tempObj1.doPolygonsOverlap(mDetectionZone_m);
        const bool isObj2InsideDetectionZone = tempObj2.doPolygonsOverlap(mDetectionZone_m);
        if (isObj1InsideDetectionZone == isObj2InsideDetectionZone) {
            // Objects of the same block are sorted by increasing distance.
            return tempObj1.calcDistance(mEgoVehicleShape_m) < tempObj2.calcDistance(mEgoVehicleShape_m);
        }
        // Take objects within detection zone first.
        return isObj1InsideDetectionZone > isObj2InsideDetectionZone;
    });
}

void SiUtility::setFirstObjOutDetZoneIdx(si::CollEnvModelPort &collEnvModelPort)
{
    // static object
    collEnvModelPort.firstStatObjOutDetZoneIdx_u8 = collEnvModelPort.numberOfStaticObjects_u8; // if no object outside zone
    for (uint8 uiIdx = 0u; uiIdx < collEnvModelPort.numberOfStaticObjects_u8; ++uiIdx) {
        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_STATIC_POLY_NU> tempStaticObj1{
            convert(collEnvModelPort.staticObjects[uiIdx].objShape_m) };
        if (!tempStaticObj1.doPolygonsOverlap(mDetectionZone_m)) {
            collEnvModelPort.firstStatObjOutDetZoneIdx_u8 = uiIdx;
            break;
        }
    }

    // dynamic object
    collEnvModelPort.firstDynObjOutDetZoneIdx_u8 = collEnvModelPort.numberOfDynamicObjects_u8; // if no object outside zone
    for (uint8 uiIdx = 0u; uiIdx < collEnvModelPort.numberOfDynamicObjects_u8; ++uiIdx) {
        // TODO: <SR> for loop with Vec2Df_POD workaround needs to be removed, as soon as templated conversion is implemented in geoml
        const LSM_GEOML::Polygon2D<ap_common::AP_COMMON_TYPES_Consts::AP_G_MAX_NUM_PTS_FOR_DYN_POLY_NU> tempDynObj1{
        convert(collEnvModelPort.dynamicObjects[uiIdx].objShape_m)};
        if (!tempDynObj1.doPolygonsOverlap(mDetectionZone_m)) {
            collEnvModelPort.firstStatObjOutDetZoneIdx_u8 = uiIdx;
            break;
        }
    }

}
