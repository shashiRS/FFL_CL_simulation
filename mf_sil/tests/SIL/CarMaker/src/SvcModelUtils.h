#ifndef __SVC_MODEL_UTILS_H__
#define __SVC_MODEL_UTILS_H__

#include <string>
#include <array>
#include <vector>

namespace gdr {
    struct GdrPointList;
}

namespace pmsd {
    struct ParkingLineList;
    struct WheelStopperList;
    struct WheelLockerList;
    struct ParkingSlotList;
    struct StopLineList;
    struct PedestrianCrossingList;
}

namespace tpp {
    struct DynamicObjectList_t;
}

namespace spp {
    struct SppPolylineList_t;
}

namespace svc_model_processing {
    struct SvcModelProcessingInput;
    struct SvcModelProcessingOutput;
    struct SvcModelProcessingSettings;
    struct DebugVizSide;
}


namespace VCEM {
    struct ODSlot;
}

struct tTrafficObj;
struct tTrafficObj_Cfg;


#ifdef USE_ENV_PLOTTER
#include "carMakerAPWrapper.h" // SVCPlotterData
#endif

class SvcModelUtils {
public:
    static constexpr char settingsFilePath[] = "../../../conf/package/svc_model_processing/svc_model_params.json";

    static void assignODSlots(std::vector<VCEM::ODSlot> const& _odSlots);

    // MAIN INPUT POPULATION FUNCTION
    static void fillSvcModelInputByTrafficObjects(svc_model_processing::SvcModelProcessingInput& input, uint64_t const timeStamp_us,
                                                  svc_model_processing::SvcModelProcessingSettings const& settings);

    // MAIN TCLGEO SCRIPT VARIABLES FUNCTION
    static void declQuants(svc_model_processing::SvcModelProcessingInput& input,
                           svc_model_processing::SvcModelProcessingOutput& output,
                           svc_model_processing::SvcModelProcessingSettings& settings);

    static void getVehicleOriginPosition();
    static void getTrafficObjects2DContureMirroring();

    static void logSvcModelProcessingInput(svc_model_processing::SvcModelProcessingInput& input);
    static void logSvcModelProcessingOutput(svc_model_processing::SvcModelProcessingOutput& output);

#ifdef USE_ENV_PLOTTER
    static void copySVCData(svc_model_processing::SvcModelProcessingOutput const& input, SVCPlotterData& output);
#endif

private:
    static std::array<float, 3U> vehicleOriginPosition;
    static double rearLeftWheelPosDefault[3U];
    static double rearRightWheelPosDefault[3U];
    static std::vector<uint8_t> trafficObjects2DContureMirroring;

    // IDENTIFIER DECLARATIONS FOR THE TCLGEO SCRIPT SUBSCRIPTIONS
    static void declQuantsSvcModelProcessingInput(std::string prefixStr, svc_model_processing::SvcModelProcessingInput& input);
    static void declQuantsGdrPointListInput(std::string prefixStr, gdr::GdrPointList& gdrPointsList);
    static void declQuantsParkingLineListInput(std::string prefixStr, pmsd::ParkingLineList& parkingLineList);
    static void declQuantsWheelStopperListInput(std::string prefixStr, pmsd::WheelStopperList& wheelStopperList);
    static void declQuantsWheelLockerListInput(std::string prefixStr, pmsd::WheelLockerList& wheelLockerList);
    static void declQuantsParkingSlotInput(std::string prefixStr, pmsd::ParkingSlotList& parkingSlotList);
    static void declQuantsStopLinesInput(std::string prefixStr, pmsd::StopLineList& stopLineList);
    static void declQuantsPedestrCrossInput(std::string prefixStr, pmsd::PedestrianCrossingList& pedCrossList);
    static void declQuantsDynamicObjectInputList(std::string prefixStr, tpp::DynamicObjectList_t& dynamicObjectList);
    static void declQuantsSppPolylineInput(std::string prefixStr, spp::SppPolylineList_t& sppPolylineList);
    static void declQuantsVizDebugValues(std::string prefixStr, svc_model_processing::DebugVizSide& _debugVizSide);
    static void declQuantsSettingsValues(std::string prefixStr, svc_model_processing::SvcModelProcessingSettings& _settings);

    // LOGGER FUNCTIONS
    static void logGdrPointList(gdr::GdrPointList& list, std::string listName);
    static void logParkingLineList(pmsd::ParkingLineList& list, std::string listName);
    static void logWheelStopperList(pmsd::WheelStopperList& list, std::string listName);

    // HELPERS
    static bool is2DContureMirroring(int32_t const trafficObjectId);
};

#endif
