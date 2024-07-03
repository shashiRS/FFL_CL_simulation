
// Component interface includes
#include "uspWrapper.h"
#include "MfSilTypes.h"

#include <cassert>
#include <cstdio>

static us_processing::UsProcessingInterface* uspInstance{};

uspWrapper & uspWrapper::getInstance()
{
    static uspWrapper instance{};
    return instance;
}

void uspWrapper::init() {
    dummyUsDrvRntCfgResponse = {};

    us_processing::UsProcessingParams uspParams{};
    us_processing::initUsProcessingParams(&uspParams);

    us_processing::UsProcessingCfgInput configInput = {};
    configInput.usProcessingParams = &uspParams;

    //Init USP
    uspInstance = &us_processing::UsProcessingInterface::getInstance();
    uspInstance->init(configInput);
}

us_processing::UsProcessingSensorParameters uspWrapper::getSensorParameters()
{
    static us_processing::UsProcessingParams uspParams{};
    us_processing::initUsProcessingParams(&uspParams);
    return uspParams.sensParams;
}

void uspWrapper::run(
    const uint64_t timeStamp_ms,
    const lsm_vedodo::OdoEstimationOutputPort &odoEstimationOutputPort,
    const ap_commonvehsigprovider::AmbientDataPort &ambientDataPort,
    const us_drv::UsDrvDetectionList& usRawInput,
    us_processing::UsProcessingPointList& uspPointList,
    us_processing::UsProcessingFilteredEchoList& usFiltered,
    us_processing::UsProcessingDistanceList& uspDistList,
    us_processing::UsProcessingDiagOutput& uspDiagOutput,
    us_processing::UsProcessingDataIntegrity& uspIntegrityOutput,
    us_drv::UsDrvRuntimeConfiguration& usDrvRntConfigRequest
)
{
    const auto isScheduleTime = timeStamp_ms % US_PROCESSING_SAMPLE_TIME_MS == 0;
    const auto isUsRawReady = usRawInput.sSigHeader.uiTimeStamp > 0U;

    if (isScheduleTime && isUsRawReady) {
        assert(uspInstance);
        //const uint64_t cycleStartTime_us = timeStamp_ms * 1000U;
        /// Prepare comm input for ambient temp.
        // us_processing::UsProcessingComInput commInput{};
        //commInput.timestamp_us_u64 = cycleStartTime_us;
        // commInput.ambientDataPort = &ambientDataPort;
        ///commInput.ignition_nu = nullptr;                 ///Unused
        ///commInput.ctrunkLidStatus_nu = nullptr;          ///Unused
        ///commInput.trailerStatus_nu = nullptr;            ///Unused
        ///commInput.doorsStatus_nu = nullptr;              ///Unused
        ///commInput.susHeight_nu = nullptr;                ///Unused

        /// Prepare input data
        us_processing::UsProcessingRequestPorts inputData{};

        inputData.ambientDataPort = &ambientDataPort;                                   ///Ambient temperature input
        ///inputData.funcInput = nullptr;                                               ///Unused
        ///inputData.ecuInput = nullptr;                                                ///Unused
        inputData.usDrvRuntimeConfigurationResponse = &dummyUsDrvRntCfgResponse;        ///Dummy runtime config response
        inputData.usDrvDetectionList = &usRawInput;                                     ///Pulse and echo input from driver
        inputData.odoEstimationOutputPort = &odoEstimationOutputPort;                   ///Odometry port input sample

        /// Prepare output data
        us_processing::UsProcessingProvidePorts outputData{};
        outputData.diagOutput = &uspDiagOutput;                                     ///Unused
        outputData.distListOutput = &uspDistList;                                   ///Unused but zero-filled
        outputData.pointListOutput = &uspPointList;                                 ///Main points output
        ///outputData.objectListOutput = nullptr;                                   ///Unused
        outputData.usDrvRuntimeConfigurationRequest = &usDrvRntConfigRequest;       ///UsDrv Runtime Configuration Request
        outputData.echoOutput = &usFiltered;                                        ///Valid filtered echos output
        outputData.integrityOutput = &uspIntegrityOutput;                           ///Integrity output
        ///outputData.debugOutput = ;                                               ///Optional
        uspInstance->run(&inputData, &outputData);
    }
}

void usp_warning(const char_t* msg)
{
    printf("[ SWC USP ] usp_warning: %s\r\n", msg);
    for (;;) {
        /// Undefined behaviour post this call, on return, but may not be fatal.
        /// Same impl. as usp_terminate for debugging.
    }
}

void usp_terminate(const char_t* msg)
{
    printf("[ SWC USP ] usp_terminate: %s\r\n", msg);

    for (;;) {
        /// Non fatal but blocking implementation for HW.
    }
}
