#include "us_em_wrapper.h"
#include "us_em/US_EM_Interface.h"
#include "us_em/us_em_generated_types.h"
#include "assert.h"
#ifdef USE_ENV_PLOTTER
#include "pod_class_converter.h"
#endif

static us_em::UsEmInterface* us_emInstance{};

UsEmWrapper & UsEmWrapper::getInstance()
{
    static UsEmWrapper instance{};
    return instance;
}

void UsEmWrapper::init()
{
    us_em::UsEmParams us_emParams;
    us_em::initUsEmParams(&us_emParams);

    const us_em::UsEmCfgInput configData = { &us_emParams };

    //Init US_EM
    us_emInstance = &(us_em::getUsEmInstance());
    us_emInstance->init(configData);
}

void UsEmWrapper::run(const lsm_vedodo::OdoEstimationOutputPort& odoEstimationOutputPort,
    const us_processing::UsProcessingPointList& uspPointList,
    const us_processing::UsProcessingDistanceList& uspDistList,
    const us_processing::UsProcessingDiagOutput& uspDiagOutput,
    us_em::UsEnvModelPort& usEnvModelPort,
    us_em::PerceptionAvailabilityPort& usPercAvailPort,
    us_em::UsEmDebugOutputPort& usEmDebugPort)
{
    assert(us_emInstance);

    // Prepare input data
    static us_em::UsEmInputData inputData{};
    inputData.odoInput = &odoEstimationOutputPort;
    inputData.uspPtsInput = &uspPointList;
    inputData.uspDistInput = &uspDistList;
    inputData.uspDiagInput = &uspDiagOutput;

    // Prepare output data
    static us_em::UsEmOutputData outputData{};
    outputData.envModelPort = &usEnvModelPort;
    outputData.percAvailPort = &usPercAvailPort;
    outputData.usEmDebugPort = &usEmDebugPort;

    us_emInstance->run(inputData, outputData);
}


#ifdef USE_ENV_PLOTTER
void UsEmWrapper::convertToCemObjectsForPlotter(const us_em::UsEnvModelPort& usEnvModelPort, MF_Plot::plotterCemObjectList& cemObjects)
{
    cemObjects.clear();
    MF_Plot::CemObjectData cemObjData{};
    // static objects
    for (uint8_t idx{ 0U }; idx < usEnvModelPort.numberOfStaticObjects_u8; ++idx) {
        cemObjData.id = usEnvModelPort.staticObjects[idx].refObjID_nu;
        cemObjData.points.clear();
        cemObjData.points.append(convert(usEnvModelPort.staticObjects[idx].objShape_m));
        cemObjects.append(cemObjData);
    }
    // dynamic objects
    for (uint8_t idx{ 0U }; idx < usEnvModelPort.numberOfDynamicObjects_u8; ++idx) {
        cemObjData.id = usEnvModelPort.dynamicObjects[idx].refObjID_nu;
        cemObjData.points.clear();
        cemObjData.points.append(convert(usEnvModelPort.dynamicObjects[idx].objShape_m));
        cemObjects.append(cemObjData);
    }
}
#endif
