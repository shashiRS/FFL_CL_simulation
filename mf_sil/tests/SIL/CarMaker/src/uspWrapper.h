#pragma once

#include "carMakerAP_DLL_common.h"
#include "mf_vedodo_types/OdoServiceFunctions.h"
#include "ap_commonvehsigprovider/ambient_data_port.h"
#include "us_drv/us_drv_consts.h"
#include "us_drv/us_drv_generated_types.h"
#include "us_drv/us_drv_detection_list.h"
#include "us_processing/US_PROCESSING_Interface.h"

static constexpr uint64_t US_PROCESSING_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };

class DllExport uspWrapper
{
public:

    static uspWrapper& getInstance();

    uspWrapper(uspWrapper const&) = delete; // prevent copying the singleton
    uspWrapper& operator= (uspWrapper const&) = delete;

    void init();

    us_processing::UsProcessingSensorParameters getSensorParameters();

    void run(
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
    );

private:
    uspWrapper() = default;
    us_drv::UsDrvRuntimeConfiguration dummyUsDrvRntCfgResponse{};

};
