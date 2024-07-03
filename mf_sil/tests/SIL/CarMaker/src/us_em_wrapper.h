#pragma once

#include "carMakerAP_DLL_common.h"
#include "lsm_vedodo/odo_estimation_output_port.h"
#include "us_processing/us_processing_generated_types.h"
#include "us_em/us_em_generated_types.h"
#include "us_em/us_em_debug_output_port.h"

#ifdef USE_ENV_PLOTTER
#include "mf_plot/MF_PlotterCem.h"
#endif

static constexpr uint64_t US_EM_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };

class DllExport UsEmWrapper
{
public:
    // @details return the instance of the UsEmWrapper; singleton since UsEmInterface is a singleton too
    static UsEmWrapper& getInstance();
    UsEmWrapper(UsEmWrapper const&) = delete; // prevent copying the singleton
    UsEmWrapper& operator= (UsEmWrapper const&) = delete;

    //!
    //! \brief      Initialize the US_EM algorithm.
    //!
    void init();

    void run(const lsm_vedodo::OdoEstimationOutputPort& odoEstimationOutputPort,
        const us_processing::UsProcessingPointList& uspPointList,
        const us_processing::UsProcessingDistanceList& uspDistList,
        const us_processing::UsProcessingDiagOutput& uspDiagOutput,
        us_em::UsEnvModelPort& usEnvModelPort,
        us_em::PerceptionAvailabilityPort& usPercAvailPort,
        us_em::UsEmDebugOutputPort& usEmDebugPort);

#ifdef USE_ENV_PLOTTER
    //!
    //! \brief      Converts the objects from the UsEnvModelPort to a plotterCemObjectList.
    //!
    static void convertToCemObjectsForPlotter(const us_em::UsEnvModelPort& usEnvModelPort, MF_Plot::plotterCemObjectList& cemObjects);
#endif
};