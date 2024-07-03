#ifndef __SVC_MODEL_WRAPPER_H__
#define __SVC_MODEL_WRAPPER_H__

#include "carMakerAP_DLL_common.h"

#include <SvcModelProcessingData.hpp>
#include <SvcModelProcessingInterface.hpp>

static constexpr uint64_t SVC_MODEL_SAMPLE_TIME_MS{ LONG_SAMPLE_TIME_MS };

class SvcModelWrapper
{
public:

    ~SvcModelWrapper();

    static SvcModelWrapper& Instance();

    svc_model_processing::SvcModelProcessingSettings AllocateSettings();
    svc_model_processing::SvcModelProcessingInput AllocateInput();
    svc_model_processing::SvcModelProcessingOutput AllocateOutput();

    void ReleaseSettings(svc_model_processing::SvcModelProcessingSettings& settings);
    void ReleaseInput(svc_model_processing::SvcModelProcessingInput& input);
    void ReleaseOutput(svc_model_processing::SvcModelProcessingOutput& output);

    void ResetSettings(svc_model_processing::SvcModelProcessingSettings& settings);
    void ResetInput(svc_model_processing::SvcModelProcessingInput& input);
    void ResetOutput(svc_model_processing::SvcModelProcessingOutput& output);

    void init(const svc_model_processing::SvcModelProcessingSettings& settings);
    void process(const svc_model_processing::SvcModelProcessingInput& input,
        svc_model_processing::SvcModelProcessingOutput& output);
    void release();

private:

    SvcModelWrapper();
};

#endif
