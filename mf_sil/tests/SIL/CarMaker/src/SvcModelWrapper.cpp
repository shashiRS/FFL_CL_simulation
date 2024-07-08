#include "SvcModelWrapper.h"

SvcModelWrapper::SvcModelWrapper()
{
    svc_model_processing::SvcModelProcessingInterface::Init();
}

SvcModelWrapper::~SvcModelWrapper()
{
    svc_model_processing::SvcModelProcessingInterface::Release();
}

SvcModelWrapper& SvcModelWrapper::Instance()
{
    static SvcModelWrapper instance{};
    return instance;
}

svc_model_processing::SvcModelProcessingSettings SvcModelWrapper::AllocateSettings()
{
    return svc_model_processing::SvcModelProcessingInterface::Instance()->AllocateSettings();
}

svc_model_processing::SvcModelProcessingInput SvcModelWrapper::AllocateInput()
{
    return svc_model_processing::SvcModelProcessingInterface::Instance()->AllocateInput();
}

svc_model_processing::SvcModelProcessingOutput SvcModelWrapper::AllocateOutput()
{
    return svc_model_processing::SvcModelProcessingInterface::Instance()->AllocateOutput();
}

void SvcModelWrapper::ReleaseSettings(svc_model_processing::SvcModelProcessingSettings& settings)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->ReleaseSettings(settings);
}

void SvcModelWrapper::ReleaseInput(svc_model_processing::SvcModelProcessingInput& input)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->ReleaseInput(input);
}

void SvcModelWrapper::ReleaseOutput(svc_model_processing::SvcModelProcessingOutput& output)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->ReleaseOutput(output);
}

void SvcModelWrapper::ResetSettings(svc_model_processing::SvcModelProcessingSettings& settings)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->ResetSettings(settings);
}

void SvcModelWrapper::ResetInput(svc_model_processing::SvcModelProcessingInput& input)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->ResetInput(input);
}

void SvcModelWrapper::ResetOutput(svc_model_processing::SvcModelProcessingOutput& output)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->ResetOutput(output);
}

void SvcModelWrapper::init(const svc_model_processing::SvcModelProcessingSettings& settings)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->create(settings);
}

void SvcModelWrapper::process(const svc_model_processing::SvcModelProcessingInput& input,
    svc_model_processing::SvcModelProcessingOutput& output)
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->process(input, output);
}

void SvcModelWrapper::release()
{
    svc_model_processing::SvcModelProcessingInterface::Instance()->destroy();
}
