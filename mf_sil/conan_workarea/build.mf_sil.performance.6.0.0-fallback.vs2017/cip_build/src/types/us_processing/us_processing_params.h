// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef US_PROCESSING_US_PROCESSING_PARAMS_H_
#define US_PROCESSING_US_PROCESSING_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "us_processing/us_processing_state_management_cfg.h"
#include "us_processing/us_processing_echo_pre_processing_cfg.h"
#include "us_processing/us_processing_reflection_tracking_cfg.h"
#include "us_processing/us_processing_object_tracking_cfg.h"
#include "us_processing/us_processing_sensor_parameters.h"
#include "us_processing/us_processing_dyn_params.h"
#include "eco/memset.h"


namespace us_processing
{

  struct UsProcessingParams
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    UsProcessingStateManagementCfg ustmParams;
    UsProcessingEchoPreProcessingCfg usepParams;
    UsProcessingReflectionTrackingCfg rftrParams;
    UsProcessingObjectTrackingCfg objtParams;
    UsProcessingSensorParameters sensParams;
    UsProcessingDynParams dynParams;
  };

  inline ::us_processing::UsProcessingParams createUsProcessingParams()
  {
    UsProcessingParams m;
    (void)::eco::memset(&m, 0U, sizeof(UsProcessingParams));
    m.sSigHeader = ::eco::createSignalHeader();
    m.ustmParams = createUsProcessingStateManagementCfg();
    m.usepParams = createUsProcessingEchoPreProcessingCfg();
    m.rftrParams = createUsProcessingReflectionTrackingCfg();
    m.objtParams = createUsProcessingObjectTrackingCfg();
    m.sensParams = createUsProcessingSensorParameters();
    m.dynParams = createUsProcessingDynParams();
    return m;
  }

} // namespace us_processing

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_processing::UsProcessingParams create_default()
  {
      return ::us_processing::createUsProcessingParams();
  }
}


#endif // US_PROCESSING_US_PROCESSING_PARAMS_H_