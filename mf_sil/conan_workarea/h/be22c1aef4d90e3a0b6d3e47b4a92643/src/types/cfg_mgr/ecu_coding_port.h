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

#ifndef CFG_MGR_ECU_CODING_PORT_H_
#define CFG_MGR_ECU_CODING_PORT_H_

#include "Platform_Types.h"
#include "cfg_mgr/ecu_coding_state.h"
#include "cfg_mgr/us_sensor_degradation_type.h"
#include "eco/memset.h"


namespace cfg_mgr
{

  struct EcuCodingPort
  {
    uint64 timestamp_us_u64;
    EcuCodingState codingState_e;
    boolean isUsSupported;
    uint8 numOfUsSensors;
    UsSensorDegradationType UsDegradationMode_e;
    boolean isPdwFrontOnlySupported;
    boolean isPdwRearOnlySupported;
    boolean isPdw360Supported;
    boolean isBrakingFrontOnlySupported;
    boolean isBrakingRearOnlySupported;
    boolean isBraking360Supported;
    boolean isSteeringSuggestSupported;
    boolean isSteeringProtectSupported;
    boolean isWhlProtectSupported;
    boolean isSemiAuParkSupported;
    boolean isFullAuParkSupported;
    boolean isRemoteParkSupported;
    boolean isBasicGarageParkSupported;
  };

  inline ::cfg_mgr::EcuCodingPort createEcuCodingPort()
  {
    EcuCodingPort m;
    (void)::eco::memset(&m, 0U, sizeof(EcuCodingPort));
    return m;
  }

} // namespace cfg_mgr

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::cfg_mgr::EcuCodingPort create_default()
  {
      return ::cfg_mgr::createEcuCodingPort();
  }
}


#endif // CFG_MGR_ECU_CODING_PORT_H_
