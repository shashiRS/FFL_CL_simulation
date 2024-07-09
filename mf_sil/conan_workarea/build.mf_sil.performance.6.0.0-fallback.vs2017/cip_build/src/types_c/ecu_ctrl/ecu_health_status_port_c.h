//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef ECU_CTRL_ECU_HEALTH_STATUS_PORT_C_H_
#define ECU_CTRL_ECU_HEALTH_STATUS_PORT_C_H_

#include "Platform_Types.h"
#include "ecu_ctrl/ecu_state_c.h"
#include "ecu_ctrl/voltage_state_c.h"
#include "ecu_ctrl/comm_state_c.h"
#include "ecu_ctrl/temp_state_c.h"
#include "eco/memset_c.h"

typedef struct
{
    uint64 timestamp_us_u64;
    float32 ecuVoltageValue_V;
    ECU_CTRL_EcuState globalState_e;
    ECU_CTRL_VoltageState ecuVoltageState_e;
    ECU_CTRL_CommState commState_e;
    ECU_CTRL_TempState temperatureState_e;
} ECU_CTRL_EcuHealthStatusPort;

inline ECU_CTRL_EcuHealthStatusPort create_ECU_CTRL_EcuHealthStatusPort(void)
{
  ECU_CTRL_EcuHealthStatusPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // ECU_CTRL_ECU_HEALTH_STATUS_PORT_C_H_