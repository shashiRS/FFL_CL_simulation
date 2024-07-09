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

#ifndef VC_ERROR_CODE_DATA_PORT_C_H_
#define VC_ERROR_CODE_DATA_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "vc/error_code_c.h"
#include "vc/hmiwarning_message_c.h"
#include "vc/hmiinstruction_set_c.h"
#include "eco/memset_c.h"

/// Error code has been displayed on HMI
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    VC_ErrorCode errorCode;
    VC_HMIWarningMessage hmiWarningMessage;
    VC_HMIInstructionSet hmiInstructionSet;
} VC_ErrorCodeDataPort;

inline VC_ErrorCodeDataPort create_VC_ErrorCodeDataPort(void)
{
  VC_ErrorCodeDataPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // VC_ERROR_CODE_DATA_PORT_C_H_