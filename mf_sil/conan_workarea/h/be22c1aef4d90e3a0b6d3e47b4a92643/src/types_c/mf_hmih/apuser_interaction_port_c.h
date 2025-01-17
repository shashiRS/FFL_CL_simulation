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

#ifndef MF_HMIH_APUSER_INTERACTION_PORT_C_H_
#define MF_HMIH_APUSER_INTERACTION_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "mf_hmih/remote_device_interaction_c.h"
#include "mf_hmih/head_unit_interaction_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Store information about user action.
typedef struct
{
    ///Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ///Common header for all structured data types (e.g. timestamp)
    ECO_SignalHeader sSigHeader;
    ///@unit{nu}
    ///Keep information about the status of the remote device.
    MF_HMIH_RemoteDeviceInteraction remoteDeviceInteraction;
    ///@unit{nu}
    ///User interaction with HMI handler
    MF_HMIH_HeadUnitInteraction headUnitInteraction;
    ///@range{0,255}
    ///Unique identifier of selected target pose. If no pose is selected, ID is 255.
    uint8 selectedTPID_nu;
    ///@range{0,255}
    ///Unique identifier of selected memory slot
    uint8 selectedMemorySlotID_nu;
    ///save request from user
    boolean saveRequest_nu;
    ///delete request from user
    boolean deleteRequest_nu;
} MF_HMIH_APUserInteractionPort;

inline MF_HMIH_APUserInteractionPort create_MF_HMIH_APUserInteractionPort(void)
{
  MF_HMIH_APUserInteractionPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.remoteDeviceInteraction = create_MF_HMIH_RemoteDeviceInteraction();
  m.headUnitInteraction = create_MF_HMIH_HeadUnitInteraction();
  m.selectedTPID_nu = 255U;
  m.selectedMemorySlotID_nu = 255U;
  return m;
}

#endif // MF_HMIH_APUSER_INTERACTION_PORT_C_H_
