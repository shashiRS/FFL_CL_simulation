// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef MF_HMIH_USER_DEFINED_SLOT_PORT_H_
#define MF_HMIH_USER_DEFINED_SLOT_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "lsm_geoml/pose_pod.h"
#include "mf_hmih/user_defined_slot_side.h"
#include "mf_hmih/user_defined_slot_type.h"
#include "Platform_Types.h"


namespace mf_hmih
{

  struct UserDefinedSlotPort
  {
    ///Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ///Common header for all structured data types (e.g. timestamp)
    ::eco::SignalHeader sSigHeader{};
    ::lsm_geoml::Pose_POD pose{};
    ///Selected side for the user defined slot
    UserDefinedSlotSide slotSide_nu{};
    ///Selected type for the user defined slot
    UserDefinedSlotType slotType_nu{};
    ///Indicates whether we are in the case that the user defined a slot via HMI and the port contains the corresponding information
    boolean userDefined_nu{};
    ///Indicates that the data in the memory slot selected by user is valid.
    boolean valid{};
  };

} // namespace mf_hmih

#endif // MF_HMIH_USER_DEFINED_SLOT_PORT_H_
