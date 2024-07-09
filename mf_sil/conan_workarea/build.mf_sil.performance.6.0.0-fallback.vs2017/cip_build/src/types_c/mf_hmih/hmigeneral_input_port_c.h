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

#ifndef MF_HMIH_HMIGENERAL_INPUT_PORT_C_H_
#define MF_HMIH_HMIGENERAL_INPUT_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "mf_hmih/hmigeneral_c.h"
#include "mf_hmih/parking_spaces_c.h"
#include "mf_hmih/parking_situation_c.h"
#include "mf_hmih/pdcsectors_c.h"
#include "mf_hmih/driving_tube_c.h"
#include "mf_hmih/general_warnings_c.h"
#include "mf_hmih/wheel_warnings_c.h"
#include "mf_hmih/lsca_warnings_c.h"
#include "mf_hmih/memory_parking_info_c.h"
#include "ap_tp/reverse_assist_availability_port_c.h"
#include "eco/memset_c.h"

/// Store all information which is send to the HMI.
typedef struct
{
    ///Provide a upconversion for old measurement to newer interfaces(needed as the first field in a port).
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ///Common header for all structured data types (e.g. timestamp)
    ECO_SignalHeader sSigHeader;
    ///@unit{0}
    ///Major screen control of HMI.
    MF_HMIH_HMIGeneral general;
    ///@unit{0}
    ///Status of the parking spaces
    MF_HMIH_ParkingSpaces parkingSpaces;
    ///@unit{0}
    ///Parking space situation around the vehicle
    MF_HMIH_ParkingSituation parkingSituation;
    ///@unit{0}
    ///Information for the PDC sectors
    MF_HMIH_PDCSectors pdcSectors;
    ///@unit{0}
    ///Driving tube
    MF_HMIH_DrivingTube drivingTube;
    ///@unit{0}
    ///General warnings from the driving function system.
    MF_HMIH_GeneralWarnings generalWarnings;
    ///@unit{0}
    ///Wheel warning information coming from WHP
    MF_HMIH_WheelWarnings wheelWarnings;
    ///@unit{0}
    ///LSCA warning information
    MF_HMIH_LscaWarnings lscaWarnings;
    MF_HMIH_MemoryParkingInfo memoryParkingInfo;
    ///Availability of the reverse assist function and information about the saved reverse path.
    AP_TP_ReverseAssistAvailabilityPort reverseAssistAvailabilityPort_nu;
} MF_HMIH_HMIGeneralInputPort;

inline MF_HMIH_HMIGeneralInputPort create_MF_HMIH_HMIGeneralInputPort(void)
{
  MF_HMIH_HMIGeneralInputPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.general = create_MF_HMIH_HMIGeneral();
  m.parkingSpaces = create_MF_HMIH_ParkingSpaces();
  m.parkingSituation = create_MF_HMIH_ParkingSituation();
  m.pdcSectors = create_MF_HMIH_PDCSectors();
  m.drivingTube = create_MF_HMIH_DrivingTube();
  m.generalWarnings = create_MF_HMIH_GeneralWarnings();
  m.wheelWarnings = create_MF_HMIH_WheelWarnings();
  m.lscaWarnings = create_MF_HMIH_LscaWarnings();
  m.memoryParkingInfo = create_MF_HMIH_MemoryParkingInfo();
  m.reverseAssistAvailabilityPort_nu = create_AP_TP_ReverseAssistAvailabilityPort();
  return m;
}

#endif // MF_HMIH_HMIGENERAL_INPUT_PORT_C_H_