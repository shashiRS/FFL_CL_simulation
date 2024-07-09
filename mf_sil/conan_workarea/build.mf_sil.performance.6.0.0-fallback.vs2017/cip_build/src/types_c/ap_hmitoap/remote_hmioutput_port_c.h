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

#ifndef AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_C_H_
#define AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_hmitoap/user_action_remote_device_c.h"
#include "eco/memset_c.h"

/// None
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{Percent}
    ///@range{0,102}
    ///Returns battery level of remote device to ensure enough usage time of the remote device for parking function
    float32 batteryLevel_perc;
    ///@unit{Pixel}
    ///@range{0,4095}
    ///x Finger position on Smartphone display 4095: Finger is not on screen (0xfff)
    uint16 fingerPositionX_px;
    ///@unit{Pixel}
    ///@range{0,4095}
    ///y Finger position on Smartphone display 4095: Finger is not on screen (0xfff)
    uint16 fingerPositionY_px;
    ///@range{0,255}
    ///User interaction with HMI (Remote Parking App)  27: Switches to a screen, where the user can see the surround-view view / us-distance view. 28 / 29: Forward / Backward-Button for Remote Maneuvering
    AP_HMITOAP_UserActionRemoteDevice userActionRemoteDevice_nu;
    ///@range{0,255}
    ///Increases on each cycle by one (to clarify: needed here)
    uint8 aliveCounter_nu;
    ///Dead Man"s Switch pressed (test purpose only)
    boolean deadMansSwitchBtn_nu;
    ///true if bluetooth device is paried
    boolean paired_nu;
    ///true if bluetooth device is connected
    boolean connected_nu;
    ///@unit{Pixel}
    ///@range{0,16383}
    ///resolution screen curent device in X
    uint16 screenResolutionX_px;
    ///@unit{Pixel}
    ///@range{0,16383}
    ///resolution screen curent device in Y
    uint16 screenResolutionY_px;
    uint8 userActionRemCounter_nu;
} AP_HMITOAP_RemoteHMIOutputPort;

inline AP_HMITOAP_RemoteHMIOutputPort create_AP_HMITOAP_RemoteHMIOutputPort(void)
{
  AP_HMITOAP_RemoteHMIOutputPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_C_H_
