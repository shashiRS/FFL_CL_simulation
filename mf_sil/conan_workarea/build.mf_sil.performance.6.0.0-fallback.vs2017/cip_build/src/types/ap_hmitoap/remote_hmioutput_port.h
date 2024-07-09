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

#ifndef AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_H_
#define AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_hmitoap/user_action_remote_device.h"
#include "eco/memset.h"


namespace ap_hmitoap
{

  /// None
  struct RemoteHMIOutputPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
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
    UserActionRemoteDevice userActionRemoteDevice_nu;
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
  };

  inline ::ap_hmitoap::RemoteHMIOutputPort createRemoteHMIOutputPort()
  {
    RemoteHMIOutputPort m;
    (void)::eco::memset(&m, 0U, sizeof(RemoteHMIOutputPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace ap_hmitoap

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_hmitoap::RemoteHMIOutputPort create_default()
  {
      return ::ap_hmitoap::createRemoteHMIOutputPort();
  }
}


#endif // AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_H_
