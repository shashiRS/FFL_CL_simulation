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

#ifndef AP_TP_DRIVEN_PATH_DATA_PORT_H_
#define AP_TP_DRIVEN_PATH_DATA_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_tp/stored_waypoint_data.h"
#include "eco/memset.h"


namespace ap_tp
{

  /// Data structure for storing way points of a driven path
  struct DrivenPathDataPort
  {
    ///@unit{nu}
    ///Version number of the driven path data port
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ///Signal information
    ::eco::SignalHeader sSigHeader;
    ///@unit{nu}
    ///@range{0,255}
    ///Counter changes when saving data requested, can overflow
    uint8 saveCounter;
    ///@unit{nu}
    ///@range{0,1}
    ///Indicator whether port contains valid data
    boolean hasValidData;
    ///Actual driven path containing samples in one driving direction
    StoredWaypointData storedDrivenPath[1000];
    ///@unit{nu}
    ///@range{0,AP_TP.AP_TP_Const.AP_P_MAX_NUM_SAMPLES_IN_DRIVEN_PATH}
    ///Number of valid elements in driven path
    uint16 numElementsInDrivenPath;
    ///Buffer for storing way points which are in the opposite direction of the driven path
    StoredWaypointData buffer[10];
    ///@unit{nu}
    ///@range{0,AP_TP.AP_TP_Const.AP_P_MAX_NUM_SAMPLES_IN_DRIVEN_PATH_BUFFER}
    ///Number of valid elements in buffer
    uint8 numElementsInBuffer;
  };

  inline ::ap_tp::DrivenPathDataPort createDrivenPathDataPort()
  {
    DrivenPathDataPort m;
    (void)::eco::memset(&m, 0U, sizeof(DrivenPathDataPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.saveCounter = 0U;
    m.hasValidData = 0;
    {
      const uint64 arraysize = (sizeof(m.storedDrivenPath) / sizeof(m.storedDrivenPath[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.storedDrivenPath[i] = createStoredWaypointData();
      }
    }
    m.numElementsInDrivenPath = 0U;
    {
      const uint64 arraysize = (sizeof(m.buffer) / sizeof(m.buffer[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.buffer[i] = createStoredWaypointData();
      }
    }
    m.numElementsInBuffer = 0U;
    return m;
  }

} // namespace ap_tp

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_tp::DrivenPathDataPort create_default()
  {
      return ::ap_tp::createDrivenPathDataPort();
  }
}


#endif // AP_TP_DRIVEN_PATH_DATA_PORT_H_
