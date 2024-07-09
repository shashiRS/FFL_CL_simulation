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

#ifndef ECO_TIME_SERVICE_DATA_H_
#define ECO_TIME_SERVICE_DATA_H_

#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace eco
{

  struct TimeServiceData
  {
    ///Returned SignalHeader Common header for all structure.
    SignalHeader sigHeader;
    ///cycle start time
    uint64 cycleStart;
    ///Estimated Cycle End Time
    uint64 estimatedCycleEnd;
  };

  inline ::eco::TimeServiceData createTimeServiceData()
  {
    TimeServiceData m;
    (void)::eco::memset(&m, 0U, sizeof(TimeServiceData));
    m.sigHeader = createSignalHeader();
    return m;
  }

} // namespace eco

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::eco::TimeServiceData create_default()
  {
      return ::eco::createTimeServiceData();
  }
}


#endif // ECO_TIME_SERVICE_DATA_H_
