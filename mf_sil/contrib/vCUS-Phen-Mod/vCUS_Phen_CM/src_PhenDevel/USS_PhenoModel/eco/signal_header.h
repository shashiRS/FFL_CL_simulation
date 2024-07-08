// Attention, this file is generated by Cobolt from template: C:\_repos\mf_sil\dbg\eco\eco.generic\codegen\templates\types\struct.h.template!

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

#ifndef ECO_SIGNAL_HEADER_H_
#define ECO_SIGNAL_HEADER_H_

#include "eco/algo_data_time_stamp.h"
#include "eco/algo_cycle_counter.h"
#include "eco/algo_signal_state.h"
#include "eco/memset.h"


namespace eco
{

  /// Common header for all structured data types
  struct SignalHeader
  {

  ///Time stamp of the signal (for SIP: T0 timestamp of input signal
  ///, otherwise: T7 timestamp to which signal has been adjusted)
    AlgoDataTimeStamp uiTimeStamp;

  ///Deprecated, not used. Should be set to 0.
    AlgoCycleCounter uiMeasurementCounter;

  ///Rolling counter of source component. Will be incremented in every
  ///call to the exec method of the component.
    AlgoCycleCounter uiCycleCounter;

  ///Validity status of the signal. If not set to AL_SIG_STATE_OK,
  ///then do not evaluate the content of the associated signal!
    AlgoSignalState eSigStatus;
  };

  inline ::eco::SignalHeader createSignalHeader()
  {
    SignalHeader m;
    (void)::eco::memset(&m, 0U, sizeof(SignalHeader));
    return m;
  }

} // namespace eco

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::eco::SignalHeader create_default()
  {
      return ::eco::createSignalHeader();
  }
}


#endif // ECO_SIGNAL_HEADER_H_