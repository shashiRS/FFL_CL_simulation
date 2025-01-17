// Attention, this file is generated by Cobolt from template: C:\_repos\mf_sil\dbg\eco\eco.generic\codegen\templates\types\enum.h.template!

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

#ifndef ECO_ALGO_SIGNAL_STATE_H_
#define ECO_ALGO_SIGNAL_STATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace eco
{
  /// \cond HIDDEN_ENUMS
  ///Algo signal state enumeration  values: enum { AL_SIG_STATE_INIT=0
  ///,AL_SIG_STATE_OK=1,AL_SIG_STATE_INVALID=2,}
  ///@range{0,2}
  class AlgoSignalState
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      AL_SIG_STATE_INIT = 0U,
      AL_SIG_STATE_OK = 1U,
      AL_SIG_STATE_INVALID = 2U
    };

    AlgoSignalState() :
      value(static_cast< underlying_type >(AL_SIG_STATE_INIT))
    {
    }

    AlgoSignalState(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    AlgoSignalState(const AlgoSignalState& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END AlgoSignalState(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    AlgoSignalState& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(AlgoSignalState&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    AlgoSignalState& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    AlgoSignalState& operator=(const AlgoSignalState& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(AlgoSignalState&) operator=(const underlying_type v)
    {
      value = v;
      return *this;
    }

    template < typename T >
    bool operator==(const T v) const
    {
      return value == static_cast< underlying_type >(v);
    }
    /// \endcond

  private:
    underlying_type value;
  };
} // namespace eco

// PRQA S 2180 --

#endif // ECO_ALGO_SIGNAL_STATE_H_
