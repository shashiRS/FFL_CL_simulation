// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\enum.h.template!

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

#ifndef MF_DRVWARNSM_PDWSTATE_H_
#define MF_DRVWARNSM_PDWSTATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_drvwarnsm
{
  /// \cond HIDDEN_ENUMS
  ///Internal state of the PDW state machine
  class PDWState
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      PDW_INT_STATE_INIT = 0U,
      PDW_INT_STATE_ACT_BTN = 1U,
      PDW_INT_STATE_ACT_R_GEAR = 2U,
      PDW_INT_STATE_ACT_AP = 3U,
      PDW_INT_STATE_ACT_AUTO = 4U,
      PDW_INT_STATE_ACT_ROLLBACK = 5U,
      PDW_INT_STATE_ACT_RA = 6U,
      PDW_INT_STATE_DEACT_INIT = 7U,
      PDW_INT_STATE_DEACT_BTN = 8U,
      PDW_INT_STATE_DEACT_SPEED = 9U,
      PDW_INT_STATE_DEACT_P_GEAR = 10U,
      PDW_INT_STATE_DEACT_EPB = 11U,
      PDW_INT_STATE_DEACT_AP_FIN = 12U,
      PDW_INT_STATE_FAILURE = 13U,
      PDW_INT_NUM_STATES = 14U
    };

    PDWState() :
      value(static_cast< underlying_type >(PDW_INT_STATE_INIT))
    {
    }

    PDWState(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    PDWState(const PDWState& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END PDWState(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    PDWState& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PDWState&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    PDWState& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    PDWState& operator=(const PDWState& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PDWState&) operator=(const underlying_type v)
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
} // namespace mf_drvwarnsm

// PRQA S 2180 --

#endif // MF_DRVWARNSM_PDWSTATE_H_
