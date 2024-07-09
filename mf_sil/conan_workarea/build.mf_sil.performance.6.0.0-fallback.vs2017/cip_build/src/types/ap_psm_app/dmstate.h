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

#ifndef AP_PSM_APP_DMSTATE_H_
#define AP_PSM_APP_DMSTATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_psm_app
{
  /// \cond HIDDEN_ENUMS
  class DMState
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      DM_INACTIVE = 0U,
      DM_DRIVER_MANEUVERING = 1U,
      DM_DRIVER_ASSISTS_AP_MANEUVER = 2U,
      DM_DRIVER_NOT_MANEUVERING = 3U,
      DM_PASSENGER_IN_VEHICLE = 4U,
      DM_NOONE_IN_VEHICLE = 5U
    };

    DMState() :
      value(static_cast< underlying_type >(DM_INACTIVE))
    {
    }

    DMState(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DMState(const DMState& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END DMState(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    DMState& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(DMState&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    DMState& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DMState& operator=(const DMState& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(DMState&) operator=(const underlying_type v)
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
} // namespace ap_psm_app

// PRQA S 2180 --

#endif // AP_PSM_APP_DMSTATE_H_
