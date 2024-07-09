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

#ifndef AP_PSM_RASTATE_H_
#define AP_PSM_RASTATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_psm
{
  /// \cond HIDDEN_ENUMS
  ///State of the Reverse Assist.
  class RAState
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      RA_INACTIVE = 0U,
      RA_RECORD = 1U,
      RA_AVG_ACTIVE = 2U,
      RA_AVG_PAUSE = 3U,
      RA_AVG_FINISHED = 4U
    };

    RAState() :
      value(static_cast< underlying_type >(RA_INACTIVE))
    {
    }

    RAState(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    RAState(const RAState& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END RAState(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    RAState& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(RAState&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    RAState& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    RAState& operator=(const RAState& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(RAState&) operator=(const underlying_type v)
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
} // namespace ap_psm

// PRQA S 2180 --

#endif // AP_PSM_RASTATE_H_
