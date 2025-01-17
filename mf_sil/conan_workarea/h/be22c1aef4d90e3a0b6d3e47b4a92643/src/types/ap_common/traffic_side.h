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

#ifndef AP_COMMON_TRAFFIC_SIDE_H_
#define AP_COMMON_TRAFFIC_SIDE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_common
{
  /// \cond HIDDEN_ENUMS
  ///
  class TrafficSide
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      LEFT_HAND_TRAFFIC = 0U,
      RIGHT_HAND_TRAFFIC = 1U,
      MAX_NUM_TRAFFIC_SIDE_TYPES = 2U
    };

    TrafficSide() :
      value(static_cast< underlying_type >(LEFT_HAND_TRAFFIC))
    {
    }

    TrafficSide(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    TrafficSide(const TrafficSide& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END TrafficSide(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    TrafficSide& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(TrafficSide&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    TrafficSide& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    TrafficSide& operator=(const TrafficSide& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(TrafficSide&) operator=(const underlying_type v)
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
} // namespace ap_common

// PRQA S 2180 --

#endif // AP_COMMON_TRAFFIC_SIDE_H_
