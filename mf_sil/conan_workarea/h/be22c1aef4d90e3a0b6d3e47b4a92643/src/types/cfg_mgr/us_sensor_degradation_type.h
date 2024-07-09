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

#ifndef CFG_MGR_US_SENSOR_DEGRADATION_TYPE_H_
#define CFG_MGR_US_SENSOR_DEGRADATION_TYPE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace cfg_mgr
{
  /// \cond HIDDEN_ENUMS
  class UsSensorDegradationType
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      ///DEGRADATION_MODE_NO_DEGRADATION data or struct is not valid
      DEGRADATION_MODE_NO_DEGRADATION = 0U,
      ///DEGRADATION_MODE_GROUP_DEGRADATION data is valid for other usage
      DEGRADATION_MODE_GROUP_DEGRADATION = 1U,
      ///DEGRADATION_MODE_SENSOR_DEGRADATION data is invalid or out of range, usage with care
      DEGRADATION_MODE_SENSOR_DEGRADATION = 2U
    };

    UsSensorDegradationType() :
      value(static_cast< underlying_type >(DEGRADATION_MODE_NO_DEGRADATION))
    {
    }

    UsSensorDegradationType(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UsSensorDegradationType(const UsSensorDegradationType& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UsSensorDegradationType(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UsSensorDegradationType& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsSensorDegradationType&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UsSensorDegradationType& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UsSensorDegradationType& operator=(const UsSensorDegradationType& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsSensorDegradationType&) operator=(const underlying_type v)
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
} // namespace cfg_mgr

// PRQA S 2180 --

#endif // CFG_MGR_US_SENSOR_DEGRADATION_TYPE_H_