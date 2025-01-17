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

#ifndef US_DRV_US_DRV_REQUEST_MEAS_MODE_H_
#define US_DRV_US_DRV_REQUEST_MEAS_MODE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace us_drv
{
  /// \cond HIDDEN_ENUMS
  class UsDrvRequestMeasMode
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      ///Measurement mode set to near range echoes
      US_MEASMODE_NEAR_RANGE_FIELD = 0U,
      ///Measurement mode set to far range echoes
      US_MEASMODE_FAR_RANGE_FIELD = 1U,
      ///Number of measurement modes and value used when no mode is available due to sensor not running
      US_MEASMODE_COUNT = 2U
    };

    UsDrvRequestMeasMode() :
      value(static_cast< underlying_type >(US_MEASMODE_NEAR_RANGE_FIELD))
    {
    }

    UsDrvRequestMeasMode(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UsDrvRequestMeasMode(const UsDrvRequestMeasMode& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UsDrvRequestMeasMode(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UsDrvRequestMeasMode& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsDrvRequestMeasMode&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UsDrvRequestMeasMode& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UsDrvRequestMeasMode& operator=(const UsDrvRequestMeasMode& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsDrvRequestMeasMode&) operator=(const underlying_type v)
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
} // namespace us_drv

// PRQA S 2180 --

#endif // US_DRV_US_DRV_REQUEST_MEAS_MODE_H_
