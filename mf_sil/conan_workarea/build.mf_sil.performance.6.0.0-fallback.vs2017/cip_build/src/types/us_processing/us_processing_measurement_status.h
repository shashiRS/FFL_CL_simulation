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

#ifndef US_PROCESSING_US_PROCESSING_MEASUREMENT_STATUS_H_
#define US_PROCESSING_US_PROCESSING_MEASUREMENT_STATUS_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace us_processing
{
  /// \cond HIDDEN_ENUMS
  class UsProcessingMeasurementStatus
  {
  public:
    typedef uint32 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      ///Point measurement status is invalid
      US_PROCESSING_MEASUREMENT_INVALID = 0U,
      ///This indicates the first detection of this particular point in the point list
      US_PROCESSING_MEASUREMENT_NEW = 1U,
      ///The point was updated with new information from a sensor in the last update cycle of this point list
      US_PROCESSING_MEASUREMENT_UPDATED = 2U,
      ///The point was predicted without any new information with respect to the previous update cycle
      US_PROCESSING_MEASUREMENT_PREDICTED = 3U
    };

    UsProcessingMeasurementStatus() :
      value(static_cast< underlying_type >(US_PROCESSING_MEASUREMENT_INVALID))
    {
    }

    UsProcessingMeasurementStatus(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UsProcessingMeasurementStatus(const UsProcessingMeasurementStatus& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UsProcessingMeasurementStatus(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UsProcessingMeasurementStatus& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsProcessingMeasurementStatus&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UsProcessingMeasurementStatus& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UsProcessingMeasurementStatus& operator=(const UsProcessingMeasurementStatus& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsProcessingMeasurementStatus&) operator=(const underlying_type v)
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
} // namespace us_processing

// PRQA S 2180 --

#endif // US_PROCESSING_US_PROCESSING_MEASUREMENT_STATUS_H_