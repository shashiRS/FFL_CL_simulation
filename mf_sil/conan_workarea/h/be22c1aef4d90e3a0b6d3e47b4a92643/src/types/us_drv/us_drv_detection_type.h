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

#ifndef US_DRV_US_DRV_DETECTION_TYPE_H_
#define US_DRV_US_DRV_DETECTION_TYPE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace us_drv
{
  /// \cond HIDDEN_ENUMS
  class UsDrvDetectionType
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      ///Bits 0..3 of this enum specify confidence level of detection
      US_DETECTION_CONFIDENCE_MASK = 15U,
      ///Detection on standard path
      US_DETECTION_NORMAL_PATH = 0U,
      ///Detection on advanced path 1 (chirp up)
      US_DETECTION_ADVANCED_PATH_1 = 16U,
      ///Detection on advanced path 2 (chirp down)
      US_DETECTION_ADVANCED_PATH_2 = 32U,
      US_DETECTION_NFD_PATH = 48U,
      ///Detected AATG threshold from first channel
      US_DETECTION_AATG_THRESHOLD_1 = 64U,
      ///Detected AATG threshold from second channel
      US_DETECTION_AATG_THRESHOLD_2 = 80U,
      ///Sensor timestamp of firing event
      US_DETECTION_FIRING_TIMESTAMP = 128U,
      ///Bitmask for detection type
      US_DETECTION_TYPE_MASK = 240U
    };

    UsDrvDetectionType() :
      value(static_cast< underlying_type >(US_DETECTION_CONFIDENCE_MASK))
    {
    }

    UsDrvDetectionType(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UsDrvDetectionType(const UsDrvDetectionType& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UsDrvDetectionType(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UsDrvDetectionType& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsDrvDetectionType&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UsDrvDetectionType& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UsDrvDetectionType& operator=(const UsDrvDetectionType& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsDrvDetectionType&) operator=(const underlying_type v)
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

#endif // US_DRV_US_DRV_DETECTION_TYPE_H_
