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

#ifndef AP_HMITOAP_PARKING_AUGMENTATION_TYPE_H_
#define AP_HMITOAP_PARKING_AUGMENTATION_TYPE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_hmitoap
{
  /// \cond HIDDEN_ENUMS
  class ParkingAugmentationType
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      PARKING_AUGMENTATION_OFF = 0U,
      PARKING_IN_ON = 1U,
      PARKING_OUT_ON = 2U
    };

    ParkingAugmentationType() :
      value(static_cast< underlying_type >(PARKING_AUGMENTATION_OFF))
    {
    }

    ParkingAugmentationType(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    ParkingAugmentationType(const ParkingAugmentationType& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END ParkingAugmentationType(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    ParkingAugmentationType& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(ParkingAugmentationType&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    ParkingAugmentationType& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    ParkingAugmentationType& operator=(const ParkingAugmentationType& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(ParkingAugmentationType&) operator=(const underlying_type v)
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
} // namespace ap_hmitoap

// PRQA S 2180 --

#endif // AP_HMITOAP_PARKING_AUGMENTATION_TYPE_H_
