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

#ifndef AP_PSM_APPLANNING_SPECIFICATION_H_
#define AP_PSM_APPLANNING_SPECIFICATION_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_psm
{
  /// \cond HIDDEN_ENUMS
  ///Additional information to consider for the planning of the requested parking maneuver.
  class APPlanningSpecification
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      APPS_INVALID = 0U,
      APPS_NONE = 1U,
      APPS_PARK_IN_FULL_MANEUVERING_AREA = 2U,
      APPS_PARK_IN_RESTRICTED_MANEUVERING_AREA = 3U,
      APPS_PARK_OUT_UNTIL_CRITICAL_POINT_REACHED = 4U,
      APPS_PARK_OUT_TO_TARGET_POSE = 5U
    };

    APPlanningSpecification() :
      value(static_cast< underlying_type >(APPS_INVALID))
    {
    }

    APPlanningSpecification(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    APPlanningSpecification(const APPlanningSpecification& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END APPlanningSpecification(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    APPlanningSpecification& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(APPlanningSpecification&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    APPlanningSpecification& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    APPlanningSpecification& operator=(const APPlanningSpecification& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(APPlanningSpecification&) operator=(const underlying_type v)
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

#endif // AP_PSM_APPLANNING_SPECIFICATION_H_
