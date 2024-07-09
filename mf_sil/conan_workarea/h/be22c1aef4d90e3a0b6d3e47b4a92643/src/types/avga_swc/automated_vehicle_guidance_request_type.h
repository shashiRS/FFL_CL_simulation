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

#ifndef AVGA_SWC_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_TYPE_H_
#define AVGA_SWC_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_TYPE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace avga_swc
{
  /// \cond HIDDEN_ENUMS
  class AutomatedVehicleGuidanceRequestType
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      AVGA_NONE = 0U,
      AVGA_BRAKING_SUPERVISION = 1U,
      AVGA_BRAKING_VISUAL_SUPERVISION = 2U,
      AVGA_BRAKING_AUDIO_VISUAL_SUPERVISION = 3U
    };

    AutomatedVehicleGuidanceRequestType() :
      value(static_cast< underlying_type >(AVGA_NONE))
    {
    }

    AutomatedVehicleGuidanceRequestType(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    AutomatedVehicleGuidanceRequestType(const AutomatedVehicleGuidanceRequestType& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END AutomatedVehicleGuidanceRequestType(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    AutomatedVehicleGuidanceRequestType& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(AutomatedVehicleGuidanceRequestType&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    AutomatedVehicleGuidanceRequestType& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    AutomatedVehicleGuidanceRequestType& operator=(const AutomatedVehicleGuidanceRequestType& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(AutomatedVehicleGuidanceRequestType&) operator=(const underlying_type v)
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
} // namespace avga_swc

// PRQA S 2180 --

#endif // AVGA_SWC_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_TYPE_H_
