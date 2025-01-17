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

#ifndef MF_HMIH_HUCURRENT_FEATURE_STATE_H_
#define MF_HMIH_HUCURRENT_FEATURE_STATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_hmih
{
  /// \cond HIDDEN_ENUMS
  ///Current status of the APA features
  class HUCurrentFeatureState
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      NOT_USED = 0U,
      NO_FEATURE_ACTIVE = 1U,
      PRE_CONDITION_NOT_MET = 2U,
      PARK_IN = 3U,
      PARK_OUT = 4U,
      GARAGE_PARK_IN = 5U,
      SVS = 6U,
      REVERSE_ASSIST = 7U
    };

    HUCurrentFeatureState() :
      value(static_cast< underlying_type >(NOT_USED))
    {
    }

    HUCurrentFeatureState(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    HUCurrentFeatureState(const HUCurrentFeatureState& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END HUCurrentFeatureState(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    HUCurrentFeatureState& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(HUCurrentFeatureState&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    HUCurrentFeatureState& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    HUCurrentFeatureState& operator=(const HUCurrentFeatureState& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(HUCurrentFeatureState&) operator=(const underlying_type v)
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
} // namespace mf_hmih

// PRQA S 2180 --

#endif // MF_HMIH_HUCURRENT_FEATURE_STATE_H_
