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

#ifndef AP_LODMC_SLOPE_ACCURACY_H_
#define AP_LODMC_SLOPE_ACCURACY_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_lodmc
{
  /// \cond HIDDEN_ENUMS
  ///Accurcay of current estimation of dynamic slope.
  class SlopeAccuracy
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      HIGH = 0U,
      MID = 1U,
      LOW = 2U,
      UNDEFINED = 3U
    };

    SlopeAccuracy() :
      value(static_cast< underlying_type >(HIGH))
    {
    }

    SlopeAccuracy(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    SlopeAccuracy(const SlopeAccuracy& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END SlopeAccuracy(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    SlopeAccuracy& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(SlopeAccuracy&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    SlopeAccuracy& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    SlopeAccuracy& operator=(const SlopeAccuracy& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(SlopeAccuracy&) operator=(const underlying_type v)
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
} // namespace ap_lodmc

// PRQA S 2180 --

#endif // AP_LODMC_SLOPE_ACCURACY_H_