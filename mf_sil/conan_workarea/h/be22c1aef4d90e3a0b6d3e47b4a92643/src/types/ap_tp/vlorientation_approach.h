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

#ifndef AP_TP_VLORIENTATION_APPROACH_H_
#define AP_TP_VLORIENTATION_APPROACH_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_tp
{
  /// \cond HIDDEN_ENUMS
  ///
  class VLOrientationApproach
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      FRONT_VL_ORIENTATION = 0U,
      REAR_VL_ORIENTATION = 1U,
      VL_ORIENTATION_AVERAGE = 2U,
      USE_BOTH_VL_PTS_NEAR_SLOT = 3U,
      MAX_NUM_VL_ORIENT_APPROACHES = 4U
    };

    VLOrientationApproach() :
      value(static_cast< underlying_type >(FRONT_VL_ORIENTATION))
    {
    }

    VLOrientationApproach(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    VLOrientationApproach(const VLOrientationApproach& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END VLOrientationApproach(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    VLOrientationApproach& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(VLOrientationApproach&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    VLOrientationApproach& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    VLOrientationApproach& operator=(const VLOrientationApproach& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(VLOrientationApproach&) operator=(const underlying_type v)
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
} // namespace ap_tp

// PRQA S 2180 --

#endif // AP_TP_VLORIENTATION_APPROACH_H_
