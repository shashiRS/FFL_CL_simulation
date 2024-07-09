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

#ifndef AP_TP_POSE_TYPE_H_
#define AP_TP_POSE_TYPE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_tp
{
  /// \cond HIDDEN_ENUMS
  ///
  class PoseType
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      T_PARALLEL_PARKING = 0U,
      T_PERP_PARKING_FWD = 1U,
      T_PERP_PARKING_BWD = 2U,
      T_ANGLED_PARKING_STANDARD = 3U,
      T_ANGLED_PARKING_REVERSE = 4U,
      T_REM_MAN_FWD = 5U,
      T_REM_MAN_BWD = 6U,
      T_PERP_PARKING_OUT_FWD = 7U,
      T_PERP_PARKING_OUT_BWD = 8U,
      T_PAR_PARKING_OUT = 9U,
      T_ANGLED_PARKING_STANDARD_OUT = 10U,
      T_ANGLED_PARKING_REVERSE_OUT = 11U,
      T_UNDO = 12U,
      T_GP_FWD = 13U,
      T_GP_BWD = 14U,
      T_GP_OUT_FWD = 15U,
      T_GP_OUT_BWD = 16U,
      T_GP_FWD_AXIS = 17U,
      T_GP_BWD_AXIS = 18U,
      T_GP_OUT_FWD_AXIS = 19U,
      T_GP_OUT_BWD_AXIS = 20U,
      T_UNDEFINED = 21U
    };

    PoseType() :
      value(static_cast< underlying_type >(T_PARALLEL_PARKING))
    {
    }

    PoseType(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    PoseType(const PoseType& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END PoseType(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    PoseType& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PoseType&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    PoseType& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    PoseType& operator=(const PoseType& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PoseType&) operator=(const underlying_type v)
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

#endif // AP_TP_POSE_TYPE_H_