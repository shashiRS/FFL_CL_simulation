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

#ifndef AP_TP_POS_DEF_APPROACH_H_
#define AP_TP_POS_DEF_APPROACH_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_tp
{
  /// \cond HIDDEN_ENUMS
  ///
  class PosDefApproach
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      ATTACH_EGO_TO_EDGE = 0U,
      CENTERING = 1U,
      ROAD_SIDE_CENTERING = 2U,
      SHORT_SIDE_CENTERING = 3U
    };

    PosDefApproach() :
      value(static_cast< underlying_type >(ATTACH_EGO_TO_EDGE))
    {
    }

    PosDefApproach(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    PosDefApproach(const PosDefApproach& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END PosDefApproach(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    PosDefApproach& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PosDefApproach&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    PosDefApproach& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    PosDefApproach& operator=(const PosDefApproach& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PosDefApproach&) operator=(const underlying_type v)
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

#endif // AP_TP_POS_DEF_APPROACH_H_
