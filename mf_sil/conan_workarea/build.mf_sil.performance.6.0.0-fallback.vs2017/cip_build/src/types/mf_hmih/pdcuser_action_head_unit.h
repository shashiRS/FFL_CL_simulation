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

#ifndef MF_HMIH_PDCUSER_ACTION_HEAD_UNIT_H_
#define MF_HMIH_PDCUSER_ACTION_HEAD_UNIT_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_hmih
{
  /// \cond HIDDEN_ENUMS
  ///User interaction with HMI handler (Head Unit) regarding PDW
  class PDCUserActionHeadUnit
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      PDC_NO_USER_ACTION = 0U,
      PDC_TAP_ON_ACT_DEACT_BTN = 1U,
      PDC_TAP_ON_MUTE_BTN = 2U
    };

    PDCUserActionHeadUnit() :
      value(static_cast< underlying_type >(PDC_NO_USER_ACTION))
    {
    }

    PDCUserActionHeadUnit(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    PDCUserActionHeadUnit(const PDCUserActionHeadUnit& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END PDCUserActionHeadUnit(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    PDCUserActionHeadUnit& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PDCUserActionHeadUnit&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    PDCUserActionHeadUnit& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    PDCUserActionHeadUnit& operator=(const PDCUserActionHeadUnit& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PDCUserActionHeadUnit&) operator=(const underlying_type v)
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

#endif // MF_HMIH_PDCUSER_ACTION_HEAD_UNIT_H_