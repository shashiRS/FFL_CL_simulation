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

#ifndef MF_HMIH_APUSER_ACTION_HEAD_UNIT_H_
#define MF_HMIH_APUSER_ACTION_HEAD_UNIT_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_hmih
{
  /// \cond HIDDEN_ENUMS
  ///User interaction with HMI handler (Head Unit)
  class APUserActionHeadUnit
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      AP_NO_USER_ACTION = 0U,
      AP_TAP_ON_START_SELECTION = 1U,
      AP_TAP_ON_START_PARKING = 2U,
      AP_TAP_ON_INTERRUPT = 3U,
      AP_TAP_ON_CONTINUE = 4U,
      AP_TAP_ON_UNDO = 5U,
      AP_TAP_ON_CANCEL = 6U,
      AP_TAP_ON_REDO = 7U,
      AP_TAP_ON_START_REMOTE_PARKING = 8U,
      AP_TAP_ON_SWITCH_DIRECTION = 9U,
      AP_TAP_ON_SWITCH_ORIENTATION = 10U,
      AP_TAP_ON_PREVIOUS_SCREEN = 11U,
      AP_TOGGLE_AP_ACTIVE = 12U,
      AP_TAP_ON_FULLY_AUTOMATED_PARKING = 13U,
      AP_TAP_ON_SEMI_AUTOMATED_PARKING = 14U,
      AP_TAP_ON_START_KEY_PARKING = 15U,
      AP_TAP_ON_GP = 16U,
      AP_TAP_ON_RM = 17U,
      AP_TAP_SWITCH_TO_REMOTE_APP = 18U,
      AP_TAP_SWITCH_TO_REMOTE_KEY = 19U,
      AP_TAP_ON_EXPLICIT_SCANNING = 20U,
      AP_TAP_ON_REVERSE_ASSIST = 21U,
      AP_TAP_ON_USER_SLOT_DEFINE = 22U,
      AP_TAP_ON_MEMORY_PARKING = 23U,
      AP_TAP_ON_USER_SLOT_REFINE = 24U,
      AP_TAP_ON_USER_SLOT_SAVE = 25U,
      AP_TAP_ON_USER_SLOT_CLOSE = 26U,
      AP_TAP_ON_USER_SLOT_DELETE = 27U,
      AP_TAP_ON_CROSS = 28U
    };

    APUserActionHeadUnit() :
      value(static_cast< underlying_type >(AP_NO_USER_ACTION))
    {
    }

    APUserActionHeadUnit(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    APUserActionHeadUnit(const APUserActionHeadUnit& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END APUserActionHeadUnit(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    APUserActionHeadUnit& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(APUserActionHeadUnit&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    APUserActionHeadUnit& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    APUserActionHeadUnit& operator=(const APUserActionHeadUnit& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(APUserActionHeadUnit&) operator=(const underlying_type v)
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

#endif // MF_HMIH_APUSER_ACTION_HEAD_UNIT_H_
