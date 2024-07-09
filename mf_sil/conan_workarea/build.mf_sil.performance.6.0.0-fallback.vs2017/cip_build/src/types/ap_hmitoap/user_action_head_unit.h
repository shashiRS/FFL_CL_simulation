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

#ifndef AP_HMITOAP_USER_ACTION_HEAD_UNIT_H_
#define AP_HMITOAP_USER_ACTION_HEAD_UNIT_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_hmitoap
{
  /// \cond HIDDEN_ENUMS
  ///User interaction with HMI (Head Unit)
  ///Gesture suported by HMI.
  ///Types of blindspot detection suported by HMI.
  ///Types of parking augmentation types suported by HMI.
  class UserActionHeadUnit
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      NO_USER_ACTION = 0U,
      TAP_ON_PARKING_SPACE_LEFT_1 = 1U,
      TAP_ON_PARKING_SPACE_LEFT_2 = 2U,
      TAP_ON_PARKING_SPACE_LEFT_3 = 3U,
      TAP_ON_PARKING_SPACE_LEFT_4 = 4U,
      TAP_ON_PARKING_SPACE_RIGHT_1 = 5U,
      TAP_ON_PARKING_SPACE_RIGHT_2 = 6U,
      TAP_ON_PARKING_SPACE_RIGHT_3 = 7U,
      TAP_ON_PARKING_SPACE_RIGHT_4 = 8U,
      TAP_ON_PARKING_SPACE_FRONT_1 = 9U,
      TAP_ON_PARKING_SPACE_FRONT_2 = 10U,
      TAP_ON_PARKING_SPACE_FRONT_3 = 11U,
      TAP_ON_PARKING_SPACE_FRONT_4 = 12U,
      TAP_ON_PARKING_SPACE_REAR_1 = 13U,
      TAP_ON_PARKING_SPACE_REAR_2 = 14U,
      TAP_ON_PARKING_SPACE_REAR_3 = 15U,
      TAP_ON_PARKING_SPACE_REAR_4 = 16U,
      TAP_ON_START_SELECTION = 17U,
      TAP_ON_START_PARKING = 18U,
      TAP_ON_INTERRUPT = 19U,
      TAP_ON_CONTINUE = 20U,
      TAP_ON_UNDO = 21U,
      TAP_ON_CANCEL = 22U,
      TAP_ON_REDO = 23U,
      TAP_ON_START_REMOTE_PARKING = 24U,
      TAP_ON_SWITCH_DIRECTION = 25U,
      TAP_ON_SWITCH_ORIENTATION = 26U,
      TAP_ON_PREVIOUS_SCREEN = 27U,
      TOGGLE_AP_ACTIVE = 28U,
      TAP_ON_LSCA_RELEASE_BRAKE = 29U,
      TAP_ON_FULLY_AUTOMATED_PARKING = 30U,
      TAP_ON_SEMI_AUTOMATED_PARKING = 31U,
      TAP_ON_START_KEY_PARKING = 32U,
      TAP_ON_GP = 33U,
      TAP_ON_RM = 34U,
      TAP_ON_PDC = 35U,
      TAP_ON_AP_PDC_TOGGLE_VIEW = 36U,
      TAP_ON_SWITCH_TO_REMOTE_KEY = 37U,
      TAP_ON_SWITCH_TO_REMOTE_APP = 38U,
      TAP_ON_WHP = 39U,
      TAP_ON_USER_SLOT_LEFT_PAR = 40U,
      TAP_ON_USER_SLOT_LEFT_PERP_BWD = 41U,
      TAP_ON_USER_SLOT_LEFT_PERP_FWD = 42U,
      TAP_ON_USER_SLOT_RIGHT_PAR = 43U,
      TAP_ON_USER_SLOT_RIGHT_PERP_BWD = 44U,
      TAP_ON_USER_SLOT_RIGHT_PERP_FWD = 45U,
      TAP_ON_USER_SLOT_MOVE_UP = 46U,
      TAP_ON_USER_SLOT_MOVE_DOWN = 47U,
      TAP_ON_USER_SLOT_MOVE_LEFT = 48U,
      TAP_ON_USER_SLOT_MOVE_RIGHT = 49U,
      TAP_ON_USER_SLOT_ROT_CLKWISE = 50U,
      TAP_ON_USER_SLOT_ROT_CTRCLKWISE = 51U,
      TAP_ON_USER_SLOT_RESET = 52U,
      TAP_ON_USER_SLOT_SAVE = 53U,
      TAP_ON_EXPLICIT_SCANNING = 54U,
      TAP_ON_REVERSE_ASSIST = 55U,
      TAP_ON_MUTE = 56U,
      TAP_ON_MEMORY_PARKING = 57U,
      TAP_ON_MEMORY_SLOT_1 = 58U,
      TAP_ON_MEMORY_SLOT_2 = 59U,
      TAP_ON_MEMORY_SLOT_3 = 60U,
      TAP_ON_USER_SLOT_REFINE = 61U,
      TAP_ON_USER_SLOT_CLOSE = 62U,
      TAP_ON_USER_SLOT_DEFINE = 63U,
      TAP_ON_LVMD = 64U,
      TAP_ON_LSCA = 65U,
      TAP_ON_USER_SLOT_DELETE = 66U,
      TAP_ON_LVMD_MUTE_AUDIO = 67U,
      TAP_ON_SVS_ACTIVE = 68U,
      TAP_ON_SVS_2D = 69U,
      TAP_ON_SVS_3D = 70U,
      TAP_ON_SVS_KERB_VIEW_FRONT = 71U,
      TAP_ON_SVS_KERB_VIEW_REAR = 72U,
      TAP_ON_SVS_CROSS_BUTTON = 73U
    };

    UserActionHeadUnit() :
      value(static_cast< underlying_type >(NO_USER_ACTION))
    {
    }

    UserActionHeadUnit(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UserActionHeadUnit(const UserActionHeadUnit& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UserActionHeadUnit(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UserActionHeadUnit& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UserActionHeadUnit&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UserActionHeadUnit& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UserActionHeadUnit& operator=(const UserActionHeadUnit& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UserActionHeadUnit&) operator=(const underlying_type v)
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

#endif // AP_HMITOAP_USER_ACTION_HEAD_UNIT_H_
