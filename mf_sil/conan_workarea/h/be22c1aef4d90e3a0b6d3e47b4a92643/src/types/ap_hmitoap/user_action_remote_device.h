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

#ifndef AP_HMITOAP_USER_ACTION_REMOTE_DEVICE_H_
#define AP_HMITOAP_USER_ACTION_REMOTE_DEVICE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_hmitoap
{
  /// \cond HIDDEN_ENUMS
  ///    User interaction with HMI (Remote Parking App)
  ///
  ///27: Switches to a screen, where the user can see the surround-view view / us-distance view.
  ///28 / 29: Forward / Backward-Button for Remote Maneuvering
  class UserActionRemoteDevice
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      REM_NO_USER_ACTION = 0U,
      REM_TAP_ON_PARKING_SPACE_1 = 1U,
      REM_TAP_ON_PARKING_SPACE_2 = 2U,
      REM_TAP_ON_PARKING_SPACE_3 = 3U,
      REM_TAP_ON_PARKING_SPACE_4 = 4U,
      REM_APP_STARTED = 16U,
      REM_APP_CLOSED = 17U,
      REM_TAP_ON_START_PARKING = 18U,
      REM_TAP_ON_INTERRUPT = 19U,
      REM_TAP_ON_CONTINUE = 20U,
      REM_TAP_ON_UNDO = 21U,
      REM_TAP_ON_CANCEL = 22U,
      REM_TAP_ON_REDO = 23U,
      REM_TAP_ON_PARK_IN = 24U,
      REM_TAP_ON_PARK_OUT = 25U,
      REM_TAP_ON_REM_MAN = 26U,
      REM_TAP_ON_REM_SV = 27U,
      REM_TAP_ON_REM_FWD = 28U,
      REM_TAP_ON_REM_BWD = 29U,
      REM_TAP_ON_PREVIOUS_SCREEN = 30U,
      REM_TAP_ON_GP = 31U,
      REM_TAP_ON_SWITCH_TO_HEAD_UNIT = 32U
    };

    UserActionRemoteDevice() :
      value(static_cast< underlying_type >(REM_NO_USER_ACTION))
    {
    }

    UserActionRemoteDevice(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UserActionRemoteDevice(const UserActionRemoteDevice& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UserActionRemoteDevice(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UserActionRemoteDevice& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UserActionRemoteDevice&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UserActionRemoteDevice& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UserActionRemoteDevice& operator=(const UserActionRemoteDevice& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UserActionRemoteDevice&) operator=(const underlying_type v)
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

#endif // AP_HMITOAP_USER_ACTION_REMOTE_DEVICE_H_
