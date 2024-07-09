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

#ifndef AP_VEHSTATESIGPROVIDER_KEY_FOB_USER_ACTION_H_
#define AP_VEHSTATESIGPROVIDER_KEY_FOB_USER_ACTION_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_vehstatesigprovider
{
  /// \cond HIDDEN_ENUMS
  ///User action from the key fob
  class KeyFobUserAction
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      AP_KEY_NO_USER_ACTION = 0U,
      AP_KEY_TAP_ON_START_PARKING = 1U,
      AP_KEY_TAP_ON_INTERRUPT = 2U,
      AP_KEY_TAP_ON_CONTINUE = 3U,
      AP_KEY_TAP_ON_UNDO = 4U,
      AP_KEY_TAP_ON_CANCEL = 5U,
      AP_KEY_TAP_ON_REDO = 6U,
      AP_KEY_FOB_MANEUVER_AUTHORIZED = 7U,
      AP_KEY_FOB_MANEUVER_ABORT = 8U,
      AP_KEY_FOB_MANEUVER_PAUSED = 9U,
      AP_KEY_FOB_ACTIVATED = 10U
    };

    KeyFobUserAction() :
      value(static_cast< underlying_type >(AP_KEY_NO_USER_ACTION))
    {
    }

    KeyFobUserAction(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    KeyFobUserAction(const KeyFobUserAction& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END KeyFobUserAction(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    KeyFobUserAction& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(KeyFobUserAction&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    KeyFobUserAction& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    KeyFobUserAction& operator=(const KeyFobUserAction& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(KeyFobUserAction&) operator=(const underlying_type v)
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
} // namespace ap_vehstatesigprovider

// PRQA S 2180 --

#endif // AP_VEHSTATESIGPROVIDER_KEY_FOB_USER_ACTION_H_