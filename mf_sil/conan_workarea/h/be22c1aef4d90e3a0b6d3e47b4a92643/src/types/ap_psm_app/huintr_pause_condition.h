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

#ifndef AP_PSM_APP_HUINTR_PAUSE_CONDITION_H_
#define AP_PSM_APP_HUINTR_PAUSE_CONDITION_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_psm_app
{
  /// \cond HIDDEN_ENUMS
  ///This signal Indicates the Pause condition messages
  class HUIntrPauseCondition
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      NO_MSG = 0U,
      DRIVER_DOOR_NOT_CLOSED = 1U,
      CO_DRIVER_DOOR_NOT_CLOSED = 2U,
      DYN_STAT_OBJ_TRAJ = 3U,
      SEAT_BELT_OPEN = 4U,
      ORVM_CLOSE = 5U,
      BRAKE_PRESS_TH = 6U,
      APA_HOLD_RELEASED = 7U,
      REAR_PSG_DOOR_NOT_CLOSED = 8U,
      TRUNK_LID_NOT_CLOSED = 9U,
      TAIL_GATE_NOT_CLOSED = 10U
    };

    HUIntrPauseCondition() :
      value(static_cast< underlying_type >(NO_MSG))
    {
    }

    HUIntrPauseCondition(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    HUIntrPauseCondition(const HUIntrPauseCondition& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END HUIntrPauseCondition(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    HUIntrPauseCondition& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(HUIntrPauseCondition&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    HUIntrPauseCondition& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    HUIntrPauseCondition& operator=(const HUIntrPauseCondition& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(HUIntrPauseCondition&) operator=(const underlying_type v)
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
} // namespace ap_psm_app

// PRQA S 2180 --

#endif // AP_PSM_APP_HUINTR_PAUSE_CONDITION_H_
