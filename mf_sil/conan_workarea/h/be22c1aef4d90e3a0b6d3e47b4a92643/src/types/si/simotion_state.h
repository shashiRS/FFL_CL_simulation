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

#ifndef SI_SIMOTION_STATE_H_
#define SI_SIMOTION_STATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace si
{
  /// \cond HIDDEN_ENUMS
  ///motion state of the vehicle
  class SIMotionState
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      SI_ODO_STANDSTILL = 0U,
      SI_ODO_NO_STANDSTILL = 1U
    };

    SIMotionState() :
      value(static_cast< underlying_type >(SI_ODO_STANDSTILL))
    {
    }

    SIMotionState(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    SIMotionState(const SIMotionState& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END SIMotionState(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    SIMotionState& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(SIMotionState&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    SIMotionState& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    SIMotionState& operator=(const SIMotionState& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(SIMotionState&) operator=(const underlying_type v)
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
} // namespace si

// PRQA S 2180 --

#endif // SI_SIMOTION_STATE_H_
