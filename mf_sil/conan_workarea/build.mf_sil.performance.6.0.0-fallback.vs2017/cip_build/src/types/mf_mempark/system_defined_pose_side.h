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

#ifndef MF_MEMPARK_SYSTEM_DEFINED_POSE_SIDE_H_
#define MF_MEMPARK_SYSTEM_DEFINED_POSE_SIDE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_mempark
{
  /// \cond HIDDEN_ENUMS
  ///Pose side of the target pose provided by the system
  class SystemDefinedPoseSide
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      SYSTEM_DEF_POSE_CURRENT_EGO = 0U,
      SYSTEM_DEF_POSE_RIGHT = 1U,
      SYSTEM_DEF_POSE_LEFT = 2U,
      SYSTEM_DEF_POSE_FRONT = 3U,
      SYSTEM_DEF_POSE_REAR = 4U,
      MAX_NUM_SYSTEM_DEF_POSE_SIDES = 5U
    };

    SystemDefinedPoseSide() :
      value(static_cast< underlying_type >(SYSTEM_DEF_POSE_CURRENT_EGO))
    {
    }

    SystemDefinedPoseSide(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    SystemDefinedPoseSide(const SystemDefinedPoseSide& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END SystemDefinedPoseSide(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    SystemDefinedPoseSide& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(SystemDefinedPoseSide&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    SystemDefinedPoseSide& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    SystemDefinedPoseSide& operator=(const SystemDefinedPoseSide& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(SystemDefinedPoseSide&) operator=(const underlying_type v)
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
} // namespace mf_mempark

// PRQA S 2180 --

#endif // MF_MEMPARK_SYSTEM_DEFINED_POSE_SIDE_H_
