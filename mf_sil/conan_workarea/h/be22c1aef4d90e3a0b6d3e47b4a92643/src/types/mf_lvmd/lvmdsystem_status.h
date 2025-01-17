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

#ifndef MF_LVMD_LVMDSYSTEM_STATUS_H_
#define MF_LVMD_LVMDSYSTEM_STATUS_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_lvmd
{
  /// \cond HIDDEN_ENUMS
  ///lvmd system status
  class LVMDSystemStatus
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      LVMD_Status_OFF = 0U,
      LVMD_Status_IDLE = 1U,
      LVMD_Status_INACTIVE = 2U,
      LVMD_Status_ACTIVE = 3U,
      LVMD_Status_FAILURE = 4U
    };

    LVMDSystemStatus() :
      value(static_cast< underlying_type >(LVMD_Status_OFF))
    {
    }

    LVMDSystemStatus(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    LVMDSystemStatus(const LVMDSystemStatus& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END LVMDSystemStatus(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    LVMDSystemStatus& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(LVMDSystemStatus&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    LVMDSystemStatus& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    LVMDSystemStatus& operator=(const LVMDSystemStatus& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(LVMDSystemStatus&) operator=(const underlying_type v)
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
} // namespace mf_lvmd

// PRQA S 2180 --

#endif // MF_LVMD_LVMDSYSTEM_STATUS_H_
