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

#ifndef AP_VEHSTATESIGPROVIDER_ACCSTATUS_H_
#define AP_VEHSTATESIGPROVIDER_ACCSTATUS_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace ap_vehstatesigprovider
{
  /// \cond HIDDEN_ENUMS
  ///Status ACC
  class ACCStatus
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      ACC_OFF = 0U,
      ACC_INIT = 1U,
      ACC_STANDBY = 2U,
      ACC_ACTIVE = 3U,
      ACC_OVERRIDE = 4U,
      ACC_TURN_OFF = 5U,
      ACC_ERROR_REVERSIBLE = 6U,
      ACC_ERROR_IRREVERSIBLE = 7U
    };

    ACCStatus() :
      value(static_cast< underlying_type >(ACC_OFF))
    {
    }

    ACCStatus(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    ACCStatus(const ACCStatus& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END ACCStatus(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    ACCStatus& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(ACCStatus&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    ACCStatus& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    ACCStatus& operator=(const ACCStatus& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(ACCStatus&) operator=(const underlying_type v)
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

#endif // AP_VEHSTATESIGPROVIDER_ACCSTATUS_H_
