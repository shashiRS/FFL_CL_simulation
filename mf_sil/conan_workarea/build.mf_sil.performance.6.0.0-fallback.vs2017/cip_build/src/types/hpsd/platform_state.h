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

#ifndef HPSD_PLATFORM_STATE_H_
#define HPSD_PLATFORM_STATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace hpsd
{
  /// \cond HIDDEN_ENUMS
  ///An enumeration describing the possible states of the platform running the application
  class PlatformState
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      ///Platform (ADCU) during startup. It is not possible to run the application yet.
      STARTUP = 0U,
      ///The platform can already support fastPDW functionality. IU, SPU and CVU are operational.
      FAST_PDW = 1U,
      ///All the cores have been started.
      NORMAL_OPERATION = 2U,
      ///The system has initiated shutdown.
      SHUTDOWN = 3U
    };

    PlatformState() :
      value(static_cast< underlying_type >(STARTUP))
    {
    }

    PlatformState(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    PlatformState(const PlatformState& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END PlatformState(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    PlatformState& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PlatformState&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    PlatformState& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    PlatformState& operator=(const PlatformState& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(PlatformState&) operator=(const underlying_type v)
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
} // namespace hpsd

// PRQA S 2180 --

#endif // HPSD_PLATFORM_STATE_H_