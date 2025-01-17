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

#ifndef PDCP_DRV_TUBE_DISPLAY_H_
#define PDCP_DRV_TUBE_DISPLAY_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace pdcp
{
  /// \cond HIDDEN_ENUMS
  ///Information of where the driving tube should be displayed
  class DrvTubeDisplay
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      PDC_DRV_TUBE_NONE = 0U,
      PDC_DRV_TUBE_FRONT = 1U,
      PDC_DRV_TUBE_REAR = 2U
    };

    DrvTubeDisplay() :
      value(static_cast< underlying_type >(PDC_DRV_TUBE_NONE))
    {
    }

    DrvTubeDisplay(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DrvTubeDisplay(const DrvTubeDisplay& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END DrvTubeDisplay(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    DrvTubeDisplay& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(DrvTubeDisplay&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    DrvTubeDisplay& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DrvTubeDisplay& operator=(const DrvTubeDisplay& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(DrvTubeDisplay&) operator=(const underlying_type v)
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
} // namespace pdcp

// PRQA S 2180 --

#endif // PDCP_DRV_TUBE_DISPLAY_H_
