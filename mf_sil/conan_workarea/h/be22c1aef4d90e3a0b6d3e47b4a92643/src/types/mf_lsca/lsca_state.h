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

#ifndef MF_LSCA_LSCA_STATE_H_
#define MF_LSCA_LSCA_STATE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_lsca
{
  /// \cond HIDDEN_ENUMS
  class LSCA_STATE
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      LSCA_STATE_DEACTIVATED = 0U,
      LSCA_STATE_ACTIVATED = 1U,
      LSCA_STATE_INTERVENTION = 2U,
      LSCA_STATE_ERROR = 3U
    };

    LSCA_STATE() :
      value(static_cast< underlying_type >(LSCA_STATE_DEACTIVATED))
    {
    }

    LSCA_STATE(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    LSCA_STATE(const LSCA_STATE& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END LSCA_STATE(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    LSCA_STATE& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(LSCA_STATE&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    LSCA_STATE& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    LSCA_STATE& operator=(const LSCA_STATE& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(LSCA_STATE&) operator=(const underlying_type v)
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
} // namespace mf_lsca

// PRQA S 2180 --

#endif // MF_LSCA_LSCA_STATE_H_
