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

#ifndef MF_MEMPARK_LOCALIZATION_STATUS_H_
#define MF_MEMPARK_LOCALIZATION_STATUS_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_mempark
{
  /// \cond HIDDEN_ENUMS
  ///Current status of the localization request if the memory parking
  ///is ready to offer slot or path or reason for failure if it
  ///cannot localize path or slot.
  class LocalizationStatus
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      LOCALIZE_STATUS_INIT = 0U,
      LOCALIZE_STATUS_UNAVAIL_OUTSIDE_ZONE = 1U,
      LOCALIZE_STATUS_ACTIVE = 2U,
      LOCALIZE_STATUS_SUCCESS_OFFER = 3U,
      LOCALIZE_STATUS_SUCCESS_NO_OFFER = 4U,
      LOCALIZE_STATUS_FAILED = 5U
    };

    LocalizationStatus() :
      value(static_cast< underlying_type >(LOCALIZE_STATUS_INIT))
    {
    }

    LocalizationStatus(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    LocalizationStatus(const LocalizationStatus& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END LocalizationStatus(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    LocalizationStatus& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(LocalizationStatus&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    LocalizationStatus& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    LocalizationStatus& operator=(const LocalizationStatus& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(LocalizationStatus&) operator=(const underlying_type v)
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

#endif // MF_MEMPARK_LOCALIZATION_STATUS_H_
