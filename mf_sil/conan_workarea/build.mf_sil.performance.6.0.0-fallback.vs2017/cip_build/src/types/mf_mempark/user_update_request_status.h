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

#ifndef MF_MEMPARK_USER_UPDATE_REQUEST_STATUS_H_
#define MF_MEMPARK_USER_UPDATE_REQUEST_STATUS_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_mempark
{
  /// \cond HIDDEN_ENUMS
  ///Current status of the slot update request from user. It indicates
  ///if the update request was successful and if the last request
  ///pushed the update to maximum allowed limits.
  class UserUpdateRequestStatus
  {
  public:
    typedef uint16 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      UPDATE_STATUS_INIT = 0U,
      UPDATE_STATUS_SUCCESS = 1U,
      UPDATE_STATUS_SUCCESS_MAX_C_ANGLE = 2U,
      UPDATE_STATUS_SUCCESS_MAX_CC_ANGLE = 3U,
      UPDATE_STATUS_SUCCESS_MAX_LONG_POS_UP = 4U,
      UPDATE_STATUS_SUCCESS_MAX_LONG_POS_DOWN = 5U,
      UPDATE_STATUS_SUCCESS_MAX_LAT_POS_LEFT = 6U,
      UPDATE_STATUS_SUCCESS_MAX_LAT_POS_RIGHT = 7U,
      UPDATE_STATUS_SUCCESS_MAX_ATTEMPTS = 8U,
      UPDATE_STATUS_FAILED = 9U
    };

    UserUpdateRequestStatus() :
      value(static_cast< underlying_type >(UPDATE_STATUS_INIT))
    {
    }

    UserUpdateRequestStatus(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UserUpdateRequestStatus(const UserUpdateRequestStatus& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UserUpdateRequestStatus(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UserUpdateRequestStatus& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UserUpdateRequestStatus&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UserUpdateRequestStatus& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UserUpdateRequestStatus& operator=(const UserUpdateRequestStatus& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UserUpdateRequestStatus&) operator=(const underlying_type v)
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

#endif // MF_MEMPARK_USER_UPDATE_REQUEST_STATUS_H_