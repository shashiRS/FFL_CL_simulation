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

#ifndef US_EM_STATIC_OBJ_HEIGHT_TYPE_H_
#define US_EM_STATIC_OBJ_HEIGHT_TYPE_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace us_em
{
  /// \cond HIDDEN_ENUMS
  ///Height class of the object
  class StaticObjHeightType
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      SO_HI_UNKNOWN = 0U,
      SO_HI_WHEEL_TRAVERSABLE = 1U,
      SO_HI_BODY_TRAVERSABLE = 2U,
      SO_HI_DOOR_OPENABLE = 3U,
      SO_HI_HIGH_OBSTACLE = 4U,
      SO_HI_HANGING_OBJECT = 5U,
      SO_HI_LOWER_BUMPER_HEIGHT = 6U,
      MAX_NUM_HEIGHT_TYPES = 7U
    };

    StaticObjHeightType() :
      value(static_cast< underlying_type >(SO_HI_UNKNOWN))
    {
    }

    StaticObjHeightType(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    StaticObjHeightType(const StaticObjHeightType& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END StaticObjHeightType(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    StaticObjHeightType& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(StaticObjHeightType&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    StaticObjHeightType& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    StaticObjHeightType& operator=(const StaticObjHeightType& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(StaticObjHeightType&) operator=(const underlying_type v)
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
} // namespace us_em

// PRQA S 2180 --

#endif // US_EM_STATIC_OBJ_HEIGHT_TYPE_H_
