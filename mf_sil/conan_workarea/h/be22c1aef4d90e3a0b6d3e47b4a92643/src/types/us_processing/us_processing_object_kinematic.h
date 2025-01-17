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

#ifndef US_PROCESSING_US_PROCESSING_OBJECT_KINEMATIC_H_
#define US_PROCESSING_US_PROCESSING_OBJECT_KINEMATIC_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace us_processing
{
  /// \cond HIDDEN_ENUMS
  class UsProcessingObjectKinematic
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      US_PROCESSING_OBJ_KN_UNKOWN = 0U,
      US_PROCESSING_OBJ_KN_STATIC = 1U,
      US_PROCESSING_OBJ_KN_DYNAMIC = 2U
    };

    UsProcessingObjectKinematic() :
      value(static_cast< underlying_type >(US_PROCESSING_OBJ_KN_UNKOWN))
    {
    }

    UsProcessingObjectKinematic(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    UsProcessingObjectKinematic(const UsProcessingObjectKinematic& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END UsProcessingObjectKinematic(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    UsProcessingObjectKinematic& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsProcessingObjectKinematic&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    UsProcessingObjectKinematic& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    UsProcessingObjectKinematic& operator=(const UsProcessingObjectKinematic& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(UsProcessingObjectKinematic&) operator=(const underlying_type v)
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
} // namespace us_processing

// PRQA S 2180 --

#endif // US_PROCESSING_US_PROCESSING_OBJECT_KINEMATIC_H_
