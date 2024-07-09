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

#ifndef MF_HMIH_HU_FRAME_READY_H_
#define MF_HMIH_HU_FRAME_READY_H_

#include "Platform_Types.h"
#include "eco/deprecation.h"

// PRQA S 2180 ++
/* date: 2020-03-31, reviewer: PR reviewers of https://github-am.geo.conti.de/ADAS/eco/pull/1100, CCBIssueId: https://jira-adas.zone2.agileci.conti.de/browse/SEP-1137, reason: implicit construction is desired here */

namespace mf_hmih
{
  /// \cond HIDDEN_ENUMS
  ///Indicates the readiness of the frames
  class HuFrameReady
  {
  public:
    typedef uint8 underlying_type;
#ifdef _MSC_VER
    enum do_not_use : underlying_type
#else
    enum do_not_use
#endif
    {
      NO_USED = 0U,
      IN_TRANSITION = 1U,
      FRAME_ERROR = 2U,
      EARLY_RVC_READY = 3U,
      APA_READY = 4U,
      PARK_IN_READY = 5U,
      PARK_OUT_READY = 6U,
      GARAGE_PARKING_READY = 7U,
      REVERSE_ASSIST_READY = 8U,
      RCTA_READY = 9U,
      BVM_READY = 10U,
      TWO_D_VIEW_READY = 11U,
      THREE_D_VIEW_READY = 12U,
      FRONT_CORNER_VIEW_READY = 13U,
      REAR_CORNER_VIEW_READY = 14U
    };

    HuFrameReady() :
      value(static_cast< underlying_type >(NO_USED))
    {
    }

    HuFrameReady(const do_not_use v) :
      value(static_cast<underlying_type >(v))
    {
    }

    HuFrameReady(const HuFrameReady& v) :
      value(static_cast<underlying_type >(v))
    {
    }

    DEPRECATED_BEGIN DEPRECATED_END HuFrameReady(const underlying_type v) :
      value(v)
    {
    }

    operator underlying_type() const
    {
      return value;
    }

    HuFrameReady& operator|=(const do_not_use v)
    {
      value |= static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(HuFrameReady&) operator|=(const underlying_type v)
    {
      value |= v;
      return *this;
    }

    HuFrameReady& operator=(const do_not_use v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    HuFrameReady& operator=(const HuFrameReady& v)
    {
      value = static_cast< underlying_type >(v);
      return *this;
    }

    DEPRECATED(HuFrameReady&) operator=(const underlying_type v)
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
} // namespace mf_hmih

// PRQA S 2180 --

#endif // MF_HMIH_HU_FRAME_READY_H_
