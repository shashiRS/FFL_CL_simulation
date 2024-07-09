// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

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

#ifndef SI_DIAGNOSTIC_EVENT_SERIALIZABLE_H_
#define SI_DIAGNOSTIC_EVENT_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "si/diagnostic_event.h"
#include "eco/memset.h"


namespace si
{

  struct DiagnosticEventSerializable
  {
    ///@unit{nu}
    ///@range{0,SI.Constants.MAX_NUM_OF_DIAGNOSTIC_EVENT_IDS}
    ::lsm_geoml::size_type actualSize;
    ///@unit{nu}
    ///Array holding diagnostic events
    DiagnosticEvent array[4];
  };

  inline ::si::DiagnosticEventSerializable createDiagnosticEventSerializable()
  {
    DiagnosticEventSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(DiagnosticEventSerializable));
    {
      const uint64 arraysize = (sizeof(m.array) / sizeof(m.array[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.array[i] = createDiagnosticEvent();
      }
    }
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::DiagnosticEventSerializable create_default()
  {
      return ::si::createDiagnosticEventSerializable();
  }
}


#endif // SI_DIAGNOSTIC_EVENT_SERIALIZABLE_H_