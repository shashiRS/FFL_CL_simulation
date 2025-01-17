// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef MF_HMIH_LSCA_WARNINGS_H_
#define MF_HMIH_LSCA_WARNINGS_H_

#include "mf_lsca/lsca_warning_status.h"


namespace mf_hmih
{

  /// LSCA warning information
  struct LscaWarnings
  {
    ///Indicate on what part of the ego body will the collision be.
    ::mf_lsca::LSCA_WARNING_STATUS warningBody_nu{};
    ///Indicate what wheel is affected by a collision with a low object.
    ::mf_lsca::LSCA_WARNING_STATUS warningWheel_nu{};
    ///Indicate where is the object relative to the ego vehicle.
    ::mf_lsca::LSCA_WARNING_STATUS warningObject_nu{};
  };

} // namespace mf_hmih

#endif // MF_HMIH_LSCA_WARNINGS_H_
