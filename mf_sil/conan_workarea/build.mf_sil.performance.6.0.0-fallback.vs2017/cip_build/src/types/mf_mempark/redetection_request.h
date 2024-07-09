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

#ifndef MF_MEMPARK_REDETECTION_REQUEST_H_
#define MF_MEMPARK_REDETECTION_REQUEST_H_

#include "Platform_Types.h"
#include "lsm_geoml/pose_pod.h"
#include "eco/memset.h"


namespace mf_mempark
{

  /// Elements of the localization request slots
  struct RedetectionRequest
  {
    uint8 mapID;
    ::lsm_geoml::Pose_POD startPose;
    boolean initialLocalizationRequest;
  };

  inline ::mf_mempark::RedetectionRequest createRedetectionRequest()
  {
    RedetectionRequest m;
    (void)::eco::memset(&m, 0U, sizeof(RedetectionRequest));
    m.mapID = 255U;
    m.startPose = ::lsm_geoml::createPose_POD();
    return m;
  }

} // namespace mf_mempark

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_mempark::RedetectionRequest create_default()
  {
      return ::mf_mempark::createRedetectionRequest();
  }
}


#endif // MF_MEMPARK_REDETECTION_REQUEST_H_