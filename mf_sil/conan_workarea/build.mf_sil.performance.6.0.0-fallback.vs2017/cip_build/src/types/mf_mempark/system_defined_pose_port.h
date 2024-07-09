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

#ifndef MF_MEMPARK_SYSTEM_DEFINED_POSE_PORT_H_
#define MF_MEMPARK_SYSTEM_DEFINED_POSE_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "lsm_geoml/pose_pod.h"
#include "mf_mempark/system_defined_pose_side.h"
#include "mf_mempark/system_defined_pose_type.h"
#include "eco/memset.h"


namespace mf_mempark
{

  /// Port containing the information of a pose with was saved by the system. This information can be used to park to this pose using the AUP stack
  struct SystemDefinedPosePort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    boolean valid;
    uint8 targetPoseID;
    ::lsm_geoml::Pose_POD currentEgoInReference;
    ::lsm_geoml::Pose_POD targetPoseInReference;
    float32 curvature_1pm;
    SystemDefinedPoseSide side;
    SystemDefinedPoseType type;
  };

  inline ::mf_mempark::SystemDefinedPosePort createSystemDefinedPosePort()
  {
    SystemDefinedPosePort m;
    (void)::eco::memset(&m, 0U, sizeof(SystemDefinedPosePort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.currentEgoInReference = ::lsm_geoml::createPose_POD();
    m.targetPoseInReference = ::lsm_geoml::createPose_POD();
    return m;
  }

} // namespace mf_mempark

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_mempark::SystemDefinedPosePort create_default()
  {
      return ::mf_mempark::createSystemDefinedPosePort();
  }
}


#endif // MF_MEMPARK_SYSTEM_DEFINED_POSE_PORT_H_