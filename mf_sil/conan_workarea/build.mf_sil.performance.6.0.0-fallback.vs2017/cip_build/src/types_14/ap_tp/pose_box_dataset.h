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

#ifndef AP_TP_POSE_BOX_DATASET_H_
#define AP_TP_POSE_BOX_DATASET_H_

#include "ap_tp/pose_box_data.h"


namespace ap_tp
{

  /// 
  struct PoseBoxDataset
  {
    ///@unit{nu}
    ///Linking of Target Pose to Parking Box @min: 0 @max: 0 @unit: nu
    PoseBoxData data[8]{};
  };

} // namespace ap_tp

#endif // AP_TP_POSE_BOX_DATASET_H_
