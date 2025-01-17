//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\enum.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef US_DRV_US_DRV_DETECTION_TYPE_C_H_
#define US_DRV_US_DRV_DETECTION_TYPE_C_H_

#include "Platform_Types.h"

typedef uint8 US_DRV_UsDrvDetectionType;

///Bits 0..3 of this enum specify confidence level of detection
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_CONFIDENCE_MASK 15U
///Detection on standard path
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_NORMAL_PATH 0U
///Detection on advanced path 1 (chirp up)
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_ADVANCED_PATH_1 16U
///Detection on advanced path 2 (chirp down)
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_ADVANCED_PATH_2 32U
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_NFD_PATH 48U
///Detected AATG threshold from first channel
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_AATG_THRESHOLD_1 64U
///Detected AATG threshold from second channel
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_AATG_THRESHOLD_2 80U
///Sensor timestamp of firing event
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_FIRING_TIMESTAMP 128U
///Bitmask for detection type
#define US_DRV_US_DRV_DETECTION_TYPE_US_DETECTION_TYPE_MASK 240U


#endif // US_DRV_US_DRV_DETECTION_TYPE_C_H_
