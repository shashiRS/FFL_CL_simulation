//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

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

#ifndef AP_HMITOAP_VISU_INPUT_DATA_C_H_
#define AP_HMITOAP_VISU_INPUT_DATA_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_hmitoap/screen_types_c.h"
#include "ap_hmitoap/gesture_code_c.h"
#include "ap_hmitoap/blind_spot_view_c.h"
#include "ap_hmitoap/parking_augmentation_type_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    sint16 firstClickEventX_px_u16;
    sint16 firstClickEventY_px_u16;
    sint16 secondClickEventX_px_u16;
    sint16 secondClickEventY_px_u16;
    AP_HMITOAP_ScreenTypes HmiOutUserActScreenReq_u8;
    AP_HMITOAP_GestureCode GestureCode_nu_u8;
    AP_HMITOAP_BlindSpotView blindSpotActivated_nu;
    AP_HMITOAP_ParkingAugmentationType parkingAugmentationType_nu;
    uint8 GestureFinger_nu_u8;
    uint8 gestureCounter_nu;
    uint8 visibilityTags_nu;
    boolean videoRecorderRequest_nu;
    boolean screenCaptureRequest_nu;
    boolean isSequence_nu;
} AP_HMITOAP_VisuInputData;

inline AP_HMITOAP_VisuInputData create_AP_HMITOAP_VisuInputData(void)
{
  AP_HMITOAP_VisuInputData m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_HMITOAP_VISU_INPUT_DATA_C_H_
