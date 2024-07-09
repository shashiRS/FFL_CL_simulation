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

#ifndef AP_HMITOAP_VISU_INPUT_DATA_H_
#define AP_HMITOAP_VISU_INPUT_DATA_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_hmitoap/screen_types.h"
#include "ap_hmitoap/gesture_code.h"
#include "ap_hmitoap/blind_spot_view.h"
#include "ap_hmitoap/parking_augmentation_type.h"
#include "eco/memset.h"


namespace ap_hmitoap
{

  struct VisuInputData
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    sint16 firstClickEventX_px_u16;
    sint16 firstClickEventY_px_u16;
    sint16 secondClickEventX_px_u16;
    sint16 secondClickEventY_px_u16;
    ScreenTypes HmiOutUserActScreenReq_u8;
    GestureCode GestureCode_nu_u8;
    BlindSpotView blindSpotActivated_nu;
    ParkingAugmentationType parkingAugmentationType_nu;
    uint8 GestureFinger_nu_u8;
    uint8 gestureCounter_nu;
    uint8 visibilityTags_nu;
    boolean videoRecorderRequest_nu;
    boolean screenCaptureRequest_nu;
    boolean isSequence_nu;
  };

  inline ::ap_hmitoap::VisuInputData createVisuInputData()
  {
    VisuInputData m;
    (void)::eco::memset(&m, 0U, sizeof(VisuInputData));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace ap_hmitoap

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_hmitoap::VisuInputData create_default()
  {
      return ::ap_hmitoap::createVisuInputData();
  }
}


#endif // AP_HMITOAP_VISU_INPUT_DATA_H_
