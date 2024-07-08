// Attention, this file is generated by Cobolt from template: C:\_repos\mf_sil\dbg\eco\eco.generic\codegen\templates\types\struct.h.template!

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

#ifndef US_DRV_US_DRV_PDCM_FRAME_FORMAT_ERRORS_H_
#define US_DRV_US_DRV_PDCM_FRAME_FORMAT_ERRORS_H_

#include "us_drv/us_drv_error_status.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvPdcmFrameFormatErrors
  {
    UsDrvErrorStatus noNewData;
    UsDrvErrorStatus dataBufferOvr;
    UsDrvErrorStatus dataLoss;
    UsDrvErrorStatus packetCountErr;
  };

  inline ::us_drv::UsDrvPdcmFrameFormatErrors createUsDrvPdcmFrameFormatErrors()
  {
    UsDrvPdcmFrameFormatErrors m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvPdcmFrameFormatErrors));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvPdcmFrameFormatErrors create_default()
  {
      return ::us_drv::createUsDrvPdcmFrameFormatErrors();
  }
}


#endif // US_DRV_US_DRV_PDCM_FRAME_FORMAT_ERRORS_H_