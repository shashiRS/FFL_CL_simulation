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

#ifndef MF_HMIH_PDCSECTOR_INFO_H_
#define MF_HMIH_PDCSECTOR_INFO_H_

#include "Platform_Types.h"
#include "pdcp/criticality_level.h"
#include "eco/memset.h"


namespace mf_hmih
{

  /// Information for the PDC sectors from the left side (numbering is done from front towards rear)
  struct PDCSectorInfo
  {
    ///Distance between the car contour and the closest obstacle for this sector
    float32 smallestDistance_m;
    ///@range{0,3}
    ///Criticality level for this sector (the higher the criticality, the closer the obstacle is to the car).
    ::pdcp::CriticalityLevel criticalityLevel_nu;
    ///@range{0,3}
    ///Slice number within the criticality level (numbering is done from the outside of the car towards the car contour).
    uint8 slice_nu;
    ///Sector ID (unique over all sectors)
    uint8 sectorID_nu;
    ///Indicates if the obstacle that determines the criticality of this sector intersects the driving tube
    boolean intersectsDrvTube_nu;
    ///Indicates if this sector was scanned (scanned means that PDCP had valid information to calculate the smallestDistance_nu).
    boolean scanned_nu;
  };

  inline ::mf_hmih::PDCSectorInfo createPDCSectorInfo()
  {
    PDCSectorInfo m;
    (void)::eco::memset(&m, 0U, sizeof(PDCSectorInfo));
    return m;
  }

} // namespace mf_hmih

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_hmih::PDCSectorInfo create_default()
  {
      return ::mf_hmih::createPDCSectorInfo();
  }
}


#endif // MF_HMIH_PDCSECTOR_INFO_H_
