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

#ifndef SI_PULL_CORNER_POLY_H_
#define SI_PULL_CORNER_POLY_H_

#include "si/quadrilateral_serializable.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Corner pulling polygon.
  struct PullCornerPoly
  {
    ///@brief Rectangular RoI centered around the front left corner of the slot.
    QuadrilateralSerializable fl;
    ///@brief Rectangular RoI centered around the front right corner of the slot.
    QuadrilateralSerializable fr;
    ///@brief RoI for curb alignment.
    QuadrilateralSerializable curb;
    ///@unit{nu}
    ///@brief Defines whether front left RoI is on or not.
    boolean flOn_nu;
    ///@unit{nu}
    ///@brief Axis aligned bounding box around the RoI for curb alignment.
    boolean frOn_nu;
    ///@range{0,3.4028237e+38}
    ///@unit{m}
    ///@brief Distance from the front left RoI.
    float32 flDist_m;
    ///@range{0,3.4028237e+38}
    ///@unit{m}
    ///@brief Distance from the front right RoI.
    float32 frDist_m;
  };

  inline ::si::PullCornerPoly createPullCornerPoly()
  {
    PullCornerPoly m;
    (void)::eco::memset(&m, 0U, sizeof(PullCornerPoly));
    m.fl = createQuadrilateralSerializable();
    m.fr = createQuadrilateralSerializable();
    m.curb = createQuadrilateralSerializable();
    m.flOn_nu = 0;
    m.frOn_nu = 0;
    m.flDist_m = 0.0F;
    m.frDist_m = 0.0F;
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::PullCornerPoly create_default()
  {
      return ::si::createPullCornerPoly();
  }
}


#endif // SI_PULL_CORNER_POLY_H_