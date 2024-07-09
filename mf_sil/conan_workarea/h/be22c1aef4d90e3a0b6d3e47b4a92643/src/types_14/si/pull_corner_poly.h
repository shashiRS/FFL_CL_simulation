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

#ifndef SI_PULL_CORNER_POLY_H_
#define SI_PULL_CORNER_POLY_H_

#include "si/quadrilateral_serializable.h"
#include "Platform_Types.h"


namespace si
{

  /// @brief Corner pulling polygon.
  struct PullCornerPoly
  {
    ///@brief Rectangular RoI centered around the front left corner of the slot.
    QuadrilateralSerializable fl{};
    ///@brief Rectangular RoI centered around the front right corner of the slot.
    QuadrilateralSerializable fr{};
    ///@brief RoI for curb alignment.
    QuadrilateralSerializable curb{};
    ///@unit{nu}
    ///@brief Defines whether front left RoI is on or not.
    boolean flOn_nu{0};
    ///@unit{nu}
    ///@brief Axis aligned bounding box around the RoI for curb alignment.
    boolean frOn_nu{0};
    ///@range{0,3.4028237e+38}
    ///@unit{m}
    ///@brief Distance from the front left RoI.
    float32 flDist_m{0.0F};
    ///@range{0,3.4028237e+38}
    ///@unit{m}
    ///@brief Distance from the front right RoI.
    float32 frDist_m{0.0F};
  };

} // namespace si

#endif // SI_PULL_CORNER_POLY_H_
