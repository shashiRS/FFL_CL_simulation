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

#ifndef SI_OPTIMIZATION_RESULT_H_
#define SI_OPTIMIZATION_RESULT_H_

#include "Platform_Types.h"
#include "si/quadrilateral_serializable.h"


namespace si
{

  /// @brief Structure containing the optional optimization result.
  struct OptimizationResult
  {
    ///@unit{nu}
    ///@brief Describes whether the optimization result is valid or not.
    boolean valid_nu{0};
    ///@brief End shape after the optimization.
    QuadrilateralSerializable shape{};
  };

} // namespace si

#endif // SI_OPTIMIZATION_RESULT_H_
