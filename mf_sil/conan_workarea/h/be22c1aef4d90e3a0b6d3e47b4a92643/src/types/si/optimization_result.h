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

#ifndef SI_OPTIMIZATION_RESULT_H_
#define SI_OPTIMIZATION_RESULT_H_

#include "Platform_Types.h"
#include "si/quadrilateral_serializable.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Structure containing the optional optimization result.
  struct OptimizationResult
  {
    ///@unit{nu}
    ///@brief Describes whether the optimization result is valid or not.
    boolean valid_nu;
    ///@brief End shape after the optimization.
    QuadrilateralSerializable shape;
  };

  inline ::si::OptimizationResult createOptimizationResult()
  {
    OptimizationResult m;
    (void)::eco::memset(&m, 0U, sizeof(OptimizationResult));
    m.valid_nu = 0;
    m.shape = createQuadrilateralSerializable();
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::OptimizationResult create_default()
  {
      return ::si::createOptimizationResult();
  }
}


#endif // SI_OPTIMIZATION_RESULT_H_
