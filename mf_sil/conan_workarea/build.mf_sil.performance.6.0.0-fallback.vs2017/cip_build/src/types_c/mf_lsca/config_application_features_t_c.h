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

#ifndef MF_LSCA_CONFIG_APPLICATION_FEATURES_T_C_H_
#define MF_LSCA_CONFIG_APPLICATION_FEATURES_T_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ///If more than "numberOfPedestriansUntilCrowded_nu" pedestrians (including on bicycle) are in front of the car, treat them as static objects
    boolean treatCrowdedPedestriansAsStatic_nu;
    ///Number of pedestrians (including on bicycle) until all pedestrians are considered static objects (until fewer than this number of pedestrians are detected)
    uint8 numberOfPedestriansUntilCrowded_nu;
    ///Ignore dynamic objects of type "vehicle"
    boolean ignoreDynamicVehicles_nu;
    ///Check if loDMC handshake was successful or not (and disable lsca or put it in error if not)
    boolean checkLoDmcHandshake_nu;
} MF_LSCA_configApplicationFeatures_t;

inline MF_LSCA_configApplicationFeatures_t create_MF_LSCA_configApplicationFeatures_t(void)
{
  MF_LSCA_configApplicationFeatures_t m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_LSCA_CONFIG_APPLICATION_FEATURES_T_C_H_