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

#ifndef MF_LSCA_CONFIG_GENERAL_T_C_H_
#define MF_LSCA_CONFIG_GENERAL_T_C_H_

#include "Platform_Types.h"
#include "cml/vec2_df_pod_c.h"
#include "eco/memset_c.h"

/// Struct that contains all relevant general parameter data
typedef struct
{
    uint32 simpleShapeActualSize_nu;
    ///4-point-shape for dynamic object calculations
    CML_Vec2Df_POD simpleShape[4];
    uint32 forwardLeftIndicesActualSize_nu;
    ///Indices of points for forward left motion      #maybe this can be removed, using the bodyIndexLocationInfo?
    uint8 forwardLeftIndices[17];
    uint32 forwardRightIndicesActualSize_nu;
    ///Indices of points for forward right motion     #maybe this can be removed, using the bodyIndexLocationInfo?
    uint8 forwardRightIndices[17];
    uint32 backwardLeftIndicesActualSize_nu;
    ///Indices of points for backward left motion     #maybe this can be removed, using the bodyIndexLocationInfo?
    uint8 backwardLeftIndices[17];
    uint32 backwardRightIndicesActualSize_nu;
    ///Indices of points for backward right motion    #maybe this can be removed, using the bodyIndexLocationInfo?
    uint8 backwardRightIndices[17];
    uint32 bodyIndexLocationFrontActualSize_nu;
    ///Indices of points on the body front
    uint8 bodyIndexLocationFront[2];
    uint32 bodyIndexLocationBackActualSize_nu;
    ///Indices of points on the body back
    uint8 bodyIndexLocationBack[2];
    uint32 bodyIndexLocationLeftActualSize_nu;
    ///Indices of points on the body left
    uint8 bodyIndexLocationLeft[2];
    uint32 bodyIndexLocationRightActualSize_nu;
    ///Indices of points on the body right
    uint8 bodyIndexLocationRight[2];
    uint32 bodyIndexLocationFrontLeftActualSize_nu;
    ///Indices of points on the body left-front corner
    uint8 bodyIndexLocationFrontLeft[2];
    uint32 bodyIndexLocationFrontRightActualSize_nu;
    ///Indices of points on the body right-front corner
    uint8 bodyIndexLocationFrontRight[2];
    uint32 bodyIndexLocationBackLeftActualSize_nu;
    ///Indices of points on the body left-back corner
    uint8 bodyIndexLocationBackLeft[2];
    uint32 bodyIndexLocationBackRightActualSize_nu;
    ///Indices of points on the body right-back corner
    uint8 bodyIndexLocationBackRight[2];
    ///Overall function switch
    boolean LscaActive_nu;
    ///Force lsca off when AP is off
    boolean onlyBackupMode;
    ///Steering proposal switch
    boolean proposalActive_nu;
    ///Steering resistance switch
    boolean resistanceActive_nu;
    ///Static brake switch
    boolean brakeStaticActive_nu;
    ///Dynamic brake switch
    boolean brakeDynamicActive_nu;
    ///Door opening protection switch for static objects
    boolean doorProtectionStatActive_nu;
    ///Door opening protection switch for dynamic objects
    boolean doorProtectionDynActive_nu;
    ///Reverse crossing traffic switch for dynamic objects
    boolean reverseAssistDynActive_nu;
    ///Pmp static switch
    boolean pedalMisapplicationActive_nu;
} MF_LSCA_configGeneral_t;

inline MF_LSCA_configGeneral_t create_MF_LSCA_configGeneral_t(void)
{
  MF_LSCA_configGeneral_t m;
  (void) ECO_memset (&m, 0, sizeof(m));
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.simpleShape) / sizeof(m.simpleShape[0])); ++i)
    {
      m.simpleShape[i] = create_CML_Vec2Df_POD();
    }
  }
  return m;
}

#endif // MF_LSCA_CONFIG_GENERAL_T_C_H_
