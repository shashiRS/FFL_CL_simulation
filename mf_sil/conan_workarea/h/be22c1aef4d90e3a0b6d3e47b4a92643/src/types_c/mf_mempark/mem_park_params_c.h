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

#ifndef MF_MEMPARK_MEM_PARK_PARAMS_C_H_
#define MF_MEMPARK_MEM_PARK_PARAMS_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Memory Parking Parameters
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///None
    boolean functionActive_nu;
    ///Scan roi
    float32 scanRoiHalfWidth_m;
    ///Scan roi
    float32 scanRoiBackExtension_m;
    ///Scan roi
    float32 scanRoiFrontExtension_m;
    ///ICP covergence criterion
    uint8 minRequiredRelocalizationProb_perc;
    ///developer parameters
    float32 memParkDeveloperParam_0;
    ///Generic member for developer parameter.
    float32 memParkDeveloperParam_1;
    ///Generic member for developer parameter.
    float32 memParkDeveloperParam_2;
    ///Generic member for developer parameter.
    float32 memParkDeveloperParam_3;
    ///Generic member for developer parameter.
    float32 memParkDeveloperParam_4;
    ///Generic member for developer parameter.
    float32 memParkDeveloperParam_5;
    ///Generic member for developer parameter.
    float32 memParkDeveloperParam_6;
    ///Generic member for developer parameter.
    float32 memParkDeveloperParam_7;
} MF_MEMPARK_MemParkParams;

inline MF_MEMPARK_MemParkParams create_MF_MEMPARK_MemParkParams(void)
{
  MF_MEMPARK_MemParkParams m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // MF_MEMPARK_MEM_PARK_PARAMS_C_H_
