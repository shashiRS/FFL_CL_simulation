//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\enum.h.template!

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

#ifndef MF_MEMPARK_STORING_STATUS_C_H_
#define MF_MEMPARK_STORING_STATUS_C_H_

#include "Platform_Types.h"

///The status of the storing, loading and deleting data.
typedef uint8 MF_MEMPARK_StoringStatus;

#define MF_MEMPARK_STORING_STATUS_STORING_STATUS_INIT 0U
#define MF_MEMPARK_STORING_STATUS_STORING_STATUS_IDLE 1U
#define MF_MEMPARK_STORING_STATUS_STORING_STATUS_ONGOING 2U
#define MF_MEMPARK_STORING_STATUS_STORING_STATUS_SUCCESS 3U
#define MF_MEMPARK_STORING_STATUS_STORING_STATUS_FAILED 4U
#define MF_MEMPARK_STORING_STATUS_STORING_STATUS_NOT_VALID 5U


#endif // MF_MEMPARK_STORING_STATUS_C_H_
