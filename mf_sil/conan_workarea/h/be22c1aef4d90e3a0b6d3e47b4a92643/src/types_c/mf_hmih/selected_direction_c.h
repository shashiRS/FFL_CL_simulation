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

#ifndef MF_HMIH_SELECTED_DIRECTION_C_H_
#define MF_HMIH_SELECTED_DIRECTION_C_H_

#include "Platform_Types.h"

///If selection of direction is possible, the user can choose the direction of the vehicle in the parking space. The result of the user selection is indicated in this signal. If selection of direction is not possible, the signal value indicates the only possible parking direction.
typedef uint8 MF_HMIH_SelectedDirection;

#define MF_HMIH_SELECTED_DIRECTION_SEL_DIR_FORWARDS 0U
#define MF_HMIH_SELECTED_DIRECTION_SEL_DIR_BACKWARDS 1U


#endif // MF_HMIH_SELECTED_DIRECTION_C_H_