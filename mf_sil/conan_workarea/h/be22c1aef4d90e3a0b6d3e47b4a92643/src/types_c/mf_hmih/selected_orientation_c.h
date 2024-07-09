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

#ifndef MF_HMIH_SELECTED_ORIENTATION_C_H_
#define MF_HMIH_SELECTED_ORIENTATION_C_H_

#include "Platform_Types.h"

///If orientation is uncertain, the user can choose the final orientation by a button. The result of the user selection is indicated in this signal. If selection of orientation is not possible, the signal value indicates the only possible parking orientation.
typedef uint8 MF_HMIH_SelectedOrientation;

#define MF_HMIH_SELECTED_ORIENTATION_SEL_ORI_PARALLEL 0U
#define MF_HMIH_SELECTED_ORIENTATION_SEL_ORI_PERPENDICULAR 1U
#define MF_HMIH_SELECTED_ORIENTATION_SEL_ORI_ANGLED_STANDARD 2U
#define MF_HMIH_SELECTED_ORIENTATION_SEL_ORI_ANGLED_REVERSE 3U


#endif // MF_HMIH_SELECTED_ORIENTATION_C_H_
