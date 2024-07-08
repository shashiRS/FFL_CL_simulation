/*! \file **********************************************************************

  @brief                  This file contains all RTE autosar platform types


*****************************************************************************/

// PRQA S 0047,0048 ++
// date: 2019-12-03, reviewer: uids8870
// reason: It is a project need to use 64bit values, and all our used compilers support it 

/* REF 1 - MISRA-C:2004 19.4 VIOLATION: Macro contains unacceptable items
This violation is due to the C type keyword 'boolean' used
inside the macro. This cannot be fixed since AUTOSAR requires
TRUE and FALSE to be defined like this.
******************************************************************************/
/* [PLATFORM002]: It is not allowed to add any extension to this file. */
/* Any extension invalidates the AUTOSAR conformity */

/* PROG044 */
#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H

/* [PLATFORM012] */
#define PLATFORM_VENDOR_ID			43
#define PLATFORM_AR_MAJOR_VERSION	2
#define PLATFORM_AR_MINOR_VERSION	1
#define PLATFORM_AR_PATCH_VERSION	0
#define PLATFORM_SW_MAJOR_VERSION	0
#define PLATFORM_SW_MINOR_VERSION	9
#define PLATFORM_SW_PATCH_VERSION	1

/* [PLATFORM057] */
#define CPU_TYPE_8 8
#define CPU_TYPE_16 16
#define CPU_TYPE_32 32
#define MSB_FIRST 0
#define LSB_FIRST 1
#define HIGH_BYTE_FIRST 0
#define LOW_BYTE_FIRST 1
/* [PLATFORM010],[PLATFORM031] */
#define CPU_TYPE (CPU_TYPE_32)            /* [PLATFORM004],[PLATFORM044],[PLATFORM045] - CPU_TYPE_32, CPU_TYPE_16, CPU_TYPE_8 */
#define CPU_BIT_ORDER (LSB_FIRST)         /* [PLATFORM043],[PLATFORM038],[PLATFORM048] - MSB_FIRST (bit), LSB_FIRST (little) */
#define CPU_BYTE_ORDER (LOW_BYTE_FIRST)  /* [PLATFORM046],[PLATFORM050],[PLATFORM039],[PLATFORM050] - HIGH_BYTE_FIRST, LOW_BYTE_FIRST */

#ifdef WIN32

#include "sys_types.h"

typedef uint8_t boolean;          /* [PLATFORM026],[PLATFORM027] */
typedef int8_t sint8;              /* [PLATFORM016] */
typedef uint8_t uint8;            /* [PLATFORM013] */
typedef int16_t sint16;            /* [PLATFORM017] */
typedef uint16_t uint16;          /* [PLATFORM014] */
typedef int32_t sint32;             /* [PLATFORM018] */
typedef uint32_t uint32;           /* [PLATFORM015] */
typedef uint32_t uint8_least;       /* [PLATFORM020],[PLATFORM005] */
typedef uint32_t uint16_least;      /* [PLATFORM021],[PLATFORM005] */
typedef uint32_t uint32_least;      /* [PLATFORM022],[PLATFORM005] */
typedef int32_t sint8_least;         /* [PLATFORM023],[PLATFORM005] */
typedef int32_t sint16_least;       	/* [PLATFORM024],[PLATFORM005] */
typedef int32_t sint32_least;        /* [PLATFORM025],[PLATFORM005] */
typedef uint64_t    uint64;   /* 0 .. 18446744073709551615   */
typedef float32_t float32;                  /* [PLATFORM041] */
typedef float64_t float64;                 /* [PLATFORM042] */
typedef int64_t sint64;

#else

#include <stdint.h>

typedef unsigned char boolean;          /* [PLATFORM026],[PLATFORM027] */
typedef int8_t sint8;              /* [PLATFORM016] */
typedef uint8_t uint8;            /* [PLATFORM013] */
typedef int16_t sint16;            /* [PLATFORM017] */
typedef uint16_t uint16;          /* [PLATFORM014] */
typedef int32_t sint32;             /* [PLATFORM018] */
typedef uint32_t uint32;           /* [PLATFORM015] */
typedef unsigned int uint8_least;       /* [PLATFORM020],[PLATFORM005] */
typedef unsigned int uint16_least;      /* [PLATFORM021],[PLATFORM005] */
typedef unsigned int uint32_least;      /* [PLATFORM022],[PLATFORM005] */
typedef signed int sint8_least;         /* [PLATFORM023],[PLATFORM005] */
typedef signed int sint16_least;       	/* [PLATFORM024],[PLATFORM005] */
typedef signed int sint32_least;        /* [PLATFORM025],[PLATFORM005] */
typedef unsigned long long    uint64;   /* 0 .. 18446744073709551615   */
typedef float float32;                  /* [PLATFORM041] */
typedef double float64;                 /* [PLATFORM042] */
typedef signed long long sint64;

#endif

/* [PLATFORM54], [PLATFORM55], [PLATFORM56] */
/* MISRA-C:2004 19.4 VIOLATION: Refer to REF 1 above */
#ifndef TRUE
    #define TRUE (1U)
#endif
#ifndef FALSE
    #define FALSE (0U)
#endif

#ifndef b_TRUE
    #define b_TRUE ((boolean) 1)
#endif
#ifndef b_FALSE
    #define b_FALSE ((boolean) 0)
#endif

/// Description: Null pointer for initialization / comparison \n
/// Range:       constant void* 0
#ifndef NULL
#ifdef  __cplusplus
#define NULL           (0)
#else //-- __cplusplus
// This definition is guarded with ifndef to prevent duplicates.
#define NULL           ((void *)0)
#endif //-- __cplusplus
#endif //-- NULL

// PRQA S 0047,0048 --

/* !LINKSTO Base.Types.SizeType,1 */
/** \brief Type definition of platform specific size type (generated, depending on parameter 'Cpu.Type') */
typedef uint32 usize;

/// Description: Calculates number of elements in array
/// @param{in] a : array
/// @return number of elements in array
#define ARRAY_LEN(a) (sizeof( a ) / sizeof(*(a)))

//-- The following 3 macros (CFG_OFF, CFG_ON and CFG_ENABLED) are from coding_guideline.docx: R 246 (C, C++, recommended)
// PRQA S 3410 ++
// 2017-07-10; uid40533
// summary: Msg(3:3410) Macro parameter not enclosed in ().
// reason: Missing bracket comes with another macro
// PRQA S 3411 ++
// 2017-07-10; uid40533
// summary: Msg(3:3411) Macro defined with unbalanced brackets, parentheses or braces.
// reason: This macros style is recommended explicitly in coding guideline.
// PRQA S 1020 ++
// 2017-07-06; uid42962
// summary: Msg(3:1020) Avoid macros.
//          Use constants, enumerators, typedefs or inline functions instead. MISRA-C++ Rules 16-0-4, 16-2-1
// reason: Macros used in #if statements -> they cannot be changed to constants

/// Description: Define for disabling feature flag. The "(" will match the ")" of the CFG_ENABLED macro.
#define CFG_OFF (0

/// Description: Define for enabling feature flag. The "(" will match the ")" of the CFG_ENABLED macro.
#define CFG_ON (1

/// Description: Definition for checking feature configuration. The usage of this definition ensures that \n
///              the configuration flag to be checked is also defined as either CFG_ON or CFG_OFF. \n
/// @param[in] feature : Flag to be checked (Range: CFG_OFF or CFG_ON)
///
/// @code{.c}
///       #define FEATURE_FLAG CFG_ON
/// @endcode
///
/// @code{.c}
///       #if CFG_ENABLED( FEATURE_FLAG )
///           // flag enabled. do something
///       #else
///           // flag disabled. do something else.
///       #endif
/// @endcode
#define CFG_ENABLED(feature) feature)

//#include <Platform_Types_Ext.h>

#endif /* #ifndef PLATFORM_TYPES_H */
