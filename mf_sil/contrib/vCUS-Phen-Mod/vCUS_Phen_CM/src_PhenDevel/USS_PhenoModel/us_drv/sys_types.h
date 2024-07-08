/**
 * @brief       Declares global basic types.
 *
 * @copyright   2015 Application Solutions (Electronics and Vision) Limited / Continental Corporation
 * @author      Vaclav Mocek <Vaclav.Mocek@continental-corporation.com>
 *
 * @details     Declares global basic types.
 *
 * @file
 *
 * @ingroup     core-sys
 *
 */

#ifndef sys_types_h
#define sys_types_h

#ifdef __GNUC__
    #ifndef __cplusplus
        #include <stdbool.h>
    #endif
#endif

// TODO: <VM> We need a feature flag for INT types.
#if ( defined(__GNUC__) || defined (__TI_COMPILER_VERSION__) || defined(__clang__))
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#endif

#ifndef __bool_true_false_are_defined
    #ifndef __cplusplus
        #define bool    int
        #define true    1
        #define false   0
    #endif
    #define __bool_true_false_are_defined   1
#endif

#if !( defined(__GNUC__) || defined(__TI_COMPILER_VERSION__) || defined(__clang__))
typedef   signed    __int8      int8_t;
typedef unsigned    __int8      uint8_t;
typedef   signed    __int16     int16_t;
typedef unsigned    __int16     uint16_t;
typedef   signed    __int32     int32_t;
typedef unsigned    __int32     uint32_t;
typedef   signed    __int64     int64_t;
typedef unsigned    __int64     uint64_t;
#endif

// MISRA-C:2004 float and char recommended typedefs
typedef char    char_t;
typedef float   float32_t;
typedef signed char sint8_t;
typedef double  float64_t;

// MISRA-C:2012 common typedefs for completeness. See mc3_types.h (Example Suite)
typedef bool bool_t;
#endif // sys_types_h
