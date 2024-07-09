// COMPANY:   Continental Automotive
// COMPONENT: Eco
#ifndef ECO_DEPRECATION_H_
#define ECO_DEPRECATION_H_

// The behavior is highly dependent on compiler and compiler version.
// Marking symbols deprecated is not part of standard C++ until C++14.
// All the different versions are required to reflect the possible different locations where a symbol has to be declared
// deprecated.
// Examples:
//
//  DEPRECATED_VAR(int a) = 0;
//
//  struct DEPRECATED_STRUCT(MyStruct);
//
//  struct DEPRECATED_STRUCT_BEGIN MyStruct
//  {
//  } DEPRECATED_STRUCT_END;
//
// void DEPRECATED_FUNCT(myFunct());
//
// void DEPRECATED_FUNCT(myFunct())
// {
// }
//
#if defined(__clang__)
  #define DEPRECATED_STRING __attribute__ ((deprecated))
  #define DEPRECATED_STRUCT_BEGIN DEPRECATED_STRING
  #define DEPRECATED_STRUCT_END
  #define DEPRECATED_FUNCT_BEGIN
  #define DEPRECATED_FUNCT_END DEPRECATED_STRING
#elif defined(__GNUC__)
  #define DEPRECATED_STRING __attribute__ ((deprecated))
  #define DEPRECATED_STRUCT_BEGIN DEPRECATED_STRING
  #define DEPRECATED_STRUCT_END
  #define DEPRECATED_FUNCT_BEGIN
  #define DEPRECATED_FUNCT_END DEPRECATED_STRING
#elif defined(_MSC_VER)
  #define DEPRECATED_STRING __declspec(deprecated)
  #define DEPRECATED_STRUCT_BEGIN DEPRECATED_STRING
  #define DEPRECATED_STRUCT_END
  #define DEPRECATED_FUNCT_BEGIN DEPRECATED_STRING
  #define DEPRECATED_FUNCT_END
#else
  #define DEPRECATED_STRING
  #define DEPRECATED_STRUCT_BEGIN
  #define DEPRECATED_STRUCT_END
  #define DEPRECATED_FUNCT_BEGIN
  #define DEPRECATED_FUNCT_END
  #pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#endif

// This section is for avoiding redefinition and aligning because of outdated definitions
#ifdef DEPRECATED_BEGIN
#undef DEPRECATED_BEGIN
#endif
#ifdef DEPRECATED_END
#undef DEPRECATED_END
#endif
#ifdef DEPRECATED_BEGIN
#undef DEPRECATED_BEGIN
#endif
#ifdef DEPRECATED
#undef DEPRECATED
#endif
#define _DEPRECATION_H_
// End of redefinition avoidance section

#define DEPRECATED_BEGIN DEPRECATED_FUNCT_BEGIN
#define DEPRECATED_END DEPRECATED_FUNCT_END
#define DEPRECATED(SYMBOL) DEPRECATED_STRUCT_BEGIN SYMBOL DEPRECATED_STRUCT_END

#define DEPRECATED_STRUCT(SYMBOL) DEPRECATED_STRUCT_BEGIN SYMBOL DEPRECATED_STRUCT_END
#define DEPRECATED_FUNCT(SYMBOL) DEPRECATED_FUNCT_BEGIN SYMBOL DEPRECATED_FUNCT_END
#define DEPRECATED_VAR(SYMBOL) DEPRECATED_FUNCT(SYMBOL)


#endif // ECO_DEPRECATION_H_
