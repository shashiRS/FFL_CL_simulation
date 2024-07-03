#pragma once

#ifdef CAR_MAKER_AP_DLL_EXPORT
#define DllExport __declspec(dllexport)
#else
#define DllExport __declspec(dllimport)
#endif

#include "Platform_Types.h"

//Sample times
static constexpr uint64_t SHORT_SAMPLE_TIME_MS{ 10U };
#ifdef VARIANT_CUS_ONLY
static constexpr uint64_t LONG_SAMPLE_TIME_MS{ 40U };
#else
static constexpr uint64_t LONG_SAMPLE_TIME_MS{ 33U };
#endif
