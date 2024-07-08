#ifndef _USSMATHUTILS_H__
#define _USSMATHUTILS_H__

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif


#define VEC_Abs(va, vb) \
		sqrt( fabs((vb)[0] - (va)[0]) * fabs((vb)[0] - (va)[0]) \
			+ fabs((vb)[1] - (va)[1]) * fabs((vb)[1] - (va)[1]) \
			+ fabs((vb)[2] - (va)[2]) * fabs((vb)[2] - (va)[2]) )

#define DistanceToTofFactor(T) \
	((((T) - 273.15) * 0.6) + 331.5) / 1000 // in ms -> Necessary for ToF-Counter

#define GetSoundVelinAir(T) \
	( 331.5 + (0.6 * T) )

#ifdef __cplusplus
}
#endif

#endif		/* #ifndef _MATHUTILS_H__ */