/*
******************************************************************************
**  CarMaker - Version 8.1.1
**  Continental Teves AG & CO. oHG
**  - Stefan Hagenmueller -
**
******************************************************************************
*/

#ifndef _USS_Ext_H__
#define _USS_Ext_H__

#define _NO_OF_SENSORS_MAX_ 12
#define _NO_TOS_MAX_		300

#define _NO_OF_TARGS_MAX_ 10
#define _NO_OF_ECHOES_MAX_ 20
#define _NO_OF_TARGS_STRUCT_MAX_ 720

#define	_Sensor_Deaf_Time_ 2

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

	int		vCUS_InitData(void);
	int		vCUS_ReflectionGenerator(void);
	void	vCUS_Cleanup(void);

#ifdef __cplusplus
}
#endif

typedef struct tInt_var {
	int			ID;
	int			noTargets;
	double		Fr0_position_NP[20][2];
	double		NPDir_Fr0[20];
	double		distance[20];
	int			Detc_TO_ID[20];
	int			Detc_TO_Surface_ID[20];
} tInt_var;

extern tInt_var *int_var;

typedef struct tVector_calc {
	double NormalDir[4];
	double NormalsCombined[4];
	double NormalDirDefault[2];
	double DirVector[2];
} tVector_calc;

extern tVector_calc *env_vector;

typedef struct tTrafficObject {
	double		boundary_points[4][2];								// Boundary points for traffic objects [Left/Bottom | Rigth/Bottom | Rigth/Top | Left/Top] [x | y]
	double		boundary_pointsAngles[_NO_OF_SENSORS_MAX_][4];		// Boundary point angles for traffic objects relative to Fr0 from sensor [Sensor_ID] [Left/Bottom | Rigth/Bottom | Rigth/Top | Left/Top]
	double		boundary_pointsAngles_min[_NO_OF_SENSORS_MAX_];		// Boundary point minimum angles for traffic objects relative to Fr0 from sensor [Sensor_ID]
	double		boundary_pointsAngles_max[_NO_OF_SENSORS_MAX_];		// Boundary point maximum angles for traffic objects relative to Fr0 from sensor [Sensor_ID]
	double		normals[4][2];										// Normal vectors for bounding box [bottom | left | top | right] [x | y]
	double		NearestPointDir_Fr0[_NO_OF_SENSORS_MAX_];			// Global NearestPoint azimuth angle in Fr0 for each sensor
} tTrafficObject;

extern tTrafficObject *TrfObj;


typedef struct tSensorParam {
	double		orientation;										// Sensor orientation around z-axis
	double		Fr1_position[3];									// Sensor position in Fr1
} tSensorParam;

extern tSensorParam *SensParam;


typedef struct tSensorData {
	double		Fr0_position_Sens[3];								// Sensor position in Fr0
	double		Fr0_position_NP[20][2];								// Nearest Point / Reflection point position in Fr0 for TARGET [x | y]
	double		NPDir_Fr0[20];										// NearestPointDirection of reflection point in Fr0 for TARGET
	double		distance[20];										// Distance to Nearest Point / Reflection Point for TARGET
	int			Detc_TO_ID[20];										// Detected Traffic Object ID for TARGET
	int			Detc_TO_Surface_ID[20];								// Detected Traffic Object Surface ID for TARGET
} tSensorData;

extern tSensorData *SensData;

#endif	/* #ifndef _USS_Ext_H__ */