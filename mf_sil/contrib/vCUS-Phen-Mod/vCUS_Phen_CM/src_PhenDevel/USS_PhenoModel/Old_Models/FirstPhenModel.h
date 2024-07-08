/*
******************************************************************************
**  CarMaker - Version 8.1.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
*/

#ifndef _USER_H__
#define _USER_H__

#include <Global.h>
#include <Vehicle/MBSUtils.h>

#ifdef __cplusplus
extern "C" {
#endif



extern int UserCalcCalledByAppTestRunCalc;


#define N_USEROUTPUT	10
#define NO_MAX_TOS		300
#define NO_MAX_SENS		20

/* Struct for user variables. */
typedef struct tUser {
    /* For debugging purposes */
    double Out[N_USEROUTPUT];
} tUser;

typedef struct tTrafficObject {
	double		boundary_points[4][2];							// Boundary points for traffic objects [Left/Bottom | Rigth/Bottom | Rigth/Top | Left/Top] [x | y]
	double		boundary_pointsAngles[NO_MAX_SENS][4];			// Boundary point angles for traffic objects relative to Fr0 from sensor [Sensor_ID] [Left/Bottom | Rigth/Bottom | Rigth/Top | Left/Top]
	double		boundary_pointsAngles_min[NO_MAX_SENS];			// Boundary point minimum angles for traffic objects relative to Fr0 from sensor [Sensor_ID]
	double		boundary_pointsAngles_max[NO_MAX_SENS];			// Boundary point maximum angles for traffic objects relative to Fr0 from sensor [Sensor_ID]
	double		normals[4][2];									// Normal vectors for bounding box [bottom | left | top | right] [x | y]
	double		NearestPointDir_Fr0[NO_MAX_SENS];				// Global NearestPoint azimuth angle in Fr0 for each sensor
} tTrafficObject;

typedef struct tSensorParam {
	double		orientation;									// Sensor orientation around z-axis
	double		Fr1_position[3];								// Sensor position in Fr1
} tSensorParam;

typedef struct tSensorData {
	double		Fr0_position_Sens[3];							// Sensor position in Fr0
	double		Fr0_position_NP[20][2];							// Nearest Point / Reflection point position in Fr0 for TARGET [x | y]
	double		NPDir_Fr0[20];									// NearestPointDirection of reflection point in Fr0 for TARGET
	double		distance[20];									// Distance to Nearest Point / Reflection Point for TARGET
	int			Detc_TO_ID[20];									// Detected Traffic Object ID for TARGET
	int			Detc_TO_Surface_ID[20];							// Detected Traffic Object Surface ID for TARGET
} tSensorData;

extern tUser User;


int 	User_Init_First		(void);
int 	User_Init		(void);
void	User_PrintUsage		(const char *Pgm);
char  **User_ScanCmdLine	(int argc, char **argv);
int 	User_Start		(void);
int	User_Register		(void);
void	User_DeclQuants		(void);
int 	User_ShutDown		(int ShutDownForced);
int 	User_End		(void);
void 	User_Cleanup		(void);

int	User_TestRun_Start_atBegin		(void);
int	User_TestRun_Start_atEnd		(void);
int	User_TestRun_Start_StaticCond_Calc	(void);
int	User_TestRun_Start_Finalize		(void);
int	User_TestRun_RampUp			(double dt);
int	User_DrivMan_Calc			(double dt);
int	User_VehicleControl_Calc		(double dt);
int	User_Brake_Calc				(double dt);
int	User_Traffic_Calc			(double dt);
int	User_Calc				(double dt);
int	User_Check_IsIdle			(int IsIdle);
int	User_TestRun_End_First 			(void);
int	User_TestRun_End 			(void);

void 	User_In  (const unsigned CycleNo);
void	User_Out (const unsigned CycleNo);


/* User_<> functions,
** - called from SimCore and in CM_Main.c,
** - already defined in SimCore.h
*/
int 	User_Param_Get		(void);
int 	User_Param_Add		(void);
int 	User_ApoMsg_Eval (int channel, char *msg, int len, int who);
void 	User_ApoMsg_Send (double T, const unsigned CycleNo);


#define User_TestRun_Start   User_TestRun_Start__deprecated_function__Change_to__User_TestRun_Start_XYZ;


#ifdef __cplusplus
}
#endif

#endif	/* #ifndef _USER_H__ */
