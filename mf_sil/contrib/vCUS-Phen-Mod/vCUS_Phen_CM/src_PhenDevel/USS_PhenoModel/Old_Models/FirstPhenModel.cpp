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
**
** Functions
** ---------
**
** Initialization
**
**	User_Init_First ()
**	User_PrintUsage ()
**	User_ScanCmdLine ()
**
**	User_AppLogFilter ()
**
**	User_Init ()
**	User_Register ()
**	User_DeclQuants ()
**
**	User_Param_Add ()
**	User_Param_Get ()
**
**
** Main TestRun Start/End:
**
**	User_TestRun_Start_atBegin ()
**	User_TestRun_Start_atEnd ()
**	User_TestRun_Start_StaticCond_Calc ()
**	User_TestRun_Start_Finalize ()
**	User_TestRun_RampUp ()
**
**	User_TestRun_End_First ()
**	User_TestRun_End ()
**
**
** Main Cycle:
**
**	User_In ()
**
**	User_DrivMan_Calc ()
** 	User_Traffic_Calc ()
**	User_VehicleControl_Calc ()
**	User_Brake_Calc ()           in Vhcl_Calc ()
**	User_Calc ()
**	User_Check_IsIdle ()
**
**	User_Out ()
**
**
** APO Communication:
**
**	User_ApoMsg_Eval ()
**	User_ApoMsg_Send ()
**
**	User_ShutDown ()
**	User_End ()
**	User_Cleanup ()
**
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(XENO)
# include <mio.h>
#endif

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include <ADASRP.h>

#include <rbs.h>

#include "IOVec.h"
#include "User.h"
#include <Vehicle\Sensor_Object.h>

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */


int UserCalcCalledByAppTestRunCalc = 0;


tUser	User;

/* Define Trfobj handle & initiallize handle */
tTrafficObject	*TrfObj = NULL;
tSensorParam	*SensParam = NULL;
tSensorData		*SensData = NULL;

int nTC, nOS;
int debug = 1;

/*
** User_Init_First ()
**
** First, low level initialization of the User module
**
** Call:
** - one times at start of program
** - no realtime conditions
**
*/

int
User_Init_First (void)
{
    memset (&User, 0, sizeof(User));


    return 0;
}



/*
** User_PrintUsage ()
**
** Print the user/application specific programm arguments
*/

void
User_PrintUsage (const char *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
	if (IO_GetDefault() != NULL)
	    printf(" -io %-12s Default I/O configuration\n", IO_GetDefault());
	const tIOConfig *cf;
	for (cf=IO_GetConfigurations(); cf->Name!=NULL; cf++)
	    LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
    }
#endif
}



/*
** User_ScanCmdLine ()
**
** Scan application specific command line arguments
**
** Return:
** - argv: last unscanned argument
** - NULL: error or unknown argument
*/

char **
User_ScanCmdLine (int argc, char **argv)
{
    const char *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("none" /* or "demoapp", "demorbs,demofr" etc. */);

    while (*++argv) {
	if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
	    if (IO_Select(*++argv) != 0)
		return NULL;
	} else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
	    User_PrintUsage(Pgm);
	    SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
	    return  NULL;
	} else if ((*argv)[0] == '-') {
	    LogErrF(EC_General, "Unknown option '%s'", *argv);
	    return NULL;
	} else {
	    break;
	}
    }

    return argv;
}



/*
** User_Init ()
**
** Basic initialization of the module User.o
**
** Call:
** - once at program start
** - no realtime conditions
*/

int
User_Init (void)
{
    return 0;
}



int
User_Register (void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}



/*
** User_DeclQuants ()
**
** Add user specific quantities to the dictionary
**
** Call:
** - once at program start
** - no realtime conditions
*/

void
User_DeclQuants (void)
{
    int i;

    for (i=0; i<N_USEROUTPUT; i++) {
	char sbuf[32];
	sprintf (sbuf, "UserOut_%02d", i);
	DDefDouble (NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }
#if !defined(LABCAR)
    RBS_DeclQuants();
#endif
}


/*
** User_Param_Add ()
**
** Update all modified application specific parameters in the test stand
** parameter file (ECUParameters).
**
** If the variable SimCore.TestRig.ECUParam.Modified set to 1 somewhere else
** CarMaker calls this function to let the user add or change all necessary
** entries before the file is written.
** So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
** The next TestRun start or end, CarMaker calls this function and writes
** the file to the harddisk.
**
** Call:
** - in a separate thread (no realtime contitions)
** - when starting a new test run
*/

int
User_Param_Add (void)
{
#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;
#endif

    return 0;
}



/*
** User_Param_Get ()
**
** Update all modified application specific parameters from the test stand
** parameter file (ECUParameters).
**
** Call:
** - in a separate thread (no realtime conditions)
** - if User_Param_Get() wasn't called
** - when starting a new test run, if
**   - the files SimParameters and/or
**   - ECUParameters
**   are modified since last reading
**
** return values:
**  0	ok
** -1	no testrig parameter file
** -2	testrig parameter error
** -3	i/o configuration specific error
** -4	no simulation parameters
** -5	simulation parameters error
** -6	FailSafeTester parameter/init error
*/

int
User_Param_Get (void)
{
    int rv = 0;

#if defined(CM_HIL)
    /*** testrig / ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -2;
#endif

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL)
	return -4;

    return rv;
}



/*
** User_TestRun_Start_atBegin ()
**
** Special things before a new simulation starts like
** - reset user variables to their default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - after (standard) infofiles are read in
** - before reading parameters for Environment, DrivMan, Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atBegin (void)
{
    int rv = 0;
    int i;

    for (i=0; i<N_USEROUTPUT; i++)
	User.Out[i] = 0.0;


    if (IO_None)
	return rv;

#if defined(CM_HIL)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -6;
#endif

    return rv;
}




/*
** User_TestRun_Start_atEnd ()
**
** Special things before a new simulation starts like
** - reset user variables to there default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - at the end, behind reading parameters for Environment, DrivMan,
**   Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atEnd (void)
{
	float  *sensor_orientation = NULL, *sensor_position = NULL;
	int nCols;
	char pre[64], sbuf[256];

	nTC = iGetIntOpt(SimCore.TestRun.Inf, "Traffic.N", 0);
	nOS = iGetIntOpt(SimCore.Vhcl.Inf, "Sensor.Object.N", 0);

	// Obj = (tTrafficObj *)calloc(nTC, sizeof(tTrafficObj));
	TrfObj = (tTrafficObject *)calloc(nTC, sizeof(tTrafficObject));
	SensParam = (tSensorParam *)calloc(nOS, sizeof(tSensorParam));
	SensData = (tSensorData *)calloc(nOS, sizeof(tSensorData));

	/* DDict -> Traffic */
	for (int i = 0; i < nTC; i++) {
		/* Generate DDict Entries for all edges */
		for (int j = 0; j < 4; j++) {
			TrfObj[i].boundary_points[j][0] = 0;
			TrfObj[i].boundary_points[j][1] = 0;

			sprintf(sbuf, "TrafficData.%d.Edge.%d.PosX", i, j);
			DDefDouble4(NULL, sbuf, "m", &TrfObj[i].boundary_points[j][0], DVA_None);
			sprintf(sbuf, "TrafficData.%d.Edge.%d.PosY", i, j);
			DDefDouble4(NULL, sbuf, "m", &TrfObj[i].boundary_points[j][1], DVA_None);
		}
	}

	/* DDict/IFile -> Sensor */
	for (int i = 0; i < nOS; i++) {

		sprintf(pre, "Sensor.Object.%d.rot", i);
		if ((sensor_orientation = iGetTableFltOpt2(SimCore.Vhcl.Inf, pre, NULL, 1, &nCols)) == NULL)
			LogErrF(EC_Init, "Can't read ''Sensor.Object.%d.rot''\n", i);
		SensParam[i].orientation = sensor_orientation[2];

		sprintf(pre, "Sensor.Object.%d.pos", i);
		if ((sensor_position = iGetTableFltOpt2(SimCore.Vhcl.Inf, pre, NULL, 1, &nCols)) == NULL)
			LogErrF(EC_Init, "Can't read ''Sensor.Object.%d.pos''\n", i);
		for (int j = 0; j < 3; j++) {
			SensParam[i].Fr1_position[j] = sensor_position[j];
			SensData[i].Fr0_position_Sens[j] = 0;
			if (debug == 1)
				Log("Position for sensor No %d | Element %d = %f\n\n", i, j, SensParam[i].Fr1_position[j]);
		}

		if (debug == 1)
			Log("Orientation for sensor No %d = %f\n\n", i, SensParam[i].orientation);

		for (int k = 0; k < 3; k++)
			SensData[i].Fr0_position_Sens[k] = 0;

		/* Generate DDict Entries for all Sensors */
		sprintf(sbuf, "SensData.%d.SensPos.x", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[0], DVA_None);
		sprintf(sbuf, "SensData.%d.SensPos.y", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[1], DVA_None);
		sprintf(sbuf, "SensData.%d.SensPos.z", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[2], DVA_None);

		for (int k = 0; k < 20; k++) {
			SensData[i].Fr0_position_NP[k][0] = 0;
			SensData[i].Fr0_position_NP[k][1] = 0;
			SensData[i].distance[k] = 0;
			SensData[i].Detc_TO_ID[k] = 0;
			SensData[i].Detc_TO_Surface_ID[k] = 0;
			SensData[i].NPDir_Fr0[k] = 0;

			sprintf(sbuf, "SensData.%d.Target.NPPos.x.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_NP[k][0], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.NPPos.y.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_NP[k][1], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.NPDir_Fr0.%d", i, k);
			DDefDouble(NULL, sbuf, "-", &SensData[i].NPDir_Fr0[k], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.distance.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].distance[k], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.Detected_TOid.%d", i, k);
			DDefInt(NULL, sbuf, "-", &SensData[i].Detc_TO_ID[k], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.Detected_TO_SurfaceID.%d", i, k);
			DDefInt(NULL, sbuf, "-", &SensData[i].Detc_TO_Surface_ID[k], DVA_None);
		}
	}

    return 0;
}



/*
** User_TestRun_Start_StaticCond_Calc ()
**
** called in non RT context
*/

int
User_TestRun_Start_StaticCond_Calc (void)
{
    return 0;
}



/*
** User_TestRun_Start_Finalize ()
**
** called in RT context
*/

int
User_TestRun_Start_Finalize (void)
{
    return 0;
}



/*
** User_TestRun_RampUp ()
**
** Perform a smooth transition of variables (e.g. I/O)
** from their current state  to the new testrun.
** This function is called repeatedly, once during each cycle, until
** it returns true (or issues an error message), so the function should
** return true if transitioning is done, false otherwise.
**
** In case of an error the function should issue an apropriate
** error message and return false;
**
** Called in RT context, in state SCState_StartSim,
** after preprocessing is done, before starting the engine.
** Please note, that in this early initialization state no calculation
** of the vehicle model takes place.
*/

int
User_TestRun_RampUp (double dt)
{
    int IsReady = 1;

    return IsReady;
}



/*
** User_TestRun_End_First ()
**
** Invoked immediately after the end of a simulation is initiated,
** but before data storage ends and before transitioning into SCState_Idle.
** - Send Scratchpad-note
** - ...
**
** Call:
** - in main task, in the main loop (real-time conditions!)
** - when a test run is finished (SimCore.State is SCState_End)
*/

int
User_TestRun_End_First (void)
{
    return 0;
}



/*
** User_TestRun_End ()
**
** Special things after the end of a simulation like
** - switch off an air compressor
** - Write something to a file
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when a test run is finished (SimCore.State is SCState_End<xyz>)
*/

int
User_TestRun_End (void)
{
    return 0;
}



/*
** User_In ()
**
** Assign quantities of the i/o vector to model variables
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - just after IO_In()
*/

void
User_In (const unsigned CycleNo)
{
    if (SimCore.State != SCState_Simulate)
	return;
}



/*
** User_DrivMan_Calc ()
**
** called
** - in RT context
** - after DrivMan_Calc()
*/

int
User_DrivMan_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}


/*
** User_VehicleControl_Calc ()
**
** called
** - in RT context
** - after VehicleControl_Calc()
*/

int
User_VehicleControl_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}



/*
** User_Brake_Calc ()
**
** called
** - in RT context
** - after Brake_Calc() in Vhcl_Calc()
*/

int
User_Brake_Calc (double dt)
{
    /* Modify the total brake torque from the brake system model Brake.Trq_tot[]
       or the target drive source torque from the brake control unit
       Brake.HydBrakeCU_IF.Trq_DriveSrc_trg[]
    */

    return 0;
}



/*
** User_Traffic_Calc ()
**
** called
** - in RT context
** - after Traffic_Calc()
*/

int
User_Traffic_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate)
	return 0;

    return 0;
}



/*
** User_Calc ()
**
** called in RT context
*/

int
User_Calc (double dt)
{
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/

	struct vector_calc {
		double NormalDir[4];
		double NormalsCombined[4];
		double NormalDirDefault[2];
		double DirVector[2];
	} env_vector;
	
	env_vector.NormalDirDefault[0]	= 1;
	env_vector.NormalDirDefault[1]	= 0;
	env_vector.DirVector[0]			= 0;
	env_vector.DirVector[1]			= 0;

	int cnt = 0;

	struct int_var {
		int			ID						= -1;
		int			noTargets				= 0;
		double		Fr0_position_NP[20][2]	= { 0 };
		double		NPDir_Fr0[20]			= { 0 };
		double		distance[20]			= { 0 };
		int			Detc_TO_ID[20]			= { 0 };
		int			Detc_TO_Surface_ID[20]	= { 0 };

	} int_var;

	/* Initialize pointers */
	tTrafficObj *Obj = NULL;
	tObjectSensor *pOS = NULL;
	tObjectSensorObj *pOSO = NULL;

	/* ****************************** */
	/* Reset variables at TestRun end */
	/* ****************************** */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		for (int j = 0; j < ObjectSensorCount; j++) {
			/* tSensorData SensData */
			for (int k = 0; k < 3; k++)
				SensData[j].Fr0_position_Sens[k] = 0;
			for (int k = 0; k < 20; k++) {
				SensData[j].Fr0_position_NP[k][0] = 0;
				SensData[j].Fr0_position_NP[k][1] = 0;
				SensData[j].NPDir_Fr0[k] = 0;
				SensData[j].distance[k] = 0;
				SensData[j].Detc_TO_ID[k] = 0;
				SensData[j].Detc_TO_Surface_ID[k] = 0;
			}
			/* tSensorParam SensParam */
			SensParam[j].orientation = 0;
			for (int k = 0; k < 3; k++)
				SensParam[j].Fr1_position[k] = 0;
		}

		for (int i = 0; i < Traffic.nObjs; i++) {
			/* tTrafficObject TrfObj */
			for (int k = 0; k < 4; k++) {
				TrfObj[i].boundary_points[k][0] = 0;
				TrfObj[i].boundary_points[k][1] = 0;
				for (int l = 0; l < NO_MAX_SENS; l++) {
					TrfObj[i].boundary_pointsAngles[l][k] = 0;
				}
				TrfObj[i].normals[k][0] = 0;
				TrfObj[i].normals[k][1] = 0;
			}
			for (int k = 0; k < NO_MAX_SENS; k++) {
				TrfObj[i].NearestPointDir_Fr0[k] = 0;
				TrfObj[i].boundary_pointsAngles_min[k] = 0;
				TrfObj[i].boundary_pointsAngles_max[k] = 0;
			}
		}

		return 0;
	}



	/* ******************************************* */
	/* Run through Object Sensors - Main Routine - */
	/* ******************************************* */
	if (ObjectSensorCount > 0 && (SimCore.State == SCState_StartLastCycle || SimCore.State == SCState_Simulate)) {
		for (int j = 0; j < ObjectSensorCount; j++) {

			/* Derive global sensor position */
			for (int k = 0; k < 3; k++)
				SensData[j].Fr0_position_Sens[k] = Vehicle.Fr1A.t_0[k] + SensParam[j].Fr1_position[k];

			for (int k = 0; k < 20; k++) {
				SensData[j].distance[k] = 0;
				SensData[j].Detc_TO_ID[k] = -1;
				SensData[j].Detc_TO_Surface_ID[k] = 0;
				SensData[j].Fr0_position_NP[k][0] = 0;
				SensData[j].Fr0_position_NP[k][1] = 0;
				SensData[j].NPDir_Fr0[k] = 0;
			}

			/* ********************************* */
			/* Extraction of Traffic information */
			/* ********************************* */
			if (Traffic.nObjs > 0) {
				for (int i = 0; i < Traffic.nObjs; i++) {

					/* Get Traffic object information */
					Obj = Traffic_GetByTrfId(i);
					if (Obj->Cfg.Envelope.nArea == 6) {
						/* Get information for sensor and TO */
						tObjectSensor *pOS = ObjectSensor_GetByIndex(j);
						tObjectSensorObj *pOSO = ObjectSensor_GetObject(j, i);

						/* Extract 2D-information & safe into struct */
						TrfObj[i].boundary_points[0][0]			= Obj->Envelope.Areas_0[0].Edge[3][0];		// Point A -> x-Coordinate
						TrfObj[i].boundary_points[0][1]			= Obj->Envelope.Areas_0[0].Edge[3][1];		// Point A -> y-Coordinate
						TrfObj[i].normals[0][0]					= Obj->Envelope.Areas_0[0].z[0];			// Normal of line A-B -> x-Compontent
						TrfObj[i].normals[0][1]					= Obj->Envelope.Areas_0[0].z[1];			// Normal of line A-B -> y-Compontent
						env_vector.DirVector[0]					= TrfObj[i].boundary_points[0][0]// A_x
																- SensData[j].Fr0_position_Sens[0];//Sens_x
																// x-Coord of Vector to Bound_Point
						env_vector.DirVector[1]					= TrfObj[i].boundary_points[0][1]// Ay
																- SensData[j].Fr0_position_Sens[1];//Sens_Y
																// y-Coord of Vector to Bound_Point
						TrfObj[i].boundary_pointsAngles[j][0]	= rad2deg *
																	acos( VEC_Scalar2D(env_vector.NormalDirDefault, env_vector.DirVector)
																		/ ( VEC_Norm2D(env_vector.NormalDirDefault) * VEC_Norm2D(env_vector.DirVector) ) );

						TrfObj[i].boundary_points[1][0]			= Obj->Envelope.Areas_0[0].Edge[2][0];		// Point B -> x-Coordinate
						TrfObj[i].boundary_points[1][1]			= Obj->Envelope.Areas_0[0].Edge[2][1];		// Point B -> y-Coordinate
						TrfObj[i].normals[1][0]					= Obj->Envelope.Areas_0[1].z[0];			// Normal of line B-C -> x-Compontent
						TrfObj[i].normals[1][1]					= Obj->Envelope.Areas_0[1].z[1];			// Normal of line B-C -> y-Compontent
						env_vector.DirVector[0]					= TrfObj[i].boundary_points[1][0]// A_x
																- (Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]);//Sens_x
																// x-Coord of Vector to Bound_Point
						env_vector.DirVector[1]					= TrfObj[i].boundary_points[1][1]// Ay
																- (Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]);//Sens_Y
																// y-Coord of Vector to Bound_Point
						TrfObj[i].boundary_pointsAngles[j][1]	= rad2deg *
																	acos( VEC_Scalar2D(env_vector.NormalDirDefault, env_vector.DirVector)
																		/ ( VEC_Norm2D(env_vector.NormalDirDefault) * VEC_Norm2D(env_vector.DirVector) ) );

						TrfObj[i].boundary_points[2][0]			= Obj->Envelope.Areas_0[2].Edge[3][0];		// Point C -> x-Coordinate
						TrfObj[i].boundary_points[2][1]			= Obj->Envelope.Areas_0[2].Edge[3][1];		// Point C -> y-Coordinate
						TrfObj[i].normals[2][0]					= Obj->Envelope.Areas_0[2].z[0];			// Normal of line C-D -> x-Compontent
						TrfObj[i].normals[2][1]					= Obj->Envelope.Areas_0[2].z[1];			// Normal of line C-D -> y-Compontent
						env_vector.DirVector[0]					= TrfObj[i].boundary_points[2][0]// A_x
																- (Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]);//Sens_x
																// x-Coord of Vector to Bound_Point
						env_vector.DirVector[1]					= TrfObj[i].boundary_points[2][1]// Ay
																- (Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]);//Sens_Y
																// y-Coord of Vector to Bound_Point
						TrfObj[i].boundary_pointsAngles[j][2]	= rad2deg *
																	acos( VEC_Scalar2D(env_vector.NormalDirDefault, env_vector.DirVector)
																		/ ( VEC_Norm2D(env_vector.NormalDirDefault) * VEC_Norm2D(env_vector.DirVector) ) );

						TrfObj[i].boundary_points[3][0]			= Obj->Envelope.Areas_0[2].Edge[2][0];		// Point D -> x-Coordinate
						TrfObj[i].boundary_points[3][1]			= Obj->Envelope.Areas_0[2].Edge[2][1];		// Point D -> y-Coordinate
						TrfObj[i].normals[3][0]					= Obj->Envelope.Areas_0[3].z[0];			// Normal of line D-A -> x-Compontent
						TrfObj[i].normals[3][1]					= Obj->Envelope.Areas_0[3].z[1];			// Normal of line D-A -> y-Compontent
						env_vector.DirVector[0]					= TrfObj[i].boundary_points[3][0]// A_x
																- (Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]);//Sens_x
																// x-Coord of Vector to Bound_Point
						env_vector.DirVector[1]					= TrfObj[i].boundary_points[3][1]// Ay
																- (Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]);//Sens_Y
																// y-Coord of Vector to Bound_Point
						TrfObj[i].boundary_pointsAngles[j][3]	= rad2deg *
																	acos( VEC_Scalar2D(env_vector.NormalDirDefault, env_vector.DirVector)
																		/ ( VEC_Norm2D(env_vector.NormalDirDefault) * VEC_Norm2D(env_vector.DirVector) ) );

						// Debug Output
						if (SimCore.State == SCState_StartLastCycle && debug == 1) {
							Log("Traffic No: %d\n", i);
							for (int j = 0; j < 4; j++) {
								Log("Point %d (x/y)->(%f/%f)\n", j, TrfObj[i].boundary_points[j][0], TrfObj[i].boundary_points[j][1]);
								Log("Point %d-%d (x/y)->(%f/%f)\n", j, (j+1), TrfObj[i].normals[j][0], TrfObj[i].normals[j][1]);
							}
							Log("------------------\n");
						}
					}
				}
			}



			/* ********************* */
			/* Evaluation ORTHOGONAL */
			/* ********************* */
			if (Traffic.nObjs > 0) {
				for (int i = 0; i < Traffic.nObjs; i++) {

					/* Get Traffic object information */
					Obj = Traffic_GetByTrfId(i);
					if (Obj->Cfg.Envelope.nArea == 6) {
						/* Get information for sensor and TO */
						tObjectSensor *pOS = ObjectSensor_GetByIndex(j);
						tObjectSensorObj *pOSO = ObjectSensor_GetObject(j, i);

						/* Calculate global angles */
						TrfObj[i].NearestPointDir_Fr0[j] = rad2deg * (Vehicle.Yaw + pOSO->NearPnt.alpha_p/*NearesPoint rotation*/) + SensParam[j].orientation/*Sensors rotation*/;

						/* For all 4 surfaces */
						for (int k = 0; k < 4; k++)
							/* Get normal vector's direction relative to Fr0 */
							env_vector.NormalDir[k] = rad2deg * acos(VEC_Scalar2D(env_vector.NormalDirDefault, TrfObj[i].normals[k])
								/ (VEC_Norm2D(env_vector.NormalDirDefault) * VEC_Norm2D(TrfObj[i].normals[k])));

						if ( (rad2deg * Obj->r_zyx[2]) <= 90 && (rad2deg * Obj->r_zyx[2]) >= 0 ) {
							env_vector.NormalDir[0] = env_vector.NormalDir[0] * (-1);
							env_vector.NormalDir[3] = env_vector.NormalDir[3] * (-1);
						}
						else if ( (rad2deg * Obj->r_zyx[2]) > 90 && (rad2deg * Obj->r_zyx[2]) <= 180 ) {
							env_vector.NormalDir[0] = env_vector.NormalDir[0] * (-1);
							env_vector.NormalDir[1] = env_vector.NormalDir[1] * (-1);
						}
						else if ((rad2deg * Obj->r_zyx[2]) < 0 && (rad2deg * Obj->r_zyx[2]) >= -90) {
							env_vector.NormalDir[2] = env_vector.NormalDir[0] * (-1);
							env_vector.NormalDir[3] = env_vector.NormalDir[1] * (-1);
						}
						else if ((rad2deg * Obj->r_zyx[2]) < -90 && (rad2deg * Obj->r_zyx[2]) >= -180) {
							env_vector.NormalDir[1] = env_vector.NormalDir[0] * (-1);
							env_vector.NormalDir[2] = env_vector.NormalDir[3] * (-1);
						}

						/* Check if normals are orthogonal to NearestPointDir */
						for (int k = 0; k < 4; k++) {
							/* Calculate result of normals + NearestPointDir (if 180 then 'ortho') */
							env_vector.NormalsCombined[k] = env_vector.NormalDir[k] - TrfObj[i].NearestPointDir_Fr0[j];

							// Debug Output
							if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1) {
								Log("TO_Point(%f|%f) , Sensor_Point(%f|%f), Angle between %f\n",
									TrfObj[i].boundary_points[k][0],
									TrfObj[i].boundary_points[k][1],
									(Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]),
									(Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]),
									TrfObj[i].boundary_pointsAngles[j][k]);
								Log("DirNo. %d: NormalDirection %f  -  NearestPointDirection %f  =  CombinedDirection %f\n",
									k, env_vector.NormalDir[k], TrfObj[i].NearestPointDir_Fr0[j], env_vector.NormalsCombined[k]);
							}

							if ( env_vector.NormalsCombined[k] >= (180 - 0.001) && env_vector.NormalsCombined[k] <= (180 + 0.001) ) {
								//Debug Output
								if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
									Log("Reflection point @ %f\n", pOSO->NearPnt.ds_p);

								/* Increase counter in case of orthogonal match of one sensor */
								cnt++;

								/* Mapping Data of one sensor */
								int_var.distance[cnt - 1]			= pOSO->NearPnt.ds_p;
								int_var.Detc_TO_ID[cnt - 1]			= i;
								int_var.Detc_TO_Surface_ID[cnt - 1] = k;
								int_var.Fr0_position_NP[cnt - 1][0] = SensData[j].Fr0_position_Sens[0] + pOSO->NearPnt.ds[0];
								int_var.Fr0_position_NP[cnt - 1][1] = SensData[j].Fr0_position_Sens[1] + pOSO->NearPnt.ds[1];
								int_var.NPDir_Fr0[cnt - 1]			= TrfObj[i].NearestPointDir_Fr0[j];
							}
							else {
								// Debug Output
								if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
									Log("No reflection point\n");
							}
						} // End of k-loop (4)

						int_var.noTargets = cnt;
						/* Set rest of cnt's distances to '0' */
						for (int k = cnt; k < 20; k++) {
							int_var.distance[k] = 0;
							int_var.Detc_TO_ID[k] = 0;
							int_var.Detc_TO_Surface_ID[k] = 0;
							int_var.Fr0_position_NP[k][0] = 0;
							int_var.Fr0_position_NP[k][1] = 0;
							int_var.NPDir_Fr0[k] = 0;
						}

						/* Calculate min/max angles for each TO */
						TrfObj[i].boundary_pointsAngles_min[j] = TrfObj[i].boundary_pointsAngles[j][0];
						TrfObj[i].boundary_pointsAngles_max[j] = TrfObj[i].boundary_pointsAngles[j][0];
						for (int k = 1; k < 4; k++) {
							if (TrfObj[i].boundary_pointsAngles[j][k] < TrfObj[i].boundary_pointsAngles_min[j])
								TrfObj[i].boundary_pointsAngles_min[j] = TrfObj[i].boundary_pointsAngles[j][k];
							if (TrfObj[i].boundary_pointsAngles[j][k] > TrfObj[i].boundary_pointsAngles_max[j])
								TrfObj[i].boundary_pointsAngles_max[j] = TrfObj[i].boundary_pointsAngles[j][k];
						}

						if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
							Log("min=%f | max=%f\n", TrfObj[i].boundary_pointsAngles_min[j], TrfObj[i].boundary_pointsAngles_max[j]);
					} // if-case (running)
				} // End of traffic-loop (nTC)
			} // if-case (nTC>0) [EVALUATION]



			/* ******************************* */
			/* Evaluation OCCLUCISON / SORTING */
			/* ******************************* */
			cnt = 1;
			for (int k = 0; k < int_var.noTargets; k++) {
				double min = 9999;

				/* Determine next target entry */
				for (int l = 0; l < int_var.noTargets; l++) {
					if (int_var.distance[l] < min) {
						min = int_var.distance[l];
						int_var.ID = l;
					}
				}

				if (k == 0) {
					SensData[j].distance[k] = int_var.distance[int_var.ID];
					SensData[j].Detc_TO_ID[k] = int_var.Detc_TO_ID[int_var.ID];
					SensData[j].Detc_TO_Surface_ID[k] = int_var.Detc_TO_Surface_ID[int_var.ID];
					SensData[j].Fr0_position_NP[k][0] = int_var.Fr0_position_NP[int_var.ID][0];
					SensData[j].Fr0_position_NP[k][1] = int_var.Fr0_position_NP[int_var.ID][1];
					SensData[j].NPDir_Fr0[k] = int_var.NPDir_Fr0[int_var.ID];

					int_var.distance[int_var.ID] = 9999;
				}
				else {
					/* Starting one after least filled target in list 'k' */
					for (int l = 0; l < k; l++) {
						if (TrfObj[SensData[j].Detc_TO_ID[l]].boundary_pointsAngles_min[j] > TrfObj[int_var.ID].NearestPointDir_Fr0[j] * (-1)
							&& TrfObj[SensData[j].Detc_TO_ID[l]].NearestPointDir_Fr0[j] * (-1) > TrfObj[int_var.ID].boundary_pointsAngles_max[j]) {

							SensData[j].distance[cnt] = int_var.distance[int_var.ID];
							SensData[j].Detc_TO_ID[cnt] = int_var.Detc_TO_ID[int_var.ID];
							SensData[j].Detc_TO_Surface_ID[cnt] = int_var.Detc_TO_Surface_ID[int_var.ID];
							SensData[j].Fr0_position_NP[cnt][0] = int_var.Fr0_position_NP[int_var.ID][0];
							SensData[j].Fr0_position_NP[cnt][1] = int_var.Fr0_position_NP[int_var.ID][1];
							SensData[j].NPDir_Fr0[cnt] = int_var.NPDir_Fr0[int_var.ID];
						}
						else {
							int_var.distance[int_var.ID] = 9999;
							break; // Go out of loop
						}
					}

					if (int_var.distance[int_var.ID] != 9999)
						cnt++;
				}
			}
			// cnt = 0; //nur temporaer!!!
			for (int k = cnt; k < 20; k++) {
				SensData[j].distance[k] = 0;
				SensData[j].Detc_TO_ID[k] = -1;
				SensData[j].Detc_TO_Surface_ID[k] = 0;
				SensData[j].Fr0_position_NP[k][0] = 0;
				SensData[j].Fr0_position_NP[k][1] = 0;
				SensData[j].NPDir_Fr0[k] = 0;
			}

			/* Reset */
			cnt = 0;
		} // End of sensor-loop (nOS)

		// Debug Output
		if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
			Log("--\n");
	} // if-case (nOS>0)

    return 0;
}



/*
** User_Check_IsIdle ()
**
** Checking, if the simulation model is in idle conditions (stand still,
** steeringwheel angle zero, cluch pedal pressed, ...).
** If reached idle state, the calculation of vehicle model and driving
** manoevers is stopped.
** Ready for start new simulation.
**
** Return:
** 1  idle state reached
** 0  else
**
** Call:
** - in main task, in the main loop
** - pay attention to realtime condition
** - while SimCore.State==SCState_EndIdleGet
*/

int
User_Check_IsIdle (int IsIdle)
{
    double val;

    /*** ECU / carmodel signals */

    /* vehicle and wheels: stand still */
    val = 0.5*kmh2ms;
    if (Vehicle.v > val
     || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
     || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
	IsIdle = 0;
    }

    /* SteerAngle: drive  straight forward position */
    val = 1.0*deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val)
	IsIdle = 0;

    return IsIdle;
}



/*
** User_Out ()
**
** Assigns model quantities to variables of the i/o vector
**
** call:
** - in the main loop
** - pay attention to realtime condition
** - just before IO_Out();
*/

void
User_Out (const unsigned CycleNo)
{
#if !defined(LABCAR)
    RBS_OutMap(CycleNo);
#endif

    if (SimCore.State != SCState_Simulate)
	return;
}



/*
** User_ApoMsg_Eval ()
**
** Communication between the application and connected GUIs.
** Evaluate messages from GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
**    skips the message
**
** Return:
**   0 : message evaluated
**  -1 : message not handled
*/

int
User_ApoMsg_Eval (int Ch, char *Msg, int len, int who)
{
#if defined(CM_HIL)
    /*** FailSafeTester */
    if (Ch == ApoCh_CarMaker) {
	if (FST_ApoMsgEval(Ch, Msg, len) <= 0)
	    return 0;
    }

#endif
    return -1;
}



/*
** User_ApoMsg_Send ()
**
** Communication between the application and connected GUIs.
** Sends messages to GUIs
**
** Call:
** - near the end of the main loop, in MainThread_FinishCycle()
** - pay attention to realtime condition
*/

void
User_ApoMsg_Send (double T, const unsigned CycleNo)
{
}



/*
** User_ShutDown ()
**
** Prepare application for shut down
**
** Call:
** - at end of program
** - no realtime conditions
*/

int
User_ShutDown (int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
	IsDown = 1;
    }

    return IsDown;
}



/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int
User_End (void)
{
    return 0;
}



/*
** User_Cleanup ()
**
** Cleanup function of the User module
**
** Call:
** - one times at end of program, just before exit
** - no realtime conditions
*/

void
User_Cleanup (void)
{
	if (TrfObj != NULL)
		free(TrfObj);

	if (SensParam != NULL)
		free(SensParam);

	if (SensData != NULL)
		free(SensData);
}
