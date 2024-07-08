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

#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include <CarMaker.h>
#pragma warning( pop )
#include <Car/Vehicle_Car.h>

#include <ADASRP.h>
#pragma warning(push)
#pragma warning(disable: 4201)
#include <rbs.h>
#pragma warning(pop)

#include "IOVec.h"
#include "User.h"
#include "mdl_APCtrl.h"
#include "VehTurnCntr.h"
#include "Mdl_RearSteering.h"
#include "PhenoModel.h"
#ifdef MOCO_REPLACES_LODMC
#include "OCTAGONBrake.h"
#include "OCTAGONEngine.h"
#endif
#ifdef PLP_LOG_ENABLED
#include "CarMakerLogger.h"
#include "log_config.h"
#endif

#ifndef VARIANT_CUS_ONLY
#include "cem_carmaker_user.h"
#endif
/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */


int UserCalcCalledByAppTestRunCalc = 0;


tUser	User;



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
    argc; /*silence unreferenced warning*/
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
#ifndef VARIANT_CUS_ONLY
    CarMaker_CEM_Init(); //eCAL initial
#endif
#ifdef PLP_LOG_ENABLED
    PLP_LOG_SET_LOGGER_INST(&loggerInstance(logger::DEFAULT_LOGGER));
    logger::loggerInstance(logger::DEFAULT_LOGGER).add(logger::CarMakerLogger::getInstance());
    logger::loggerInstance(logger::DEFAULT_LOGGER).init();
#endif

    return 0;
}



int
User_Register (void)
{
    VehicleControl_Register_mdl_APCtrl();
    VehicleControl_Register_VhclTurnCntr();
	Steering_Register_MyModel();
#ifdef MOCO_REPLACES_LODMC
    Brake_Register_OCTAGONBrake();
    Engine_Register_OCTAGONEngine();
#endif
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
#ifndef VARIANT_CUS_ONLY
    CEM200_DeclQuants(); //eCAL display
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
    vCUS_InitData();
#ifdef PLP_LOG_ERRORS_TO_WARNINGS
    // In case of simulation errors, do not abort simulation but show a warning
    LogDefineErrClass(EC_Sim, LogAction_warn, "", 0);
    // Allow 10000 (instead of 3) warnings per dt = 5 ms before the simulation is aborted.
    LogDefineWarnLimit(EC_Sim, 10000, 5.0);
#endif
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
    vCUS_TestRun_Start_Finalize();
#ifndef VARIANT_CUS_ONLY
    CarMaker_CEM_TestRun_Start(); //eCal running
#endif
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
    dt; /*silence unreferenced warning*/
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
#ifndef VARIANT_CUS_ONLY
    if (SimCore.State == SCState_EndWait || SimCore.State == SCState_EndClean)
    {
        CarMaker_CEM_TestRun_End(); //eCAL end
    }
#endif
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
    CycleNo;/*silence unreferenced warning*/
    if (SimCore.State != SCState_Simulate)
	    return;
#ifndef VARIANT_CUS_ONLY 
    if (SimCore.State == SCState_Simulate)
        fill_cem200(); //filling eCal Node
#endif
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
    dt;/*silence unreferenced warning*/
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
    dt;/*silence unreferenced warning*/
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
    dt; /*silence unreferenced warning*/
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
    dt; /*silence unreferenced warning*/
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
    dt; /*silence unreferenced warning*/
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/
#ifndef VARIANT_CUS_ONLY
    CarMaker_CEM_Calc(); //eCAL cyclic calling
#endif

    //vCUS_Main(); --> called in mdl_APCtrl.cpp to avoid 40ms delay in forwarding UsRaw to us_processing

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
    who; /*silence unreferenced warning*/
    Ch;  /*silence unreferenced warning*/
    Msg; /*silence unreferenced warning*/
    len; /*silence unreferenced warning*/
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
    T; /*silence unreferenced warning*/
    CycleNo; /*silence unreferenced warning*/
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
    ShutDownForced; /*silence unreferenced warning*/
    int IsDown = 0;

#ifndef VARIANT_CUS_ONLY
    CarMaker_CEM_Shutdown(); //eCAL shutdown and reset
#endif
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
    vCUS_Cleanup();
}
