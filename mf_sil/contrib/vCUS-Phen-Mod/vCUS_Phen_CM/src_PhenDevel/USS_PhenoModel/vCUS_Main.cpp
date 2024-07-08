#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cstdlib>
#include <algorithm>

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include "PhenoModel.h"
#include "PhenoMapping.h"
#include "USSMathUtils.h"
#include <Vehicle\Sensor_Object.h>
#include "vCUS_mfPlot.h"

#include "us_drv_def_vfa.h"

extern int nTC, nOS;
static int PauseCheck = 0;
extern double null_vec_2D[2], null_vec_3D[3];

#ifdef vCUS_HIL_MODE
#define FS_OPT     3
extern US_DRV::VFA28::PdcmRecorderModeFrame *localPDCMframe;
uint64_t fire_scheme_tab[FS_OPT][_NO_OF_SENSORS_MAX_][_NO_OF_FIRESAMPLES_] = //;
          {{{ 13, 37, 23, 29, 41, 35, 20, 44 },
            { 15, 33, 23, 50, 41, 17, 35, 29 },
            { 16, 26, 33, 18, 39, 45, 24, 31 },
            { 13, 44, 20, 35, 41, 29, 23, 37 },
            { 15, 29, 35, 17, 41, 50, 23, 33 },
            { 20, 32, 46, 36, 26, 50, 30, 38 },
            { 13, 37, 23, 29, 41, 35, 20, 44 },
            { 15, 33, 23, 50, 41, 17, 35, 29 },
            { 16, 26, 33, 18, 39, 45, 24, 31 },
            { 13, 44, 20, 35, 41, 29, 23, 37 },
            { 15, 29, 35, 17, 41, 50, 23, 33 },
            { 20, 32, 46, 36, 26, 50, 30, 38 }}, {{0}}, {{0}}};

uint64_t fire_scheme[_NO_OF_SENSORS_MAX_][_NO_OF_FIRESAMPLES_];

volatile unsigned int fs_sel[2] = {0,0};
#endif

#ifdef _MATLAB_VALIDATION_
/*===matlab include stuff*/
#include <stdio.h>
#include "engine.h"
#include "matrix.h"
/*===*/

void
runMatlabValidation(TCHAR *mainPath) {
	/*runs MATLAB validation tool - see _MATLAB_VALIDATION_ in PhenoModel.h*/

	static char validationPath[256];
	Engine *ep;				    //MATLAB engine
	mxArray *mPth = NULL;		//MATLAB path to validation tool

	/* c string containing the path to validation tool*/
	sprintf(validationPath, "%s/../vCUS_Phen_Mat", mainPath);
	
	Log("vCUS: Starting MATLAB engine for Valiation tool!\n");

	/*start engine*/
	if (!(ep = engOpen(""))) {
		LogErrStr(EC_General, "Can't start MATLAB engine!\n");
		//return;
	}
	/**/

	/* Create a MATLAB variable for the path to tool*/
	mPth = mxCreateString(validationPath);

	/*Place the variable mPth into the MATLAB workspace*/
	engPutVariable(ep, "mPth", mPth);

	/*cahnge matlab current path to tool path and call tool*/
	engEvalString(ep, "cd(mPth)");
	Log("vCUS: Runing Validation tool!\n");
	engEvalString(ep, "readCSVsAll_auto();");

	/* When finished with the string array, free its memory and close engine*/
	mxDestroyArray(mPth);
	/*
	printf("Hit return to continue\n\n");
	fgetc(stdin);
	*/
	engClose(ep);
	Log("vCUS: Validation finished, see SimOutput/Validation/ValidationResults!\n");

}
#endif

int
GetCurrentEchoes(int i)
{
	int		g = 0;
	int		t = 0;
	int		a1 = 0;
	int		a2 = 0;
	int		Rx_ID = -1;
	double	SFSP_ToF_Diff = 0;
	uint64_t	last_RxVsTx_Duration;
	uint64_t	nextRx_SFSP;

	/* Evaluate how many echoes are writen in this SFSP and save them locally for ordering them */
	struct tmpSensData {
		double		ToF[_NO_OF_ECHOES_MAX_] = { 0 };
		bool		vTarget_Flag[_NO_OF_ECHOES_MAX_] = { 0 };
		double		Fr0_position_NP[_NO_OF_ECHOES_MAX_][3] = {{ 0 }};
		double		Fr1_position_NP[_NO_OF_ECHOES_MAX_][3] = { { 0 } };
		double		Fr0_position_Sens_Tx[_NO_OF_ECHOES_MAX_][3] = {{ 0 }};
		double		Tof_Counter[_NO_OF_ECHOES_MAX_] = { 0 };
		uint64_t	Tof_Counter_State[_NO_OF_ECHOES_MAX_] = { 0 };
		double 		echoAmplitude[_NO_OF_ECHOES_MAX_] = { 0 };
		uint64_t	BurstTime[_NO_OF_ECHOES_MAX_] = { 0 };
		int			Tx[_NO_OF_ECHOES_MAX_] = { 0 };
		int			Rx[_NO_OF_ECHOES_MAX_] = { 0 };
		int			Detc_TO_ID[_NO_OF_ECHOES_MAX_] = { 0 };
	} tmpSensData;

	/* Generate local copy of old SensData to be able to do sorting */
	tSensorData OldSensData = SensData[i];
	/* Run ReflectionGeneration to determine current targets after the old targets have been checked */
	vCUS_ReflectionGenerator(i);
	/* Generate local copy of new SensData to be able to do sorting */
	tSensorData NewSensData = SensData[i];
	/* Restore Old Sensor Pointer Struct */
	SensData[i].Target = OldSensData.Target;

	/* Save Number of Echoes locally | Reset all DataTrans_Runnings to 0 if no Echoes received */
	if (NewSensData.Target.ToF[0] != 0) {
		/* Check first ToF, because CM is storing the last nEchoes for the next Sim */
		SensData[i].Target.nReflections = NewSensData.Target.nReflections;
	}
	else {
		SensData[i].Target.nReflections = 0;
	}

	if (SimParameters->DebugOpt == -1) {
		Log("- - -\nSensor %d: Got %d new Echoes: ", i, SensData[i].Target.nReflections);
		for (int a = 0; a < NewSensData.Target.nReflections; a++) {
			Log("%d: %f  ", a, (NewSensData.Target.ToF[a]));
		}
		Log("\n");
	}

	/* Deriving overlapping target measurements from last SFSP before new mapping starts */
	/* This is necessary in terms of checking, if there are: */
	/* a) Old ToF in the memory, which have been expired */
	/* b) Old ToF is overlapping inside <NewSFSP> -- <2ms> as DeafTime */
	/* c) Old ToF that lasts longer than the upcoming NewSFSP and woud therefore be overwritten but still has to count */
	for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {

		Rx_ID = SensData[i].Target.Rx[k];

		/*if Rx bursting cycle >= timeStamp_ms take this burst cycle (direct echo & burst cycle has been updated and current echo is generated by the previous burst),
		else take NextSFSP[Rx_ID] - typically this is a cross echo - the CyclenNo has not been updated - the following Rx burst is in NextSFSP */
		nextRx_SFSP				= (ToFwait->CycleNo[Rx_ID] >= ToFwait->timeStamp_ms) ? ToFwait->CycleNo[Rx_ID] : ToFwait->NextSFSP[Rx_ID];
		last_RxVsTx_Duration	= nextRx_SFSP - SensData[i].Target.vCUSBurstTime[k];

		/* a) In case no echoes are received and DataTrans_Running is false, reset all variables */
		if ( (SensData[i].Target.ToF[k] != 0) && (ToFwait->DataTrans_Running[i][k] == 0) ) {

			#ifdef vCUS_HIL_MODE
				/*PDCM frame debug: sensorID+1*/
				if (SimParameters->DebugPDCM_Frame) {
					Log("##### RX %d & Tx %d: Expired ToF for Target  %d with amp %f and ToF %f\n",
						Rx_ID + 1,
						i + 1,
						k,
						SensData[i].Target.echoAmplitude[k],
						SensData[i].Target.ToF[k]);
					PdcmFlags->nEchoesExpired = PdcmFlags->nEchoesExpired + 1;
				}
			#endif

			if (SimParameters->DebugOpt == -1)
				Log("ContiUSS: Got ContiToF=%f but no DataTrans_Running! Reset variables for Sens_%d + Target_%d\n", SensData[i].Target.ToF[k], i, k);

			ToFwait->ToF_Counter[i][k] = 0;
			ToFwait->ToF_Counter_State[i][k] = 0;

			SensData[i].Target.ToF[k] = 0;
			VEC_Assign(SensData[i].Target.Fr0_position_NP[k], null_vec_3D);
			VEC_Assign(SensData[i].Target.Fr1_position_NP[k], null_vec_3D);
			VEC_Assign(SensData[i].Target.Fr0_position_Sens_Tx[k], null_vec_3D);
			SensData[i].Target.vTarget_Flag[k] = 0;
			SensData[i].Target.vCUSBurstTime[k] = 0;
			SensData[i].Target.Tx[k] = 0;
			SensData[i].Target.Rx[k] = 0;
			SensData[i].Target.echoAmplitude[k] = 0;
			SensData[i].Target.Detc_TO_ID[k] = 0;
		}
		/* b) Evaluation of SFSP & ToF if overlapping this SFSP within 2ms Deaftime */
		/*use Rx_ID deaf time not TxID (== i)*/
		
		//else if (((SensData[i].Target.ToF[k] /*Snapshot of previous SFSP*/ - SensData[Rx_ID].TimeStamp /*Last SFSP Duration*/) > 0)
		//	&& ((SensData[i].Target.ToF[k] /*Snapshot of previous SFSP*/ - SensData[Rx_ID].TimeStamp/*Last SFSP Duration*/) <= _Sensor_Deaf_Time_)) {

		else if (((SensData[i].Target.ToF[k] /*Snapshot of previous SFSP*/ - last_RxVsTx_Duration /*Last Rx vs Tx SFSP Duration*/) > 0)
			&& ((SensData[i].Target.ToF[k] /*Snapshot of previous SFSP*/ - last_RxVsTx_Duration/*Last Rx vs Tx SFSP Duration*/) <= _Sensor_Deaf_Time_)) {

			if (SimParameters->DebugOpt == -1) {
				SFSP_ToF_Diff = SensData[i].Target.ToF[k] - last_RxVsTx_Duration;
				Log("RX %d & Tx %d: Overlapping WITHIN Deaf-Time for Target %d with %fms | Delete Target Measurement\n", Rx_ID, i, k, SFSP_ToF_Diff);
			}
			#ifdef vCUS_HIL_MODE
				/*PDCM frame debug: sensorID+1*/
				if (SimParameters->DebugPDCM_Frame) {
					Log("##### RX %d & Tx %d: Overlapping WITHIN Deaf-Time for Target %d with amp %f and ToF %f | Delete Target Measurement\n", 
							Rx_ID + 1, 
							i + 1, 
							k, 
							SensData[i].Target.echoAmplitude[k],
							SensData[i].Target.ToF[k]);
					PdcmFlags->nDeafTimeEchos = PdcmFlags->nDeafTimeEchos + 1;
				}
			#endif

			/* Reset all variables for this detected target */
			ToFwait->DataTrans_Running[i][k] = 0;
			ToFwait->ToF_Counter[i][k] = 0;
			ToFwait->ToF_Counter_State[i][k] = 0;

			SensData[i].Target.ToF[k] = 0;
			VEC_Assign(SensData[i].Target.Fr0_position_NP[k], null_vec_3D);
			VEC_Assign(SensData[i].Target.Fr1_position_NP[k], null_vec_3D);
			VEC_Assign(SensData[i].Target.Fr0_position_Sens_Tx[k], null_vec_3D);
			SensData[i].Target.vTarget_Flag[k] = 0;
			SensData[i].Target.vCUSBurstTime[k] = 0;
			SensData[i].Target.Tx[k] = 0;
			SensData[i].Target.Rx[k] = 0;
			SensData[i].Target.echoAmplitude[k] = 0;
			SensData[i].Target.Detc_TO_ID[k] = 0;
		}
		/* c) Evaluation of SFSP & ToF if overlapping this SFSP over 2ms Deaftime */
		//else if (((SensData[i].Target.ToF[k]/*Snapshot of previous SFSP*/ - SensData[Rx_ID].TimeStamp/*Last SFSP Duration*/) > _Sensor_Deaf_Time_)) {

		else if (((SensData[i].Target.ToF[k]/*Snapshot of previous SFSP*/ - last_RxVsTx_Duration/*Last Rx vs Tx SFSP Duration*/) > _Sensor_Deaf_Time_)) {

			if (SimParameters->DebugOpt == -1) {
				SFSP_ToF_Diff = SensData[i].Target.ToF[k] - last_RxVsTx_Duration;
				Log("Sensor %d: Overlapping for Target %d with %fms\n", i, k, SFSP_ToF_Diff);
			}

			#ifdef vCUS_HIL_MODE
				/*PDCM frame debug: sensorID+1*/
				if (SimParameters->DebugPDCM_Frame) {
					Log("##### RX %d & Tx %d: Overlapping over nextSFSP + deafTime for Target  %d with amp %f and ToF %f \n",
						Rx_ID + 1,
						i + 1,
						k,
						SensData[i].Target.echoAmplitude[k],
						SensData[i].Target.ToF[k]);
					PdcmFlags->nOverDTimeEchoes = PdcmFlags->nOverDTimeEchoes + 1;
				}
			#endif

			tmpSensData.Tof_Counter[t] = ToFwait->ToF_Counter[i][k];
			ToFwait->ToF_Counter[i][k] = 0;
			tmpSensData.Tof_Counter_State[t] = ToFwait->ToF_Counter_State[i][k];
			ToFwait->ToF_Counter_State[i][k] = 0;

			/* Make local copy of SensorData to be able to reorder them*/
			tmpSensData.ToF[t] = SensData[i].Target.ToF[k]; // TBD/CHECK: ToFNew-ToFOld for getting new overlapping timespan
			SensData[i].Target.ToF[k] = 0;
			VEC_Assign(tmpSensData.Fr0_position_NP[t], SensData[i].Target.Fr0_position_NP[k]);
			VEC_Assign(SensData[i].Target.Fr0_position_NP[k], null_vec_3D);
			VEC_Assign(tmpSensData.Fr1_position_NP[t], SensData[i].Target.Fr1_position_NP[k]);
			VEC_Assign(SensData[i].Target.Fr1_position_NP[k], null_vec_3D);
			VEC_Assign(tmpSensData.Fr0_position_Sens_Tx[t], SensData[i].Target.Fr0_position_Sens_Tx[k]);
			VEC_Assign(SensData[i].Target.Fr0_position_Sens_Tx[k], null_vec_3D);
			tmpSensData.vTarget_Flag[t] = SensData[i].Target.vTarget_Flag[k];
			SensData[i].Target.vTarget_Flag[k] = 0;
			tmpSensData.BurstTime[t] = SensData[i].Target.vCUSBurstTime[k];
			SensData[i].Target.vCUSBurstTime[k] = 0;
			tmpSensData.Tx[t] = SensData[i].Target.Tx[k];
			SensData[i].Target.Tx[k] = 0;
			tmpSensData.Rx[t] = SensData[i].Target.Rx[k];
			SensData[i].Target.Rx[k] = 0;
			tmpSensData.echoAmplitude[t] = SensData[i].Target.echoAmplitude[k];
			SensData[i].Target.echoAmplitude[k] = 0;
			tmpSensData.Detc_TO_ID[t] = SensData[i].Target.Detc_TO_ID[k];
			SensData[i].Target.Detc_TO_ID[k] = 0;

			t++;
			a1++;
		}
	}

	/* Reset for next loop */
	t = 0;

	/* Check ToFs of current SFSP */
	for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {

		/* Check if <NewTargetToF> is smaller than <LastTargetTof - LastSFSP> | Means, new target measurement is smaller than old ones*/
		if ((((NewSensData.Target.ToF[k]) <= tmpSensData.ToF[t]) || (tmpSensData.ToF[t] == 0))
			&& (NewSensData.Target.ToF[k] != 0)) {

			/* Set local variables as starting point for new target measurement */
			ToFwait->DataTrans_Running[i][g] = 1;
			ToFwait->ToF_Counter[i][g] = 0;
			ToFwait->ToF_Counter_State[i][g] = 0;

			/* Copy current sim data from CarMaker */
			SensData[i].Target.ToF[g] = NewSensData.Target.ToF[k];
			VEC_Assign(SensData[i].Target.Fr0_position_NP[g], NewSensData.Target.Fr0_position_NP[k]);
			VEC_Assign(SensData[i].Target.Fr1_position_NP[g], NewSensData.Target.Fr1_position_NP[k]);
			VEC_Assign(SensData[i].Target.Fr0_position_Sens_Tx[g], NewSensData.Target.Fr0_position_Sens_Tx[k]);
			SensData[i].Target.vTarget_Flag[g] = NewSensData.Target.vTarget_Flag[k];
			SensData[i].Target.Tx[g] = NewSensData.Target.Tx[k];
			SensData[i].Target.Rx[g] = NewSensData.Target.Rx[k];
			SensData[i].Target.echoAmplitude[g] = NewSensData.Target.echoAmplitude[k];
			SensData[i].Target.Detc_TO_ID[g] = NewSensData.Target.Detc_TO_ID[k];
			SensData[i].Target.vCUSBurstTime[g] = ToFwait->timeStamp_ms;

			if (SimParameters->DebugOpt == -1)
				Log("Sensor %d: NEW ToF for Target %d: %f\n", i, g, SensData[i].Target.ToF[g]);

			a2++;
		}
		else if ((((NewSensData.Target.ToF[k]) > tmpSensData.ToF[t]) && (tmpSensData.ToF[t] != 0))
			|| ((NewSensData.Target.ToF[k] == 0) && (tmpSensData.ToF[t] != 0))) {

			/* Set local variables according to old target measurement */
			ToFwait->DataTrans_Running[i][g] = 1;
			ToFwait->ToF_Counter[i][g] = tmpSensData.Tof_Counter[t];
			ToFwait->ToF_Counter_State[i][g] = tmpSensData.Tof_Counter_State[t];

			/* Copy old sim data from tmp-var*/
			SensData[i].Target.ToF[g] = tmpSensData.ToF[t];
			VEC_Assign(SensData[i].Target.Fr0_position_NP[g], tmpSensData.Fr0_position_NP[t]);
			VEC_Assign(SensData[i].Target.Fr1_position_NP[g], tmpSensData.Fr1_position_NP[t]);
			VEC_Assign(SensData[i].Target.Fr0_position_Sens_Tx[g], tmpSensData.Fr0_position_Sens_Tx[t]);
			SensData[i].Target.vTarget_Flag[g] = tmpSensData.vTarget_Flag[t];
			SensData[i].Target.vCUSBurstTime[g] = tmpSensData.BurstTime[t];
			SensData[i].Target.Tx[g] = tmpSensData.Tx[t];
			SensData[i].Target.Rx[g] = tmpSensData.Rx[t];
			SensData[i].Target.echoAmplitude[g] = tmpSensData.echoAmplitude[t];
			SensData[i].Target.Detc_TO_ID[g] = tmpSensData.Detc_TO_ID[t];

			if (SimParameters->DebugOpt == -1)
				Log("Sensor %d: OLD ToF for Target %d: %f <--\n", i, g, SensData[i].Target.ToF[g]);

			t++;
			k--;
		}
		else {

			/* Reset all variables */
			ToFwait->DataTrans_Running[i][g] = 0;
			ToFwait->ToF_Counter[i][g] = 0;
			ToFwait->ToF_Counter_State[i][g] = 0;
			SensData[i].Target.vCUSBurstTime[g] = 0;

			SensData[i].Target.ToF[g] = 0;
			VEC_Assign(SensData[i].Target.Fr0_position_NP[g], null_vec_3D);
			VEC_Assign(SensData[i].Target.Fr1_position_NP[g], null_vec_3D);
			VEC_Assign(SensData[i].Target.Fr0_position_Sens_Tx[g], null_vec_3D);
			SensData[i].Target.vTarget_Flag[g] = 0;
			SensData[i].Target.Tx[g] = 0;
			SensData[i].Target.Rx[g] = 0;
			SensData[i].Target.echoAmplitude[g] = 0;
			SensData[i].Target.Detc_TO_ID[g] = 0;

			if (SimParameters->DebugOpt == -1)
				Log("Sensor %d: NO ToF for Target %d\n", i, g);

		}
		g++;
		if (g >= _NO_OF_ECHOES_MAX_) {
			if ((a1 + SensData[i].Target.nReflections) != (t + a2))
				Log("WARNING - vCUS: Number of max reflections per sensor reached: Max->%d | Got->%d \nI'll discard the remaining %d echoes!\n", _NO_OF_ECHOES_MAX_, (a1 + SensData[i].Target.nReflections), (a1 + SensData[i].Target.nReflections)- _NO_OF_ECHOES_MAX_);

			return 0;
		}
	}

	if ((a1 + SensData[i].Target.nReflections) != (t + a2))
		LogErrF(EC_Sim, "vCUS: Number of GroundTruth echoes and converted echoes not equal!\n");

	return 0;
}


int
SensDataTrans_Eval(int j)
{
	/* Non-static variables */
	double Sum_j = 0;	/* Used for evaluation if early return in this cycle due to no job */
	int BurstCheck = 0;	/* Used for evaluation if one sensor has already been used */

	/* Reset variables at TestRun end */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		for (int i = 0; i < nOS; i++) {
			for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
				ToFwait->ToF_Counter[i][k] = 0;
				ToFwait->DataTrans_Running[i][k] = 0;
			}
		}
		return 0;
	}


	if (SimParameters->DebugOpt == -1) {
		Log("\nSensDataTrans_Eval for SensorNo %d -> CycleNo %d\nToF-Counter:\n---\nSensors   0   1   2   3   4   5   6   7   8   9   10  11\n", j, ToFwait->timeStamp_ms);
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			Log("Targ %d | %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f\n", k,
				ToFwait->ToF_Counter[0][k], ToFwait->ToF_Counter[1][k], ToFwait->ToF_Counter[2][k], ToFwait->ToF_Counter[3][k], ToFwait->ToF_Counter[4][k],
				ToFwait->ToF_Counter[5][k], ToFwait->ToF_Counter[6][k], ToFwait->ToF_Counter[7][k], ToFwait->ToF_Counter[8][k], ToFwait->ToF_Counter[9][k],
				ToFwait->ToF_Counter[10][k], ToFwait->ToF_Counter[11][k]);
		}
		Log("++++++++++++++++++++++++++++++\n");
		Log("DataTrans_Runnings:\n---\nSensors   0   1   2   3   4   5   6   7   8   9   10  11\n", j, ToFwait->timeStamp_ms);
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			Log("Targ %d | %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f\n", k,
				ToFwait->DataTrans_Running[0][k], ToFwait->DataTrans_Running[1][k], ToFwait->DataTrans_Running[2][k], ToFwait->DataTrans_Running[3][k], ToFwait->DataTrans_Running[4][k],
				ToFwait->DataTrans_Running[5][k], ToFwait->DataTrans_Running[6][k], ToFwait->DataTrans_Running[7][k], ToFwait->DataTrans_Running[8][k], ToFwait->DataTrans_Running[9][k],
				ToFwait->DataTrans_Running[10][k], ToFwait->DataTrans_Running[11][k]);
		}
		Log("++++++++++++++++++++++++++++++\n");
	}


	/* Check if DataTrans_Eval hasn't to be executed (if -> no Sensor-NewTrigger or no Sensor-Running) */
	for (int i = 0; i < nOS; i++) {
		/* Sum up the amount of current running tasks */
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			Sum_j = Sum_j + ToFwait->DataTrans_Running[i][k];
		}
	}
	/* Check if NewTrigger is false (-1 -> No Sensor Burst) and Sum_j is bigger than 0 (There's still at least one ToF in the air, so don't abort) */
	if (j == -1 && Sum_j == 0) {
		if (SimParameters->DebugOpt == -1)
			Log("Return in this cycle (no new Calls or DataTrans_Running)\n- - - - - -\n");

		Sum_j = 0;
		return 0;
	}


	/* Check if and which Sensor-NewTrigger or Sensor-Running to consider*/
	for (int i = 0; i < nOS; i++) {

		/* Only execute "GetCurrentEchoes()" once per Sensor / Burst | Will be set to 0 @end of k-loop */
		BurstCheck = 1;

		/* Check all current Echoes */
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {

			/* Evaluate ToFs if the BURSTING Sensor (j) matches the current loop-var (i) OR there's still one ToF for Sensor (i) in the air */
			if ((ToFwait->DataTrans_Running[i][k] == 1) || (j == i)) {

				if (SimParameters->DebugOpt == -1)
					Log("Sens_%d, Target_%d: ToF_Counter_State=%d\n", i, k, ToFwait->ToF_Counter_State[i][k]);

				/* If no DataTrans is running right now, get the echo information for this SFSP and save them LOCALLY (no write down to quantities yet) */
				// if (ToFwait->DataTrans_Running[i][k] == 0 && BurstCheck == 1)
				if (i==j && BurstCheck == 1)
					GetCurrentEchoes(i);

				/* ***************************************** */
				/* ****** Wait for ToF to be finsihed ****** */
				/* ***************************************** */
				if ( (SensData[i].Target.ToF[k] > ToFwait->ToF_Counter[i][k])
					&& (ToFwait->timeStamp_ms != ToFwait->ToF_Counter_State[i][k]) ) {

					if (SimParameters->DebugOpt == -1)
						Log("Tof-Counter (SensNo %d, EchoNo %d, StartCycle %d, StopCycle %f): Cnt = %f, ContiToF = %f\n",
							i, k, SensData[i].Target.vCUSBurstTime[k],
							(SensData[i].Target.vCUSBurstTime[k] + SensData[i].Target.ToF[k]),
							ToFwait->ToF_Counter[i][k],
							(SensData[i].Target.ToF[k]));

					ToFwait->ToF_Counter[i][k]++;
				}
				else if ((SensData[i].Target.ToF[k] <= ToFwait->ToF_Counter[i][k])
					&& (ToFwait->DataTrans_Running[i][k] == 1)
					&& (ToFwait->timeStamp_ms != ToFwait->ToF_Counter_State[i][k])) {

					if (SimParameters->DebugOpt == -1)
						Log("Tof-Counter (SensNo %d, EchoNo %d, StartCycle %d, StopCycle %f): Cnt = %f, ContiToF = %f  --> Time is up!!!\n",
							i, k, SensData[i].Target.vCUSBurstTime[k],
							(SensData[i].Target.vCUSBurstTime[k] + SensData[i].Target.ToF[k]),
							ToFwait->ToF_Counter[i][k],
							(SensData[i].Target.ToF[k]));

					if ((SensData[i].Target.vCUSBurstTime[k] + ToFwait->ToF_Counter[i][k]) != ToFwait->timeStamp_ms)
						LogErrF(EC_Sim, "vCUS: ToF-Counter (%f) StopCycle (%f) does not match current cycle (%d)\n",
							ToFwait->ToF_Counter[i][k], (SensData[i].Target.vCUSBurstTime[k] + ToFwait->ToF_Counter[i][k]), ToFwait->timeStamp_ms);

					/* Mapping: Whenever a ToF Counter/ State has reached it's final value */
					ReflData_Mapping(k /* Current echo */, i /* Receiver ID */, 0 /* BurstFlag -> RECV */);

					ToFwait->ToF_Counter[i][k] = 0;
					ToFwait->DataTrans_Running[i][k] = 0;
				}

				/* Set to 0 to execute "GetCurrentEchoes()" only once per Sensor / Burst */
				BurstCheck = 0;

				/* Set variable to current cycle. This loop won't be executed twice in this cycle */
				if ((j == -1) || ((ToFwait->DataTrans_Running[i][k] == 1) && (ToFwait->ToF_Counter[i][k] == 1))) {
					ToFwait->ToF_Counter_State[i][k] = ToFwait->timeStamp_ms;
					if (SimParameters->DebugOpt == -1)
						Log("Got new ToF_Counter_State=%f for Sens_%d Target_%d\n", ToFwait->ToF_Counter_State[i][k], i, k);
				}
			}
		}
	}

	if (SimParameters->DebugOpt == -1)
		Log("---------------------------------------\n");

	return 0;
}


bool
SFSP_Trigger(void)
{
	static int k[_NO_OF_SENSORS_MAX_] = { 0 };
	int _SensorCalc = 0, a = 0;
	uint64_t ErrorFlag = 0;

	if (ToFwait->SimCoreStartCycleFlag)
		for (int j = 0; j < nOS; j++) { k[j] = 0; };

	/* Reset variables at TestRun end */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		for (int j = 0; j < nOS; j++) { k[j] = 0; }
		return false;
	}

	/* Executed during runtime */
	for (int i = 0; i < nOS; i++) {
		if (ToFwait->NextSFSP[i] == ToFwait->timeStamp_ms) { // Check for each sensor if next exec is yet reached

			if (SimParameters->DebugOpt > 0)
				Log("\n\nCurrent Cycle: %d\n", ToFwait->timeStamp_ms);

			/* Calculate next sample point for the ContiUSS_SensorCalc to be executed based on parameterset */
         #ifndef vCUS_HIL_MODE
			ToFwait->NextSFSP[i] = ToFwait->NextSFSP[i] + SimParameters->fire_scheme[i][k[i]];
         #else
			ToFwait->NextSFSP[i] = ToFwait->NextSFSP[i] + fire_scheme[i][k[i]];
         #endif /* vCUS_HIL_MODE */

			/* Avoid execution for the first SimCore.CycleNo */
			if (ToFwait->SimCoreStartCycle - ToFwait->timeStamp_ms) {
				k[i] == 0 ? a = 7 : a = k[i] - 1;
				SensData[i].TimeStamp = ToFwait->timeStamp_ms - ToFwait->CycleNo[i];
            #ifndef vCUS_HIL_MODE
				ErrorFlag = SensData[i].TimeStamp - SimParameters->fire_scheme[i][a];
            #else
				ErrorFlag = SensData[i].TimeStamp - fire_scheme[i][a];
            #endif /* vCUS_HIL_MODE */
			}

			/* Set flag for sensor calculation */
			SensData[i].SFSP_Flag = 1;
			SensData[i].SFSP_cnt = k[i];

			/* Print additional information */
			if (SimParameters->DebugOpt > 0)
            #ifndef vCUS_HIL_MODE
				Log("Sensor=%d , Next Execution=%f , OldCycleNo=%d , ContiUSS[i].TimeStamp=%f, NewFireScheme=%f\n", i, ToFwait->NextSFSP[i], ToFwait->CycleNo[i], SensData[i].TimeStamp, SimParameters->fire_scheme[i][k[i]]);
            #else
				Log("Sensor=%d , Next Execution=%f , OldCycleNo=%d , ContiUSS[i].TimeStamp=%f, NewFireScheme=%f\n", i, ToFwait->NextSFSP[i], ToFwait->CycleNo[i], SensData[i].TimeStamp, fire_scheme[i][k[i]]);
            #endif /* vCUS_HIL_MODE */
			if (ErrorFlag != 0)
				LogErrF(EC_Init, "ContiUSS: Time stamp doesn't fit to fire scheme! Error Value: %f\n", ErrorFlag);

			/* Set vairables for next sample point e.g. remeber old CycleNo value*/
			ToFwait->CycleNo[i] = ToFwait->timeStamp_ms;
			k[i] < 7 ? k[i]++ : k[i] = 0;
			_SensorCalc++;
		}
	}

	/* If one or more sensors should be calculated _SensorCalc != 0 */
	if (_SensorCalc >= 1) {
		return true;
	}
	else {
		return false;
	}
}


int
SFSP_Eval(void)
{
	for (int j = 0; j < nOS; j++) {
		/* Evaluate if SFSP trigger has been used */
		if (SensData[j].SFSP_Flag == 1) {
			/* Call function according to what sensor has its Burst */
			SensDataTrans_Eval(j);

			/* Reset internal trigger status bit */
			SensData[j].SFSP_Flag = 0;
		}
	}

	return 0;
}


int
vCUS_TestRun_Start_Finalize(void)
{
	int timingDelay = 1; 

	#ifdef vCUS_HIL_MODE
		timingDelay = 12;
		ToFwait->timeStamp_ms = 0;
		//PdcmFlags->syncCnt = 0;
	#endif /* vCUS_HIL_MODE */

	/* Set starting conditions for timing */
	for (int i = 0; i < nOS; i++) {
		ToFwait->NextSFSP[i] = ToFwait->timeStamp_ms + timingDelay;
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			ToFwait->DataTrans_Running[i][k] = 0;
		}
	}
	ToFwait->SimCoreStartCycle = ToFwait->timeStamp_ms + timingDelay;		// Avoid execution of the SFSP_Trigger for the first SimCycle
	
	#ifndef vCUS_HIL_MODE
		IntReflData->ReflDataTimer = ToFwait->timeStamp_ms + SimParameters->USP_UpdateRate;	// Set usp execution counter for UsRawData writing pattern for first cycle -> Start counting (e.g. 40ms)
	#else
		IntReflData->ReflDataTimer = ToFwait->timeStamp_ms + 1;	// Set 1ms counter for PDCM Frame writing pattern for first cycle -> Start counting
	#endif

	return 0;
}



int
vCUS_Main(void) {
	/* Generate timestamp */
   #ifdef vCUS_HIL_MODE
   ToFwait->timeStamp_ms = ToFwait->timeStamp_ms + 1;
   #else
	ToFwait->timeStamp_ms = static_cast<uint64_t>(std::round(SimCore.Time * 1000));
   #endif /* vCUS_HIL_MODE */

	if (ToFwait->timeStamp_ms == ToFwait->SimCoreStartCycle) {
		ToFwait->SimCoreStartCycleFlag = true;
		PauseCheck = 0;
	}
	else {
		ToFwait->SimCoreStartCycleFlag = false;
	}

	#ifdef vCUS_HIL_MODE
		/*reset n of deaftime echoes every cycle*/
		PdcmFlags->nDeafTimeEchos = 0;
		PdcmFlags->nEchoesExpired = 0;
		PdcmFlags->nOverDTimeEchoes = 0;
	#endif

	/* Initialize structs */
	vCUS_InitReflGen();

	/* Call every cycle for running parallel to execution */
	if (SimCore.State == SCState_Simulate)
		SensDataTrans_Eval(-1);

	/* Evaluate if main sensor calculation routine should be executed based on stochastic code */
	if ( (SimCore.State == SCState_Simulate) && (SFSP_Trigger() /* True for sample times in parameterset */) ) {
		SFSP_Eval(); // Executed, if burst of sensor happend
	}
	else if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		
		/* Reset variables at simulation end */
		for (int j = 0; j < nOS; j++) {
			SensDataTrans_Eval(j);
		}
		SFSP_Trigger();
		vCUS_ResetVarTREnd();
		ReflData_Mapping(0,0,0);
	#ifdef _UsRaw_DEBUG_
			WriteUsRaw(0); /* Reset static counter for file generation */
	#endif // _UsRaw_DEBUG_
	}

	if ( (SimCore.State == SCState_Simulate) && (IntReflData->ReflDataTimer <= ToFwait->timeStamp_ms) ) {
			
	#ifndef vCUS_HIL_MODE
			if (SimParameters->DebugOpt == -2)
				Log("USP-Exec-Timer for CycleNo %d\n", ToFwait->timeStamp_ms);
			IntReflData->ReflDataTimer = ToFwait->timeStamp_ms + SimParameters->USP_UpdateRate; // Set for next usp execution cycle (e.g. 40ms)
			UsRawEcho_Mapping(IntReflData->ReflDataCnt);
			uspExec_counter++;
	#else
			if (SimParameters->DebugOpt == -2)
				Log("1ms-Timer for CycleNo %ld\n", ToFwait->timeStamp_ms);
			IntReflData->ReflDataTimer = ToFwait->timeStamp_ms + 1; // Set for next 1ms cycle
			PDCMFrame_Mapping(IntReflData->ReflDataCnt);

			/*increment syncCnt*/
			PdcmFlags->syncCnt = (uint8_t)((PdcmFlags->syncCnt + 1) % _Sync_Cnt_Max_);
	#endif

		/* Reset mapping */
		//memset(ReflectionData, 0, sizeof(ReflectionData));
		memset(ReflectionData, 0, us_drv::US_DRV_Consts::US_DRV_MAX_NUM_DETECTIONS * sizeof(tReflectionData));
		IntReflData->ReflDataCnt = 0;
	}

#ifdef _MATLAB_VALIDATION_
	/*== not >=  :only call at end, not endIdle, endClean and endWait*/
	if (SimCore.State == SCState_End && SimCore.State != SCState_Pause) {
		/* Get system data */
		TCHAR Buffer[BUFSIZE];
		DWORD dwRet = GetCurrentDirectory(BUFSIZE, Buffer);
		if (dwRet == 0) {
			LogErrStr(EC_General, "GetCurrentDirectory failed!\n");
		}
		/*Call to MATLAB validation tool if SimEnd */
		runMatlabValidation(Buffer);
	}
#endif

	return 0;
}



/* To be used in HiL mode only */
#ifdef vCUS_HIL_MODE
void vCUS_GetPDCMFrame(unsigned char *pPDCMFrBuf)
{
    //int ret = 0;

    if(pPDCMFrBuf != NULL){
        for(int idx = 0; idx < 2; idx++){  
            memcpy((uint8_t *)(pPDCMFrBuf + idx * sizeof(US_DRV::VFA28::PdcmRecorderModeFrame)), 
                   localPDCMframe[idx].rawData, sizeof(US_DRV::VFA28::PdcmRecorderModeFrame));
        }
    }

    return; //ret;
}


void vCUS_SetFireScheme(uint64_t *fsMap, unsigned int fs)
{
    //int ret = 0;

    if((fsMap != NULL) && (fs < FS_OPT)){
        memcpy((unsigned char *)(fire_scheme_tab[fs]), (unsigned char *)fsMap, sizeof(uint64_t) * _NO_OF_FIRESAMPLES_ * _NO_OF_SENSORS_MAX_);
	}

	#ifdef LOG_FS
	/* Log fire scheme */
    Log("fs_wr: %d\n", fs);
    for (int i = 0; i < _NO_OF_SENSORS_MAX_; i++) {
        for (int j = 0; j < _NO_OF_FIRESAMPLES_; j++) {
    	    Log("%d ", (int) fire_scheme_tab[fs][i][j]);
        }
        Log("\n");
    }
    Log("\n");
	#endif  /* LOG_FS */

    return; //ret;
}


void vCUS_SelFireScheme(unsigned int fs, unsigned int bus)
{
    //int ret = 0;

    if((fs < FS_OPT) && (bus < 2)){
        fs_sel[bus] = fs;
        memcpy((unsigned char *)(fire_scheme[bus * 6]), (unsigned char *)(fire_scheme_tab[fs][bus * 6]), sizeof(uint64_t) * _NO_OF_FIRESAMPLES_ * (_NO_OF_SENSORS_MAX_/2));
	}else{
		fs_sel[bus] = 0; //default fire scheme
		memcpy((unsigned char *)(fire_scheme[bus * 6]), (unsigned char *)(fire_scheme_tab[fs][bus * 6]), sizeof(uint64_t) * _NO_OF_FIRESAMPLES_ * (_NO_OF_SENSORS_MAX_/2));
	}
    
	#ifdef LOG_FS
	/* Log fire scheme */
    Log("fs_req: %d bus: %d\n", fs_sel[bus], bus);
    for (int i = 0; i < _NO_OF_SENSORS_MAX_; i++) {
        for (int j = 0; j < _NO_OF_FIRESAMPLES_; j++) {
    	    Log("%d ", (int) fire_scheme[i][j]);
        }
        Log("\n");
    }
    Log("\n");
    #endif  /* LOG_FS */

    return; //ret;
}
#endif