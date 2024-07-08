/*
******************************************************************************
**  CarMaker - Version 7.1.2
**  Continental Teves AG & CO. oHG
**  - Stefan Hagenmueller -
**
******************************************************************************
** Functions
** ---------
**
**	ContiUSS_DeclQuants()
**	ContiUSS_Init()
**	ContiUSS_TestRun_Start_Finalize
**	ContiUSS_New()
**	ContiUSS_Calc()
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32) && !defined(INTIME)
#  include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <CarMaker.h>
#include <Car/Vehicle_Car.h>
#include <Car/Sensor_USonicRSI.h>
#include <Log.h>
#include <DataDict.h>
#include <DirectVarAccess.h>

#include "User.h"
#include "USSensor_Extension.h"


/* ContiUSS internal global variables */
struct _ContiUSS_int_var {
	double	NextSFSP[_NO_OF_SENSORS_MAX_] = { -1 };
	double	NFD_RisingEdge[_NO_OF_SENSORS_MAX_] = { 1 };
	double	CorrelatorTarget[_NO_OF_SENSORS_MAX_] = { 0 };

	double	CurEchoType[_NO_OF_SENSORS_MAX_] = { 0 };

	double	DataTrans_Running[_NO_OF_SENSORS_MAX_][_NO_OF_ECHOES_MAX_] = { 0 };

	double	ToF_Counter[_NO_OF_SENSORS_MAX_][_NO_OF_ECHOES_MAX_] = { 0 };
	/* Used for evaluation if one target measurment ToF-Count has already been increased in this cycle */
	double	ToF_Counter_State[_NO_OF_SENSORS_MAX_][_NO_OF_ECHOES_MAX_] = { 0 };

	double	SimCoreStartCycle = -1;
} _localvar;

/* Definition of Global ContiUSS struct */
tContiUSS *ContiUSS = NULL;
tContiUSS_DEBUG	*ContiUSS_DEBUG = NULL;

tp_ContiUSS p_ContiUSS;
tContiUSSParam ContiUSSParam;
tContiUSS_UsRaw	ContiUSS_UsRaw = { 0 };


int
ContiUSS_TestRun_Start_atBegin ()
{
	char   *path, pre[64], filename[64];
	tInfos *ContiUSS_Parameterset_Info = InfoNew();
	
	/* Define 'fname' as the Antenna Map textfile and generate path*/
	sprintf(filename, "ContiUSS_Parameterset");
	path = PathJoin(3, SimCore.TestRig.DataDir, "Sensor", filename);

	if ((path = InfoLocateFile(path, 0)) == NULL) {
		/* try absolute/relative path to the testrun file */
		path = filename;
	}

	/* Read the content of the file */
	if (iRead2(NULL, ContiUSS_Parameterset_Info, path, "Sensor") != 0) {
		LogErrF(EC_Init, "ContiUSS: error reading USS Parameterset\n");
	}

	/* Check if proper FileIdent is used */
	sprintf(pre, "FileIdent");
	if (strcmp(iGetStrOpt(ContiUSS_Parameterset_Info, pre, ""), "ContiUSS-ParamSet") != 0) {
		LogErrF(EC_Init, "ContiUSS: Wrong file type for USS Parameterset\n");
	}

	/* Read internal variables */
	sprintf(pre, "kappa");
	ContiUSSParam.kappa = iGetDblOpt(ContiUSS_Parameterset_Info, pre, 1.4);
	sprintf(pre, "R_s");
	ContiUSSParam.R_s = iGetDblOpt(ContiUSS_Parameterset_Info, pre, 287.053);
	sprintf(pre, "NFD_near_max_dist_m");
	ContiUSSParam.NFD_near_max = iGetDblOpt(ContiUSS_Parameterset_Info, pre, 1.5);
	sprintf(pre, "NFD_far_min_dist_m");
	ContiUSSParam.NFD_far_min = iGetDblOpt(ContiUSS_Parameterset_Info, pre, 0.5);
	sprintf(pre, "NFD_dead_time_ms");
	ContiUSSParam.NFD_dead_time = iGetDblOpt(ContiUSS_Parameterset_Info, pre, 150.0);
	sprintf(pre, "T_Correlator_mean");
	ContiUSSParam.T_Correlator_mean = iGetDblOpt(ContiUSS_Parameterset_Info, pre, 8.0);
	sprintf(pre, "T_US_Driver_Calc");
	ContiUSSParam.T_US_Driver_Calc = iGetDblOpt(ContiUSS_Parameterset_Info, pre, 5.0);

	sprintf(pre, "UseFireSchemeFromFile");
	ContiUSSParam.UseFireSchemeFromFile = iGetInt(ContiUSS_Parameterset_Info, pre);
	if (ContiUSSParam.UseFireSchemeFromFile > 0)
		Log("PRE-SIM: USER FILE FOR CONTI-USS FIRE SCHEME ENABELD\n");

	/* Executed in case of Fire Scheme File */
	if (ContiUSSParam.UseFireSchemeFromFile != 0) {
		int		nFire_Samples = 8, nSensor_No = 12, MapNo = 0;
		int		cols;
		float	*Fire_Samples = NULL, *Sensor_No = NULL, *FireScheme = NULL;

		/* Read x , y Vector and z Map */
		/* ******************* */
		/*        ______       */
		/* x  --->|     |      */
		/*        |     |--> z */
		/* y  --->|     |      */
		/*        ------       */
		/* ******************* */

		sprintf(pre, "nFire_Samples");
		nFire_Samples = iGetInt(ContiUSS_Parameterset_Info, pre);

		sprintf(pre, "nSensor_No");
		nSensor_No = iGetInt(ContiUSS_Parameterset_Info, pre);

		Fire_Samples = (float *)calloc(nFire_Samples, sizeof(float));
		Sensor_No = (float *)calloc(nSensor_No, sizeof(float));

		/* x -> Fire_Samples*/
		for (int i = 0; i < nFire_Samples; i++) {
			Fire_Samples[i] = (float)i + 1;
		}

		/* y -> Sensor_No*/
		for (int i = 0; i < nSensor_No; i++) {
			Sensor_No[i] = (float)i + 1;
		}

		/* z -> FireScheme*/
		sprintf(pre, "MapNo");
		MapNo = iGetInt(ContiUSS_Parameterset_Info, pre);
		sprintf(pre, "FireSchemeMap%d", MapNo);
		Log("\nThe read map %s is as follows...\n", pre);

		if ((FireScheme = iGetTableFltOpt2(ContiUSS_Parameterset_Info, pre, NULL, nFire_Samples, &cols)) == NULL) {
			LogErrF(EC_Init, "ContiUSS: Can't get mapping data, %d columns, key '%s'. Initialization failed, please check InfoFile", nFire_Samples, pre);
		}

		/* Check if table has been read properly */
		if (FireScheme != NULL) {

			/* Initialize Antenna Map */
			p_ContiUSS.fire_scheme_map = LM2DInit(Sensor_No, nSensor_No, Fire_Samples, nFire_Samples, FireScheme, 1, 1);

			/* Log read file */
			Log("x-Dir: FireSamples\n");
			Log("y-Dir: Sensors\n\n");
			for (int i = 0; i < nSensor_No; i++) {
				for (int j = 0; j < nFire_Samples; j++) {
					/* Read the map: i->FireSamples, j->Sensor_No */
					p_ContiUSS.fire_scheme[i][j] = LM2DEval(p_ContiUSS.fire_scheme_map, i + 1, j + 1);
					Log("%2.0f ", p_ContiUSS.fire_scheme[i][j]);
				}
				Log("\n");
			}
			Log("\n");
		}

		if (Fire_Samples != NULL)
			free(Fire_Samples);

		if (Sensor_No != NULL)
			free(Sensor_No);
	}

	sprintf(pre, "DebugOpt");
	ContiUSSParam.DebugOpt = iGetInt(ContiUSS_Parameterset_Info, pre);
	if (ContiUSSParam.DebugOpt > 0)
		Log("PRE-SIM: DEBUG-OPTION ENABELD\n\n");
	if (ContiUSSParam.DebugOpt > 1)
		Log("PRE-SIM: DEBUG-OPTION RAY-INFORMATION ENABELD\n\n");

	return 0;
}



int
ContiUSS_TestRun_Start_atEnd ()
{
	char *SensName;
	char sbuf[256];

	if (SimCore.State >= SCState_Start && USonicRSICount) {

		/* Allocate pointers for <ContiUSS...> */
		ContiUSS = (tContiUSS *)calloc(USonicRSICount, sizeof(tContiUSS));
		ContiUSS_DEBUG = (tContiUSS_DEBUG *)calloc(USonicRSICount, sizeof(tContiUSS_DEBUG));

		/* Generate DDict Entries for <ContiUSS_UsRaw> */
		for (int i = 0; i < _NO_OF_TARGS_STRUCT_MAX_; i++) {
			
			// ContiUSS_UsRaw.ContiUSS_UsData_as[i].TimeStamp_us_u64 = 0;
			/* typical values for the US Raw echo structure: */
			ContiUSS_UsRaw.ContiUSS_UsData_as[i].TimeStamp_us_u64 = 148652140;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.TimeStamp_us_u64", i);
			DDefULLong(NULL, sbuf, "us", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].TimeStamp_us_u64, DVA_None);

			// ContiUSS_UsRaw.ContiUSS_UsData_as[i].ToF_8us_u16 = 0;
			/* typical values for the US Raw echo structure: */
			ContiUSS_UsRaw.ContiUSS_UsData_as[i].ToF_8us_u16 = 11616;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.ToF_8us_u16", i);
			DDefUShort(NULL, sbuf, "8us", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].ToF_8us_u16, DVA_None);

			// ContiUSS_UsRaw.ContiUSS_UsData_as[i].RxSensorId_nu_u8 = 0;
			/* typical values for the US Raw echo structure: */
			ContiUSS_UsRaw.ContiUSS_UsData_as[i].RxSensorId_nu_u8 = 6;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.RxSensorId_nu_u8", i);
			DDefUChar(NULL, sbuf, "-", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].RxSensorId_nu_u8, DVA_None);

			// ContiUSS_UsRaw.ContiUSS_UsData_as[i].TxSensorId_nu_u8 = 0;
			/* typical values for the US Raw echo structure: */
			ContiUSS_UsRaw.ContiUSS_UsData_as[i].TxSensorId_nu_u8 = 6;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.TxSensorId_nu_u8", i);
			DDefUChar(NULL, sbuf, "-", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].TxSensorId_nu_u8, DVA_None);

			// ContiUSS_UsRaw.ContiUSS_UsData_as[i].SyncCntCurTag_ms_u8 = 0;
			/* typical values for the US Raw echo structure: */
			ContiUSS_UsRaw.ContiUSS_UsData_as[i].SyncCntCurTag_ms_u8 = 200;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.SyncCntCurTag_ms_u8", i);
			DDefUChar(NULL, sbuf, "ms", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].SyncCntCurTag_ms_u8, DVA_None);

			// ContiUSS_UsRaw.ContiUSS_UsData_as[i].MeasTag_ms_u8 = 0;
			/* typical values for the US Raw echo structure: */
			ContiUSS_UsRaw.ContiUSS_UsData_as[i].MeasTag_ms_u8 = 144;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.MeasTag_ms_u8", i);
			DDefUChar(NULL, sbuf, "ms", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].MeasTag_ms_u8, DVA_None);

			// ContiUSS_UsRaw.ContiUSS_UsData_as[i].Amplitude_nu_u8 = 0;
			/* typical values for the US Raw echo structure: */
			ContiUSS_UsRaw.ContiUSS_UsData_as[i].Amplitude_nu_u8 = 101;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.Amplitude_nu_u8", i);
			DDefUChar(NULL, sbuf, "-", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].Amplitude_nu_u8, DVA_None);

			ContiUSS_UsRaw.ContiUSS_UsData_as[i].DopplerFreq_1ps_i8 = 0;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.DopplerFreq_1ps_i8", i);
			DDefUChar(NULL, sbuf, "-", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].DopplerFreq_1ps_i8, DVA_None);

			ContiUSS_UsRaw.ContiUSS_UsData_as[i].TimeDomainConfidenceLevel_nu_u4 = 5;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.TimeDomainConfidenceLevel_nu_u4", i);
			DDefUChar(NULL, sbuf, "-", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].TimeDomainConfidenceLevel_nu_u4, DVA_None);

			ContiUSS_UsRaw.ContiUSS_UsData_as[i].CodingConfidenceLevel_nu_u4 = 4;
			sprintf(sbuf, "ContiUSS_UsRaw.ContiUSS_UsData_as.%01d.CodingConfidenceLevel_nu_u4", i);
			DDefUChar(NULL, sbuf, "-", &ContiUSS_UsRaw.ContiUSS_UsData_as[i].CodingConfidenceLevel_nu_u4, DVA_None);
		}

		/* Get <ContiUSS> specific InfoFile parameters */
		ContiUSSParam.MaxRefl = iGetIntOpt(SimCore.Vhcl.Inf, "Sensor.USonicRSI.Reflections", 1);
		ContiUSSParam.CycleTime = iGetIntOpt(SimCore.Vhcl.Inf, "Sensor.USonicRSI.CycleTime", 1);

		if (ContiUSSParam.DebugOpt > 0)
			Log("Current Parameter-Set:\n No. of Sensors = %d\n No. of max. Reflections = %d\n Cycle time = %d\n\n", USonicRSICount, ContiUSSParam.MaxRefl, ContiUSSParam.CycleTime);
		
		for (int i = 0; i < USonicRSICount; i++) {

			/* Read Infofile Parameters */
			sprintf(sbuf, "Sensor.USonicRSI.%01d.name", i);
			SensName = iGetStrOpt(SimCore.Vhcl.Inf, sbuf, NULL);
			sprintf(sbuf, "Sensor.USonicRSI.%01d.nEchoesMax", i);
			ContiUSSParam.nEchoesMax[i] = iGetIntOpt(SimCore.Vhcl.Inf, sbuf, 1);
			sprintf(sbuf, "Sensor.USonicRSI.%01d.Frequency", i);
			ContiUSSParam.Frequency[i] = iGetDblOpt(SimCore.Vhcl.Inf, sbuf, 1);

			/* Allocate pointers for <ContiUSS->ContiUSSEcho> & <ContiUSS->ContiUSS_PDCMFrame>*/
			ContiUSS[i].ContiUSSEcho = (tContiUSSEcho *)calloc(_NO_OF_ECHOES_MAX_, sizeof(tContiUSSEcho));

			/* Pointer to IPG-USSRSI-API <nEchoes> */
			ContiUSS[i].nEchoes = &USonicRSI[i].nEchoes;

			/* Set default values to parameters and variables */
			ContiUSS[i].TimeStamp = 0;
			ContiUSS[i].IntTrigger = 0;
			ContiUSS[i].ExtTrigger = 0;

			/* Set default values for flags */
			ContiUSS[i].ContiUSSFlags.FireFlag = 0;
			ContiUSS[i].ContiUSSFlags.NFD = 0;

			/* Generate DDict Entries for <ContiUSS> */
			sprintf(sbuf, "ContiUSS.%s.nEchoes", SensName);
			DDefInt(NULL, sbuf, "", ContiUSS[i].nEchoes, DVA_None);
			sprintf(sbuf, "ContiUSS.%s.T_Sensor", SensName);
			DDefDouble4(NULL, sbuf, "K", &ContiUSS[i].T_Sensor, DVA_IO_Out);
			sprintf(sbuf, "ContiUSS.%s.TimeStamp", SensName);
			DDefDouble(NULL, sbuf, "ms", &ContiUSS[i].TimeStamp, DVA_None);
			sprintf(sbuf, "ContiUSS.%s.ExternTrigger", SensName);
			DDefInt(NULL, sbuf, "", &ContiUSS[i].ExtTrigger, DVA_IO_In);
			sprintf(sbuf, "ContiUSS.%s.FireFlag", SensName);
			DDefInt(NULL, sbuf, "", &ContiUSS[i].ContiUSSFlags.FireFlag, DVA_None);
			sprintf(sbuf, "ContiUSS.%s.NFD", SensName);
			DDefInt(NULL, sbuf, "", &ContiUSS[i].ContiUSSFlags.NFD, DVA_IO_In);


			for (int j = 0; j < ContiUSSParam.nEchoesMax[i]; j++) {

				/* Pointer to IPG-USSRSI-API struct <tEcho> */
				ContiUSS[i].ContiUSSEcho[j].TimeOF = &USonicRSI[i].Echo[j].TimeOF;
				ContiUSS[i].ContiUSSEcho[j].LengthOF = &USonicRSI[i].Echo[j].LengthOF;
				ContiUSS[i].ContiUSSEcho[j].SPA = &USonicRSI[i].Echo[j].SPA;
				ContiUSS[i].ContiUSSEcho[j].SPL = &USonicRSI[i].Echo[j].SPL;
				ContiUSS[i].ContiUSSEcho[j].nRefl = &USonicRSI[i].Echo[j].nRefl;
				ContiUSS[i].ContiUSSEcho[j].Tx = &USonicRSI[i].Echo[j].Tx;

				/* For measurement compatibility, will be useless in future */
				ContiUSS[i].ContiUSSEcho[j].ContiTimeOF_Temp = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiTimeOF = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiLengthOF = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiSPA = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiSPL = 0;
				ContiUSS[i].ContiUSSEcho[j].ContinRefl = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiTx = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiBurstTime = 0;

				/* For debugging only */
				ContiUSS_DEBUG[i].SFSP_ToF_Diff[j] = 0;

				/* Generate DDict Entries for <ContiUSS->ContiUSSEcho> */
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContiLengthOF", SensName, j);
				DDefDouble4(NULL, sbuf, "m", &ContiUSS[i].ContiUSSEcho[j].ContiLengthOF, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContiTimeOF", SensName, j);
				DDefDouble4(NULL, sbuf, "s", &ContiUSS[i].ContiUSSEcho[j].ContiTimeOF, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContiTimeOF_Temp", SensName, j);
				DDefDouble4(NULL, sbuf, "s", &ContiUSS[i].ContiUSSEcho[j].ContiTimeOF_Temp, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContiSPA", SensName, j);
				DDefDouble4(NULL, sbuf, "Pa", &ContiUSS[i].ContiUSSEcho[j].ContiSPA, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContiSPL", SensName, j);
				DDefDouble4(NULL, sbuf, "dB//1muPa", &ContiUSS[i].ContiUSSEcho[j].ContiSPL, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContinRefl", SensName, j);
				DDefInt(NULL, sbuf, "", &ContiUSS[i].ContiUSSEcho[j].ContinRefl, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContiTx", SensName, j);
				DDefInt(NULL, sbuf, "", &ContiUSS[i].ContiUSSEcho[j].ContiTx, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.ContiBurstTime", SensName, j);
				DDefInt(NULL, sbuf, "", &ContiUSS[i].ContiUSSEcho[j].ContiBurstTime, DVA_None);
				sprintf(sbuf, "ContiUSS.%s.Echo.%01d.DEBUG_SFSP_ToF", SensName, j);
				DDefDouble4(NULL, sbuf, "ms", &ContiUSS_DEBUG[i].SFSP_ToF_Diff[j], DVA_None);

				/* ********************** INFO ******************************** */
				/* To get access to the content of the ContiUSS Pointer use		*/
				/* e.g. ... *ContiUSS[i].ContiUSSEcho[j].LengthOF ...			*/
				/* **************************************************************/
			}
		}
	}

	return 0;
}



int
ContiUSS_TestRun_Start_Finalize (void)
{
	/* Set starting conditions for timing */
	for (int i = 0; i < USonicRSICount; i++) {
		_localvar.NextSFSP[i] = SimCore.CycleNo + 1;
		_localvar.NFD_RisingEdge[i] = 1;
		_localvar.CorrelatorTarget[i] = 0;
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			_localvar.DataTrans_Running[i][k] = 0;
		}
		_localvar.CurEchoType[i] = 0;
	}
	_localvar.SimCoreStartCycle = SimCore.CycleNo + 1;

	return 0;
}



bool
InternTrigger (void)
{
	static int k[_NO_OF_SENSORS_MAX_] = { 0 };
	static int _CycleNo[_NO_OF_SENSORS_MAX_] = { 0 };
	int _SensorCalc = 0, a = 0;
	double ErrorFlag = 0;

	/* Reset variables at TestRun end */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		for (int j = 0; j < USonicRSICount; j++) { k[j] = 0; }
		return false;
	}

	for (int i = 0; i < USonicRSICount; i++) {
		if (_localvar.NextSFSP[i] == SimCore.CycleNo) {

			if (ContiUSSParam.DebugOpt > 0)
				Log("\n\nCurrent Cycle: %d\n", SimCore.CycleNo);

			/* Calculate next sample point for the ContiUSS_SensorCalc to be executed based on parameterset */
			_localvar.NextSFSP[i] = _localvar.NextSFSP[i] + p_ContiUSS.fire_scheme[i][k[i]];

			/* Avoid execution for the first SimCore.CycleNo */
			if (_localvar.SimCoreStartCycle - SimCore.CycleNo) {
				k[i] == 0 ? a = 7 : a = k[i]-1;
				ContiUSS[i].TimeStamp = SimCore.CycleNo - _CycleNo[i];
				ErrorFlag = ContiUSS[i].TimeStamp - p_ContiUSS.fire_scheme[i][a];
			}

			/* Set flag for sensor calculation */
			ContiUSS[i].IntTrigger = 1;

			/* Print additional information */
			if (ContiUSSParam.DebugOpt > 0)
				Log("Sensor=%d , Next Execution=%f , OldCycleNo=%d , ContiUSS[i].TimeStamp=%f, NewFireScheme=%f\n", i, _localvar.NextSFSP[i], _CycleNo[i], ContiUSS[i].TimeStamp, p_ContiUSS.fire_scheme[i][k[i]]);
			if (ErrorFlag != 0)
				LogErrF(EC_Init, "ContiUSS: Time stamp doesn't fit to fire scheme! Error Value: %f\n", ErrorFlag);

			/* Set vairables for next sample point e.g. remeber old CycleNo value*/
			_CycleNo[i] = SimCore.CycleNo;
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



bool
ExternTrigger (void)
{
	static int k[_NO_OF_SENSORS_MAX_] = { 0 };
	static int _CycleNo[_NO_OF_SENSORS_MAX_] = { 0 };
	int _SensorCalc = 0;

	/* Reset variables at TestRun end */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		for (int j = 0; j < USonicRSICount; j++) { k[j] = 0; }
		return false;
	}

	/* Executed every time the DVA variable ContiUSS.<SensorName>.ExternTrigger is set to '1' */
	for (int i = 0; i < USonicRSICount; i++) {
		if (ContiUSS[i].ExtTrigger != 0) {

			/* Avoid execution for the first SimCore.CycleNo */
			if (_localvar.SimCoreStartCycle - SimCore.CycleNo) {
				ContiUSS[i].TimeStamp = SimCore.CycleNo - _CycleNo[i];
			}

			/* Set time stamp for first call */
			if (k[i] == 0) {
				ContiUSS[i].TimeStamp = 0;
				k[i] = 1;
			}

			/* Print additional information */
			if (ContiUSSParam.DebugOpt > 0)
				Log("Sensor %d , _CycleNo[i] %d , ContiUSS[i].TimeStamp %f\n", i, _CycleNo[i], ContiUSS[i].TimeStamp);

			/* Set vairables for next sample point e.g. remeber old CycleNo value*/
			_CycleNo[i] = SimCore.CycleNo;
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
ToggleFlag (int a)
{
	if (a == 0)
		a = 1;
	else if (a == 1)
		a = 0;
	else
		LogErrF(EC_Sim, "ContiUSS: Toggle Flag not possible\n");

	return a;
}



int
TemperatureManipulation (int i)
{
	/* Calculation of TimeOF based on IPG's LengthOF */
	for (int j = 0; j < ContiUSSParam.nEchoesMax[i]; j++) {
		if (ContiUSS[i].T_Sensor != 0)
			ContiUSS[i].ContiUSSEcho[j].ContiTimeOF_Temp = (*ContiUSS[i].ContiUSSEcho[j].LengthOF / sqrt(ContiUSSParam.kappa * ContiUSSParam.R_s * ContiUSS[i].T_Sensor) * 1000.0);

		if (ContiUSSParam.DebugOpt > 1)
			Log("SensorNo_%d/RayNo_%d -> LoF:%f , kappa:%f , R_s:%f , T:%f , ToF:%f , ContiToF:%f\n",
				i, j,
				*ContiUSS[i].ContiUSSEcho[j].LengthOF,
				ContiUSSParam.kappa, ContiUSSParam.R_s,
				ContiUSS[i].T_Sensor,
				*ContiUSS[i].ContiUSSEcho[j].TimeOF,
				ContiUSS[i].ContiUSSEcho[j].ContiTimeOF_Temp);
	}

	return 0;
}


int
NFDMode (void)
{
	static int SwitchState[_NO_OF_SENSORS_MAX_] = { 0 };
	static int OldState[_NO_OF_SENSORS_MAX_] = { 0 };
	static int counter[_NO_OF_SENSORS_MAX_] = { 0 };


	for (int i = 0; i < USonicRSICount; i++) {

		if (OldState[i] != ContiUSS[i].ContiUSSFlags.NFD)
			_localvar.NFD_RisingEdge[i] = 0;

		if (ContiUSS[i].ContiUSSFlags.NFD == 1 && _localvar.NFD_RisingEdge[i] == 0 && SwitchState[i] == 0 && OldState[i] != ContiUSS[i].ContiUSSFlags.NFD) {
			SwitchState[i] = 1;
			counter[i]++;
		}
		else if (ContiUSS[i].ContiUSSFlags.NFD == 1 && _localvar.NFD_RisingEdge[i] == 0 && SwitchState[i] == 1) {

			if (counter[i] >= 1 && counter[i] < ContiUSSParam.NFD_dead_time) {
				counter[i]++;
			}
			else if (counter[i] == ContiUSSParam.NFD_dead_time) {
				_localvar.NFD_RisingEdge[i] = 1;
				counter[i] = 0;
				SwitchState[i] = 0;
			}
			else {
				LogErrF(EC_Sim, "ContiUSS: Switch NFD mode not possible\n");
			}
		}
		else if (ContiUSS[i].ContiUSSFlags.NFD == 0 && _localvar.NFD_RisingEdge[i] == 0 && SwitchState[i] == 0 && OldState[i] != ContiUSS[i].ContiUSSFlags.NFD) {
			SwitchState[i] = 1;
			counter[i]++;
		}
		else if (ContiUSS[i].ContiUSSFlags.NFD == 0 && _localvar.NFD_RisingEdge[i] == 0 && SwitchState[i] == 1) {

			if (counter[i] >= 1 && counter[i] < ContiUSSParam.NFD_dead_time) {
				counter[i]++;
			}
			else if (counter[i] == ContiUSSParam.NFD_dead_time) {
				_localvar.NFD_RisingEdge[i] = 1;
				counter[i] = 0;
				SwitchState[i] = 0;
			}
			else {
				LogErrF(EC_Sim, "ContiUSS: Switch NFD mode not possible\n");
			}
		}

		if (OldState[i] != ContiUSS[i].ContiUSSFlags.NFD)
			OldState[i] = ToggleFlag(OldState[i]);
	}

	return 0;
}



int
DataMapping (void)
{
	for (int i = 0; i < USonicRSICount; i++) {
		for (int j = 0; j < ContiUSSParam.nEchoesMax[i]; j++) {

			/* NFD switch */
			if (ContiUSS[i].ContiUSSFlags.NFD == 1 && _localvar.NFD_RisingEdge[i] == 1) {
				/* Cut out values bigger than NFD_near_max */

				if (*ContiUSS[i].ContiUSSEcho[j].LengthOF <= (ContiUSSParam.NFD_near_max * 2.0)) {
					ContiUSS[i].ContiUSSEcho[j].ContiLengthOF = *ContiUSS[i].ContiUSSEcho[j].LengthOF;
					ContiUSS[i].ContiUSSEcho[j].ContiTimeOF = *ContiUSS[i].ContiUSSEcho[j].TimeOF * 1000.0;
					ContiUSS[i].ContiUSSEcho[j].ContiSPA = *ContiUSS[i].ContiUSSEcho[j].SPA;
					ContiUSS[i].ContiUSSEcho[j].ContiSPL = *ContiUSS[i].ContiUSSEcho[j].SPL;
					ContiUSS[i].ContiUSSEcho[j].ContinRefl = *ContiUSS[i].ContiUSSEcho[j].nRefl;
					ContiUSS[i].ContiUSSEcho[j].ContiTx = *ContiUSS[i].ContiUSSEcho[j].Tx;
				}
				else {
					ContiUSS[i].ContiUSSEcho[j].ContiLengthOF = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiTimeOF = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiSPA = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiSPL = 0;
					ContiUSS[i].ContiUSSEcho[j].ContinRefl = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiTx = 0;
				}
			}
			else if (ContiUSS[i].ContiUSSFlags.NFD == 0 && _localvar.NFD_RisingEdge[i] == 1) {

				/* Cut out values bigger than NFD_far_min */
				ContiUSS[i].ContiUSSEcho[j].ContiLengthOF = *ContiUSS[i].ContiUSSEcho[j].LengthOF;
				if (*ContiUSS[i].ContiUSSEcho[j].LengthOF >= (ContiUSSParam.NFD_far_min * 2.0)) {
					ContiUSS[i].ContiUSSEcho[j].ContiLengthOF = *ContiUSS[i].ContiUSSEcho[j].LengthOF;
					ContiUSS[i].ContiUSSEcho[j].ContiTimeOF = *ContiUSS[i].ContiUSSEcho[j].TimeOF * 1000.0;
					ContiUSS[i].ContiUSSEcho[j].ContiSPA = *ContiUSS[i].ContiUSSEcho[j].SPA;
					ContiUSS[i].ContiUSSEcho[j].ContiSPL = *ContiUSS[i].ContiUSSEcho[j].SPL;
					ContiUSS[i].ContiUSSEcho[j].ContinRefl = *ContiUSS[i].ContiUSSEcho[j].nRefl;
					ContiUSS[i].ContiUSSEcho[j].ContiTx = *ContiUSS[i].ContiUSSEcho[j].Tx;
				}
				else {
					ContiUSS[i].ContiUSSEcho[j].ContiLengthOF = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiTimeOF = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiSPA = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiSPL = 0;
					ContiUSS[i].ContiUSSEcho[j].ContinRefl = 0;
					ContiUSS[i].ContiUSSEcho[j].ContiTx = 0;
				}
			}
			else {
				ContiUSS[i].ContiUSSEcho[j].ContiLengthOF = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiTimeOF = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiSPA = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiSPL = 0;
				ContiUSS[i].ContiUSSEcho[j].ContinRefl = 0;
				ContiUSS[i].ContiUSSEcho[j].ContiTx = 0;
			}
		}
	}
	return 0;
}



int 
GetCorrelatorTime(void)
{
	// Function to derive the current correlator time

	return 0;
}



int
GetCurrentEchoes(int i)
{
	/* Evaluate how many echoes are writen in this SFSP and save them locally*/
	/* EchoType
	0 -> No echoes
	1 -> 1st 5 direct echoes
	2 -> 2nd 5 direct echoes
	3 -> 1st 5 indirect echoes
	4 -> 2nd 5 indirect echoes*/

	/* Local variables */
	int g = 0;
	int t = 0;

	int a1 = 0;
	int a2 = 0;

	struct tmpSensData {
		double	ToF[_NO_OF_ECHOES_MAX_] = { 0 };
		double	LoF[_NO_OF_ECHOES_MAX_] = { 0 };
		double	SPL[_NO_OF_ECHOES_MAX_] = { 0 };
		double	SPA[_NO_OF_ECHOES_MAX_] = { 0 };
		int		Tx[_NO_OF_ECHOES_MAX_] = { 0 };
		int		nRefl[_NO_OF_ECHOES_MAX_] = { 0 };
		double	Tof_Counter[_NO_OF_ECHOES_MAX_] = { 0 };
		double	Tof_Counter_State[_NO_OF_ECHOES_MAX_] = { 0 };
		int		BurstTime[_NO_OF_ECHOES_MAX_] = { 0 };
	} tmpSensData;

	/* Save Number of Echoes locally | Reset all DataTrans_Runnings to 0 if no Echoes received */
	if (*ContiUSS[i].ContiUSSEcho[0].TimeOF != 0) {
		/* Check first ToF, because CM is storing the last nEchoes for the next Sim */
		ContiUSS[i].ContinEchoes = *ContiUSS[i].nEchoes;
	}
	else {
		ContiUSS[i].ContinEchoes = 0;
	}
	
	if (ContiUSSParam.DebugOpt == -1) {
		Log("- - -\nSensor %d: Got %d new Echoes: ", i, ContiUSS[i].ContinEchoes);
		for (int a = 0; a < ContiUSS[i].ContinEchoes; a++) {
			Log("%d: %f  ", a, (*ContiUSS[i].ContiUSSEcho[a].TimeOF * 1000.0));
		}
		Log("\n");
	}

	/* Deriving overlapping target measurements from last SFSP before new mapping starts */
	for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {

		/* In case no echoes are received and DataTrans_Running is false, reset all variables */
		if ( (ContiUSS[i].ContiUSSEcho[k].ContiTimeOF != 0) && (_localvar.DataTrans_Running[i][k] == 0) ) {
			
			if (ContiUSSParam.DebugOpt == -1)
				Log("ContiUSS: Got ContiToF=%f but no DataTrans_Running! Reset variables for Sens_%d + Target_%d\n", ContiUSS[i].ContiUSSEcho[k].ContiTimeOF, i, k);
			
			_localvar.ToF_Counter[i][k] = 0;
			_localvar.ToF_Counter_State[i][k] = 0;
			
			ContiUSS[i].ContiUSSEcho[k].ContiLengthOF = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiTimeOF = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiSPA = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiSPL = 0;
			ContiUSS[i].ContiUSSEcho[k].ContinRefl = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiTx = 0;
		}
		/* Evaluation of SFSP & ToF if overlapping this SFSP within 2ms Deaftime */
		else if (((ContiUSS[i].ContiUSSEcho[k].ContiTimeOF /*Snapshot of previous SFSP*/ - ContiUSS[i].TimeStamp/*Last SFSP Duration*/) > 0)
			&& ((ContiUSS[i].ContiUSSEcho[k].ContiTimeOF /*Snapshot of previous SFSP*/ - ContiUSS[i].TimeStamp/*Last SFSP Duration*/) <= _Sensor_Deaf_Time_)) {

			ContiUSS_DEBUG[i].SFSP_ToF_Diff[k] = ContiUSS[i].ContiUSSEcho[k].ContiTimeOF - ContiUSS[i].TimeStamp;
			
			if (ContiUSSParam.DebugOpt == -1)
				Log("Sensor %d: Overlapping WITHIN Deaf-Time for Target %d with %fms | Delete Target Measurement\n", i, k, ContiUSS_DEBUG[i].SFSP_ToF_Diff[k]);

			/* Reset all variables for this detected target */
			_localvar.DataTrans_Running[i][k] = 0;
			_localvar.ToF_Counter[i][k] = 0;
			_localvar.ToF_Counter_State[i][k] = 0;

			ContiUSS[i].ContiUSSEcho[k].ContiLengthOF = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiTimeOF = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiSPA = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiSPL = 0;
			ContiUSS[i].ContiUSSEcho[k].ContinRefl = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiTx = 0;
			ContiUSS[i].ContiUSSEcho[k].ContiBurstTime = 0;

			/* ATTENTION: This does not consider Time Domain Confidence Level !!! */

		}
		/* Evaluation of SFSP & ToF if overlapping this SFSP over 2ms Deaftime */
		else if (((ContiUSS[i].ContiUSSEcho[k].ContiTimeOF/*Snapshot of previous SFSP*/ - ContiUSS[i].TimeStamp/*Last SFSP Duration*/) > _Sensor_Deaf_Time_)) {
			
			ContiUSS_DEBUG[i].SFSP_ToF_Diff[k] = ContiUSS[i].ContiUSSEcho[k].ContiTimeOF - ContiUSS[i].TimeStamp;
			
			if (ContiUSSParam.DebugOpt == -1)
				Log("Sensor %d: Overlapping for Target %d with %fms\n", i, k, ContiUSS_DEBUG[i].SFSP_ToF_Diff[k]);

			/* Make local copy of SensorData to be able to reorder them*/
			tmpSensData.ToF[t] =	ContiUSS[i].ContiUSSEcho[k].ContiTimeOF;
			ContiUSS[i].ContiUSSEcho[k].ContiTimeOF = 0;
			tmpSensData.LoF[t] =	ContiUSS[i].ContiUSSEcho[k].ContiLengthOF;
			ContiUSS[i].ContiUSSEcho[k].ContiLengthOF = 0;
			tmpSensData.SPA[t] =	ContiUSS[i].ContiUSSEcho[k].ContiSPA;
			ContiUSS[i].ContiUSSEcho[k].ContiSPA = 0;
			tmpSensData.SPL[t] =	ContiUSS[i].ContiUSSEcho[k].ContiSPL;
			ContiUSS[i].ContiUSSEcho[k].ContiSPL = 0;
			tmpSensData.nRefl[t] =	ContiUSS[i].ContiUSSEcho[k].ContinRefl;
			ContiUSS[i].ContiUSSEcho[k].ContinRefl = 0;
			tmpSensData.Tx[t] =		ContiUSS[i].ContiUSSEcho[k].ContiTx;
			ContiUSS[i].ContiUSSEcho[k].ContiTx = 0;
			tmpSensData.BurstTime[t] = ContiUSS[i].ContiUSSEcho[k].ContiBurstTime;
			ContiUSS[i].ContiUSSEcho[k].ContiBurstTime = 0;

			tmpSensData.Tof_Counter[t] = _localvar.ToF_Counter[i][k];
			_localvar.ToF_Counter[i][k] = 0;
			tmpSensData.Tof_Counter_State[t] = _localvar.ToF_Counter_State[i][k];
			_localvar.ToF_Counter_State[i][k] = 0;

			t++;
			a1++;
		}
	}

	/* Reset for next loop */
	t = 0;

	/* Check ToFs of current SFSP */
	for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {

		/* Check if <NewTargetToF> is smaller than <LastTargetTof - LastSFSP> | Means, new target measurement is smaller than old ones*/
		if ( ( ((*ContiUSS[i].ContiUSSEcho[k].TimeOF * 1000.0) <= tmpSensData.ToF[t]) || (tmpSensData.ToF[t] == 0) )
				&& (*ContiUSS[i].ContiUSSEcho[k].TimeOF != 0) ) {
			
			/* Set local variables as starting point for new target measurement */
			_localvar.DataTrans_Running[i][g] = 1;
			_localvar.ToF_Counter[i][g] = 0;
			_localvar.ToF_Counter_State[i][g] = 0;

			/* Copy current sim data from CarMaker */
			ContiUSS[i].ContiUSSEcho[g].ContiLengthOF = *ContiUSS[i].ContiUSSEcho[k].LengthOF;
			ContiUSS[i].ContiUSSEcho[g].ContiTimeOF = *ContiUSS[i].ContiUSSEcho[k].TimeOF * 1000.0;
			ContiUSS[i].ContiUSSEcho[g].ContiSPA = *ContiUSS[i].ContiUSSEcho[k].SPA;
			ContiUSS[i].ContiUSSEcho[g].ContiSPL = *ContiUSS[i].ContiUSSEcho[k].SPL;
			ContiUSS[i].ContiUSSEcho[g].ContinRefl = *ContiUSS[i].ContiUSSEcho[k].nRefl;
			ContiUSS[i].ContiUSSEcho[g].ContiTx = *ContiUSS[i].ContiUSSEcho[k].Tx;
			ContiUSS[i].ContiUSSEcho[g].ContiBurstTime = SimCore.CycleNo;

			if (ContiUSSParam.DebugOpt == -1)
				Log("Sensor %d: NEW ToF for Target %d: %f\n", i, g, ContiUSS[i].ContiUSSEcho[g].ContiTimeOF);

			a2++;
		}
		else if ( ( ((*ContiUSS[i].ContiUSSEcho[k].TimeOF * 1000.0) > tmpSensData.ToF[t]) && (tmpSensData.ToF[t] != 0) )
				|| ( (*ContiUSS[i].ContiUSSEcho[k].TimeOF == 0) && (tmpSensData.ToF[t] != 0) ) ) {

			/* Set local variables according to old target measurement */
			_localvar.DataTrans_Running[i][g] = 1;
			_localvar.ToF_Counter[i][g] = tmpSensData.Tof_Counter[t];
			_localvar.ToF_Counter_State[i][g] = tmpSensData.Tof_Counter_State[t];

			/* Copy old sim data from tmp-var*/
			ContiUSS[i].ContiUSSEcho[g].ContiLengthOF = tmpSensData.LoF[t];
			ContiUSS[i].ContiUSSEcho[g].ContiTimeOF = tmpSensData.ToF[t];
			ContiUSS[i].ContiUSSEcho[g].ContiSPA = tmpSensData.SPA[t];
			ContiUSS[i].ContiUSSEcho[g].ContiSPL = tmpSensData.SPL[t];
			ContiUSS[i].ContiUSSEcho[g].ContinRefl = tmpSensData.nRefl[t];
			ContiUSS[i].ContiUSSEcho[g].ContiTx = tmpSensData.Tx[t];
			ContiUSS[i].ContiUSSEcho[g].ContiBurstTime = tmpSensData.BurstTime[t];

			if (ContiUSSParam.DebugOpt == -1)
				Log("Sensor %d: OLD ToF for Target %d: %f <--\n", i, g, ContiUSS[i].ContiUSSEcho[g].ContiTimeOF);

			t++;
			k--;
		}
		else {

			/* Reset all variables */
			_localvar.DataTrans_Running[i][g] = 0;
			_localvar.ToF_Counter[i][g] = 0;
			_localvar.ToF_Counter_State[i][g] = 0;

			ContiUSS[i].ContiUSSEcho[g].ContiLengthOF = 0;
			ContiUSS[i].ContiUSSEcho[g].ContiTimeOF = 0;
			ContiUSS[i].ContiUSSEcho[g].ContiSPA = 0;
			ContiUSS[i].ContiUSSEcho[g].ContiSPL = 0;
			ContiUSS[i].ContiUSSEcho[g].ContinRefl = 0;
			ContiUSS[i].ContiUSSEcho[g].ContiTx = 0;
			ContiUSS[i].ContiUSSEcho[g].ContiBurstTime = 0;

			if (ContiUSSParam.DebugOpt == -1)
				Log("Sensor %d: NO ToF for Target %d\n", i, g);

		}
		
		g++;

		if (g >= _NO_OF_ECHOES_MAX_) {

			if ((a1 + ContiUSS[i].ContinEchoes) != (t + a2))
				LogErrF(EC_Sim, "ContiUSS: Number of GroundTruth echoes and converted echoes not equal!\n");

			return 0;
		}
	}

	if ((a1 + ContiUSS[i].ContinEchoes) != (t + a2))
		LogErrF(EC_Sim, "ContiUSS: Number of GroundTruth echoes and converted echoes not equal!\n");

	/* Check how many echoes are currently measured
	if (ContiUSS[i].ContinEchoes < 0) {
		_localvar.CurEchoType[i] = -1;
		LogErrF(EC_Sim, "ContiUSS: SFSP triggered but negativ number of Echoes! Check 'nEchoes'\n");
	}
	else if (ContiUSS[i].ContinEchoes == 0) {
		_localvar.CurEchoType[i] = 0;
	}
	else if (ContiUSS[i].ContinEchoes <= 5) {
		_localvar.CurEchoType[i] = 1;
	}
	else if (ContiUSS[i].ContinEchoes <= 10) {
		_localvar.CurEchoType[i] = 2;
	}
	else if (ContiUSS[i].ContinEchoes <= 15) {
		_localvar.CurEchoType[i] = 3;
	}
	else if (ContiUSS[i].ContinEchoes <= 20) {
		_localvar.CurEchoType[i] = 4;
	}
	else if (ContiUSS[i].ContinEchoes > 20) {
		_localvar.CurEchoType[i] = -1;
		LogErrF(EC_Sim, "ContiUSS: More echoes than 20. Currently not supported!\n");
	}
	*/

	return 0;
}



int
SensDataTrans_Eval (int j)
{

	/* Main Procedure 
	(0) Let ToF fly by
	(1) T_Correlator
	(2) T_ASIC_Idle (based on CM_Clock)
	(3) T_ASIC Calc Time 1ms
	(4) T_US_Driver (fix by parameter -> 5ms defautl)
	(5) T_US_ALgo_idle (based on CM_Clock) */

	/* Non-static variables */
	double Sum_j = 0;	/* Used for evaluation if early return in this cycle due to no job */
	int BurstCheck = 0;	/* Used for evaluation if one sensor has already been used */

	/* Static variables */
	static int Correlator_Counter[_NO_OF_SENSORS_MAX_] = { 0 };
	static int US_Driver_Counter[_NO_OF_SENSORS_MAX_] = { 0 };
	static int ASIC_Counter[_NO_OF_SENSORS_MAX_] = { 1 };

	/* Reset variables at TestRun end */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		for (int i = 0; i < USonicRSICount; i++) {
			for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
				_localvar.ToF_Counter[i][k] = 0;
				ContiUSS[i].ContiUSSEcho[k].ContiLengthOF = 0;
				ContiUSS[i].ContiUSSEcho[k].ContiTimeOF = 0;
				ContiUSS[i].ContiUSSEcho[k].ContiSPA = 0;
				ContiUSS[i].ContiUSSEcho[k].ContiSPL = 0;
				ContiUSS[i].ContiUSSEcho[k].ContinRefl = 0;
				ContiUSS[i].ContiUSSEcho[k].ContiTx = 0;
				ContiUSS[i].ContiUSSEcho[k].ContiBurstTime = 0;
				_localvar.DataTrans_Running[i][k] = 0;
			}
		}
		return 0;
	}


	if (ContiUSSParam.DebugOpt == -1) {
		Log("\nSensDataTrans_Eval for SensorNo %d -> CycleNo %d\nToF-Counter:\n---\nSensors   0   1   2   3   4   5   6   7   8   9   10  11\n", j, SimCore.CycleNo);
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			Log("Targ %d | %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f\n", k,
				_localvar.ToF_Counter[0][k], _localvar.ToF_Counter[1][k], _localvar.ToF_Counter[2][k], _localvar.ToF_Counter[3][k], _localvar.ToF_Counter[4][k],
				_localvar.ToF_Counter[5][k], _localvar.ToF_Counter[6][k], _localvar.ToF_Counter[7][k], _localvar.ToF_Counter[8][k], _localvar.ToF_Counter[9][k],
				_localvar.ToF_Counter[10][k], _localvar.ToF_Counter[11][k]);
		}
		Log("++++++++++++++++++++++++++++++\n");
		Log("DataTrans_Runnings:\n---\nSensors   0   1   2   3   4   5   6   7   8   9   10  11\n", j, SimCore.CycleNo);
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			Log("Targ %d | %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f  %2.0f\n", k,
				_localvar.DataTrans_Running[0][k], _localvar.DataTrans_Running[1][k], _localvar.DataTrans_Running[2][k], _localvar.DataTrans_Running[3][k], _localvar.DataTrans_Running[4][k],
				_localvar.DataTrans_Running[5][k], _localvar.DataTrans_Running[6][k], _localvar.DataTrans_Running[7][k], _localvar.DataTrans_Running[8][k], _localvar.DataTrans_Running[9][k],
				_localvar.DataTrans_Running[10][k], _localvar.DataTrans_Running[11][k]);
		}
		Log("++++++++++++++++++++++++++++++\n");
	}


	/* Check if DataTrans_Eval hasn't to be executed (if -> no Sensor-NewTrigger or no Sensor-Running) */
	for (int i = 0; i < USonicRSICount; i++) {
		/* Sum up the amount of current running tasks */
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			Sum_j = Sum_j + _localvar.DataTrans_Running[i][k];
		}
	}
	/* Check if NewTrigger is false and Sum_j is bigger than 0 */
	if (j == -1 && Sum_j == 0) {
		if (ContiUSSParam.DebugOpt == -1)
			Log("Return in this cycle (no new Calls or DataTrans_Running)\n- - - - - -\n");

		Sum_j = 0;
		return 0;
	}


	/* Check if and which Sensor-NewTrigger or Sensor-Running to consider*/
	for (int i = 0; i < USonicRSICount; i++) {
		
		/* Only execute "GetCurrentEchoes()" once per Sensor / Burst | Will be set to 0 @end of k-loop */
		BurstCheck = 1;

		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {

			if ( (_localvar.DataTrans_Running[i][k] == 1) || (j == i) ) {

				if (ContiUSSParam.DebugOpt == -1)
					Log("Sens_%d, Target_%d: ToF_Counter_State=%d\n", i, k, _localvar.ToF_Counter_State[i][k]);

				/* If no DataTrans is running right now, get the echo information for this SFSP and save them LOCALLY (no write down to quantities yet) */
				if (_localvar.DataTrans_Running[i][k] == 0 && BurstCheck == 1)
					GetCurrentEchoes(i);

				/* Error checking of correltor time */
				if (_localvar.CorrelatorTarget[i] < Correlator_Counter[i]) {
					LogErrF(EC_Sim, "ContiUSS: Correlator target time problem!\n");
				}

				/* *********************************************** */
				/* ****** (-1-) Wait for ToF to be finsihed ****** */
				/* *********************************************** */
				if ( (ContiUSS[i].ContiUSSEcho[k].ContiTimeOF > _localvar.ToF_Counter[i][k])
						&& (SimCore.CycleNo != _localvar.ToF_Counter_State[i][k]) ) {

					if (ContiUSSParam.DebugOpt == -1)
						Log("Tof-Counter (SensNo %d, EchoNo %d, StartCycle %d, StopCycle %f): Cnt = %f, ContiToF = %f\n",
							i, k, ContiUSS[i].ContiUSSEcho[k].ContiBurstTime,
							(ContiUSS[i].ContiUSSEcho[k].ContiBurstTime + ContiUSS[i].ContiUSSEcho[k].ContiTimeOF),
							_localvar.ToF_Counter[i][k],
							(ContiUSS[i].ContiUSSEcho[k].ContiTimeOF));

					_localvar.ToF_Counter[i][k]++;
				}
				else if ((ContiUSS[i].ContiUSSEcho[k].ContiTimeOF <= _localvar.ToF_Counter[i][k])
						&& (_localvar.DataTrans_Running[i][k] == 1)
						&& (SimCore.CycleNo != _localvar.ToF_Counter_State[i][k]) ) {

					if (ContiUSSParam.DebugOpt == -1)
						Log("Tof-Counter (SensNo %d, EchoNo %d, StartCycle %d, StopCycle %f): Cnt = %f, ContiToF = %f  --> Time is up!!!\n",
							i, k, ContiUSS[i].ContiUSSEcho[k].ContiBurstTime,
							(ContiUSS[i].ContiUSSEcho[k].ContiBurstTime + ContiUSS[i].ContiUSSEcho[k].ContiTimeOF),
							_localvar.ToF_Counter[i][k],
							(ContiUSS[i].ContiUSSEcho[k].ContiTimeOF));

					if ((ContiUSS[i].ContiUSSEcho[k].ContiBurstTime + _localvar.ToF_Counter[i][k]) != SimCore.CycleNo)
						LogErrF(EC_Sim, "ContiUSS: ToF-Counter (%f) StopCycle (%f) does not match current cycle (%d)\n",
								_localvar.ToF_Counter[i][k], (ContiUSS[i].ContiUSSEcho[k].ContiBurstTime + _localvar.ToF_Counter[i][k]), SimCore.CycleNo);

					_localvar.ToF_Counter[i][k] = 0;
					_localvar.DataTrans_Running[i][k] = 0;
				}

				/* (-2-) Wait for Correlator to finsih it's tasks */
				if (_localvar.CorrelatorTarget[i] != Correlator_Counter[i]) {
					Correlator_Counter[i]++;
				}
				/* (-3-) Check for 1st 5 direct echoes */
				else if (ASIC_Counter == 0) {

				}
				else /*This is the end*/ {
					GetCorrelatorTime();
				}

				/* Set to 0 to execute "GetCurrentEchoes()" only once per Sensor / Burst */
				BurstCheck = 0;

				/* Set variable to current cycle. This loop won't be executed twice in this cycle */
				if ((j == -1) || ((_localvar.DataTrans_Running[i][k] == 1) && (_localvar.ToF_Counter[i][k] == 1))) {
					_localvar.ToF_Counter_State[i][k] = SimCore.CycleNo;
					if (ContiUSSParam.DebugOpt == -1)
						Log("Got new ToF_Counter_State=%f for Sens_%d Target_%d\n", _localvar.ToF_Counter_State[i][k], i, k);
				}
			}
		}

		// Erst später interessant
		ASIC_Counter[i] == 4 ? ASIC_Counter[i] = 1 : ASIC_Counter[i]++;
	}


	/* Call DataMapping module  */
	//DataMapping();
	if (ContiUSSParam.DebugOpt == -1)
		Log("---------------------------------------\n");

	return 0;
}



int
SFSP_Eval (void)
{
	for (int i = 0; i < USonicRSICount; i++) {

		/* Avoid feeding queue without having enough echoes */
		if (ContiUSSParam.nEchoesMax[i] < _NO_OF_ECHOES_MAX_) {
			LogErrF(EC_Init, "ContiUSS: Number of max. echoes for sensor %d not enough. Got %d. Expected minimum of 20\n", i, ContiUSSParam.nEchoesMax[i]);
		}

		/* Evaluate if internal or external trigger has been used */
		if (ContiUSS[i].IntTrigger == 1 || ContiUSS[i].ExtTrigger == 1) {

			/* Give rising edge to <FireFlag> */
			ContiUSS[i].ContiUSSFlags.FireFlag = 1;

			SensDataTrans_Eval(i);
			TemperatureManipulation(i);

			/* Reset internal or external trigger status bit */
			ContiUSS[i].IntTrigger == 1 ? ContiUSS[i].IntTrigger = 0 : ContiUSS[i].ExtTrigger = 0;
		}
	}

	return 0;
}



int
ContiUSS_MainCalc ()
{
	int SensorCall[_NO_OF_SENSORS_MAX_] = { 0 };

	/* ********************************** */
	/* **** (1) Cycle Initialization **** */
	/* ********************************** */
	/* a) Set rising edge for <FireFlag> down to '0' for each simulation cycle */
	/* b) Determine when and which NFD mode has to be used */
	if (SimCore.State == SCState_Simulate) {
		for (int i = 0; i < USonicRSICount; i++) { ContiUSS[i].ContiUSSFlags.FireFlag = 0; }

		/* Call every cycle for running parallel to execution */
		SensDataTrans_Eval(-1);

		/* ??? */
		NFDMode();
	}

	/* ******************************* */
	/* **** (2) Cycle Calculation **** */
	/* ******************************* */
	/* Evaluate if main sensor calculation routine should be executed based on MODE_1 (Internal Trigger ) or MODE_2 (External Trigger) */
	if ((SimCore.State == SCState_Simulate)
		&& ((ContiUSSParam.UseFireSchemeFromFile == 1 && InternTrigger() /* True for sample times in parameterset */ )
		||  (ContiUSSParam.UseFireSchemeFromFile == 0 && ExternTrigger() /* True for external function call */ )) ) {

		/* Call main ContiUSS function */
		SFSP_Eval();
	}

	/* ************************* */
	/* **** (3) Cycle Reset **** */
	/* ************************* */
	else if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {

		/* Reset variables at simulation end */
		for (int j = 0; j < USonicRSICount; j++) {
			ContiUSS[j].TimeStamp = 0;
			ContiUSS[j].ContiUSSFlags.FireFlag = 0;
			ContiUSS[j].ContiUSSFlags.NFD = 0;
			SensDataTrans_Eval(j);
		}
		InternTrigger();
		ExternTrigger();
	}

	/* Write CarMaker temperature to all sensors */
	/* ContiUSS temperature can be overtuned via DVA */
	if (SimCore.State == SCState_Simulate) {
		for (int i = 0; i < USonicRSICount; i++) { ContiUSS[i].T_Sensor = Env.Temperature; }
	}

	return 0;
}



void
ContiUSS_Cleanup(void)
{
	if (ContiUSS != NULL)
		free(ContiUSS);
}