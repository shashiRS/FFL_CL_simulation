#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include "PhenoModel.h"
#include "PhenoMapping.h"
#include "USSMathUtils.h"
#include <Vehicle\Sensor_Object.h>

#include "us_drv/us_drv_generated_types.h"
#include "us_drv_def_vfa.h"

#include "vCUS_mfPlot.h"

#if CARMAKER_NUMVER >= 100000
#define Assembly 0
#define Sensors 1
#endif /* CARMAKER_NUMVER */

/* Define Trfobj handle & initiallize handle */
tTrafficObject					*TrfObj = NULL;
tSensorParam					*SensParam = NULL;
tSensorData						*SensData = NULL;
tReflectionData					*ReflectionData = NULL;
tNpDataMfPlot					*MfPlotDataStru = NULL;
tToDatacontour					*ToDatacontour = NULL;
tVector_calc					*env_vector = NULL;
tInt_var						*int_var = NULL;
us_drv::UsDrvDetectionList		*UsRaw = NULL; //Q: Done in us_drv_types.h? A: No, via inline-function "createUsDrvDetectionList" declared in us_drv_detection_list.h
tIntReflData					*IntReflData = NULL;
tSimParameters					*SimParameters = NULL;
tToFwait						*ToFwait = NULL;
tRxState_tof					*RxState_tof = NULL;

US_DRV::VFA28::PdcmRecorderModeFrame	*localPDCMframe	= NULL;

#ifdef vCUS_HIL_MODE
tPdcmFlags			*PdcmFlags = NULL;
tPdcmBuffers		*PdcmBuffers = NULL;
tFrameMirrorBuff	*FrameMirrorBuff = NULL;
#endif

#ifdef _UsRaw_DEBUG_
	tUsRaw_debug		*UsRaw_debug = NULL;
#endif

int nTC, nOS;
double null_vec_2D[2] = { 0 };
double null_vec_3D[3] = { 0 };

#if CARMAKER_NUMVER >= 100000
int ObjSensId[2][64] = { { 0 } };
#endif /* CARMAKER_NUMVER */

int	   uspExec_counter;


void
ReadvCUSParameterFile(void)
{
	char   *path, pre[64], filename[64];


	/* ********************************** */
	/* Get sensor vCUS parameter set file */
	/* ********************************** */

	tInfos *vCUS_Parameterset_Info = InfoNew();

	/* Define 'fname' as the Antenna Map textfile and generate path*/
	sprintf(filename, "vCUS_Parameterset");
	path = PathJoin(3, SimCore.TestRig.DataDir, "Sensor", filename);

	if ((path = InfoLocateFile(path, 0)) == NULL) {
		/* try absolute/relative path to the testrun file */
		path = filename;
	}

	/* Read the content of the file */
	if (iRead2(NULL, vCUS_Parameterset_Info, path, "Sensor") != 0) {
		LogErrF(EC_Init, "vCUS: Error reading USS Parameterset\n");
	}

	/* Check if proper FileIdent is used */
	sprintf(pre, "FileIdent");
	if (strcmp(iGetStrOpt(vCUS_Parameterset_Info, pre, ""), "vCUS-ParamSet") != 0) {
		LogErrF(EC_Init, "vCUS: Wrong file type for USS Parameterset\n");
	}

	/* Load Debug information */
	sprintf(pre, "DebugOpt");
	SimParameters->DebugOpt = iGetInt(vCUS_Parameterset_Info, pre);
	if (SimParameters->DebugOpt && !SimParameters->SWSwitch_SuppressInfo)
		Log("SWSwitch: DEBUG-OPTION ENABELD\n\n");

	sprintf(pre, "DebugPDCM_Frame");
	SimParameters->DebugPDCM_Frame = iGetInt(vCUS_Parameterset_Info, pre);

	/* Load Debug Amp information */
	sprintf(pre, "DebugOpt_Amp");
	SimParameters->DebugOpt_Amp = iGetInt(vCUS_Parameterset_Info, pre);
	if (SimParameters->DebugOpt_Amp && !SimParameters->SWSwitch_SuppressInfo)
		Log("vCUS DEBUG-OPTION for Amplitude ENABLED\n\n");

	/* Load Debug USP information */
	sprintf(pre, "DebugOpt_USP");
	SimParameters->DebugOpt_USP = iGetInt(vCUS_Parameterset_Info, pre);
	if (SimParameters->DebugOpt_USP && !SimParameters->SWSwitch_SuppressInfo)
		Log("vCUS DEBUG-OPTION for USP ENABLED\n\n");

	/* Parameter information */
	sprintf(pre, "min_height_TO");
	SimParameters->min_height_TO = iGetDblOpt(vCUS_Parameterset_Info, pre, 0.02);
	sprintf(pre, "USP_UpdateRate");
	SimParameters->USP_UpdateRate = iGetDblOpt(vCUS_Parameterset_Info, pre, 40);
	sprintf(pre, "HiLOffsetDelay");
	SimParameters->HiLOffsetDelay = iGetDblOpt(vCUS_Parameterset_Info, pre, 0);


	/* Load Stochastic Code information */
	sprintf(pre, "SWSwitch_SFSPon");
	SimParameters->SWSwitch_SFSPon = iGetInt(vCUS_Parameterset_Info, pre);
	sprintf(pre, "SWSwitch_SuppressInfo");
	SimParameters->SWSwitch_SuppressInfo = iGetInt(vCUS_Parameterset_Info, pre);
	sprintf(pre, "SWSwitch_CrossEchoes");
	SimParameters->SWSwitch_CrossEchoes = iGetInt(vCUS_Parameterset_Info, pre);
	sprintf(pre, "SWSwitch_DirectEchoes");
	SimParameters->SWSwitch_DirectEchoes = iGetInt(vCUS_Parameterset_Info, pre);
	sprintf(pre, "SWSwitch_CustomInRange");
	SimParameters->SWSwitch_CustomInRange = iGetInt(vCUS_Parameterset_Info, pre);
	sprintf(pre, "SWSwitch_RxCorection_ToF");
	SimParameters->SWSwitch_RxCorection_ToF = iGetInt(vCUS_Parameterset_Info, pre);
	sprintf(pre, "SWSwitch_RCPmsCorrection");
	SimParameters->SWSwitch_RCPmsCorrection = iGetInt(vCUS_Parameterset_Info, pre);

	if (SimParameters->SWSwitch_SFSPon > 0) {
		if (!SimParameters->SWSwitch_SuppressInfo)
			Log("SWSwitch: USER FILE FOR vCUS FIRE SCHEME ENABELD\n");

		int		nFire_Samples, nSensor_No, MapNo;
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
		nFire_Samples = iGetIntOpt(vCUS_Parameterset_Info, pre, 8);

		sprintf(pre, "nSensor_No");
		nSensor_No = iGetIntOpt(vCUS_Parameterset_Info, pre, 12);

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
		MapNo = iGetIntOpt(vCUS_Parameterset_Info, pre, 0);
		sprintf(pre, "FireSchemeMap%d", MapNo);
		if (!SimParameters->SWSwitch_SuppressInfo)
			Log("\nThe read map %s is as follows...\n", pre);

		if ((FireScheme = iGetTableFltOpt2(vCUS_Parameterset_Info, pre, NULL, nFire_Samples, &cols)) == NULL) {
			LogErrF(EC_Init, "ContiUSS: Can't get mapping data, %d columns, key '%s'. Initialization failed, please check InfoFile", nFire_Samples, pre);
		}

		/* Check if table has been read properly */
		if (FireScheme != NULL) {

			/* Initialize Antenna Map */
			SimParameters->fire_scheme_map = LM2DInit(Sensor_No, nSensor_No, Fire_Samples, nFire_Samples, FireScheme, 1, 1);

			/* Log read file */
			if (!SimParameters->SWSwitch_SuppressInfo) {
				Log("x-Dir: FireSamples\n");
				Log("y-Dir: Sensors\n\n");
			}
			for (int i = 0; i < nSensor_No; i++) {
				for (int j = 0; j < nFire_Samples; j++) {
					/* Read the map: i->FireSamples, j->Sensor_No */
					SimParameters->fire_scheme[i][j] = (uint64_t) LM2DEval(SimParameters->fire_scheme_map, i + 1, j + 1);
					if (!SimParameters->SWSwitch_SuppressInfo)
						Log("%d ", (int) SimParameters->fire_scheme[i][j]);
				}
				if (!SimParameters->SWSwitch_SuppressInfo)
					Log("\n");
			}
			if (!SimParameters->SWSwitch_SuppressInfo)
				Log("\n");
		}

		if (Fire_Samples != NULL)
			free(Fire_Samples);

		if (Sensor_No != NULL)
			free(Sensor_No);
	}
	else {
		if (!SimParameters->SWSwitch_SuppressInfo)
			Log("SWSwitch: USER FILE FOR vCUS FIRE SCHEME DISABLED\n");
	}
	/* ************* End vCUS param file ************** */



	/* ******************************************* */
	/* Get sensor characterisitc & load param file */
	/* ******************************************* */
	tInfos *vCUS_SensCharacteristic_Info = InfoNew();
	int    nRows;
	float  *Gain = NULL;

	sprintf(pre, "SensorCharacteristicFileName");
	char *SensCharFileName = iGetStrOpt(vCUS_Parameterset_Info, pre, "vCUS_SensorCharacteristic_default");
	if (!SimParameters->SWSwitch_SuppressInfo)
		Log("Selected Path for Sensor Characterisitc File Name\n '%s'\n", SensCharFileName);

	/* Define 'fname' as the Antenna Map textfile and generate path*/
	path = PathJoin(3, SimCore.TestRig.DataDir, "Sensor", SensCharFileName);

	if ((path = InfoLocateFile(path, 0)) == NULL) {
		/* try absolute/relative path to the testrun file */
		path = SensCharFileName;
	}

	/* Read the content of the file */
	if (iRead2(NULL, vCUS_SensCharacteristic_Info, path, "Sensor") != 0) {
		LogErrF(EC_Init, "vCUS: Error reading Sensor Characteristic Parameterset\n");
	}

	/* Check if proper FileIdent is used */
	sprintf(pre, "FileIdent");
	if (strcmp(iGetStrOpt(vCUS_SensCharacteristic_Info, pre, ""), "vCUS-USSensorCharacteristic") != 0) {
		LogErrF(EC_Init, "vCUS: Wrong file type for Sensor Characteristic Parameterset\n");
	}

	/* Read constant values */
	sprintf(pre, "Type");
	char *AmplitudeType = iGetStrOpt(vCUS_SensCharacteristic_Info, pre, "");
	if ( (strcmp(AmplitudeType, "SPA") == 0) || (strcmp(AmplitudeType, "SPL") == 0) ) {
		if (strcmp(AmplitudeType, "SPA") == 0)
			SimParameters->SensChar.Type = 0;
		else
			SimParameters->SensChar.Type = 1;
	}
	else {
		LogErrF(EC_Init, "vCUS: Unknown Type for Sensor Characteristic Amplitude\n");
	}
	sprintf(pre, "Azimuth");
	double MaxAzimuth = iGetDblOpt(vCUS_SensCharacteristic_Info, pre, 180);
	sprintf(pre, "Elevation");
	double MaxElevation = iGetDblOpt(vCUS_SensCharacteristic_Info, pre, 180);
	sprintf(pre, "DeltaFoV");
	double MapDelta = iGetDblOpt(vCUS_SensCharacteristic_Info, pre, 5.0);

	sprintf(pre, "SPA0Range");
	SimParameters->SensChar.SPANormRange = iGetDblOpt(vCUS_SensCharacteristic_Info, pre, 0.45);

	/* ***** Read x,y for determining z ***** */
	/* x */
	if (fmod(MaxAzimuth, MapDelta) != 0) {
		LogErrF(EC_Init, "vCUS: 'DeltaFoV' doesn't fit to Azimuth entries\n");
	}
	int NoEntriesAzi = int(MaxAzimuth / MapDelta);
	int j = 0;
	double *EntriesAzi = NULL;
	EntriesAzi = (double *)calloc(NoEntriesAzi + 1, sizeof(double));
	for (int i = 0; i <= NoEntriesAzi; i++) {
		EntriesAzi[i] = i * MapDelta;
	}
	NoEntriesAzi = NoEntriesAzi + 1;

	/* y */
	if (fmod(MaxElevation, MapDelta) != 0) {
		LogErrF(EC_Init, "vCUS: 'DeltaFoV' doesn't fit to Elevation entries\n");
	}
	int NoEntriesEle = int(MaxElevation / MapDelta);
	j = 0;
	double *EntriesEle = NULL;
	EntriesEle = (double *)calloc(NoEntriesEle + 1, sizeof(double));
	for (int i = 0; i <= NoEntriesEle; i++) {
		EntriesEle[i] = i * MapDelta;
	}
	NoEntriesEle = NoEntriesEle + 1;

	if (!SimParameters->SWSwitch_SuppressInfo)
		Log("Number of entries: %d x %d\n", NoEntriesAzi, NoEntriesEle);

	/* z */
	sprintf(pre, "CharacteristicMap");
	if ((Gain = iGetTableFltOpt2(vCUS_SensCharacteristic_Info, pre, NULL, NoEntriesAzi, &nRows)) == NULL) {
		LogErrF(EC_Init, "Sensor Characteristic Map: Can't get mapping data, %d columns, key '%s'",
			NoEntriesAzi, pre);
	}

	/* Derive maximum SPA in map */
	SimParameters->SensChar.MaxSPA = 0;
	for (int i = 0; i < (NoEntriesEle * NoEntriesAzi); i++) {
		if (Gain[i] > SimParameters->SensChar.MaxSPA)
			SimParameters->SensChar.MaxSPA = Gain[i];
	}

	/* Initialize Antenna Map */
	SimParameters->SensChar.LobeMap = LM2DInit2(EntriesAzi, NoEntriesAzi, EntriesEle, NoEntriesEle, Gain, 1, 1);
	SimParameters->SensChar.p0 = LM2DEval(SimParameters->SensChar.LobeMap, 90, 90);
}

void
vCUS_InitReflGen(void)
{
	/* Set variables to 0 */
	memset(int_var, 0, sizeof(tInt_var));
	int_var->targ_ID = -1;
	for (int k = 0; k < _NO_TOS_MAX_; k++) {
		int_var->TO_ID[k] = -1;
	}
	memset(env_vector, 0, sizeof(tVector_calc));
	env_vector->NormalDirDefault[0] = 1;

	return;
}

void
vCUS_ResetVarTREnd(void)
{
  /* Reset all variables at TestRun end (User abort / TR End) */
  memset(SensParam, 0, nOS * sizeof(tSensorParam));
  memset(SensData, 0, nOS * sizeof(tSensorData));
  memset(ReflectionData, 0, us_drv::US_DRV_Consts::US_DRV_MAX_NUM_DETECTIONS * sizeof(tReflectionData));
  memset(MfPlotDataStru, 0, _MF_PLOT_HISTORY_ * sizeof(tNpDataMfPlot));
  memset(ToDatacontour, 0, _NO_OF_ECHOES_MAX_ * sizeof(tToDatacontour));
  memset(int_var, 0, sizeof(tInt_var));
  int_var->targ_ID = -1;
  for (int k = 0; k < _NO_TOS_MAX_; k++) {
    int_var->TO_ID[k] = -1;
  }
  memset(env_vector, 0, sizeof(tVector_calc));
  env_vector->NormalDirDefault[0] = 1;
  memset(UsRaw, 0, sizeof(us_drv::UsDrvDetectionList));
  memset(IntReflData, 0, sizeof(tIntReflData));
  memset(TrfObj, 0, nTC * sizeof(tTrafficObject));
  memset(SimParameters, 0, sizeof(tSimParameters));
  memset(ToFwait, 0, sizeof(tToFwait));

  /*reset daisy chain PDCM buffers*/
  #ifdef vCUS_HIL_MODE
  memset(localPDCMframe, 0, PdcmFlags->nBuses * sizeof(US_DRV::VFA28::PdcmRecorderModeFrame));

  int nBuses;
  if (nOS <= 6) {
    nBuses = 1;
  }
  else if ((nOS > 6) && (nOS % 2 == 0)) {
    nBuses = 2;
  }
  else {
    /*this should not happen - error message upstream*/
    Log("WARNING - vCUS: Incorrect number of sensors per Daisy chain bus!");
  }

  memset(PdcmBuffers, 0, nBuses * sizeof(tPdcmBuffers));
  memset(PdcmFlags, 0, sizeof(tPdcmFlags));
  memset(FrameMirrorBuff, 0, sizeof(tFrameMirrorBuff));
  #endif

  #ifdef _UsRaw_DEBUG_
  memset(UsRaw_debug, 0, sizeof(tUsRaw_debug));
  #endif

  return;
}

int
vCUS_InitData(void)
{
	float	*sensor_orientation = NULL, *sensor_position = NULL;
#if CARMAKER_NUMVER >= 100000
   float *FoVdata = NULL;
   int		nRows;
#endif /* CARMAKER_NUMVER */
   
	int		nCols;
	char	pre[64], sbuf[256];
	double	nFaces;
	uspExec_counter = 0;

#if CARMAKER_NUMVER >= 100000
    vCUS_ReadVhclInfo();
	//Log("\nnOS: %d \n", nOS);
#else 
	nTC = iGetIntOpt(SimCore.TestRun.Inf, "Traffic.N", 0);
	nOS = iGetIntOpt(SimCore.Vhcl.Inf, "Sensor.Object.N", 0);
#endif /* CARMAKER_NUMVER */

	TrfObj			= (tTrafficObject *)	calloc(nTC, sizeof(tTrafficObject));
	SensParam		= (tSensorParam *)		calloc(nOS, sizeof(tSensorParam));
	SensData		= (tSensorData *)		calloc(nOS, sizeof(tSensorData));
	ReflectionData	= (tReflectionData *)	calloc(us_drv::US_DRV_Consts::US_DRV_MAX_NUM_DETECTIONS, sizeof(tReflectionData));
	MfPlotDataStru	= (tNpDataMfPlot *)		calloc(_MF_PLOT_HISTORY_, sizeof(tNpDataMfPlot));
	ToDatacontour	= (tToDatacontour *)	calloc(_NO_OF_ECHOES_MAX_, sizeof(tToDatacontour));
	env_vector		= (tVector_calc *)		calloc(1, sizeof(tVector_calc));
	int_var			= (tInt_var *)			calloc(1, sizeof(tInt_var));
	UsRaw			= (us_drv::UsDrvDetectionList *)	calloc(1, sizeof(us_drv::UsDrvDetectionList));
	IntReflData		= (tIntReflData *)		calloc(1, sizeof(tIntReflData));
	SimParameters	= (tSimParameters *)	calloc(1, sizeof(tSimParameters));
	ToFwait			= (tToFwait *)			calloc(1, sizeof(tToFwait));
	RxState_tof     = (tRxState_tof	*)		calloc(1, sizeof(tRxState_tof));

	
	/*initialise buffers for daisy chain PDCM frame*/
	#ifdef vCUS_HIL_MODE
	int nOS_perBus, nBss;
	if (nOS <= 6) {
		nOS_perBus = nOS;
		nBss = 1;
	}
	else if ((nOS > 6) && (nOS % 2 == 0)) {
		nOS_perBus = nOS / 2;
		nBss = 2;
	}
	else {
		/*this should not happen - error message upstream*/
		Log("WARNING - vCUS: Incorrect number of sensors per Daisy chain bus!");
	}

	PdcmBuffers = (tPdcmBuffers *)calloc(nBss, sizeof(tPdcmBuffers));
	PdcmFlags	= (tPdcmFlags *)calloc(1, sizeof(tPdcmFlags));
	FrameMirrorBuff = (tFrameMirrorBuff *)calloc(1, sizeof(tFrameMirrorBuff));

	/*initialize flags*/
	PdcmFlags->nBuses = nBss;
	PdcmFlags->ussPerBus = nOS_perBus;

	for (int i = 0; i < nBss; i++)
		PdcmFlags->nextSensorID[i]		= -1;
	
	for (int i = 0; i < nOS; i++){
		PdcmFlags->flag_overflow[i]		= 0;
		PdcmFlags->flag_wait4Turn[i]	= 0;
		PdcmFlags->muxDataCnt[i]		= 0;
	}
	PdcmFlags->syncCnt = 0U;
	/*flags done*/

	localPDCMframe = (US_DRV::VFA28::PdcmRecorderModeFrame *)	calloc(PdcmFlags->nBuses, sizeof(US_DRV::VFA28::PdcmRecorderModeFrame));
	#endif

	for (int m = 0; m < _MF_PLOT_HISTORY_; m++) {
		MfPlotDataStru[m].vCUS_CycleId = -1;
	}

	#ifdef _UsRaw_DEBUG_
		UsRaw_debug = (tUsRaw_debug *)calloc(1, sizeof(tUsRaw_debug));
	#endif

	/* Initializing */
	for (int k = 0; k < us_drv::US_DRV_Consts::US_DRV_MAX_NUM_DETECTIONS; k++) {
		UsRaw->detections[k].sensorId = 255U;
	}
	int_var->targ_ID = -1;
	for (int k = 0; k < _NO_TOS_MAX_; k++) {
		int_var->TO_ID[k] = -1;
	}
	env_vector->NormalDirDefault[0] = 1;

	/* Read vCUS Parameter File */
	ReadvCUSParameterFile();

	/* DDict/IFile -> Sensor */
	for (int i = 0; i < nOS; i++) {
#if CARMAKER_NUMVER >= 100000
		sprintf(pre, "Sensor.Param.%d.FoV", ObjSensId[Sensors][i]);
		FoVdata = iGetTableFltOpt2(SimCore.Vhcl.Inf, pre, NULL, 1, &nCols);
		SensParam[i].Azimuth = (double)FoVdata[0];
		SensParam[i].Elevation = (double)FoVdata[1];
		//Log("Sensor.Param.%d.FoV: %f %f \n", i, FoVdata[0], FoVdata[1]);

		sprintf(pre, "Sensor.Param.%d.Range", ObjSensId[Sensors][i]);
		SensParam[i].Range = iGetDblOpt(SimCore.Vhcl.Inf, pre, 9.0);

		sprintf(pre, "Sensor.%d.rot", ObjSensId[Assembly][i]);
#else
		sprintf(pre, "Sensor.Object.%d.rot", i);
#endif /* CARMAKER_NUMVER */
		if ((sensor_orientation = iGetTableFltOpt2(SimCore.Vhcl.Inf, pre, NULL, 1, &nCols)) == NULL)
			LogErrF(EC_Init, "Can't read ''Sensor.Object.%d.rot''\n", i);
		SensParam[i].orientation = sensor_orientation[2];
#if CARMAKER_NUMVER >= 100000
		//Log("Sensor.%d.rot: %f, %f, %f, \n", i, sensor_orientation[0], sensor_orientation[1], sensor_orientation[2]);
		
		sprintf(pre, "Sensor.%d.pos", ObjSensId[Assembly][i]);
#else
		sprintf(pre, "Sensor.Object.%d.pos", i);
#endif /* CARMAKER_NUMVER */
		if ((sensor_position = iGetTableFltOpt2(SimCore.Vhcl.Inf, pre, NULL, 1, &nCols)) == NULL)
			LogErrF(EC_Init, "Can't read ''Sensor.Object.%d.pos''\n", i);
		for (int j = 0; j < 3; j++) {
			SensParam[i].Fr1_position[j] = sensor_position[j];
		}
#if CARMAKER_NUMVER >= 100000
		//Log("Sensor.%d.pos: %f, %f, %f, \n\n", i, sensor_position[0], sensor_position[1], sensor_position[2]);
#else
		sprintf(pre, "Sensor.Object.%d.alpha", i);
		SensParam[i].Azimuth = iGetDblOpt(SimCore.Vhcl.Inf, pre, 180.0);
		sprintf(pre, "Sensor.Object.%d.theta", i);
		SensParam[i].Elevation = iGetDblOpt(SimCore.Vhcl.Inf, pre, 80.0);
		sprintf(pre, "Sensor.Object.%d.range", i);
		SensParam[i].Range = iGetDblOpt(SimCore.Vhcl.Inf, pre, 8.0);
#endif /* CARMAKER_NUMVER */
		/* Generate DDict Entries for all Sensors Fr0 + NP for visualization*/
		sprintf(sbuf, "SensData.%d.SensPos.x", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[0], DVA_None);
		sprintf(sbuf, "SensData.%d.SensPos.y", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[1], DVA_None);
		sprintf(sbuf, "SensData.%d.SensPos.z", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[2], DVA_None);

#if CARMAKER_NUMVER >= 100000
		/* Generate DDict Entries for sensors ToF data*/
		PDCM_DDict(i);
#else
		/*report n of reflections  SensData[i].Target.nReflections*/
		sprintf(sbuf, "SensData.%d.Target.nReflections", i);
		DDefInt(NULL, sbuf, "m", &SensData[i].Target.nReflections, DVA_None);
#endif /* CARMAKER_NUMVER */
		for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
			sprintf(sbuf, "SensData.%d.Target.NPPos.x.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].Target.Fr0_position_NP[k][0], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.NPPos.y.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].Target.Fr0_position_NP[k][1], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.NPPos.z.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].Target.Fr0_position_NP[k][2], DVA_None);
		}
	}

#ifndef vCUS_HIL_MODE
	for (int k = 0; k < _NO_OF_ECHOES_MAX_; k++) {
		sprintf(sbuf, "ReflectionData.%d.Rx", k);
		DDefInt(NULL, sbuf, "m", &ReflectionData[k].Rx, DVA_None);
		sprintf(sbuf, "ReflectionData.%d.TOid", k);
		DDefInt(NULL, sbuf, "m", &ReflectionData[k].Detc_TO_ID, DVA_None);

		sprintf(sbuf, "ReflectionData.%d.Tx", k);
		DDefInt(NULL, sbuf, "m", &ReflectionData[k].Tx, DVA_None);

		sprintf(sbuf, "ReflectionData.%d.TimeTag", k);
		DDefULLong(NULL, sbuf, "m", &ReflectionData[k].TimeTag, DVA_None);
		sprintf(sbuf, "ReflectionData.%d.TimeOF", k);
		DDefULLong(NULL, sbuf, "m", &ReflectionData[k].TimeOF, DVA_None);

		sprintf(sbuf, "ReflectionData.%d.Amp", k);
		DDefUShort(NULL, sbuf, "m", &ReflectionData[k].Amplitude, DVA_None);

		sprintf(sbuf, "ReflectionData.%d.NPPos.x", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_NP[0], DVA_None);
		sprintf(sbuf, "ReflectionData.%d.NPPos.y", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_NP[1], DVA_None);
		sprintf(sbuf, "ReflectionData.%d.NPPos.z", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_NP[2], DVA_None);

		sprintf(sbuf, "ReflectionData.%d.RxPos.x", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_Sens[0], DVA_None);
		sprintf(sbuf, "ReflectionData.%d.RxPos.y", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_Sens[1], DVA_None);
		sprintf(sbuf, "ReflectionData.%d.RxPos.z", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_Sens[2], DVA_None);

		sprintf(sbuf, "ReflectionData.%d.TxPos.x", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_Sens_Tx[0], DVA_None);
		sprintf(sbuf, "ReflectionData.%d.TxPos.y", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_Sens_Tx[1], DVA_None);
		sprintf(sbuf, "ReflectionData.%d.TxPos.z", k);
		DDefDouble4(NULL, sbuf, "m", &ReflectionData[k].Fr0_position_Sens_Tx[2], DVA_None);

		sprintf(sbuf, "ReflectionData.%d.CycleNo", k);
		DDefInt(NULL, sbuf, "m", &ReflectionData[k].CycleNo, DVA_None);
	}

	for (int m = 0; m < _MF_PLOT_HISTORY_ ; m++) {

		for (int k = 0; k < _MAX_ARR_VCUS2MFPLOT_; k++) {
			sprintf(sbuf, "MfPlotDataStru.%d.CoordArrayFr1.x.%d", m, k);
			DDefFloat(NULL, sbuf, "m", &MfPlotDataStru[m].CoordArrayFr1[k][0], DVA_None);
			sprintf(sbuf, "MfPlotDataStru.%d.CoordArrayFr1.y.%d", m, k);
			DDefFloat(NULL, sbuf, "m", &MfPlotDataStru[m].CoordArrayFr1[k][1], DVA_None);

			sprintf(sbuf, "MfPlotDataStru.%d.CoordArrayFr0.x.%d", m, k);
			DDefFloat(NULL, sbuf, "m", &MfPlotDataStru[m].CoordArrayFr0[k][0], DVA_None);
			sprintf(sbuf, "MfPlotDataStru.%d.CoordArrayFr0.y.%d", m, k);
			DDefFloat(NULL, sbuf, "m", &MfPlotDataStru[m].CoordArrayFr0[k][1], DVA_None);
		}
	}
	 /*output n of events for every cycle*/
	sprintf(sbuf, "ReflDataCnt");
	DDefUShort(NULL, sbuf, " ", &IntReflData->ReflDataCnt, DVA_None);

	#ifdef vCUS_HIL_MODE
		/*for HiL PDCM debug*/
		sprintf(sbuf, "nPdcmFramePayloads");
		DDefInt(NULL, sbuf, " ", &PdcmFlags->nPayldPDCM, DVA_None);
		sprintf(sbuf, "nPdcmFrameBursts");
		DDefInt(NULL, sbuf, " ", &PdcmFlags->nBurstsPDCM, DVA_None);
		sprintf(sbuf, "nDeafTimeRejected");
		DDefInt(NULL, sbuf, " ", &PdcmFlags->nDeafTimeEchos, DVA_None);
		sprintf(sbuf, "nOverNextSFSPEchoes");
		DDefInt(NULL, sbuf, " ", &PdcmFlags->nOverDTimeEchoes, DVA_None);
		sprintf(sbuf, "nExpiredEchoes");
		DDefInt(NULL, sbuf, " ", &PdcmFlags->nEchoesExpired, DVA_None);
		sprintf(sbuf, "SyncCnt");
		DDefUChar(NULL, sbuf, " ", &PdcmFlags->syncCnt, DVA_None);
	#endif
   #endif /* vCUS_HIL_MODE */

	/*for all traffic objects - max _NO_OF_ECHOES_MAX_ traffic objects (50)*/
	sprintf(sbuf, "TrafficObjNumber");
	DDefInt(NULL, sbuf, " ", &Traffic.nObjs, DVA_None);
	tTrafficObj *Obj;
	
	for (int i = 0; i < Traffic.nObjs; i++) {
		Obj = Traffic_GetByTrfId(i);

		//Traf obj name
		strcpy(ToDatacontour[i].ToName, Obj->Cfg.Name);
		sprintf(sbuf, "TraffObjOutline.%d.ToName", i);
		DDefChar(NULL, sbuf, ToDatacontour[i].ToName, "N", DVA_None);

		//Traf obj type
		strcpy(ToDatacontour[i].ToType, Obj->Cfg.Info);
		sprintf(sbuf, "TraffObjOutline.%d.ToType", i);
		DDefChar(NULL, sbuf, ToDatacontour[i].ToType, "T", DVA_None);

		sprintf(sbuf, "TraffObjOutline.%d.ToID", i);//Traf obj id
		DDefInt(NULL, sbuf, " ", &ToDatacontour[i].ToID, DVA_None);
		sprintf(sbuf, "TraffObjOutline.%d.ToHeight", i);// Traf obj height
		DDefDouble4(NULL, sbuf, "m", &ToDatacontour[i].ToHeight, DVA_None);
		
		sprintf(sbuf, "TraffObjOutline.%d.nDataP", i);// Traf obj n data points / number of TO faces
		DDefInt(NULL, sbuf, " ", &ToDatacontour[i].nDataP, DVA_None);

		//get the number of faces for contour
		bool isCuboid = Obj->Cfg.Envelope.Mode == EnvelopeMode_Cuboid;
		nFaces = isCuboid ? 4 : (Obj->Cfg.Envelope.nArea);

		/*for all tiles - max TRF_CONTOURPNTS tiles (200)*/
		for (int k = 0; k < nFaces; k++) {
			sprintf(sbuf, "TraffObjOutline.%d.Point0xyEdge.%d.x", i, k);
			DDefDouble4(NULL, sbuf, "m", &ToDatacontour[i].Point0xyEdge[k][0], DVA_None);
			sprintf(sbuf, "TraffObjOutline.%d.Point0xyEdge.%d.y", i, k);
			DDefDouble4(NULL, sbuf, "m", &ToDatacontour[i].Point0xyEdge[k][1], DVA_None);
		}
	}

	/*############moving vehicle correction debugging&validation*/
	/*when using make sure rxCorrection_egoMotion is called only once
	i.e., use with one sensor for direct echoes or call only once in vCUS_ReflectionGenerator for cross echoes*/
	/*
	sprintf(sbuf, "RxState_tof.isDirectEcho");
	DDefInt(NULL, sbuf, "m", &RxState_tof->isDirectEcho, DVA_None);
	sprintf(sbuf, "RxState_tof.egoVelocity");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->egoVelocity, DVA_None);
	sprintf(sbuf, "RxState_tof.initialToF");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->initialToF, DVA_None);
	sprintf(sbuf, "RxState_tof.correctedToF");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->correctedToF, DVA_None);
	sprintf(sbuf, "RxState_tof.initialLof");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->initialLof, DVA_None);
	sprintf(sbuf, "RxState_tof.correctedLof");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->correctedLof, DVA_None);
	sprintf(sbuf, "RxState_tof.Rx_displacement");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->Rx_displacement, DVA_None);
	sprintf(sbuf, "RxState_tof.NP_displacement");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->NP_displacement, DVA_None);
	sprintf(sbuf, "RxState_tof.vehYaw");
	DDefDouble4(NULL, sbuf, "m", &RxState_tof->vehYaw, DVA_None);
	*/
	/*############*/

	/* WORK PACKAGE SILVIU */
	for (int i = 0; i < nOS; i++) {
		getVirtualSensorPos(i, nOS);
	}
	return 0;
}



void
vCUS_Cleanup(void)
{
	if (TrfObj != NULL)
		free(TrfObj);

	if (SensParam != NULL)
		free(SensParam);

	if (SensData != NULL)
		free(SensData);

	if (ReflectionData != NULL)
		free(ReflectionData);

	if (MfPlotDataStru != NULL)
		free(MfPlotDataStru);

	if (ToDatacontour != NULL)
		free(ToDatacontour);

	if (env_vector != NULL)
		free(env_vector);

	if (int_var != NULL)
		free(int_var);

	if (IntReflData != NULL)
		free(IntReflData);

	if (SimParameters != NULL)
		free(SimParameters);

	/*cleanup daisy chain pdcm buffers*/
	#ifdef vCUS_HIL_MODE
	if (PdcmBuffers != NULL)
		free(PdcmBuffers);
	if (PdcmFlags != NULL)
		free(PdcmFlags);
	if (FrameMirrorBuff != NULL)
		free(FrameMirrorBuff);
	#endif

	#ifdef _UsRaw_DEBUG_
		if (UsRaw_debug != NULL)
			free(UsRaw_debug);
	#endif
}

#if CARMAKER_NUMVER >= 100000
void
PDCM_DDict(int ID)
{
	char sbuf[256];

	for(int m = 0; m < _NO_OF_ECHOES_MAX_; m++)
	{
		sprintf(sbuf, "SensData[%d].Target.ToF[%d]", ID, m);
		DDefDouble(NULL, sbuf, "ms", &SensData[ID].Target.ToF[m], DVA_None);
	}
}

void	
vCUS_ReadVhclInfo(void)
{
	char sbuf[256], name[256];
	char *SensName[64];
	int TotalnOS_Assembly = 0, TotalnOS_Sensors = 0, k1 = 0, k2 = 0, i;
	
	nTC = iGetIntOpt(SimCore.TestRun.Inf, "Traffic.N", 0);
	TotalnOS_Assembly = iGetIntOpt(SimCore.Vhcl.Inf, "Sensor.N", 0);
	TotalnOS_Sensors = iGetIntOpt(SimCore.Vhcl.Inf, "Sensor.Param.N", 0);

	for (i = 0; i < TotalnOS_Assembly; i++)
	{
		sprintf(sbuf, "Sensor.%d.name", i);
		SensName[i] = iGetStr(SimCore.Vhcl.Inf, sbuf);
		sprintf(name, "USS%02d", k1);
		if (strcmp(SensName[i], name) == 0)
		{
			ObjSensId[Assembly][k1++] = i;
		}
	}
	nOS = k1;

	for (i = 0; i < TotalnOS_Sensors; i++)
	{
		sprintf(sbuf, "Sensor.Param.%d.Name", i);
		SensName[i] = iGetStr(SimCore.Vhcl.Inf, sbuf);
		sprintf(name, "USS%02d", k2);
		if (strcmp(SensName[i], name) == 0)
		{
			ObjSensId[Sensors][k2++] = i;
		}
	}
}
#endif /* CARMAKER_NUMVER */