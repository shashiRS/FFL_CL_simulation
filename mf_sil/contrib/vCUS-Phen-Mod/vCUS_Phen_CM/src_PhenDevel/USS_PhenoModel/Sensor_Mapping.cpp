#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <cassert>
#include <cstdlib>
#include <algorithm>

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include "ReflectionFunc.h"
#include "PhenoModel.h"
#include "PhenoMapping.h"
#include "USSMathUtils.h"
#include <Vehicle\Sensor_Object.h>
#include <Vehicle/Sensor_Inertial.h>
#include <Environment.h>

#include "us_drv/us_drv_generated_types.h"
#include "us_drv_def_vfa.h"

#include "vCUS_mfPlot.h"

#ifdef vCUS_HIL_MODE
extern us_drv::UsRaw *UsRaw;
extern US_DRV::VFA28::PdcmRecorderModeFrame *localPDCMframe;
#endif

extern double null_vec_2D[2], null_vec_3D[3];
extern int nTC, nOS;

int
SensData_Mapping(int recvUssID, int BurstUssID, int EchoType, int cnt) {
	/*recvUssID receiving ultrasonic sensor id
	BurstUssID bursting (sending) USS id*/
	
	double LoF, azimuthRx, azimuthTx, elevationRx, elevationTx;
	double distanceNpRx, distanceNpTx;
	double TO_Reflectivity = 1;

	VEC_Assign(SensData[BurstUssID].Target.Fr0_position_NP[cnt], int_var->Fr0_position_NP[int_var->targ_ID]);
	
	/*convert NP x y z to FR1 - this is not middle of the rear axis!*/
	double sinCalc, cosCalc;
	sinCalc = sin(Vehicle.Yaw);
	cosCalc = cos(Vehicle.Yaw);

	/* this is FR0 to FR1
		SensData[BurstUssID].Target.Fr1_position_NP[cnt][0] = SensData[BurstUssID].Target.Fr0_position_NP[cnt][0] * cosCalc
			+ SensData[BurstUssID].Target.Fr0_position_NP[cnt][1] * sinCalc // sin(Vehicle.Fr1A.r_zyx[2]) -
			- (Vehicle.Fr1A.t_0[0] * cosCalc + Vehicle.Fr1A.t_0[1] * sinCalc);
		SensData[BurstUssID].Target.Fr1_position_NP[cnt][1] = - SensData[BurstUssID].Target.Fr0_position_NP[cnt][0] * sinCalc
			+ SensData[BurstUssID].Target.Fr0_position_NP[cnt][1] * cosCalc
			- (- Vehicle.Fr1A.t_0[0] * sinCalc + Vehicle.Fr1A.t_0[1] * cosCalc);
	*/

	/* this is FR0 to inertial*/
	SensData[BurstUssID].Target.Fr1_position_NP[cnt][0] = SensData[BurstUssID].Target.Fr0_position_NP[cnt][0] * cosCalc
		+ SensData[BurstUssID].Target.Fr0_position_NP[cnt][1] * sinCalc // sin(Vehicle.Fr1A.r_zyx[2]) -
		- (InertialSensor[1].Pos_0[0] * cosCalc + InertialSensor[1].Pos_0[1] * sinCalc);
	SensData[BurstUssID].Target.Fr1_position_NP[cnt][1] = -SensData[BurstUssID].Target.Fr0_position_NP[cnt][0] * sinCalc
		+ SensData[BurstUssID].Target.Fr0_position_NP[cnt][1] * cosCalc
		- (-InertialSensor[1].Pos_0[0] * sinCalc + InertialSensor[1].Pos_0[1] * cosCalc);
	
	/*z not used atm*/
	SensData[BurstUssID].Target.Fr1_position_NP[cnt][2] = SensData[BurstUssID].Target.Fr0_position_NP[cnt][2] - Vehicle.Fr1A.t_0[2];//unless there is some roll&pitch

	/* Distinguish between DIRECT(0) and CROSS(1) echoes */
	if (EchoType == 1) {
		distanceNpTx = VEC_Abs(SensData[BurstUssID].Fr0_position_Sens, SensData[BurstUssID].Target.Fr0_position_NP[cnt]);
		distanceNpRx = VEC_Abs(SensData[recvUssID].Fr0_position_Sens, SensData[BurstUssID].Target.Fr0_position_NP[cnt]);
		SensData[BurstUssID].Target.vTarget_Flag[cnt] = 1;
		
	}
	else if (EchoType == 0) {
		distanceNpTx = VEC_Abs(SensData[BurstUssID].Fr0_position_Sens, SensData[recvUssID].Target.Fr0_position_NP[cnt]);
		//distanceNpRx = distanceNpTx;
		distanceNpRx = VEC_Abs(int_var->Rx_coord_corrected[int_var->targ_ID], SensData[recvUssID].Target.Fr0_position_NP[cnt]);
		SensData[BurstUssID].Target.vTarget_Flag[cnt] = 0;
	}

	LoF											= distanceNpTx + distanceNpRx;
	SensData[BurstUssID].Target.ToF[cnt]		= 2 * ( LoF / (2 * DistanceToTofFactor(Env.Temperature) ) ); // Calculate in ms
	SensData[BurstUssID].Target.Tx[cnt]			= BurstUssID;//geometry is calculated when BurstUssID is sending a burst
	
	/*geometry is calculated for neighboring sensors (which are receiving), when BurstUssID is sending, 
	in geometry calculations recvUssID represents one of the neighboring sensors, or BurstUssID for direct echoes*/ 
	SensData[BurstUssID].Target.Rx[cnt]			= recvUssID;
	
	SensData[BurstUssID].Target.Detc_TO_ID[cnt]	= int_var->Detc_TO_ID[int_var->targ_ID];
	VEC_Assign(SensData[BurstUssID].Target.Fr0_position_Sens_Tx[cnt], SensData[BurstUssID].Fr0_position_Sens);

	/*set up amplitude calculation*/
	/*============================*/
	double Tx_direction, Rx_direction;
	//double Tx_elevation, Rx_elevation;/*no info on USS elevation, assume zero*/
	double TxFOV[2], RxFOV[2], vehYaw_DegTx, vehYaw_DegRx;
	double azimuthRx_Fr0, azimuthTx_Fr0;

	/*calculate min max angles of sensor FOV and USS direction*/
	vehYaw_DegTx = rad2deg * Vehicle.Yaw; //Tx FoV doesn't change between transmission and reception
	vehYaw_DegRx = int_var->egoYaw_Reception[int_var->targ_ID];
	/* rad2deg * Vehicle.Yaw can be e.g. -224 , correct this*/
	if (vehYaw_DegTx >= -360 && vehYaw_DegTx < -180) vehYaw_DegTx += 360;
	if (vehYaw_DegRx >= -360 && vehYaw_DegRx < -180) vehYaw_DegRx += 360;
	getSensorFOV(recvUssID, vehYaw_DegRx, RxFOV, &Rx_direction);
	getSensorFOV(BurstUssID, vehYaw_DegTx, TxFOV, &Tx_direction);
	
	/*convert azimuth from FR0 to +/- 90 relative to USS direction*/
	azimuthRx_Fr0	= int_var->NPDirRx_Fr0[int_var->targ_ID];
	azimuthTx_Fr0	= int_var->NPDirTx_Fr0[int_var->targ_ID];
	doAnglesConvention(RxFOV, &azimuthRx_Fr0);
	doAnglesConvention(TxFOV, &azimuthTx_Fr0);
	azimuthRx		= azimuthRx_Fr0 - Rx_direction;
	azimuthTx		= azimuthTx_Fr0 - Tx_direction;

	/*assumes sensor elevation == 0; now in FR0 -  the same with +/- 90  relative to USS direction*/
	elevationRx		= int_var->NPElevRx_Fr0[int_var->targ_ID];
	elevationTx		= int_var->NPElevTx_Fr0[int_var->targ_ID];

	/*call Amp func & map*/
	SensData[BurstUssID].Target.echoAmplitude[cnt] =
		EvalAmplitudeMapping(azimuthTx, elevationTx, distanceNpTx, azimuthRx, elevationRx, distanceNpRx, TO_Reflectivity);
	/*============================*/

	/*Temporary code for debugging
	Log("to map amp %f id %d count %d\n", SensData[BurstUssID].Target.echoAmplitude[cnt], BurstUssID, cnt);
	
	int TOid = SensData[BurstUssID].Target.Detc_TO_ID[cnt];
	Log("Rx: id %d direction %f, FoV min %f max %f Fr0 npDir %f relative azimuth %f elevation %f TOid %d echoType %d\n",
		recvUssID, Rx_direction, RxFOV[0], RxFOV[1], azimuthRx_Fr0, azimuthRx, elevationRx, TOid, EchoType);
	Log("Tx: id %d direction %f, FoV min %f max %f Fr0 npDir %f relative azimuth %f elevation %f TOid %d echoType %d\n", 
		BurstUssID, Tx_direction, TxFOV[0], TxFOV[1], azimuthTx_Fr0, azimuthTx, elevationTx, TOid, EchoType);
	/**/

	return 0;
}


int
intvar_Mapping(int i, int Tx, int Rx, int k , int cnt) {
	/*mapping for direct echoes*/

	//tObjectSensorObj *pOSO = ObjectSensor_GetObject(Tx, i);
	//int_var->targ_distance[cnt] = pOSO->NearPnt.ds_p;

	if (cnt >= _NO_OF_ECHOES_MAX_) {
		Log("WARNING - vCUS: Number of max echoes reached : Max->%d | Got->%d !\nI'll discard the remaining %d echoes!", _NO_OF_ECHOES_MAX_, cnt + 1, (cnt + 1)-_NO_OF_ECHOES_MAX_);
		return 0;
	}

	int_var->Detc_TO_ID[cnt]			= i;
	int_var->Detc_TO_Surface_ID[cnt]	= k;
	VEC_Assign(int_var->Fr0_position_NP[cnt], TrfObj[i].NearestPointPos_Fr0[Tx]);
	VEC_Assign(int_var->Rx_coord_corrected[cnt], SensData[Rx].Fr0_position_Sens);//without ToF correction (see below)
	int_var->NPDir_Fr0[cnt]				= TrfObj[i].NearestPointDir_Fr0[Tx];
	int_var->Tx[cnt]					= Tx;
	int_var->Rx[cnt]					= Rx;
	
	/*for debugging: store uncorrected NP position*/
	VEC_Assign(int_var->NP_uncorrected[cnt], int_var->Fr0_position_NP[cnt]);

	/*azimuth */
	int_var->NPDirRx_Fr0[cnt] = TrfObj[i].NearestPointDir_Fr0[Tx];//without ToF correction (see below)
	int_var->NPDirTx_Fr0[cnt] = TrfObj[i].NearestPointDir_Fr0[Tx];
	
	/*elevation*/
	int_var->NPElevRx_Fr0[cnt] = TrfObj[i].NearestPointElev_Fr0[Tx];// Rx to Np elevation
	int_var->NPElevTx_Fr0[cnt] = TrfObj[i].NearestPointElev_Fr0[Tx];// Rx to Np elevation

	/*ego yaw for Rx FoV calculation*/
	int_var->egoYaw_Reception[cnt] = rad2deg * Vehicle.Yaw;//without ToF correction (see below)

	int_var->targ_distance[cnt] = VEC_Abs(SensData[Tx].Fr0_position_Sens, int_var->Fr0_position_NP[cnt]);

	/*change Rx parameters if ToF correction is on*/
	if (SimParameters->SWSwitch_RxCorection_ToF) {
		/*update NP coordinates*/
		VEC_Assign(int_var->Fr0_position_NP[cnt], RxState_tof->nearestPoint_Coord);

		/*update Rx coordinates*/
		VEC_Assign(int_var->Rx_coord_corrected[cnt], RxState_tof->coord_RxFr0_updated);
		
		/*update Rx to Np azimuth*/
		int_var->NPDirRx_Fr0[cnt] = RxState_tof->Rx2Np_orientation;
		int_var->NPDirTx_Fr0[cnt] = RxState_tof->Tx2Np_orientation;

		/*elevation after ToF taken the same as before ToF -- to update, Np is changing*/

		/*vehicle yaw at reception*/
		int_var->egoYaw_Reception[cnt] = RxState_tof->egoYaw_Final;
	}
	   	 
	return 0;
}

int
intvar_Mapping(int i, int Tx, int Rx, int k, int cnt, int nUSSs) {
	/*overloaded function for CrossEchoes	
			-- maps NPs for cross echoes to sensor Rx
			-- before calling check if there is a virtual NP for cross echo TrfObj[i].vNpExists[Tx || Rx]*/

	//int_var-> has no flag for virtual sensor: if Rx != Tx, data is for cross reflection
	//nUSSs - number of sensors in current simulation

	if (cnt >= _NO_OF_ECHOES_MAX_) {
		Log("WARNING - vCUS: Number of max echoes reached : Max->%d | Got->%d \n", _NO_OF_ECHOES_MAX_, cnt+1);
		return 0;
	}

	int ussRight, ussLeft, vUssID;//real sensors to the right and left of Tx, and virtual sensor ID
	ussLeft		= (Tx == 0) ? (nUSSs - 1) : (Tx - 1);
	ussRight	= (Tx == (nUSSs - 1)) ? 0 : (Tx + 1);
	
	int_var->Detc_TO_ID[cnt] = i;
	int_var->Detc_TO_Surface_ID[cnt] = k;
	int_var->Tx[cnt] = Tx;
	int_var->Rx[cnt] = Rx;

	VEC_Assign(int_var->Rx_coord_corrected[cnt], SensData[Rx].Fr0_position_Sens);//without ToF correction (see below)

	/*do assignments based on Rx and Tx values*/
	if (Rx == ussLeft) {
		vUssID = Rx;
		/*if Rx is at the left(counterclockwise), the virtual sensor ID is the same with Rx
		config is Rx - vUssID - Tx*/

		//double-check if virtual NP exists
		if (!TrfObj[i].vNpExists[vUssID]) {
			LogErrF(EC_Init, "vCUS: Bad function call, no virtual NP exists!");
		}

		/*- if Rx is left of Tx take data from virtual sensor Rx*/
		VEC_Assign(int_var->Fr0_position_NP[cnt], TrfObj[i].vNearestPointPos_Fr0[vUssID]);
		
		//direction of nearest point of NP for virtual sensor ussLeft as seen from the right (from real USS Tx)
		int_var->NPDir_Fr0[cnt]		= TrfObj[i].vNpDirRight[Tx];

		/*azimuth of NP from Rx and Tx*/
		int_var->NPDirRx_Fr0[cnt] = TrfObj[i].vNpDirLeft[vUssID];// Rx to Np direction
		int_var->NPDirTx_Fr0[cnt] = TrfObj[i].vNpDirRight[vUssID];// Tx to Np direction

		/*elevation of NP from Rx and Tx*/
		int_var->NPElevRx_Fr0[cnt] = TrfObj[i].vNpElevLeft[vUssID];// Rx to Np elevation, Rx to the left of vUSS
		int_var->NPElevTx_Fr0[cnt] = TrfObj[i].vNpElevRight[vUssID];// Tx to Np elevation, Tx to the right of vUSS

	}
	else if (Rx == ussRight) {
		vUssID = Tx;
		/*if Rx is at the right(clockwise), the virtual sensor ID is the same with Tx
		config is Tx - vUssID - Rx*/

		//double-check if virtual NP exists
		if (!TrfObj[i].vNpExists[vUssID]) {
			LogErrF(EC_Init, "vCUS: Bad function call, no virtual NP exists!");
		}

		/*- if Rx is right of Tx take data from virtual sensor Tx */
		VEC_Assign(int_var->Fr0_position_NP[cnt], TrfObj[i].vNearestPointPos_Fr0[vUssID]);

		//direction of nearest point of NP for virtual sensor Tx as seen from the left (from real USS Tx)
		int_var->NPDir_Fr0[cnt]		= TrfObj[i].vNpDirLeft[Tx];

		/*azimuth of NP from Rx and Tx*/
		int_var->NPDirRx_Fr0[cnt] = TrfObj[i].vNpDirRight[vUssID];// Rx to Np direction
		int_var->NPDirTx_Fr0[cnt] = TrfObj[i].vNpDirLeft[vUssID];// Tx to Np direction

		/*elevation of NP from Rx and Tx*/
		int_var->NPElevRx_Fr0[cnt] = TrfObj[i].vNpElevRight[vUssID];// Rx to Np elevation, Rx to the right of vUSS
		int_var->NPElevTx_Fr0[cnt] = TrfObj[i].vNpElevLeft[vUssID];// Tx to Np elevation,  Tx to the left of vUSS

	}
	else {
		LogErrF(EC_Init, "vCUS: Cross-echo, no neighboring sensor detected!");
	}

	/*for debugging: store uncorrected NP position*/
	VEC_Assign(int_var->NP_uncorrected[cnt], int_var->Fr0_position_NP[cnt]);

	/*ego yaw for Rx FoV calculation*/
	int_var->egoYaw_Reception[cnt] = rad2deg * Vehicle.Yaw;//without ToF correction (see below)

	/* distance from current NP to sensor Tx (i.e., sending the cross-echo) */
	int_var->targ_distance[cnt] = VEC_Abs(SensData[Tx].Fr0_position_Sens, int_var->Fr0_position_NP[cnt]);
	
	/*change Rx parameters if ToF correction is on*/
	if (SimParameters->SWSwitch_RxCorection_ToF) {
		/*update np coordinates*/
		VEC_Assign(int_var->Fr0_position_NP[cnt], RxState_tof->nearestPoint_Coord);

		/*update Rx coordinates*/
		VEC_Assign(int_var->Rx_coord_corrected[cnt], RxState_tof->coord_RxFr0_updated);

		/*update Rx & Tx to Np azimuth*/
		int_var->NPDirRx_Fr0[cnt] = RxState_tof->Rx2Np_orientation;
		int_var->NPDirTx_Fr0[cnt] = RxState_tof->Tx2Np_orientation;
		
		/*elevation after ToF taken the same as before ToF -- to update, Np is changing*/

		/*vehicle yaw at reception*/
		int_var->egoYaw_Reception[cnt] = RxState_tof->egoYaw_Final;
	}


	return 0;
}

int
mfPlot_Mapping (tReflectionData *pRD, int reflCnt) {
	/*get data for mfplot if it's and echo, not a burst; only show first _NO_OF_ECHOES_MAX_ NPs*/
	if ((pRD[reflCnt].BurstIndic == 0) && (MfPlotDataStru[0].nReflPnts < _MAX_ARR_VCUS2MFPLOT_)) {

		/*do not show duplicate coordinates in mf_plot*/
		if ((MfPlotDataStru[0].nReflPnts > 0) &&
			(pRD[reflCnt].Fr1_position_NP[0] == MfPlotDataStru[0].CoordArrayFr1[MfPlotDataStru[0].nReflPnts - 1][0] &&
				pRD[reflCnt].Fr1_position_NP[1] == MfPlotDataStru[0].CoordArrayFr1[MfPlotDataStru[0].nReflPnts - 1][1]))
			return 0;

		/*add data to position 0, last cycle is always in pos 0*/
		MfPlotDataStru[0].CoordArrayFr1[MfPlotDataStru[0].nReflPnts][0] = (float)pRD[reflCnt].Fr1_position_NP[0];
		MfPlotDataStru[0].CoordArrayFr1[MfPlotDataStru[0].nReflPnts][1] = (float)pRD[reflCnt].Fr1_position_NP[1];
		MfPlotDataStru[0].CoordArrayFr0[MfPlotDataStru[0].nReflPnts][0] = (float)pRD[reflCnt].Fr0_position_NP[0];
		MfPlotDataStru[0].CoordArrayFr0[MfPlotDataStru[0].nReflPnts][1] = (float)pRD[reflCnt].Fr0_position_NP[1];
		MfPlotDataStru[0].mfp_CycleId = uspExec_counter % _MF_PLOT_HISTORY_;
		MfPlotDataStru[0].vCUS_CycleId = uspExec_counter;

		/*number of reflections != NoElements because elements can also be bursts*/
		MfPlotDataStru[0].nReflPnts++;
	}

	return 0;
}


int
UsRawEcho_Mapping(uint16_t NoElements) {
	
	/*reset mfplot data: remove this when saving history of 100 cycles*/
	//memset(MfPlotDataStru, 0, _MF_PLOT_HISTORY_ * sizeof(tNpDataMfPlot));
	static const int RDBsize = 100;

	static const struct tReflectionData EmptyReflectionDataBuffStruct; // Empty struct for clearing BuffData before mapping
	static struct tReflectionData ReflectionDataBuff[RDBsize]; // Buffered Data with 20 overtaken Reflections from one mapping cycle to the other
	static int ReflDataBuffCnt = 0;

	/* Clear Buffer Data */
	if (ToFwait->timeStamp_ms == static_cast<uint64_t>(SimParameters->USP_UpdateRate)) {
		ReflDataBuffCnt = 0;
		for (int a = 0; a < RDBsize; a++) {
			ReflectionDataBuff[a] = EmptyReflectionDataBuffStruct;
		}
	}

	uint16_t tmpTimeTag = 0;
	uint64_t resizedTimeTag = 0;
	int reportID = -1;
	uint64_t syncCnt_ms_abs = 0;

	bool isPulse = 0;
	bool isMeasurement = 0;
	bool isTxSensorEven = 0;
	bool isChirpUpMeasurementFromEvenSensor = 0;

	/* mf_plot */
	/*move existing mf_plot data one index higher*/
	shift_mfPlot_stru();
	/*clear position 0*/
	MfPlotDataStru[0] = { 0 };

	/* Set Signal Header */
	memset(UsRaw->detections, 0, 1800 * sizeof(us_drv::UsDrvDetection));

	/* This is the 40ms/100ms time step derived by vCUS_Main (depending on USP call timer) */
	uint64_t uiTimeStamp = ToFwait->timeStamp_ms * ((uint64_t) 1e3);

	UsRaw->numDetections = 0; // Increased during loop
	UsRaw->uiVersionNumber = us_drv::createUsDrvDetectionList_InterfaceVersion().UsDrvDetectionList_VERSION;
	UsRaw->sSigHeader.uiMeasurementCounter = 0; // Remains 0
	UsRaw->sSigHeader.uiTimeStamp = uiTimeStamp + SimParameters->HiLOffsetDelay; // Assignment to the signal header
	UsRaw->sSigHeader.eSigStatus = eco::AlgoSignalState::AL_SIG_STATE_OK;
	UsRaw->sSigHeader.uiCycleCounter++;

	/* Sensor State for running sensors */
	for (int k = 0; k < us_drv::US_DRV_Consts::US_DRV_MAX_NUM_SENSORS; k++) {
		k < _NO_OF_SENSORS_MAX_ ? UsRaw->sensorState[k] = us_drv::UsDrvSensorState::US_SENSOR_STATE_RUNNING_NORMAL_MODE : UsRaw->sensorState[k] = us_drv::UsDrvSensorState::US_SENSOR_STATE_OFF;
	}

	if (SimParameters->DebugOpt_USP)
		Log("\n\nNew ReflectionData Mapping: uiTimeStamp=%u, uiMeasurementCounter=%u, uiCycleCounter=%u\n\n",
			uiTimeStamp, UsRaw->sSigHeader.uiMeasurementCounter, UsRaw->sSigHeader.uiCycleCounter);

	/* Write UsRawData and mf_plot data */
	int j = 0; // Cnt for regular ReflData (not buffered data)
	const int InitReflDataBufCnt{ NoElements + ReflDataBuffCnt }; // Necessary to create a copy since ReflDataBuffCnt changes over time in this loop and would affect the for-loop condition
	for (uint16_t k = 0; k <  InitReflDataBufCnt; k++) {
		if (k >= us_drv::US_DRV_Consts::US_DRV_MAX_NUM_DETECTIONS) {
			break; /* Neglect overflow data */
		}

		// UsRaw->numDetections is incr/cnt for final struct
		if (k < ReflDataBuffCnt) { // Due to "looking into the future" when deriving the Report-Time (SyncCnt) there is the possibility that events must be reported in the next package -> save here
			isPulse = (ReflectionDataBuff[k].Tx == -1);
			isMeasurement = !isPulse;
			isTxSensorEven = ReflectionDataBuff[k].Tx % 2 == 0;
			isChirpUpMeasurementFromEvenSensor = isMeasurement && isTxSensorEven;
								   
			/* Detection Information were reportID represents for each sensor the point of time when to report on the bus */
			if (ReflectionDataBuff[k].Rx <= _NO_OF_USS_PER_BUS_ - 1) // if currentID < NoBusSensors
			{
				reportID = ReflectionDataBuff[k].Rx; // 0 for S1 / 1 for S2 / ...
			}
			else {
				reportID = (ReflectionDataBuff[k].Rx - _NO_OF_USS_PER_BUS_) % _NO_OF_USS_PER_BUS_; // if currentID > NoBusSensors
			}

			/* Create a syncCount that shows the 8bit-ms-timestep in which the event happened -> Cast Reflection Time Tag in us to 8bit var in ms */
			float event_ceil_syncCnt_abs = ceil((float)ReflectionDataBuff[k].TimeTag / ((float) 1e3));
			// In absolute timeframe based on timestamp "0" (ceil, because Rx data can be x.7645 ms an should then be sorted to x+1ms)

			uint64_t lowBound = floor(ReflectionDataBuff[k].TimeTag / ((uint64_t)(1e3 * _NO_OF_USS_PER_BUS_))) * _NO_OF_USS_PER_BUS_;
			// Derive in abs. timeframe the "startingpoint" of the SyncCnts e.g. 0,6,12 for Sensor 0 Sync -> 0,1,2,3,4,5 |6,7,8,9,10,11 |12,13,14,15,16 etc

			if ((uint64_t)event_ceil_syncCnt_abs == ((uint64_t)lowBound + reportID)) { /* Capture events (most likely bursts) that happen directly at the syncCnt for each sensor */
				syncCnt_ms_abs = (uint64_t)event_ceil_syncCnt_abs;
			}
			else { /* "Future" syncCnt based on evaluation of current time stamp */
				syncCnt_ms_abs = (_NO_OF_USS_PER_BUS_ - ((uint64_t)(event_ceil_syncCnt_abs - reportID) % _NO_OF_USS_PER_BUS_)) + (uint64_t)event_ceil_syncCnt_abs; // calc of absolute sync cnt value
			}

			/* Time Tag generation for BUFFERED DATA*/
			UsRaw->detections[UsRaw->numDetections].syncCnt_ms = (unsigned char)(syncCnt_ms_abs % (uint64_t)256U); /* in 256ms timeframe */
			UsRaw->detections[UsRaw->numDetections].relEcuTimestamp_us = (signed int)(uiTimeStamp - (syncCnt_ms_abs*1e3)); // Delta in time between "package time" and syncCnt
			UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us = (uint16)((syncCnt_ms_abs*1e3) - ReflectionDataBuff[j].TimeTag); // Delta in time between syncCnt and event time tag

			if (isMeasurement && SimParameters->SWSwitch_RCPmsCorrection) {
				UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us = UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us + _RCPmsCorrection;
			}

			/* Amplitude and ID */
			UsRaw->detections[UsRaw->numDetections].sensorId = ReflectionDataBuff[k].Rx + _USS_USRAW_ID_FACTOR_; /* Sensor ID depending on factor */
			UsRaw->detections[UsRaw->numDetections].amplitude = ReflectionDataBuff[k].Amplitude;

			/* mfplot mapping */
			mfPlot_Mapping(ReflectionDataBuff, UsRaw->numDetections);

			// Reset ReflDataBuffCnt after Buffer has been mapped
			if (k == ReflDataBuffCnt - 1) {
				ReflDataBuffCnt = 0;
				for (int a = 0; a < RDBsize; a++) {
					ReflectionDataBuff[a] = EmptyReflectionDataBuffStruct;
				}
			}

			if (SimParameters->DebugOpt_USP)
				Log("(%u) Ground Truth: ts-%u, burst-%d, Tx=%d, Rx=%d, TimeOF=%u, ReflPoint=%.2f/%.2f/%.2f, SensPosRx=%.2f/%.2f/%.2f, SensPosTx=%.2f/%.2f/%.2f\n",
					UsRaw->numDetections,
					ReflectionDataBuff[k].TimeTag,
					ReflectionDataBuff[k].BurstIndic,
					ReflectionDataBuff[k].Tx,
					ReflectionDataBuff[k].Rx,
					ReflectionDataBuff[k].TimeOF,
					ReflectionDataBuff[k].Fr0_position_NP[0],
					ReflectionDataBuff[k].Fr0_position_NP[1],
					ReflectionDataBuff[k].Fr0_position_NP[2],
					ReflectionDataBuff[k].Fr0_position_Sens[0],
					ReflectionDataBuff[k].Fr0_position_Sens[1],
					ReflectionDataBuff[k].Fr0_position_Sens[2],
					ReflectionDataBuff[k].Fr0_position_Sens_Tx[0],
					ReflectionDataBuff[k].Fr0_position_Sens_Tx[1],
					ReflectionDataBuff[k].Fr0_position_Sens_Tx[2]
				);
		}
		else {
			isPulse = (ReflectionData[j].Tx == -1);
			isMeasurement = !isPulse;
			isTxSensorEven = ReflectionData[j].Tx % 2 == 0;
			isChirpUpMeasurementFromEvenSensor = isMeasurement && isTxSensorEven;

			/* Detection Information */
			if (ReflectionData[j].Rx <= _NO_OF_USS_PER_BUS_ - 1) // if currentID < NoBusSensors
			{
				reportID = ReflectionData[j].Rx; // 0 for S1 / 1 for S2 / ...
			}
			else {
				reportID = (ReflectionData[j].Rx - _NO_OF_USS_PER_BUS_) % _NO_OF_USS_PER_BUS_; // if currentID > NoBusSensors
			}

			/* Create a syncCount that shows the 8bit-ms-timestep in which the event happened -> Cast Reflection Time Tag in us to 8bit var in ms */
			float event_ceil_syncCnt_abs = ceil((float)ReflectionData[j].TimeTag / ((float) 1e3));
			// In absolute timeframe based on timestamp "0" (ceil, because Rx data can be x.7645 ms an should then be sorted to x+1ms)

			uint64_t lowBound = floor(ReflectionData[j].TimeTag / ((uint64_t)(1e3 * _NO_OF_USS_PER_BUS_))) * _NO_OF_USS_PER_BUS_;
			// Derive in abs. timeframe the "startingpoint" of the SyncCnts e.g. 0,6,12 for Sensor 0 Sync -> 0,1,2,3,4,5 |6,7,8,9,10,11 |12,13,14,15,16 etc

			if ((uint64_t)event_ceil_syncCnt_abs == ((uint64_t)lowBound + reportID)) { /* Capture events (most likely bursts) that happen directly at the syncCnt for each sensor */
				syncCnt_ms_abs = (uint64_t)event_ceil_syncCnt_abs;
			}
			else { /* "Future" syncCnt based on evaluation of current time stamp */
				syncCnt_ms_abs = (_NO_OF_USS_PER_BUS_ - ((uint64_t)(event_ceil_syncCnt_abs - reportID) % _NO_OF_USS_PER_BUS_)) + (uint64_t)event_ceil_syncCnt_abs; // calc of absolute sync cnt value
				if (syncCnt_ms_abs * ((uint64_t) 1e3) > uiTimeStamp) {
					ReflectionDataBuff[ReflDataBuffCnt] = ReflectionData[j];
					ReflDataBuffCnt++;
					j++; // Skip this data and buffer it, but don't write it into UsRaw->detections
					continue;
				}
			}

			/* Time Tag generation for CURRENT DATA*/
			UsRaw->detections[UsRaw->numDetections].syncCnt_ms = (unsigned char)(syncCnt_ms_abs % (uint64_t)256U); /* in 256ms timeframe */
			UsRaw->detections[UsRaw->numDetections].relEcuTimestamp_us = (signed int)(uiTimeStamp - (syncCnt_ms_abs*1e3)); // Delta in time between syncCnt and event time tag
			UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us = (uint16)((syncCnt_ms_abs*1e3) - ReflectionData[j].TimeTag); // Delta in time between syncCnt and event time tag

			if (isMeasurement && SimParameters->SWSwitch_RCPmsCorrection) {
				UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us = UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us + _RCPmsCorrection;
			}

			/* Amplitude and ID */
			UsRaw->detections[UsRaw->numDetections].sensorId = ReflectionData[j].Rx + _USS_USRAW_ID_FACTOR_; /* Sensor ID depending on factor */
			UsRaw->detections[UsRaw->numDetections].amplitude = ReflectionData[j].Amplitude;

			/* mfplot mapping */
			mfPlot_Mapping(ReflectionData, UsRaw->numDetections);

			if (SimParameters->DebugOpt_USP)
				Log("(%u) Ground Truth: ts-%u, burst-%d, Tx=%d, Rx=%d, TimeOF=%u, ReflPoint=%.2f/%.2f/%.2f, SensPosRx=%.2f/%.2f/%.2f, SensPosTx=%.2f/%.2f/%.2f\n",
					UsRaw->numDetections,
					ReflectionData[j].TimeTag,
					ReflectionData[j].BurstIndic,
					ReflectionData[j].Tx,
					ReflectionData[j].Rx,
					ReflectionData[j].TimeOF,
					ReflectionData[j].Fr0_position_NP[0],
					ReflectionData[j].Fr0_position_NP[1],
					ReflectionData[j].Fr0_position_NP[2],
					ReflectionData[j].Fr0_position_Sens[0],
					ReflectionData[j].Fr0_position_Sens[1],
					ReflectionData[j].Fr0_position_Sens[2],
					ReflectionData[j].Fr0_position_Sens_Tx[0],
					ReflectionData[j].Fr0_position_Sens_Tx[1],
					ReflectionData[j].Fr0_position_Sens_Tx[2]
					);

			j++;
		}
     
		/* Chirp Tune information
			US_DETECTION_CONFIDENCE_MASK				= 0000 1111
			US_DETECTION_ADVANCED_PATH_1/ChUp	= 16U	= 0001		FullField -> 31U	= 0001 1111
			US_DETECTION_ADVANCED_PATH_2/ChDwn	= 32U	= 0010		FullField -> 47U	= 0010 1111
			US_DETECTION_FIRING_TIMESTAMP		= 128U	= 1000		FullField -> 143U	= 1000 1111
		*/
		if (isMeasurement) {
			isChirpUpMeasurementFromEvenSensor ? UsRaw->detections[UsRaw->numDetections].detectionType = 47U : UsRaw->detections[UsRaw->numDetections].detectionType = 31U;
			UsRaw->detections[UsRaw->numDetections].phaseDerivative = 0;
		}
		else {
			UsRaw->detections[UsRaw->numDetections].detectionType = 128U;
			UsRaw->detections[UsRaw->numDetections].phaseDerivative = UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us / 1000;
		}

		if (SimParameters->DebugOpt_USP)
			Log("Detections -> Amp:%u, DetectType:%d, Phase:%u, relECUTime:%d, sensorID:%d, sensorTime:%u, syncCnt:%d\n",
				UsRaw->detections[UsRaw->numDetections].amplitude,
				UsRaw->detections[UsRaw->numDetections].detectionType,
				UsRaw->detections[UsRaw->numDetections].phaseDerivative,
				UsRaw->detections[UsRaw->numDetections].relEcuTimestamp_us,
				UsRaw->detections[UsRaw->numDetections].sensorId,
				UsRaw->detections[UsRaw->numDetections].sensorTimestamp_us,
				UsRaw->detections[UsRaw->numDetections].syncCnt_ms);

		UsRaw->numDetections++;

		/* Debugging / Testing content */
		#ifdef _UsRaw_DEBUG_
			UsRaw_debug->usData_as_debug[k].vCUS_Tx = (ReflectionData[UsRaw->numDetections].Tx == -1) ? ReflectionData[UsRaw->numDetections].Tx :
														(ReflectionData[UsRaw->numDetections].Tx + _USS_USRAW_ID_FACTOR_);
			UsRaw_debug->usData_as_debug[k].vCUS_TOid = ReflectionData[UsRaw->numDetections].Detc_TO_ID;
			UsRaw_debug->usData_as_debug[k].vCUS_ToF = ReflectionData[UsRaw->numDetections].TimeOF;
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_NP_x = ReflectionData[UsRaw->numDetections].Fr0_position_NP[0];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_NP_y = ReflectionData[UsRaw->numDetections].Fr0_position_NP[1];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_NP_z = ReflectionData[UsRaw->numDetections].Fr0_position_NP[2];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_x = ReflectionData[UsRaw->numDetections].Fr0_position_Sens[0];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_y = ReflectionData[UsRaw->numDetections].Fr0_position_Sens[1];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_z = ReflectionData[UsRaw->numDetections].Fr0_position_Sens[2];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_Tx_x = ReflectionData[UsRaw->numDetections].Fr0_position_Sens_Tx[0];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_Tx_y = ReflectionData[UsRaw->numDetections].Fr0_position_Sens_Tx[1];
			UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_Tx_z = ReflectionData[UsRaw->numDetections].Fr0_position_Sens_Tx[2];
		#endif
	}

	#ifdef _UsRaw_DEBUG_
		/* Write UsRaw package to external file */
		WriteUsRaw(NoElements);
	#endif

	return 0;
}



int
ReflData_Mapping(int k, int ID, bool BurstFlag) {
	/* BurstFlag indicates if this is called by a burst event (1) or a receiving event (0) */
	tTrafficObj *Obj = NULL;
	static int cnt = 0;

	if (ToFwait->SimCoreStartCycleFlag)
		cnt = 0;

	/* Reset counter after sim has finished */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		cnt = 0;
		return 0;
	}

	/* Main routine for "growing" the ReflectionData struct */
	if (IntReflData->ReflDataCnt <= us_drv::US_DRV_Consts::US_DRV_MAX_NUM_DETECTIONS) { // if max. number of reflections per usp exec timing are exceeded
		if (BurstFlag) { /* Msg represents a BURST */
			/* New version of correlator check for Burst/Recv */
			ReflectionData[IntReflData->ReflDataCnt].BurstIndic = 1U;

			/* Old version of correlator check for Burst/Recv */
				//for (int p = 0; p < 8; p++) {
				//	/* For BURST Msg -> ID = TxID */
				//	if ( SimParameters->fire_scheme[ID][p] == (ToFwait->NextSFSP[ID] - ToFwait->timeStamp_ms) )
				//		ReflectionData[IntReflData->ReflDataCnt].BurstIndic = 127 + p; /* Represent the Burst-ID of the SFSP */
				//}

			ReflectionData[IntReflData->ReflDataCnt].TimeOF = 0;
			ReflectionData[IntReflData->ReflDataCnt].Detc_TO_ID = -1;
			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr0_position_NP, null_vec_3D);
			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr1_position_NP, null_vec_3D);
			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr0_position_Sens, null_vec_3D);
			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr0_position_Sens_Tx, SensData[ID].Fr0_position_Sens);

			/* For BURST Msg -> ID = TxID */
			ReflectionData[IntReflData->ReflDataCnt].Tx = -1; /* There is no receiver for BURST Msg ... */
			ReflectionData[IntReflData->ReflDataCnt].Rx = ID; /* ... but the sensors expects the info in Rx */

			/* The time tags (in us) for BURST Msg is generated by using SimCycle (in ms) */
			ReflectionData[IntReflData->ReflDataCnt].TimeTag = ToFwait->timeStamp_ms * ( (uint64_t) 1e3 );

			/*Outgoing burst: There is no Echo amplitude*/
			ReflectionData[IntReflData->ReflDataCnt].Amplitude = 53000; // TODO: Give MAX Number -> In real. they put the ringing info here around 53,000Hz

			/*store SFSP count*/
			ReflectionData[IntReflData->ReflDataCnt].SFSP_cnt = SensData[ID].SFSP_cnt;
		}
		else { /* Msg represents a RSCV Wave */
			/* New version of correlator check for Burst/Recv */
			ReflectionData[IntReflData->ReflDataCnt].BurstIndic = 0U;

			/* Old version of correlator check for Burst/Recv */
				///* This is a counter for all Rx msg between 0..126 */
				//cnt >= 126 ? cnt = 0 : cnt++;
				//ReflectionData[IntReflData->ReflDataCnt].BurstIndic = cnt;

			ReflectionData[IntReflData->ReflDataCnt].TimeOF = (uint64_t) (1e3 * SensData[ID].Target.ToF[k]); // ToF in ms to us
			ReflectionData[IntReflData->ReflDataCnt].Detc_TO_ID = SensData[ID].Target.Detc_TO_ID[k];

			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr0_position_NP, SensData[ID].Target.Fr0_position_NP[k]);
			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr1_position_NP, SensData[ID].Target.Fr1_position_NP[k]);
			//VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr0_position_Sens_Tx, SensData[ID].Target.Fr0_position_Sens_Tx[k]);

			/* For RECV Msg -> ID = RxID */
			ReflectionData[IntReflData->ReflDataCnt].Tx = SensData[ID].Target.Tx[k];//ID;// SensData[ID].Target.Tx[k];//ID is typically the sensor sending a burst
			ReflectionData[IntReflData->ReflDataCnt].Rx = SensData[ID].Target.Rx[k]; //ID;

			/* Time tag has to be in us / CM runs in ms, so the correct time tag has to be calculated by sum up ToF + time point of sending */
			ReflectionData[IntReflData->ReflDataCnt].TimeTag = ReflectionData[IntReflData->ReflDataCnt].TimeOF + (uint64_t) (1e3 * SensData[ID].Target.vCUSBurstTime[k]);

			ReflectionData[IntReflData->ReflDataCnt].Amplitude = (uint16_t) SensData[ID].Target.echoAmplitude[k];
			/*log for debugging
			Log("mapped amp %f id %d count %d\n", SensData[ID].Target.echoAmplitude[k], ID, k);
			/**/

			/*map SFSP burst index for the current Rx - USS doesn't know which is the Tx*/
			ReflectionData[IntReflData->ReflDataCnt].SFSP_cnt = SensData[SensData[ID].Target.Rx[k]].SFSP_cnt; 

			ReflectionData[IntReflData->ReflDataCnt].CycleNo = uspExec_counter;
      
			/*Rx sensor position*/
			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr0_position_Sens, SensData[SensData[ID].Target.Rx[k]].Fr0_position_Sens);

			/*Tx sensor position */
			VEC_Assign(ReflectionData[IntReflData->ReflDataCnt].Fr0_position_Sens_Tx, SensData[ID].Target.Fr0_position_Sens_Tx[k]);
		}

		IntReflData->ReflDataCnt++;
	}
	else {
		LogWarnStr(EC_General,"Too many reflections to be assigned to RefelctionData struct. This leads to a loss of data!\n");
	}

	return 0;
}


/* Evaluate sensor characteristic based on azimuth/elevation angle */
double EvalAmplitudeMapping(double Azimuth_Tx, double Elevation_Tx, double Distance_Tx, double Azimuth_Rx, double Elevation_Rx, double Distance_Rx, double Reflectivity) {
	double	G_tr_Tx, G_tr_Rx, alpha, ampl;
	int		res;

	if (SimParameters->DebugOpt_Amp) {
		Log("Tx -> Azi(%0.2f) + Ele(%0.2f) + Dist(%0.2f)\n", Azimuth_Tx, Elevation_Tx, Distance_Tx);
		Log("Rx -> Azi(%0.2f) + Ele(%0.2f) + Dist(%0.2f)\n", Azimuth_Rx, Elevation_Rx, Distance_Rx);
	}

	/* Derive Azimuth & Elevation */
	/* Convert <90,85..0,-5,-10..-90> to <0,5..90,95,100,..180> */
	Azimuth_Tx = (Azimuth_Tx * (-1) ) + 90.0;
	Elevation_Tx = (Elevation_Tx * (-1)) + 90.0;
	Azimuth_Rx = (Azimuth_Rx * (-1)) + 90.0;
	Elevation_Rx = (Elevation_Rx * (-1)) + 90.0;

	/* Get directivity> Evaluate Map via: LM2DEval(SensParam->SensChar.LobeMap, <Ele> 90..0..-90, <Azi> 90..0..-90) */
	G_tr_Tx = LM2DEval(SimParameters->SensChar.LobeMap, Azimuth_Tx, Elevation_Tx); // p0 -> Reference SPA according to Map
	G_tr_Rx = LM2DEval(SimParameters->SensChar.LobeMap, Azimuth_Rx, Elevation_Rx); // p0 -> Reference SPA according to Map

	/* tbd: Is this the proper calculation */
	alpha = 0.1151 / GetSoundVelinAir(Env.Temperature - 273.15); // Attenuation coefficient

	/* Calculate the amplitude */
	ampl =	(G_tr_Tx * G_tr_Rx * Reflectivity\
			* exp((alpha * (-1) * ( (Distance_Tx + Distance_Rx) - (2.0 * SimParameters->SensChar.SPANormRange))))) \
			/ (SimParameters->SensChar.p0 * ( (Distance_Tx + Distance_Rx) / SimParameters->SensChar.SPANormRange));

	/* In case of SPL output */
	if (SimParameters->SensChar.Type)
		ampl = 20 * log10(ampl / 0.00002);

	if (SimParameters->DebugOpt_Amp) {
		Log("G_tr_Tx(%f) / G_tr_Rx(%f)\n",  G_tr_Tx, G_tr_Rx);
		Log("Calculated Amplitude in <Pa> %f\n", ampl);
	}

	/* Convert <Pa> to <bit> */
	if (ampl < SimParameters->SensChar.MaxSPA) {
		res = static_cast<uint16_t>(std::round( ampl * (0xFFFF / SimParameters->SensChar.MaxSPA)) );
	}
	else {
		res = 0xFFFF;
	}

	if (SimParameters->DebugOpt_Amp)
		Log("Calculated Amplitude in <bit> %d\n\n", res);

	return res;
}

/* To be used in HiL mode only */
#ifdef vCUS_HIL_MODE
int
PDCMFrame_Mapping(uint16_t NoElements) {
/*maps data to the PDCM frame - daisy chain*/

	memset(localPDCMframe, 0, PdcmFlags->nBuses * sizeof(US_DRV::VFA28::PdcmRecorderModeFrame));

	int		j, payloadCount, busSensor;
	bool	isBurst, isEcho, copyFromWait4Turn, isSensorOnBus;
	bool	isHeadEmpty, hederNotDone, sensorSends;
	int 	sendingUSS_ID[_N_OF_BUSES_MAX_]; /*for each bus*/

	/*reset some counters*/
	PdcmFlags->nBurstsPDCM = 0;
	PdcmFlags->nPayldPDCM = 0;

	for (int i = 0; i < PdcmFlags->nBuses; i++) {
		/*for each bus generate pdcm frame and write buffers*/

		/*reset data*/
		memset(FrameMirrorBuff, 0, sizeof(tFrameMirrorBuff));
		isHeadEmpty		= false;
		hederNotDone	= true;
		sensorSends		= false;

		/*determine sending sensor*/
		if (PdcmFlags->nextSensorID[i] == -1) {
			/*no data was sent yet - set to the first sensor on each bus*/
			sendingUSS_ID[i]	= i * PdcmFlags->ussPerBus;
		}
		else {
			sendingUSS_ID[i]	= PdcmFlags->nextSensorID[i];
		}

		/*reset some flags*/
		PdcmFlags->lastSlot_ID[i]	= -1; /*initialize to -1 : nothing written yet in PDCM slots*/
		PdcmFlags->headerDone[i]	= 0;
		payloadCount				= 0;

		/*check buffer*/
		copyFromWait4Turn = PdcmFlags->flag_wait4Turn[sendingUSS_ID[i]];
		if (copyFromWait4Turn) {
			map_Buffer2PDCM(i, sendingUSS_ID[i]);	//takes from buffer position 0 and shift left buffer data
													/*do not write header in buffer - header contains data specific to transmission time*/
		}

		/*set an empty header in case there are no elements for the current sending sensor*/
		isHeadEmpty = true;
		int dummyK	= 0;
		if (PdcmFlags->headerDone[i] == 0)
			writeHeader(i, dummyK, sendingUSS_ID[i], isHeadEmpty);

		if (NoElements >= 0) {/*redundant?*/
			for (int k = 0; k < NoElements; k++) {
				/*go through elements: echoes or bursts*/

				j				= ReflectionData[k].Rx; /*current sensor*/
				sensorSends		= j == sendingUSS_ID[i];
				busSensor		= (j / PdcmFlags->ussPerBus);
				isSensorOnBus	= busSensor == i;

				/*sensor can send:*/
				if (sensorSends && isSensorOnBus ){

					/*initialize flags*/
					isBurst			= ReflectionData[k].BurstIndic == 128;
					isEcho			= ReflectionData[k].BurstIndic == 0;
					hederNotDone	= PdcmFlags->headerDone[i] == 0;

					/*set payload count with every element*/
					payloadCount = PdcmFlags->lastSlot_ID[i] + 1;/* ==  slot ID for new data*/

					if (isBurst)
						PdcmFlags->nBurstsPDCM = PdcmFlags->nBurstsPDCM + 1;

					/*fill in header if this sensor sends in this ms*/
					if (hederNotDone || isBurst) {
						/* ### PDCM Header Mapping: only if not done for this sensor or if there is a burst after it was mapped*/
						isHeadEmpty = false;

						/************/
						writeHeader(i, k, j, isHeadEmpty);
					}

					/*only 5 meas will be mapped at once; unless there is a burst after all payloads have been filled - takes first payload: see below*/
					if ((payloadCount >= _MAX_NO_PDCM_ECHOES_) && isEcho) {
						/*place data in overflow buffer, but only echoes - bursts go first in this frame */

						/************/
						write_PdcmBuffer(i, j, k);
						PdcmFlags->flag_wait4Turn[j]	= 1;
						PdcmFlags->flag_overflow[j]		= 1;
						continue; /*skip the rest*/
					}

					/* PDCM Payload Slot Mapping */
					if (isBurst) {

						/*if this is a burst and some data has been already mapped in payloads:
						first shift all payloads to the right*/
						if (payloadCount > 0) {
							sortPDCM(i); /*sort before shifting*/
							sortMirrorBuffer();
							shiftPayLdRight(0, payloadCount, i, j); /*shift right and moves the payload to the buffer if no place in pdcm*/
						}

						/*set burst sync tag*/
						PdcmFlags->burstSyncTag[j]	= PdcmFlags->syncCnt;
						PdcmFlags->burstTStamp[j]	= ReflectionData[k].TimeTag;

						/*check that there are no bursts already in the frame*/
						for (int iPld = 0; iPld < _MAX_NO_PDCM_ECHOES_; iPld++) {
							if (localPDCMframe[i].rec.echoes[iPld].rec.selectedSignal == 5)
								LogErrF(EC_Sim, "vCUS::PDCMFrame_Mapping: Two bursts to be mapped to PDCM frame. Please investigate!\n");
						}

						/***********place echo data in the first payload*/
						writePayload(i, 0, k, isBurst);

						/*payload added or shifted to right: increase last payload slot ID with 1, but max = (_MAX_NO_PDCM_ECHOES_ - 1)*/
						PdcmFlags->lastSlot_ID[i] = (payloadCount < _MAX_NO_PDCM_ECHOES_) ? payloadCount : (_MAX_NO_PDCM_ECHOES_ - 1); 
					}
					else if (isEcho) {
						assert(isEcho && (!isBurst));

						/************place echo data in the first payload*/
						writePayload(i, payloadCount, k, isBurst);
						PdcmFlags->lastSlot_ID[i] = payloadCount; /*payload added: increase last payload slot ID with 1, pyloadCount to be increased above*/
					}

				}
				else if ((!sensorSends) && isSensorOnBus) {/*if the current sensor cannot send in this ms but corresponds to bus - place in buffer*/
					/************/
					write_PdcmBuffer(i, j, k);
					PdcmFlags->flag_wait4Turn[j] = 1;
				}
				else {/*go to next element*/
					continue;
				}/*end if sensor sends*/

			}/*end for NoElements*/

		}/*end if NoElements >= 0*/

		/*after writting  all payloads check if everything fits and set header rx trans delay flag*/
		localPDCMframe[i].rec.header.rec.rxTransDelay	= PdcmFlags->flag_overflow[sendingUSS_ID[i]];
		localPDCMframe[i].rec.header.rec.echosLost		= PdcmFlags->echoesLost[sendingUSS_ID[i]];
		
		PdcmFlags->echoesLost[sendingUSS_ID[i]]			= 0; //reset flag
		
		/*make sure payloads are receptionTime - sorted*/
		sortPDCM(i);

		/*set to next sensor: = (SensID + 1) % nOS*/
		PdcmFlags->nextSensorID[i] = ((sendingUSS_ID[i] + 1) % PdcmFlags->ussPerBus) + (i * PdcmFlags->ussPerBus);

		/*OPTIONAL: sort buffer of next sensor - time of flight not really relevant for sorting*/
		/*sortBuff(i, PdcmFlags->nextSensorID[i]);*/

		/*increase mux data cnt for this sensor*/
		PdcmFlags->muxDataCnt[sendingUSS_ID[i]] = PdcmFlags->muxDataCnt[sendingUSS_ID[i]] + 1;

		if (SimParameters->DebugPDCM_Frame) {
			logPDCM_frame(i);
			PdcmFlags->nPayldPDCM = PdcmFlags->nPayldPDCM + PdcmFlags->lastSlot_ID[i] + 1;
		}

	}/*end for i < n Buses*/

	if (SimParameters->DebugPDCM_Frame) {
		Log("N payloads %d, out of which: %d bursts and %d echoes; n deafTime rejected: %d\n\n",
				PdcmFlags->nPayldPDCM, 
				PdcmFlags->nBurstsPDCM, 
				PdcmFlags->nPayldPDCM - PdcmFlags->nBurstsPDCM, 
				PdcmFlags->nDeafTimeEchos);
		Log("=========================\n");
	}


	return 0;
}
#endif