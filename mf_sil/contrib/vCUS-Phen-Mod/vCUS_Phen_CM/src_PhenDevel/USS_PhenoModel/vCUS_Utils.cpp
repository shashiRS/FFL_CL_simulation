#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <Log.h>
#include <cassert>
#include <algorithm>

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include "PhenoModel.h"
#include "PhenoMapping.h"
#include "USSMathUtils.h"
#include <Vehicle\Sensor_Object.h>

#include "us_drv_def_vfa.h"
//#include "us_drv_types.h"

#ifdef vCUS_HIL_MODE
extern US_DRV::UsRaw_t *UsRaw;
extern US_DRV::VFA28::PdcmRecorderModeFrame *localPDCMframe;
#endif


#ifdef _UsRaw_DEBUG_
int
WriteUsRaw(uint16_t NoElements)
{
	volatile static int file_counter = 0;
	static char pwd[256];
	
	/* Save up to 9999 meas per file */
	if (file_counter > 9999)
		return 0;

	/* Reset after SimEnd */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		file_counter = 0;
		return 0;
	}

	/* Get system data */
	FILE *filepointer;
	TCHAR Buffer[BUFSIZE];
	DWORD dwRet = GetCurrentDirectory(BUFSIZE, Buffer);
	if (dwRet == 0) {
		LogErrStr(EC_General, "GetCurrentDirectory failed!\n");
	}
	SYSTEMTIME st;
	GetLocalTime(&st);

	if (file_counter == 0) { /* Only generate folder once per simulation */
		sprintf(pwd, "%s/SimOutput/Validation/%4d-%02d-%02d_%02dh-%02dm-%02ds-%03dms",\
					Buffer, st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
		CreateDirectoryA(pwd, NULL);
	}
	char str[256];
	sprintf(str, "%s/TimeStamps_%04d.csv", pwd, file_counter);
	
	file_counter++;
	filepointer = fopen(str, "w+");

	char Values_TextArray[65535] = { 0 };
	char Labels_TextArray[1048] = { 0 };
	char tmpStr[256] = { 0 };

	/* Errorhandling */
	if (filepointer == NULL)
		Log("Error opening file! \n");

	/* Write text labels once per file */
	char csv_separator[8] = ";";
	if (_CSV_CountryCode_)
		sprintf(csv_separator,",");

	/* Write first row of file: LABELS */
	sprintf(Labels_TextArray,	"UsRaw_timestamp_us_u64%s \
								UsRaw_tof_8us_u14%s \
								UsRaw_nfd_nu_u1%s \
								UsRaw_virtualisation_nu_u1%s \
								UsRaw_rxSensorId_nu_u8%s \
								UsRaw_measTag_ms_u8%s \
								UsRaw_amplitude_nu_u8%s \
								UsRaw_timeDomainConfidenceLevel_nu_u4%s \
								vCUS_Tx%s \
								vCUS_TOid%s \
								vCUS_ToF%s \
								vCUS_Fr0_position_NP_x%s \
								vCUS_Fr0_position_NP_y%s \
								vCUS_Fr0_position_NP_z%s \
								vCUS_Fr0_position_Sens_Tx_x%s \
								vCUS_Fr0_position_Sens_Tx_y%s \
								vCUS_Fr0_position_Sens_Tx_z%s \
								vCUS_Fr0_position_Sens_x%s \
								vCUS_Fr0_position_Sens_y%s \
								vCUS_Fr0_position_Sens_z\n",\
								csv_separator, csv_separator, csv_separator, csv_separator, csv_separator,\
								csv_separator, csv_separator, csv_separator, csv_separator, csv_separator,\
								csv_separator, csv_separator, csv_separator, csv_separator, csv_separator,\
								csv_separator, csv_separator, csv_separator, csv_separator);
	strcat(Values_TextArray, Labels_TextArray);

	/* Loop through the number of Msg in UsRaw and write content to file */
	for (int k = 0; k < NoElements; k++) {
		sprintf(tmpStr, "%lld%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%d%s%lld%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f%s%f\n",\
						UsRaw->usData_as[k].timestamp_us_u64, csv_separator, \
						UsRaw->usData_as[k].tof_8us_u14, csv_separator, \
						UsRaw->usData_as[k].nfd_nu_u1, csv_separator, \
						UsRaw->usData_as[k].virtualisation_nu_u1, csv_separator, \
						UsRaw->usData_as[k].rxSensorId_nu_u8, csv_separator, \
						UsRaw->usData_as[k].measTag_ms_u8, csv_separator, \
						UsRaw->usData_as[k].amplitude_nu_u8, csv_separator, \
						UsRaw->usData_as[k].timeDomainConfidenceLevel_nu_u4, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Tx, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_TOid, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_ToF, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_NP_x, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_NP_y, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_NP_z, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_Tx_x, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_Tx_y, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_Tx_z, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_x, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_y, csv_separator, \
						UsRaw_debug->usData_as_debug[k].vCUS_Fr0_position_Sens_z);
		strcat(Values_TextArray, tmpStr);
	}

	/* Generate "new line" after each loop iteration */
	fprintf(filepointer, "%s\n", Values_TextArray);
	fclose(filepointer);

	return 0;
}
#endif

#ifdef vCUS_HIL_MODE
void
write_PdcmBuffer(int buffID, int sensID, int k) {

	int USSbus_ID, cntWaitBuff;
	bool isBurst, isEcho;

	USSbus_ID	= sensID - PdcmFlags->ussPerBus * buffID;/*sensor index for buffers*/
	cntWaitBuff = PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].cntElements;
	isBurst		= ReflectionData[k].BurstIndic == 128;
	isEcho		= ReflectionData[k].BurstIndic == 0;

	/*store ecoes in the payload section of the buffer*/
	/************/
	if (isBurst) {

		/*if this is a burst and some data has been already mapped in payloads:
		first shift all payloads to the right*/
		if (cntWaitBuff != 0)
			shift_Buff_Right(0, buffID, USSbus_ID);//also updates buffer count

		/*set burst sync tag*/
		PdcmFlags->burstSyncTag[sensID] = PdcmFlags->syncCnt;
		PdcmFlags->burstTStamp[sensID]	= ReflectionData[k].TimeTag;

		buffer_writePayLD(buffID, USSbus_ID, 0, k);
		if (cntWaitBuff == 0)//an element was added in position 0 - update count
			PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].cntElements = 1;

	}
	else if (isEcho) {

		buffer_writePayLD(buffID, USSbus_ID, cntWaitBuff, k);
		/*increment n of stored echoes*/
		PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].cntElements = PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].cntElements + 1 ;
	}
	/************/
}

void
buffer_writePayLD(int buffID, int USSbus_ID, int cntWaitBuff, int k) {
	/*writes payload data to w4t buffer*/
	bool isTxEven, isFull_w4tBuff, isBurst, keepBufferBurst, isEcho;
	int ussId, selSig;

	selSig			= 5U;
	ussId			= ReflectionData[k].Rx;
	keepBufferBurst = true;
	isBurst			= ReflectionData[k].BurstIndic == 128;
	isEcho			= !isBurst;

	/*shift left and ovewrite data if buffer is full & is echo, if it's burst leave cntWaitBuff == 0 and overwrite the first element*/
	isFull_w4tBuff = cntWaitBuff >= _MAX_USS_BUFF_SIZE_;
	if (isFull_w4tBuff && isEcho){
		shiftBuffLeft(buffID, ussId, keepBufferBurst);
		
		/*one echo was lost for this sensor*/
		PdcmFlags->echoesLost[ussId] = 1; 
		if (SimParameters->DebugPDCM_Frame)
			Log("WARNING - vCUS::buffer_writePayLD: EchoLost - Buffer full, echo removed for new one. Sensor %d.\n\n", ussId+1);

		cntWaitBuff = PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].cntElements;/*update buffer count*/
	}

	//this is a problem: if event to write is a burst but also the first buffer position is a burst
	if (isBurst && (PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].isBurst[0] == 1)){
		LogErrF(EC_Sim, "vCUS::buffer_writePayLD: Two bursts in buffer. Please investigate!\n");
	}

	/*write buffer payload*/
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].receptSyncCnt[cntWaitBuff] = PdcmFlags->syncCnt;
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].TxID[cntWaitBuff]	= ReflectionData[k].Tx;
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].burstSyncRec[cntWaitBuff]	= PdcmFlags->burstSyncTag[ussId];
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].relativeToF[cntWaitBuff]	= ReflectionData[k].TimeTag - PdcmFlags->burstTStamp[ussId];
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].isBurst[cntWaitBuff]		= isBurst;
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].timeOF[cntWaitBuff]		= isBurst ? _BURST_RING_TIME_ : (ReflectionData[k].TimeOF); 
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].amplitude[cntWaitBuff]		= isBurst ? _BURST_FREQUENCY_ : ReflectionData[k].Amplitude;

	/*not defined here- will hold reception time from right shifted overflow pdcm payload*/
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].rcptTime[cntWaitBuff] = 0;

	/*!< phase derivative of the standard path for the echoes received range from -128 to 127 */
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].phsDer[cntWaitBuff]		= 0U; 
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].cConfLevel[cntWaitBuff]	= isBurst ? 0U : 12U;/*!< US Coding Confidence Level 0..15 */

	/*rxPath indicates ch up(1) or down(2) of the incoming echo
		for Tx: even sensors - ch up/ odd - ch down*/
	if (!isBurst) {
		isTxEven = (ReflectionData[k].Tx +1) % 2 == 0;//add 1 to bring in range 1-12
		selSig = isTxEven ? 3U : 2U;
	}
	PdcmBuffers[buffID].Wait4TurnBuff[USSbus_ID].selSignal[cntWaitBuff] = selSig;

	/*set data flag in buffer & flags*/
	PdcmFlags->flag_wait4Turn[ussId] = 1;
}

/*map current header*/
void
writeHeader(int busID, int k, int USS_ID, bool isEmpty) {
	/*function writes header for event k & the current sensor on DC bus busID*/

	/*a burst is already mapped in the PDCM frame - from buffer*/
	bool isPayloadBurst;
	isPayloadBurst = (localPDCMframe[busID].rec.echoes[0].rec.selectedSignal == 5);

	localPDCMframe[busID].rec.header.rec.physicalAddress = isEmpty ? (uint8_t)(USS_ID + 1) : (uint8_t)(ReflectionData[k].Rx + 1); /*in each ms send only nBuses frames, this bus == i*/

	/*burst or echo*/
	if (isEmpty) {
		localPDCMframe[busID].rec.header.rec.burstFlag = isPayloadBurst ? 1U : 0U;
	}
	else {
		if ((ReflectionData[k].BurstIndic == 128) || isPayloadBurst)
			localPDCMframe[busID].rec.header.rec.burstFlag = 1U;
		else if (ReflectionData[k].BurstIndic == 0)
			localPDCMframe[busID].rec.header.rec.burstFlag = 0U;
	}

	localPDCMframe[busID].rec.header.rec.pdcmMode			= 0; /*always 0 in PDCM mode*/

	/*check if treansmission is delayed - ie data placed in overflow buffer- to implement flag reset*/
	localPDCMframe[busID].rec.header.rec.rxTransDelay		= PdcmFlags->flag_overflow[USS_ID];

	localPDCMframe[busID].rec.header.rec.echosLost			= 0; /*set to 0 for now*/
	localPDCMframe[busID].rec.header.rec.blankingFlag		= 0; /*set to 0 for now*/
	localPDCMframe[busID].rec.header.rec.burstErrorFlag		= 0; /*set to 0 for now*/
	localPDCMframe[busID].rec.header.rec.sumErrorFlag		= 0; /*set to 0 for now*/
	localPDCMframe[busID].rec.header.rec.multiplexDataCnt	= PdcmFlags->muxDataCnt[USS_ID] % 8; /*0 to 7, reset @ 7 */

	localPDCMframe[busID].rec.header.rec.syncCntCurrent		= PdcmFlags->syncCnt; 
	localPDCMframe[busID].rec.header.rec.AATG1_TH			= 0; /*set to 0 for now*/
	localPDCMframe[busID].rec.header.rec.AATG2_TH			= 0; /*set to 0 for now*/
	localPDCMframe[busID].rec.header.rec.multiplexedData	= 0; /*set to 0 for now*/

	PdcmFlags->headerDone[busID] = 1;

}

void
writePayload(int busID, int payldCnt, int k, bool isBurst) {

	int burst_STag, UssID, burstDelay;
	bool isEcho = !isBurst;
	uint8_t selSig = 5U;
	uint16_t tempRecTime = 0;
	uint64_t relativeToF;
	
	UssID		= ReflectionData[k].Rx;
	burst_STag	= PdcmFlags->burstSyncTag[UssID];
	if (isEcho) {
		/*check for surst sync tag vs sync count mismatch - ensures correct rec time calculation if sync count is reset since last burst:
		if burst sync tag is 255 and sync cnt at reporting is 0, the difference should be 1: 0 - 255-256 = 1*/
		if (burst_STag > PdcmFlags->syncCnt) {
			burst_STag = burst_STag - 256;
			if (SimParameters->DebugPDCM_Frame)
				Log("WARNING - vCUS::writePayload: Burst sync tag and sync cnt missmatch : BST->%d | SyncCnt->%d \n\n", burst_STag, PdcmFlags->syncCnt);
		}

		/*(syncCnt - burstSyncTag) * 1000 - relative ToF*/
		relativeToF = ReflectionData[k].TimeTag - PdcmFlags->burstTStamp[UssID];
		//tempRecTime = (uint16_t)((PdcmFlags->syncCnt - burst_STag) * 1000 - ReflectionData[k].TimeOF);
		tempRecTime = (uint16_t)((PdcmFlags->syncCnt - burst_STag + 1) * 1000 - relativeToF);
	}

	/*reject echoes with rec time > 65 ms*/
	if ((tempRecTime >= _REC_TIME_MAX_) && (isEcho)) {
		PdcmFlags->echoesLost[UssID] = 1;
		if (SimParameters->DebugPDCM_Frame)
			Log("WARNING - vCUS::writePayload: EchoLost - Reception time exceeds max, echo was not mapped to PDCM frame, sensor %d.\n\n", UssID+1);
		return;
	}

	burstDelay = PdcmFlags->syncCnt - burst_STag;

	if (isBurst) {
		localPDCMframe[busID].rec.echoes[payldCnt].burstInfo.ringingTime	= (uint16_t)_BURST_RING_TIME_;
		localPDCMframe[busID].rec.echoes[payldCnt].burstInfo.ringingFreq	= (uint16_t)_BURST_FREQUENCY_;
		localPDCMframe[busID].rec.echoes[payldCnt].burstInfo.notUsed1		= 0U;
		localPDCMframe[busID].rec.echoes[payldCnt].burstInfo.burstDelay		= (uint16_t)burstDelay ;
		localPDCMframe[busID].rec.echoes[payldCnt].burstInfo.selectedSignal = selSig;
		localPDCMframe[busID].rec.echoes[payldCnt].burstInfo.notUsed1		= 0U;
	}
	else if (isEcho) {
		bool isTxEven = (ReflectionData[k].Tx + 1) % 2 == 0; //add 1 to bring in range 1-12
		selSig = isTxEven ? 3U : 2U;
		localPDCMframe[busID].rec.echoes[payldCnt].rec.receptionTime			= tempRecTime;
		localPDCMframe[busID].rec.echoes[payldCnt].rec.amplitude				= ReflectionData[k].Amplitude;
		localPDCMframe[busID].rec.echoes[payldCnt].rec.phaseDerivative			= 0U;
		localPDCMframe[busID].rec.echoes[payldCnt].rec.codingConfidenceLevel	= 12U;/*!< US Coding Confidence Level 0..15 */
		localPDCMframe[busID].rec.echoes[payldCnt].rec.selectedSignal			= selSig; // selectedSignal is 5 for bursts - not valid for normal path & NFD!!!!

		/*place data in PDCM frame mirror buffer */
		int iMirror = FrameMirrorBuff->cntElements;
		FrameMirrorBuff->isBurst[iMirror]		= 0;
		FrameMirrorBuff->burstSyncRec[iMirror]	= burst_STag;
		FrameMirrorBuff->timeOF[iMirror]		= ReflectionData[k].TimeOF;
		FrameMirrorBuff->relativeToF[iMirror]	= relativeToF;
		FrameMirrorBuff->rcptTime[iMirror]		= tempRecTime;
		FrameMirrorBuff->amplitude[iMirror]		= ReflectionData[k].Amplitude;
		FrameMirrorBuff->phsDer[iMirror]		= 0U;
		FrameMirrorBuff->cConfLevel[iMirror]	= 12U;
		FrameMirrorBuff->selSignal[iMirror]		= localPDCMframe[busID].rec.echoes[payldCnt].rec.selectedSignal;
		FrameMirrorBuff->cntElements			= iMirror + 1;
	}

}

void
map_Buffer2PDCM(int busID, int USS_ID) {
	/*transfer data from wait4turn buffer to PDCM frame*/

	bool		burstEvent = false, burstOffSlot, burstOnSlot, keepBuffBurst;
	int			iBuffElem, USSbus_ID, startFrame, burstSyncRecpt, nInBuffer, totalFrames, nPayloads;
	uint16_t	receptTime;

	iBuffElem	= 0;
	USSbus_ID	= USS_ID - PdcmFlags->ussPerBus * busID;/*sensor index for buffers*/
	startFrame	= PdcmFlags->lastSlot_ID[busID] + 1; /*the first free PDCM payload slot - should always be 0*/
	nInBuffer	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements;
	totalFrames = startFrame + nInBuffer;
	nPayloads	= (_MAX_NO_PDCM_ECHOES_ >= totalFrames) ? totalFrames : _MAX_NO_PDCM_ECHOES_;

	/*sanity check: mapping should start at PDCM payload slot position 0*/
	if (startFrame != 0)
		LogErrF(EC_Sim, "Mapping should strart at PDCM payload slot position 0, got (%d); please investigate!\n", startFrame);

	/*check if there is burst in the buffer at other positions than 0- error*/
	for (int i = 1; i < nInBuffer; i++) {
		burstOffSlot = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[i] == 1; /*bursts in other positions than 0:*/
		if (burstOffSlot)
			LogErrF(EC_Sim, "vCUS::map_Buffer2PDCM: Burst in wrong position (not first in buffer). Please investigate!\n");
	}

	/* if there is a burst in w4t -  map it to the first pdcm slot*/
	keepBuffBurst	= false;
	burstOnSlot		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[0] == 1;
	if (burstOnSlot) {

		PdcmFlags->nBurstsPDCM = PdcmFlags->nBurstsPDCM + 1;

		if (PdcmFlags->lastSlot_ID[busID] > 0)
			/*first shift the pdcm frame right*/
			shiftPayLdRight(0, startFrame, busID, USS_ID);

		/*place the burst in the first payload slot of the pdcm frame*/
		localPDCMframe[busID].rec.echoes[0].burstInfo.ringingTime		= (uint16_t)_BURST_RING_TIME_ ;
		localPDCMframe[busID].rec.echoes[0].burstInfo.ringingFreq		= (uint16_t)_BURST_FREQUENCY_ ;
		localPDCMframe[busID].rec.echoes[0].burstInfo.notUsed1			= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[iBuffElem];// 0U
		localPDCMframe[busID].rec.echoes[0].burstInfo.burstDelay		= PdcmFlags->syncCnt - PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[iBuffElem];
		localPDCMframe[busID].rec.echoes[0].burstInfo.selectedSignal	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[iBuffElem];
		localPDCMframe[busID].rec.echoes[0].burstInfo.notUsed2			= 0U;

		/*overrite in buffer the burst mapped to PDCM frame*/
		shiftBuffLeft(busID, USS_ID, keepBuffBurst);

		/*increment counters*/
		PdcmFlags->lastSlot_ID[busID]	= PdcmFlags->lastSlot_ID[busID] + 1;
		startFrame						= PdcmFlags->lastSlot_ID[busID] + 1;

		/*recalc mapping parameters*/
		nInBuffer	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements;
		totalFrames = startFrame + nInBuffer;
		nPayloads	= (_MAX_NO_PDCM_ECHOES_ >= totalFrames) ? totalFrames : _MAX_NO_PDCM_ECHOES_;
	}

	/*warning if PDCM frame is full*/
	if (startFrame >= _MAX_NO_PDCM_ECHOES_) {
		if (SimParameters->DebugPDCM_Frame)
			Log("WARNING - vCUS::map_Buffer2PDCM: Number of max PDCM payload slots reached : Max->%d | Got->%d \n\n", _MAX_NO_PDCM_ECHOES_, startFrame);
		PdcmFlags->flag_overflow[USS_ID] = 1;
		return;
	}

	/*set overflow flag if buffer elements do not fit in the pdcm frame*/
	if (totalFrames >= _MAX_NO_PDCM_ECHOES_) {
		PdcmFlags->flag_overflow[USS_ID] = 1;
	}
	else if (totalFrames < _MAX_NO_PDCM_ECHOES_) {
		PdcmFlags->flag_overflow[USS_ID] = 0;
	}

	/*loop for payload*/
	for (int i = startFrame; i < nPayloads; i++) {

		/*calculate reception time*/ 
		/****************/
		burstSyncRecpt	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[iBuffElem];

		/*check for surst sync tag vs sync count mismatch - ensures correct rec time calculation if sync count has been reset since the last burst:
		if burst sync tag is 255 and sync cnt at reporting is 0, the difference should be 1: 0 - 255-256 = 1*/
		if (burstSyncRecpt > PdcmFlags->syncCnt) {
			burstSyncRecpt = burstSyncRecpt - 256;
			if (SimParameters->DebugPDCM_Frame)
				Log("WARNING - vCUS::map_Buffer2PDCM: Burst sync tag and sync cnt missmatch : BST->%d | SyncCnt->%d \n\n", burstSyncRecpt, PdcmFlags->syncCnt);
		}

		/*(syncCnt - burstSyncTag) * 1000 - relative ToF; relative ToF = timestamp echo - timestamp last burst on current sensor*/
		receptTime = (uint16_t)((PdcmFlags->syncCnt - burstSyncRecpt + 1) * 1000 - PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[iBuffElem]);//timeOF[iBuffElem]);
		/****************/

		/*reject echoes with rec time > 65 ms*/
		if (receptTime >= _REC_TIME_MAX_) {
			PdcmFlags->echoesLost[USS_ID] = 1;
			shiftBuffLeft(busID, USS_ID, keepBuffBurst);/*discard this buffer element*/
			i = i - 1;/*reset payload slot ID*/
			if (SimParameters->DebugPDCM_Frame)
				Log("WARNING - vCUS::map_Buffer2PDCM: EchoLost -  Reception time exceeds max, echo was not mapped to PDCM frame, sensor %d\n\n", USS_ID+1);
			continue;
		}

		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[iBuffElem] = receptTime; //will be mapped to PDCM mirror buffer
		
		/*map data to the PDCM frame*/
		localPDCMframe[busID].rec.echoes[i].rec.receptionTime			= receptTime;
		localPDCMframe[busID].rec.echoes[i].rec.amplitude				= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[iBuffElem];
		localPDCMframe[busID].rec.echoes[i].rec.phaseDerivative			= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[iBuffElem];
		localPDCMframe[busID].rec.echoes[i].rec.codingConfidenceLevel	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[iBuffElem];
		localPDCMframe[busID].rec.echoes[i].rec.selectedSignal			= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[iBuffElem];

		PdcmFlags->lastSlot_ID[busID] = i;

		/*keep in temporrary PDCM mirror buffer */
		writeMirrorBuffer(i, iBuffElem, busID, USSbus_ID);

		/*overwrite the buffer element mapped to the PDCM frame payload*/
		shiftBuffLeft(busID, USS_ID, keepBuffBurst);
	}
}

void
shiftBuffLeft(int busID, int ussId, bool keepBurst) {
	/*shifts payloads to the left
	payldCnt is the last free payload
	busId - current bus Id
	ussId - current sensor Id*/

	int USSbus_ID, buffLastElem;

	USSbus_ID		= ussId - PdcmFlags->ussPerBus * busID;/*sensor index for buffers*/
	buffLastElem	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements - 1; //the last free payload

	for (int m = 0; m < buffLastElem; m++) {

		if ((PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[m] == 1) && keepBurst) {
			if (m != 0)
				LogErrF(EC_Sim, "vCUS::shiftBuffLeft: Burst in wrong position (not first in buffer) please investigate!\n\n");/*sanity check*/
			continue;/*do not overwrite this slot if it's a burst*/
		}

		/*move each member to the left by one starting from the last*/
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[m] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[m]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[m]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[m]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[m]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[m]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[m]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[m]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[m + 1];
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[m]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[m + 1];
	}

	/*reset the last one, it was copied to lastOne-1*/
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[buffLastElem]	= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[buffLastElem]	= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[buffLastElem]		= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[buffLastElem]		= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[buffLastElem]		= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[buffLastElem]		= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[buffLastElem]	= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[buffLastElem]		= { 0 };
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[buffLastElem]		= { 0 };

	/*moved all elements to the left, overriten element 0: -1 elements*/
	PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements = buffLastElem;

	/*if no more elements in buffer: reset the buffer for this sensor & bus*/
	if (buffLastElem == 0) {
		/*reset w4t buffer*/
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID] = { 0 };
		/*reset flag*/
		PdcmFlags->flag_wait4Turn[ussId]	= 0;
		PdcmFlags->flag_overflow[ussId]		= 0;
	}
}

void
shift_Buff_Right(int startPld, int busID, int USSbus_ID) {
	/*shifts elements to the right in w4t buffer -  if full moves the last in the o-flow buffer
	w4t_cnt is the last free payload
	USSbus_ID - sensor index for buffers*/

	int w4t_cnt, resetPayld, lastPayload;

	w4t_cnt		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements;
	lastPayload = (w4t_cnt >= _MAX_USS_BUFF_SIZE_) ? (_MAX_USS_BUFF_SIZE_ - 2) : (w4t_cnt - 1);

	if (w4t_cnt < _MAX_USS_BUFF_SIZE_) {

		/*burst in the first position*/
		if ((PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[0] == 1) && (startPld == 0))
			startPld = 1;

		for (int m = lastPayload; m >= startPld; m--) {
			/*move each member to the right by one starting from the last*/
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[m];
			PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[m + 1] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[m];
		}
		/*buffer cntElements is incremented in the parent function*/

		/*reset startPld- it was copied to  startPld+1*/
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[startPld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[startPld] = 0;
		/*increment n of stored echoes*/
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements = w4t_cnt + 1;
	}
	else {//buffer is full: eliminate oldest in position 0 if is not burst, else position 1
		resetPayld = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[0] ? 1 : 0;
		if ((PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[0] == 1) && (PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[1] == 1))
			LogErrF(EC_Sim, "vCUS::shift_Buff_Right: Burst in wrong position (not first in buffer) please investigate!\n");/*sanity check*/

		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[resetPayld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[resetPayld] = 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[resetPayld]	= 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[resetPayld]		= 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[resetPayld]	= 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[resetPayld]		= 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[resetPayld]	= 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[resetPayld]	= 0;
		PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[resetPayld]		= 0;

		/*do not increment count - buffer already full*/
		//PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements = w4t_cnt
	}

}

void
shiftPayLdRight(int startPld, int payldCnt, int i, int j) {
	/*shifts payloads to the right
	payldCnt is the last free payload
	i - current bus Id
	j - current sensor Id*/

	int isLastPayldBurst, USSbus_ID, startPayload, bufferCnt, lastPayLd, overWriteID;

	USSbus_ID		= j - PdcmFlags->ussPerBus * i;/*sensor index for buffers*/
	startPayload	= payldCnt - 1;
	bufferCnt		= PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].cntElements;
	lastPayLd		= _MAX_NO_PDCM_ECHOES_ - 1;
	overWriteID		= 0;

	/*the PDCM frame is full but need to make room for the burst at 0*/
	if (payldCnt >= _MAX_NO_PDCM_ECHOES_) {

		/*first shift data in buffer to clear position 0*/
		if (bufferCnt > 0) {
			shift_Buff_Right(0, i, USSbus_ID);
		}

		/*move the last mirror buffer payload to start of the buffer*/
		int iMirror = FrameMirrorBuff->cntElements - 1;

		/*check that the pdcm payload to transfer matches the mirror buffer data*/
		if ((localPDCMframe[i].rec.echoes[lastPayLd].rec.amplitude != FrameMirrorBuff->amplitude[iMirror]) || 
			(FrameMirrorBuff->rcptTime[iMirror] != localPDCMframe[i].rec.echoes[lastPayLd].rec.receptionTime))
			LogErrF(EC_Sim, "vCUS::shiftPayLdRight: PDCM mirror buffer vs PDCM frame missmatch, please investigate!\n");

		/*write in buffer at position 1 if 0 is a burst- should not be because the burst was already mapped*/
		if (PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].isBurst[0] == 1) {
			overWriteID = 1;
			Log("WARNING - vCUS::shiftPayLdRight: Burst in buffer position 0 should have been mapped to PDCM. Please investigate!\n\n");
		}

		/*retrieve data from the pdcm frame payload and PDCM mirror buffer*/
		isLastPayldBurst													= localPDCMframe[i].rec.echoes[lastPayLd].rec.selectedSignal == 5U;

		// last payload should not be a burst
		if (isLastPayldBurst)
			LogErrF(EC_Sim, "vCUS::shiftPayLdRight: Last payload is burst (not first in buffer) please investigate!\n");

		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].rcptTime[overWriteID]		= localPDCMframe[i].rec.echoes[lastPayLd].rec.receptionTime;
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].timeOF[overWriteID]			= FrameMirrorBuff->timeOF[iMirror];
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].amplitude[overWriteID]		= localPDCMframe[i].rec.echoes[lastPayLd].rec.amplitude;
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].phsDer[overWriteID]			= localPDCMframe[i].rec.echoes[lastPayLd].rec.phaseDerivative;
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].cConfLevel[overWriteID]		= localPDCMframe[i].rec.echoes[lastPayLd].rec.codingConfidenceLevel;
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].selSignal[overWriteID]		= localPDCMframe[i].rec.echoes[lastPayLd].rec.selectedSignal;
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].isBurst[overWriteID]		= isLastPayldBurst;
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].burstSyncRec[overWriteID]	= FrameMirrorBuff->burstSyncRec[iMirror];
		PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].relativeToF[overWriteID]	= FrameMirrorBuff->relativeToF[iMirror];

		
		/* set count if count was not increased in shift_Buff_Right)- redundancy*/
		if ((bufferCnt == PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].cntElements) && (bufferCnt < _MAX_NO_PDCM_ECHOES_))
			PdcmBuffers[i].Wait4TurnBuff[USSbus_ID].cntElements		= bufferCnt + 1;

		/*set flags*/
		PdcmFlags->flag_wait4Turn[j]							= 1;
		PdcmFlags->flag_overflow[j]								= 1;

		/*adjust startPayload if payldCnt >= _MAX_NO_PDCM_ECHOES_*/
		startPayload = _MAX_NO_PDCM_ECHOES_ - 2; // == 3 for daisy ch. PDCM frame with payload slots 0-4
	}

	for (int m = startPayload; m >= startPld; m--) {
		localPDCMframe[i].rec.echoes[m + 1].rec.receptionTime			= localPDCMframe[i].rec.echoes[m].rec.receptionTime;
		localPDCMframe[i].rec.echoes[m + 1].rec.amplitude				= localPDCMframe[i].rec.echoes[m].rec.amplitude;
		localPDCMframe[i].rec.echoes[m + 1].rec.phaseDerivative			= localPDCMframe[i].rec.echoes[m].rec.phaseDerivative;
		localPDCMframe[i].rec.echoes[m + 1].rec.codingConfidenceLevel	= localPDCMframe[i].rec.echoes[m].rec.codingConfidenceLevel;
		localPDCMframe[i].rec.echoes[m + 1].rec.selectedSignal			= localPDCMframe[i].rec.echoes[m].rec.selectedSignal;
	}

	/*reset startPld to 0*/
	localPDCMframe[i].rec.echoes[startPld].rec.receptionTime			= 0;
	localPDCMframe[i].rec.echoes[startPld].rec.amplitude				= 0;
	localPDCMframe[i].rec.echoes[startPld].rec.phaseDerivative			= 0;
	localPDCMframe[i].rec.echoes[startPld].rec.codingConfidenceLevel	= 0;
	localPDCMframe[i].rec.echoes[startPld].rec.selectedSignal			= 0;
}

void
sortPDCM(int busID) {
/*sorts PDCM payloads on reception time: largest rec time first*/

	bool 		isNotSorted, isBurst;
	int 		startPayload, lastPayload;
	uint16_t 	buff_rcptTime, buff_amplitude;
	uint8_t		buff_phsDer, buff_cConfLevel, buff_selSignal;

	/*initialize */
	isBurst			= (localPDCMframe[busID].rec.echoes[0].rec.selectedSignal == 5);
	isNotSorted		= true;
	lastPayload		= PdcmFlags->lastSlot_ID[busID];
	startPayload	= isBurst ? 1 : 0; /*leave burst in place*/

	/*sort*/
	while (isNotSorted) {
		isNotSorted = false; /*set to exit while - reset below if not sorted*/

		/*go through payloads*/
		for (int i = startPayload; i < lastPayload; i++) {

			/*check if payloads should be swapped*/
			if (localPDCMframe[busID].rec.echoes[i].rec.receptionTime < localPDCMframe[busID].rec.echoes[i + 1].rec.receptionTime) {
				isNotSorted = true; /*found not sorted payload*/

				/*data in temp buffer*/
				buff_rcptTime	= localPDCMframe[busID].rec.echoes[i].rec.receptionTime;
				buff_amplitude	= localPDCMframe[busID].rec.echoes[i].rec.amplitude;
				buff_phsDer		= localPDCMframe[busID].rec.echoes[i].rec.phaseDerivative;
				buff_cConfLevel = localPDCMframe[busID].rec.echoes[i].rec.codingConfidenceLevel;
				buff_selSignal	= localPDCMframe[busID].rec.echoes[i].rec.selectedSignal;

				/***swap payloads*/
				localPDCMframe[busID].rec.echoes[i].rec.receptionTime			= localPDCMframe[busID].rec.echoes[i + 1].rec.receptionTime;
				localPDCMframe[busID].rec.echoes[i].rec.amplitude				= localPDCMframe[busID].rec.echoes[i + 1].rec.amplitude;
				localPDCMframe[busID].rec.echoes[i].rec.phaseDerivative			= localPDCMframe[busID].rec.echoes[i + 1].rec.phaseDerivative;
				localPDCMframe[busID].rec.echoes[i].rec.codingConfidenceLevel	= localPDCMframe[busID].rec.echoes[i + 1].rec.codingConfidenceLevel;
				localPDCMframe[busID].rec.echoes[i].rec.selectedSignal			= localPDCMframe[busID].rec.echoes[i + 1].rec.selectedSignal;
				/*get from buffer*/
				localPDCMframe[busID].rec.echoes[i + 1].rec.receptionTime			= buff_rcptTime;
				localPDCMframe[busID].rec.echoes[i + 1].rec.amplitude				= buff_amplitude;
				localPDCMframe[busID].rec.echoes[i + 1].rec.phaseDerivative			= buff_phsDer;
				localPDCMframe[busID].rec.echoes[i + 1].rec.codingConfidenceLevel	= buff_cConfLevel;
				localPDCMframe[busID].rec.echoes[i + 1].rec.selectedSignal			= buff_selSignal;
			}//end if swap payloads
		}//end for n payloads
	}//end while not sorted

}// end func

void
sortBuff(int busID, int ussId) {
/*sort buffer on time of flight*/

	bool 		isNotSorted;
	int 		startBuffElem, lastBuffElem, isBurst, USSbus_ID, buff_isBurst, buff_BurstTag;
	uint8_t		buff_phsDer, buff_cConfLevel, buff_selSignal;
	uint16_t 	buff_rcptTime, buff_amplitude;
	uint64_t	buff_timeOF, buff_relativeToF;

	/*initialize */
	USSbus_ID = ussId - PdcmFlags->ussPerBus * busID;/*sensor index for buffers*/
	isBurst = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[0];
	isNotSorted = true;
	lastBuffElem = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cntElements - 2;
	startBuffElem = isBurst ? 1 : 0; /*leave burst in place*/

	/*sort*/
	while (isNotSorted) {
		isNotSorted = false; /*set to exit while - reset below if not sorted*/

		/*go through payloads*/
		for (int i = startBuffElem; i < lastBuffElem; i++) {

			/*no bursts should be in other than position 0*/
			if (PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[i])
				LogErrF(EC_Sim, "vCUS::sortBuff: Burst in wrong position (not first in buffer) please investigate!\n");

			/*check if payloads should be swapped*/
			if (PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[i] <
					PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[i + 1]) {

				isNotSorted = true; /*found not sorted payload*/

				/*data in temp buffer*/
				buff_BurstTag		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[i];
				buff_relativeToF	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[i];
				buff_timeOF			= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[i];
				buff_rcptTime		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[i];
				buff_amplitude		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[i];
				buff_phsDer			= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[i];
				buff_cConfLevel		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[i];
				buff_selSignal		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[i];
				buff_isBurst		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[i];

				/***swap payloads*/
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[i] = PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[i]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[i]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[i]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[i]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[i]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[i]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[i]	= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[i + 1];
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[i]		= PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[i + 1];
				/*get from buffer*/
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].burstSyncRec[i + 1] = buff_BurstTag;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].relativeToF[i + 1]	= buff_relativeToF;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].timeOF[i + 1]		= buff_timeOF;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].rcptTime[i + 1]		= buff_rcptTime;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].amplitude[i + 1]	= buff_amplitude;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].phsDer[i + 1]		= buff_phsDer;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].cConfLevel[i + 1]	= buff_cConfLevel;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].selSignal[i + 1]	= buff_selSignal;
				PdcmBuffers[busID].Wait4TurnBuff[USSbus_ID].isBurst[i + 1]		= buff_isBurst;
			}//end if swap payloads
		}//end for n payloads
	}//end while not sorted
}//end func

void logPDCM_frame(int iBus) {
	
	if (iBus == 0) {
		Log("                         \n");
		
	}

	Log("Header: phyAddr %d; burstFlag %d; pdcmMode %d; rxTrDelay %d; echosLost %d; blankingFlag %d; burstErrFlag %d; sumErrorFlag %d; muxDataCnt %d; syncCnt %d; AATG1TH %d; AATG2TH %d; muxData %d; \n",
		localPDCMframe[iBus].rec.header.rec.physicalAddress,
		localPDCMframe[iBus].rec.header.rec.burstFlag,
		localPDCMframe[iBus].rec.header.rec.pdcmMode,
		localPDCMframe[iBus].rec.header.rec.rxTransDelay,
		localPDCMframe[iBus].rec.header.rec.echosLost,
		localPDCMframe[iBus].rec.header.rec.blankingFlag,
		localPDCMframe[iBus].rec.header.rec.burstErrorFlag,
		localPDCMframe[iBus].rec.header.rec.sumErrorFlag,
		localPDCMframe[iBus].rec.header.rec.multiplexDataCnt,
		localPDCMframe[iBus].rec.header.rec.syncCntCurrent,
		localPDCMframe[iBus].rec.header.rec.AATG1_TH,
		localPDCMframe[iBus].rec.header.rec.AATG2_TH,
		localPDCMframe[iBus].rec.header.rec.multiplexedData);

	for (int m = 0; m < _MAX_NO_PDCM_ECHOES_; m++) {
		/*if (localPDCMframe[j].rec.echoes[m].rec.receptionTime == 0)
			continue;/*skip payload if empty*/

		if (localPDCMframe[iBus].rec.echoes[m].rec.selectedSignal == 5U) {
			Log("***** payload %d: \t ringTime %5d;\t ringFreq %5d;\t notUsed %2d;\t burstDelay %2d;\t selSig %d;\t ToF %d\n",
				m,
				localPDCMframe[iBus].rec.echoes[0].burstInfo.ringingTime ,
				localPDCMframe[iBus].rec.echoes[0].burstInfo.ringingFreq ,
				localPDCMframe[iBus].rec.echoes[0].burstInfo.notUsed1 ,
				localPDCMframe[iBus].rec.echoes[0].burstInfo.burstDelay ,
				localPDCMframe[iBus].rec.echoes[0].burstInfo.selectedSignal ,
				FrameMirrorBuff->timeOF[m]);
		}
		else {
			Log("***** payload %d: \t recTime %5d;\t Amp %5d;\t phaseDer %2d;\t ConfLvl %2d;\t selSig %d;\t ToF %d\n",
				m,
				localPDCMframe[iBus].rec.echoes[m].rec.receptionTime,
				localPDCMframe[iBus].rec.echoes[m].rec.amplitude,
				localPDCMframe[iBus].rec.echoes[m].rec.phaseDerivative,
				localPDCMframe[iBus].rec.echoes[m].rec.codingConfidenceLevel,
				localPDCMframe[iBus].rec.echoes[m].rec.selectedSignal,
				FrameMirrorBuff->timeOF[m]);
		}
	}
	Log(" \n");
}

void
writeMirrorBuffer(int iMirror, int iBUffer, int busId, int USS_busId) {
/*maps normal buffer to PDCM frame mirror buffer 
- data may be restored to buffer after PDCM frame  mapping  if taken out of frame for burst*/

	FrameMirrorBuff->isBurst[iMirror]		= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].isBurst[iBUffer];
	FrameMirrorBuff->burstSyncRec[iMirror]	= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].burstSyncRec[iBUffer];
	FrameMirrorBuff->timeOF[iMirror]		= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].timeOF[iBUffer];
	FrameMirrorBuff->relativeToF[iMirror]	= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].relativeToF[iBUffer];
	FrameMirrorBuff->rcptTime[iMirror]		= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].rcptTime[iBUffer];
	FrameMirrorBuff->amplitude[iMirror]		= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].amplitude[iBUffer];
	FrameMirrorBuff->phsDer[iMirror]		= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].phsDer[iBUffer];
	FrameMirrorBuff->cConfLevel[iMirror]	= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].cConfLevel[iBUffer];
	FrameMirrorBuff->selSignal[iMirror]		= PdcmBuffers[busId].Wait4TurnBuff[USS_busId].selSignal[iBUffer];
	FrameMirrorBuff->cntElements			= iMirror + 1;
}

void
sortMirrorBuffer() {

	bool isNotSorted;
	int			buff_isBurst, buffburstSyncTag, lastElem;
	uint8_t		buff_phsDer, buff_cConfLevel, buff_selSignal;
	uint16_t	buff_rcptTime, buff_amplitude;
	uint64_t	buff_timeOF, buff_relativeToF;

	lastElem	= FrameMirrorBuff->cntElements - 1;
	isNotSorted = true;

	while(isNotSorted){
		isNotSorted = false;
		
		for (int i = 0; i < lastElem; i++) {
			if (FrameMirrorBuff->rcptTime[i] < FrameMirrorBuff->rcptTime[i + 1]) {
				isNotSorted = true;

				/*data in temp buffer*/
				buff_isBurst		= FrameMirrorBuff->isBurst[i];
				buffburstSyncTag	= FrameMirrorBuff->burstSyncRec[i];
				buff_timeOF			= FrameMirrorBuff->timeOF[i];
				buff_relativeToF	= FrameMirrorBuff->relativeToF[i];
				buff_rcptTime		= FrameMirrorBuff->rcptTime[i];
				buff_amplitude		= FrameMirrorBuff->amplitude[i];
				buff_phsDer			= FrameMirrorBuff->phsDer[i];
				buff_cConfLevel		= FrameMirrorBuff->cConfLevel[i];
				buff_selSignal		= FrameMirrorBuff->selSignal[i];

				/*swap elements*/
				FrameMirrorBuff->isBurst[i]			= FrameMirrorBuff->isBurst[i + 1];
				FrameMirrorBuff->burstSyncRec[i]	= FrameMirrorBuff->burstSyncRec[i + 1];
				FrameMirrorBuff->timeOF[i]			= FrameMirrorBuff->timeOF[i + 1];
				FrameMirrorBuff->relativeToF[i]		= FrameMirrorBuff->relativeToF[i + 1];
				FrameMirrorBuff->rcptTime[i]		= FrameMirrorBuff->rcptTime[i + 1];
				FrameMirrorBuff->amplitude[i]		= FrameMirrorBuff->amplitude[i + 1];
				FrameMirrorBuff->phsDer[i]			= FrameMirrorBuff->phsDer[i + 1];
				FrameMirrorBuff->cConfLevel[i]		= FrameMirrorBuff->cConfLevel[i + 1];
				FrameMirrorBuff->selSignal[i]		= FrameMirrorBuff->selSignal[i + 1];
					/*retreieve i from buffers*/
				FrameMirrorBuff->isBurst[i + 1]			= buff_isBurst;
				FrameMirrorBuff->burstSyncRec[i + 1]	= buffburstSyncTag;
				FrameMirrorBuff->timeOF[i + 1]			= buff_timeOF;
				FrameMirrorBuff->relativeToF[i + 1]		= buff_relativeToF;
				FrameMirrorBuff->rcptTime[i + 1]		= buff_rcptTime;
				FrameMirrorBuff->amplitude[i + 1]		= buff_amplitude;
				FrameMirrorBuff->phsDer[i + 1]			= buff_phsDer;
				FrameMirrorBuff->cConfLevel[i + 1]		= buff_cConfLevel;
				FrameMirrorBuff->selSignal[i + 1]		= buff_selSignal;
			}//end if not sorted
		}//end for i elements
	}//end while	
}//end func

#endif