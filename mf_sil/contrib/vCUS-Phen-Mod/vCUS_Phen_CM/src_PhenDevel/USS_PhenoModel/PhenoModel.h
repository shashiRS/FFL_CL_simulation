#ifndef _USS_Ext_H__
#define _USS_Ext_H__

/* System parameters */
#define _CSV_CountryCode_ 0 /* Country Code for different setups/layouts: 0->Germany / 1->Romania */
#define _NO_OF_SENSORS_MAX_ 12
#define _NO_TOS_MAX_		50 // 5 for testing | 50 for sim
#define _NO_TOPOINTS_MAX_	100
#define _NO_OF_ECHOES_MAX_	60 // 5 for testing | 60 for sim
#define _NO_OF_FIRESAMPLES_ 8
#define _NO_OF_BUSES_ 2
#define _NO_OF_USS_PER_BUS_ 6
#define BUFSIZE 1048
#define _RCPmsCorrection 1000

//#define CARMAKER_NUMVER  100000
#if CARMAKER_NUMVER >= 100000
#define TRF_INFOSIZE    80
#define TRF_NAMESIZE    10
#define TRF_ATTRIBSIZE  10
#define TRF_CONTOURPNTS 200
#endif /* CARMAKER_NUMVER */

/* Internal sensor Parameters */
#define _MaxNPDirDeviation_	1 // in deg
#define	_Sensor_Deaf_Time_ 0.00001 //2

/* HiL-API specific defines */
#define _MAX_NO_PDCM_ECHOES_ 5 //6 for p2p; 5 for daisy chain PDCM frame

//#define US_DRV_MAX_QUEUE_LENGTH_U16 static_cast<uint16_t>(36U * 10U * 2U) // 36=2*18Sens * 10*2 = 10 * Dir/Indir
#define USS_MAX_SENSOR_CNT_U8 static_cast<uint8_t>(18U) // Currently only 12 supported

#include <stdint.h>
#include <stdbool.h>

/* HINT: Please comment or uncomment the following #define in order to choose HIL or SIL mode*/
//#define vCUS_HIL_MODE

#ifndef vCUS_HIL_MODE
#include "us_drv_def_vfa.h"
#include "us_drv/us_drv_generated_types.h"
#else
#include "us_drv_types.h"
#endif // define if not running in vCUS_HIL_MODE

#ifdef __cplusplus
extern "C" {
#endif

	int		vCUS_InitData(void);
	int		vCUS_Main(void);
	int		vCUS_TestRun_Start_Finalize(void);
	int		vCUS_ReflectionGenerator(int);
	void	vCUS_Cleanup(void);
	void	vCUS_InitReflGen(void);
	void	vCUS_ResetVarTREnd(void);
	void	getVirtualSensorPos(int, int);
	void	getGlobalSensorPosition(int);
   
#if CARMAKER_NUMVER >= 100000
	void	PDCM_DDict(int ID);
	void	vCUS_ReadVhclInfo(void);
#endif /* CARMAKER_NUMVER */

	/* For vCUS_HIL_MODE */
#ifdef vCUS_HIL_MODE
	void    vCUS_GetPDCMFrame(unsigned char *);
	void    vCUS_SetFireScheme(uint64_t *, unsigned int );
	void    vCUS_SelFireScheme(unsigned int , unsigned int );
#endif /* vCUS_HIL_MODE */

#ifdef __cplusplus
}
#endif


typedef struct tToFwait {
	uint64_t	NextSFSP[_NO_OF_SENSORS_MAX_];		//next burst time per sensor
	uint64_t	CycleNo[_NO_OF_SENSORS_MAX_];		//last burst time per sensor
	double		DataTrans_Running[_NO_OF_SENSORS_MAX_][_NO_OF_ECHOES_MAX_];
	double		ToF_Counter[_NO_OF_SENSORS_MAX_][_NO_OF_ECHOES_MAX_];
				/* Used for evaluation if one target measurment ToF-Count has already been increased in this cycle */
	uint64_t	ToF_Counter_State[_NO_OF_SENSORS_MAX_][_NO_OF_ECHOES_MAX_];
	uint64_t	SimCoreStartCycle;
	bool		SimCoreStartCycleFlag;
	uint64_t	timeStamp_ms;
} tToFwait;
extern tToFwait *ToFwait;



typedef struct tSensCharacteristic {
	bool	Type;								/* Amplitude Type: 0->SPA / 1->SPL */
	double	SPANormRange;						/* Measurement Range for Gain Map Meas */
	double	MaxSPA;								/* Maximum SPA in Gain Map */
	double	p0;									/* SPA @ 0 Azi / 0 Ele */
	tLM2D	*LobeMap;							/* 2D Map of Gain Map Meas */
}tSensCharacteristic;

typedef struct tSimParameters {
	int			SWSwitch_SFSPon;											/* Flag for usage of Stochastic Code -> 1-on / 0-off */
	int			SWSwitch_CrossEchoes;										/* Flag for cross echoes generation*/
	int			SWSwitch_DirectEchoes;										/* Flag for direct echoes generation*/
	int			SWSwitch_CustomInRange;										/* Use the CM-independent determination of the in-range TOs: SWSwitch_CustomInRange = 1 (sim slower)*/
	int			SWSwitch_RxCorection_ToF;									/* Switch off and on the correction for Rx movement between burst/pulse and received echo*/
	int			SWSwitch_RCPmsCorrection;									/* For older USP veriosns there need to be a 1000us correction o0f the receptions */
	int			SWSwitch_SuppressInfo;										/* Suppress all logs in SessionLog */
	double		min_height_TO;												/* TO detection threshold, no reflections from TOs below this height*/
	int			USP_UpdateRate;												/* USP Package Update rate (e.g. 33ms, 40ms, 100ms) */
	uint64_t	fire_scheme[_NO_OF_SENSORS_MAX_][_NO_OF_FIRESAMPLES_];		/* SFSP Values */
	tLM2D		*fire_scheme_map;											/* SFSP 2D linear map */
	int			DebugOpt;													/* Debug Flag: 0->Off | 1->Standard Debug */
	int			DebugOpt_Amp;												/* Debug Flag for Amplitude Calc */
	int			DebugOpt_USP;												/* Debug Flag for ReflectionData and USP output */
	int			DebugPDCM_Frame;											/* Debug flag for PDCM frame - if set shows PDCM frame in Log */
	tSensCharacteristic	SensChar;											/* Array with Sensor Characteristics e.g. sending/receiving beam pattern */
	int			isSIL;														/* Solution shall be compiled for SiL or HiL */
	uint64_t	HiLOffsetDelay;												/* Delay added to the time stamp */
} tSimParameters;
extern tSimParameters *SimParameters;

typedef struct tInt_var {
	int			targ_ID;
	int			TO_ID[_NO_TOS_MAX_];
	int			noTargets;
	double		Fr0_position_NP[_NO_OF_ECHOES_MAX_][3];
	double		NPDir_Fr0[_NO_OF_ECHOES_MAX_];							/*should be removed, only Rx and/or Tx directions should be used*/
	double		NPDirRx_Fr0[_NO_OF_ECHOES_MAX_];						/*azimuth of NP from Rx USS*/
	double		NPDirTx_Fr0[_NO_OF_ECHOES_MAX_];						/*azimuth of NP from Tx USS*/
	double		NPElevRx_Fr0[_NO_OF_ECHOES_MAX_];						/*elevation of NP from Rx USS*/
	double		NPElevTx_Fr0[_NO_OF_ECHOES_MAX_];						/*elevation of NP from Tx USS*/
	double		targ_distance[_NO_OF_ECHOES_MAX_];						/*Might be occluded -> Will be checked in vCUS_OcclusionEval() */
	double		Rx_coord_corrected[_NO_OF_ECHOES_MAX_][3];				/*Rx coordinates corrected for vehicle movement during ToF*/
	double      egoYaw_Reception[_NO_OF_ECHOES_MAX_];					/*ego yaw at reception, after correcting for movement during ToF*/
	int			Detc_TO_ID[_NO_OF_ECHOES_MAX_];
	int			Detc_TO_Surface_ID[_NO_OF_ECHOES_MAX_];
	int			Tx[_NO_OF_ECHOES_MAX_];
	int			Rx[_NO_OF_ECHOES_MAX_];

	/*for debugging*/
	double		NP_uncorrected[_NO_OF_ECHOES_MAX_][3];					/*NP coordinates without correction for ego movement*/
} tInt_var;
extern tInt_var *int_var;


typedef struct tIntReflData {
	uint64_t	ReflDataTimer;														/* uspExec_counter timer for writing ReflData to UsRawData (e.g. 40ms)*/
	uint16_t	ReflDataCnt;														/* Reflection counter for "growing" ReflectionData */
} tIntReflData;
extern tIntReflData *IntReflData;


typedef struct tVector_calc {
	double NormalDir[_NO_TOPOINTS_MAX_];
	double NormalsCombined[_NO_TOPOINTS_MAX_];
	double NormalDirDefault[2];
	double PosVector[2];
} tVector_calc;
extern tVector_calc *env_vector;


typedef struct tTrafficObject {
	double		boundary_points[_NO_TOPOINTS_MAX_][2];								// Boundary points for traffic objects [clockwise] [x | y]
	/***redundant computations of boundary_pointsAngles for previous occulssion version***/
	/*
	double		boundary_pointsAngles[_NO_OF_SENSORS_MAX_][_NO_TOPOINTS_MAX_];		// Boundary point angles for traffic objects relative to Fr0 from sensor [clockwise]
	double		boundary_pointsAngles_min[_NO_OF_SENSORS_MAX_];						// Boundary point minimum angles for traffic objects relative to Fr0 from sensor [Sensor_ID]
	double		boundary_pointsAngles_max[_NO_OF_SENSORS_MAX_];						// Boundary point maximum angles for traffic objects relative to Fr0 from sensor [Sensor_ID]
	*/
	double		normals[_NO_TOPOINTS_MAX_][2];										// Normal vectors for bounding box [clockwise] [x | y]
	double		NearestPointDir_Fr0[_NO_OF_SENSORS_MAX_];							// Global NearestPoint azimuth angle in Fr0 for each sensor
	double		NearestPointElev_Fr0[_NO_OF_SENSORS_MAX_];							// Global NearestPoint elevation angle in Fr0 for each sensor
	double		NearestPointPos_Fr0[_NO_OF_SENSORS_MAX_][3];						// Global NearestPoint position in Fr0 for each sensor
	double		OccNearestPointPos_Fr0[_NO_OF_SENSORS_MAX_][3];						// Global NearestPoint position in Fr0 for each sensor for occlusion determination
	double		vNearestPointPos_Fr0[_NO_OF_SENSORS_MAX_][3];						// Global NearestPoint position in Fr0 for each virtual sensor (for cross-echoes)
	double		vNpDirLeft[_NO_OF_SENSORS_MAX_];									// Global virtual NearestPoint azimuth angle in Fr0 for the counterclockwise sensor
	double		vNpDirRight[_NO_OF_SENSORS_MAX_];									// Global virtual NearestPoint azimuth angle in Fr0 for the clockwise sensor
	double		vNpDistLeft[_NO_OF_SENSORS_MAX_];									// Global virtual NearestPoint distance the counterclockwise sensor
	double		vNpDistRight[_NO_OF_SENSORS_MAX_];									// Global virtual NearestPoint distance clockwise sensor
	double		vNpElevLeft[_NO_OF_SENSORS_MAX_];									// Global virtual NearestPoint elevation angle in Fr0 for the counterclockwise sensor
	double		vNpElevRight[_NO_OF_SENSORS_MAX_];									// Global virtual NearestPoint elevation angle in Fr0 for the clockwise sensor
	double		vLoF[_NO_OF_SENSORS_MAX_];											// LoF for the current virtual NP: LoF from real sensor to the right to real sensor to the left
	double		vNearestPointDir_Fr0[_NO_OF_SENSORS_MAX_];							// Global NearestPoint azimuth angle in Fr0 for each virtual sensor
	double		vNpA[_NO_OF_SENSORS_MAX_];											// Ellipse parameter of indirect echo - a
	double		vNpB[_NO_OF_SENSORS_MAX_];											// Ellipse parameter of indirect echo - b
	int			vNpSurface[_NO_OF_SENSORS_MAX_];									// TO surface for the virtual NP
	bool		vNpExists[_NO_OF_SENSORS_MAX_];										// Flag for virtual NP
	bool		inRange[_NO_OF_SENSORS_MAX_];										// in range Flag for custom in range function
	bool		NPisCorner[_NO_OF_SENSORS_MAX_];									// flag for NP on a TO corner
	bool		virtNPisCorner[_NO_OF_SENSORS_MAX_];								// flag for virtual NP on a TO corner
} tTrafficObject;
extern tTrafficObject *TrfObj;



typedef struct tSensorParam {
	double				orientation;										// Sensor orientation around z-axis
	double				Fr1_position[3];									// Sensor position in Fr1
	double				vFr1_position[3];									// virtual Sensor position in Fr1
	double				Range;												// Max. range of sensor in m
	double				Azimuth;											// Max. hFoV
	double				Elevation;											// Max. vFoV
} tSensorParam;
extern tSensorParam *SensParam;


typedef struct tvCUSEcho {
	double		Fr0_position_NP[_NO_OF_ECHOES_MAX_][3];								// Non-occluded Nearest Point / Reflection point position in Fr0 for TARGET [x | y]
	double		Fr1_position_NP[_NO_OF_ECHOES_MAX_][3];								// Reflection point position in Fr1 for TARGET [x | y | z]
	double		Fr0_position_Sens_Tx[_NO_OF_ECHOES_MAX_][3];						// Tx Sensor position in Fr0
	bool		vTarget_Flag[_NO_OF_ECHOES_MAX_];									// Flag indicating if it's a real or virtual target (direct or cross echo) -> 0-direct | 1-indirect
	double		ToF[_NO_OF_ECHOES_MAX_];											// Time-of-Flight to Non-occluded target/v-target
	double		echoAmplitude[_NO_OF_ECHOES_MAX_];									// Digital amplitude of the incoming wave (at Rx)
	uint64_t	vCUSBurstTime[_NO_OF_ECHOES_MAX_];									// Time stamp for burst time
	int			Tx[_NO_OF_ECHOES_MAX_];												// Transmitting Sensor ID (not necessary for UsRaw but for ground truth)
	int			Rx[_NO_OF_ECHOES_MAX_];												// Receiving sesor, for UsRaw
	int			Detc_TO_ID[_NO_OF_ECHOES_MAX_];										// Detected Traffic Object ID for TARGET
	int			nReflections;														// Total number of reflections (dir + crs)
} tvCUSEcho;

typedef struct tSensorData {
	double			Fr0_position_Sens[3];												// Sensor position in Fr0
	double			Fr0_position_vSens[3];												// Sensor position in Fr0 of virtual Sensors for Cross Echoes
	double			Fr0_vUssDirection;													// Direction of the virtual sensor in FR0 (realSensRight+realSensLeft)/2
	uint64_t		TimeStamp;															// Current/Last sensor fire scheme trigger sample point ("0" means, no sensor triggering)
	int				SFSP_Flag;															// Flag for SFSP for calling sensor calc
	int				SFSP_cnt;															// SFSP firing index for PDCM header mapping 
	tvCUSEcho		Target;																// Non-occluded target information
} tSensorData;
extern tSensorData *SensData;

typedef struct tReflectionData {
	double		Fr0_position_NP[3];								// Nearest Point / Reflection point position in Fr0 for TARGET [x | y]
	double		Fr1_position_NP[3];								// Nearest Point / Reflection point position in Fr1 for TARGET [x | y | z]
	double		Fr0_position_Sens[3];							// Sensor position in Fr0
	double		Fr0_position_Sens_Tx[3];						// Tx Sensor position in Fr0
	uint64_t	TimeOF;											// Time of flight in us of a Reflection Point for TARGET
	int			Detc_TO_ID;										// Detected Traffic Object ID for TARGET
	uint64_t	TimeTag;										// System time for sensor burst or incoming wave (depends on BurstFlag) in us
	int			BurstIndic;										// Indication for Tx/Rx a burst incl. SFSP-ID
																/*	pulseOrEchoTag = echoIn.measTag_ms_u8 & 0x80U; /// 0x80 = 128U = 0b1000 0000
																	isPulse = (pulseOrEchoTag == 128U);
																	isEcho = (pulseOrEchoTag < 12U); */
	int			Rx;												// Virtual receiver of the burst
	int			Tx;												// Virtual transceiver of the burst
	int			SFSP_cnt;										// SFSP count 
	uint16_t	Amplitude;									    // Amplitude of measurement
	int			CycleNo;										// keeps track of the usp execution cycle count for the current echo (e.g. 40ms)
} tReflectionData;
extern tReflectionData *ReflectionData;

typedef struct tRxState_tof {						// structure holding the current state of the Rx sensor during Rx position/orientation correction for movement during ToF
	int		rxId;									// current receiving sensor
	int		txId;									// current transmitting sensor
	int		startNP_Tile;							// the TO tile for the initial NP position (uncorrected)
	int		TOid;									// current TO id
	double	distDiff;								// difference in distance travelled by ego vehicle during the first tof (calculated without position correction) and the second tof after Rx position correction 
	double	egoYaw_Final;							// ego vehicle yaw after ToF- takes into account yaw rate -- in radian !!!
	double  initialNP_coord[3];						// stores the initial NP coordinates
	double	nearestPoint_Coord[3];					// current NP coordinates
	double	coord_RxFr0_FirstToF[3];				// coordinates of Rx in Fr0 after corrected for ego movement during the first ToF
	double	Fr1Rx_start[3];							// Rx coordinates in FR1 (calculated at start-- always the same)
	double	coord_RxFr0_updated[3];					// updated Rx coordinates in Fr0 --same as coord_RxFr0_FirstToF
	double	limFoVAngle_corrected[2];				// FoV limit angles at the new movement-corrected position, based on new yaw
	double	RxDirection_corrected;					// Rx direction corrected for movement during ToF, based on new yaw
	double	Rx2Np_orientation;						// Orientation of Rx to Np direction after movement correction and NP update during ToF
	double  Tx2Np_orientation;						// Orientation of Tx to Np direction after Rx movement correction and NP update during ToF
	bool	NpInFov_updated;						// updated flag for NP in FoV of Rx after Rx movement correction during ToF
	bool	npIsUpdated;							// flag - original NP is replaced - 1; or is kept 0

	//for debugging 
	double initialToF;								// initial time of flight without ego movement correction, in ms
	double correctedToF;							// tof with Rx and NP coordinates corrected for ego movement
	double egoVelocity;								// eg velocity in the movement direction (sqrt (vx^2+vy^2))
	double Rx_displacement;							// distance between Rx initial and corrected position
	double NP_displacement;							// same but for NP
	double initialLof;								// uncorrected LoF
	double correctedLof;							// LoF calculated with the corrected NP and Rx 
	double vehYaw;									// vehicle Yaw in rad
	int	isDirectEcho;								// flag 1 if it's a direct echo, 0 if cross
}tRxState_tof;
extern tRxState_tof *RxState_tof;

//traffic object countour coordinates output to DAVs
typedef struct tToDatacontour {
	char	ToName[TRF_NAMESIZE];						// Traffic object name 
	char 	ToType[TRF_INFOSIZE];						// Traffic object info
	int		ToID;										// TO ID
	double	ToHeight;									// TO height
	int		nDataP;										// number of datapoints for outline (number of tiles/planes forming the object)
	double	Point0xyEdge[TRF_CONTOURPNTS][2];			// mapped to DAVs : outlie of the traffic objects
}tToDatacontour;
extern tToDatacontour *ToDatacontour;

#ifdef vCUS_HIL_MODE
/*data structures for PDCMFrame_Mapping */
#define _REC_TIME_MAX_ 65000 // in us? - 65 ms
#define _N_OF_BUSES_MAX_  2
#define _N_USS_PER_BUS_MAX_  6
#define _MAX_USS_BUFF_SIZE_ 140
#define _BURST_RING_TIME_ 42 //burst ring down time about 550 [us] for NFD, not known for FFD , used != 0
#define _BURST_FREQUENCY_ 53000// burst frequency [Hz]

typedef struct tWait4TurnBuff {
	/*payloads*/
	int 		cntElements;										// number of current buffer elements 
	int			isBurst[_MAX_USS_BUFF_SIZE_];						// indicates if the event is a burst - 1 or echo -0
	int			burstSyncRec[_MAX_USS_BUFF_SIZE_];					// reception burst sync tag - see Daisy Chain PDCM frame reception time calculation
	int			TxID[_MAX_USS_BUFF_SIZE_];							// tx id for debugging
	int			receptSyncCnt[_MAX_USS_BUFF_SIZE_];					// echo reception sync count for debugging
	uint64_t	timeOF[_MAX_USS_BUFF_SIZE_];						// actual time of flight
	uint64_t	relativeToF[_MAX_USS_BUFF_SIZE_];					// relative time of flight calculated as the echoTimeStamp - LastBurstTimestamp
	uint16_t 	rcptTime[_MAX_USS_BUFF_SIZE_];						// reception time; 16 bit
	uint16_t 	amplitude[_MAX_USS_BUFF_SIZE_];						// amplitude of echo; 16 bit
	uint8_t		phsDer[_MAX_USS_BUFF_SIZE_];						// phase derrivative - the standard path for the echoes received range from -128 to 127; 8 bit
	uint8_t		cConfLevel[_MAX_USS_BUFF_SIZE_];					// freq coding confidence level - echo quality: 0 to 15; 4 bit
	uint8_t 	selSignal[_MAX_USS_BUFF_SIZE_];						// former Rx path  1 no ch up/down, 2/3 for ch up/down, 4 - NFD, 5 for bursts, 3 bit
} tWait4TurnBuff;

typedef struct tFrameMirrorBuff {
	int 		cntElements;										// number of current buffer elements 
	int			isBurst[_MAX_NO_PDCM_ECHOES_];						// indicates if the event is a burst - 1 or echo -0
	int			burstSyncRec[_MAX_NO_PDCM_ECHOES_];					// reception burst sync tag - see Daisy Chain PDCM frame reception time calculation
	uint64_t	timeOF[_MAX_NO_PDCM_ECHOES_];						// actual time of flight
	uint64_t	relativeToF[_MAX_NO_PDCM_ECHOES_];					// relative time of flight calculated as the echoTimeStamp - LastBurstTimestamp
	uint16_t 	rcptTime[_MAX_NO_PDCM_ECHOES_];						// reception time; 16 bit
	uint16_t 	amplitude[_MAX_NO_PDCM_ECHOES_];						// amplitude of echo; 16 bit
	uint8_t		phsDer[_MAX_NO_PDCM_ECHOES_];						// phase derrivative - the standard path for the echoes received range from -128 to 127; 8 bit
	uint8_t		cConfLevel[_MAX_NO_PDCM_ECHOES_];					// freq coding confidence level - echo quality: 0 to 15; 4 bit
	uint8_t 	selSignal[_MAX_NO_PDCM_ECHOES_];						// former Rx path  1 no ch up/down, 2/3 for ch up/down, 4 - NFD, 5 for bursts, 3 bit
	double		ToF[_MAX_NO_PDCM_ECHOES_];							//ToF for debugging
} tFrameMirrorBuff;
extern tFrameMirrorBuff *FrameMirrorBuff;

typedef struct tPdcmFlags {
	int			flag_overflow[_NO_OF_SENSORS_MAX_];						// flag for overflow for each sensor; initialize to 0		
	int			flag_wait4Turn[_NO_OF_SENSORS_MAX_];					// flag for waiting 4 turn for each sensor; initialize to 0
	uint8_t		muxDataCnt[_NO_OF_SENSORS_MAX_];						// multiplex data counter for each sensor - PDCM header info
	int			burstSyncTag[_NO_OF_SENSORS_MAX_];						// sync count at the burst time
	uint64_t	burstTStamp[_NO_OF_SENSORS_MAX_];						// timestamp of last burst - for rec time calculation
	int			nextSensorID[_N_OF_BUSES_MAX_];							// the sensor ID to send the next PDCM frame  for each daisy chain bus
	int			headerDone[_N_OF_BUSES_MAX_];							// flag is set io the header was done for this bus
	uint8_t		syncCnt;												// keeps track of sync count - PDCM header info
	int			lastSlot_ID[_N_OF_BUSES_MAX_];							// keeps track of the last ocupied payload slot in the current PDCM frame
	int			nBuses;													// number of daisy chain buses for the current sim - 1 for nOS <= 6; and 2 for nOS > 6 - here only even nOS accepted
	int			ussPerBus;												// number of sensor per daisy chain bus
	int			nBurstsPDCM;											// number of bursts in each PDCM frame (all buses)
	int			nPayldPDCM;												// number of payloads in each PDCM frame (all buses) - subtract bursts to get echoes :o)
	int			nDeafTimeEchos;											// number of echoes falling in sensor deaf time every CM cycle (1 ms), HiL mode only, resets every cycle; 
	int			nOverDTimeEchoes;										// number of echoes running over the next SFSP burst + deaftime
	int			nEchoesExpired;											// number of expired echoes - see GetCurrentEchoes in vCUS_Main
	uint8_t		echoesLost[_NO_OF_SENSORS_MAX_];						// if echoes recept time > _REC_TIME_MAX_
	uint16_t	receptTime;												// temporary reception time
}tPdcmFlags;
extern tPdcmFlags *PdcmFlags;

typedef struct tPdcmBuffers {
	tWait4TurnBuff		Wait4TurnBuff[_N_USS_PER_BUS_MAX_];			// buffer for data for sensor waiting for turn to transmit - mirrors PDCM frame - 
}tPdcmBuffers;
extern tPdcmBuffers *PdcmBuffers;
#endif

/*	Define in case of debugging / testing */
/* of vCUS<->USP interface / output file */
/* validation Else comment out the #define */
/* for MATLAB validation tool see 30 lines down */

//#define _UsRaw_DEBUG_
	/*requires Sil mode, see Parameterset: isSIL = 1*/
#ifdef _UsRaw_DEBUG_
	struct UsRawEcho_debug {
		int vCUS_Tx;
		int vCUS_TOid;
		uint64_t vCUS_ToF;
		double vCUS_Fr0_position_NP_x;
		double vCUS_Fr0_position_NP_y;
		double vCUS_Fr0_position_NP_z;
		double vCUS_Fr0_position_Sens_x;
		double vCUS_Fr0_position_Sens_y;
		double vCUS_Fr0_position_Sens_z;
		double vCUS_Fr0_position_Sens_Tx_x;
		double vCUS_Fr0_position_Sens_Tx_y;
		double vCUS_Fr0_position_Sens_Tx_z;

		/*runs the matlab validation tool at the end of the sim
		requires MATLAB with parallel computing toolbox, and
		correct setup of environment variables and library dependencies, see wiki or
		stackoverflow.com/questions/8800439/problems-including-matlab-engine-h-for-c-code*/
		/*to run validation add this:*/
		//#define _MATLAB_VALIDATION_ 
	};

	typedef struct tUsRaw {
		UsRawEcho_debug usData_as_debug[US_DRV_MAX_QUEUE_LENGTH_U16];           /*!< Us Data array */
	} tUsRaw_debug;
	extern tUsRaw_debug *UsRaw_debug;
#endif /* #ifndef _UsRaw_DEBUG_ */

#ifndef vCUS_HIL_MODE
extern us_drv::UsDrvDetectionList *UsRaw;
#endif
//extern US_DRV::VFA28::PdcmRecorderModeFrame *localPDCMframe;

#endif	/* #ifndef _USS_Ext_H__ */
