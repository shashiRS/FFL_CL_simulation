/*
******************************************************************************
**  CarMaker - Version 7.1
**  Continental Teves AG & CO. oHG
**  - Stefan Hagenmueller -
**
******************************************************************************
*/

#ifndef _USS_Ext_H__
#define _USS_Ext_H__

#define _NO_OF_SENSORS_MAX_ 12

#define _NO_OF_TARGS_MAX_ 10
#define _NO_OF_ECHOES_MAX_ 20
#define _NO_OF_TARGS_STRUCT_MAX_ 720

#define	_Sensor_Deaf_Time_ 2

#include <stdint.h>

int		ContiUSS_TestRun_Start_atBegin	(void);
int		ContiUSS_TestRun_Start_atEnd	(void);
int		ContiUSS_TestRun_Start_Finalize	(void);
int		ContiUSS_MainCalc				(void);
void	ContiUSS_Cleanup				(void);

typedef struct p_ContiUSS {
	double				fire_scheme[_NO_OF_SENSORS_MAX_][_NO_OF_SENSORS_MAX_];	// {Sensor: ContiUSS Parameterset} Values
	tLM2D				*fire_scheme_map;										// {Sensor: ContiUSS Parameterset} 2D linear map
} tp_ContiUSS;

typedef struct ContiUSSParam {
	int					CycleTime;												// {Vehicle Info File Parameter} Cycletime of Vehicle Info File
	int					MaxRefl;												// {Vehicle Info File Parameter} Number of max. reflections
	int					DebugOpt;												// Debugging Option Used if == 1
	int					UseFireSchemeFromFile;									// Fire Scheme derived from file if
	int					CompatibilityV13;										// Measurement compytibility mode for ContiUSS 1.3 and older (ONLY FOR DEBUGING)
	double				kappa;													// Adiabatic index, Default value 1.4
	double				R_s;													// Molar gas constant [J/kg/K], Default Value 287.053 J/kg/K
	double				NFD_near_max;											// Maximum possible distance for NFD near mode
	double				NFD_far_min;											// Minimum possible distance for NFD far mode
	double				NFD_dead_time;											// Time delay between triggering NFD and taking into account
	double				T_Correlator_mean;										// Meantime of the correlator, min. 1, max. 14
	double				T_US_Driver_Calc;										// Time duration of US Driver
	int					nEchoesMax[_NO_OF_SENSORS_MAX_];						// {Vehicle Info File Parameter} Number of max. echoes
	double				Frequency[_NO_OF_SENSORS_MAX_];							// {Vehicle Info File Parameter} Sending frequency
} tContiUSSParam;

typedef struct ContiUSSFlags {
	int					NFD;													// NFD Flag Vodafone Sensor
	int					FireFlag;												// Calculation flag for sensor calc -> Rising Edge
} tContiUSSFlags;

typedef struct ContiUSSEcho {
	double				ContiTimeOF;											// Continental calculated Time of flight [s]
	double				ContiTimeOF_Temp;										// Continental calculated Time of flight [s] incl. add. temperature depend.
	double				ContiLengthOF;											// Continental copied Path length [m]
	double				ContiSPA;												// Continental copied SPA Sound pressure amplitude [Pa]
	double				ContiSPL;												// Continental copied SPL Sound pressure level [dB//1muPa]
	int					ContinRefl;												// Continental copied Number of reflections [-]
	int					ContiTx;												// Continental copied Transmitter ID [-]
	int					ContiBurstTime;											// Time stamp for burst time
	double				*TimeOF;												// {Pointer to IPG-API} Time of flight [s]
	double				*LengthOF;												// {Pointer to IPG-API} Path length [m]
	double				*SPA;													// {Pointer to IPG-API} Sound pressure amplitude [Pa]
	double				*SPL;													// {Pointer to IPG-API} Sound pressure level [dB//1muPa]
	int					*nRefl;													// {Pointer to IPG-API} Number of reflections [-]
	int					*Tx;													// {Pointer to IPG-API} Transmitter ID [-]
	int					MeasTag;												// Time stamp for burst time -> MeasTimeTag
	int					SyncCntCurTag;											// Time stamp for ASIC -> SynCntCurTag
} tContiUSSEcho;

typedef struct ContiUSS {
	int					ExtTrigger;												// Flag for external trigger for calling sensor calc (DVA!)
	int					IntTrigger;												// Flag for internal trigger for calling sensor calc
	double				T_Sensor;												// Temperature of the sensor (DVA!)
	double				TimeStamp;												// Current sensor fire scheme trigger sample point ("0" means, no sensor triggering)
	int					*nEchoes;												// {Pointer to IPG-API} Number of echoes [-]
	int					ContinEchoes;											// Continental copied Number of echoes [-]
	tContiUSSEcho		*ContiUSSEcho;											// Echo Sensor Data
	tContiUSSFlags		ContiUSSFlags;											// Flag Sensor Data
} tContiUSS;

extern tContiUSS		*ContiUSS;

typedef struct ContiUSS_DEBUG {
	double SFSP_ToF_Diff[20];														/* Difference between SFSP and current ToF */
} tContiUSS_DEBUG;

extern tContiUSS_DEBUG	*ContiUSS_DEBUG;

#endif	/* #ifndef _USS_Ext_H__ */