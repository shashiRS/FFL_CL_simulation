#ifndef _USS_MAP_H__
#define _USS_MAP_H__
#define _USS_USRAW_ID_FACTOR_ 1 //add to USS ID for usRaw output

int		intvar_Mapping(int, int, int, int, int);
int		intvar_Mapping(int, int, int, int, int, int); //overloaded for cross-echoes
int		ReflData_Mapping(int, int, bool);
int		SensData_Mapping(int, int, int, int);
int		UsRawEcho_Mapping(uint16_t);

#ifdef vCUS_HIL_MODE
int		PDCMFrame_Mapping(uint16_t);
#endif

/* EvalAmplitudeMapping(Azimuth, Elevation, Distance, Reflectivity) */
double	EvalAmplitudeMapping(double, double, double, double, double, double, double);

#ifdef _UsRaw_DEBUG_
	int		WriteUsRaw(uint16_t);
#endif

#endif

#ifdef vCUS_HIL_MODE
	void write_PdcmBuffer(int, int, int);
	void buffer_writePayLD(int, int, int, int);
	void writeHeader(int, int, int, bool);
	void writePayload(int, int, int, bool);
	void map_Buffer2PDCM(int, int);
	void shiftBuffLeft(int, int, bool);
	void shift_Buff_Right(int, int, int);
	void shiftPayLdRight(int, int, int, int);
	void sortPDCM(int);
	void sortBuff(int, int);
	void logPDCM_frame(int);
	void writeMirrorBuffer(int, int, int, int);
	void sortMirrorBuffer();
#endif