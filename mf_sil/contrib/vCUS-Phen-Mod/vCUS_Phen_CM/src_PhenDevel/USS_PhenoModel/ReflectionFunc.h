#ifndef _USS_REFL_H__
#define _USS_REFL_H__

#include <Vehicle\Sensor_Object.h>
#include "PhenoModel.h"

#define _IN_RANGE_TOLERANCE_ 0.3 //add to sensor range for in-range-custom  (in m)
#define _MOVEMENT_CORRECTION_ERROR_ 0.01 // 1 cm -  miminum accuracy of the Rx position corrected for veh movement between burst and echo
#define ZERODIFF 0.00001
#define _FOV_TOLERANCE_ 1 //degrees 

#define INLIMITS(l0, l1, v) ( ((v < l0 && v > l1) || (v < l1 && v > l0) || abs(v - l1) < ZERODIFF || abs(v - l0) < ZERODIFF))
#define CHECKAZIMUTH(alpha)  alpha = (alpha < 0) ? (alpha + 360) : alpha;   \
							 alpha = (alpha > 360) ? (alpha - 360) : alpha; 
#define NP2USS_DIRECTION(limAngleUSS, USS2NP) (USS2NP - ( (limAngleUSS[0] + limAngleUSS[1]) / 2) ) 
#define VEH_COORD_MOVEINTOF(egoNew, egoNow, vx, vy, tof)\
do { \
    (egoNew)[0] = (egoNow)[0] + vx * tof; \
    (egoNew)[1] = (egoNow)[1] + vy * tof; \
    (egoNew)[2] = (egoNow)[2]; \
} while (0)

void doAnglesConvention(double *, double *);
void dotProduct(double *, double *, double *);
void getAngleQuadrant(double *, double *, double *);
void intx2polygoAnglesSum(double *, int, double(*)[3], bool *, double *, bool *);
void intx2polygoAnglesSum(double *, int, tTrafficObj *, bool *, double *, bool *);
void nearestP_custom(tTrafficObj *, int, int *, double *, double *, double *, bool *, int);
void parametrzLine(double *, double *, int);
void parametrzLine(double *, double *, int, double *);
void planeLineIntx(double *, double *, double *, double *, bool *);
void planeParams(double *, int, tTrafficObj *);
void point2edgeDist(double(*)[3], int, int, double *, double *);
void point2edgeDist(tTrafficObj *, int, int, int, double *, double *, double*);
void vectModulus(double *, double *);
void checkCorner(tTrafficObj *, int *, int, int, double *, bool *, int);
void checkNPinFOV(double *, double *, double *, bool *, double *);
void getSensorFOV(int, double, double *, double *);
double getNpElevation(double *, double *);
void checkTO_inRange(double *, double *, int, bool *);
void shiftPayLdRight(int, int, int);
void getLineProjection(double *, double *, double *, double *);
void shift_mfPlot_stru();
void convertFr0_toFr1(float *, float *);
void convertFr0_toFr1_3D(double *, double *);
void convertFr1toFr0(double *, double *, double *, double *);
void rxCorrection_egoMotion(int, int, int, int);
void SensorRxState_afterToF(tRxState_tof *);
void recalculateNP_postToF(tRxState_tof *);
void check_for_corners(int, double, double *, double *, int, int, int *);
void check_equal_normals(int, int, double *, int *);
void outputToOutline();
#endif	/* #ifndef _USS_REFL_H__ */