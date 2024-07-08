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
#include <Vehicle\Sensor_Object.h>
#include <Vehicle/Sensor_Inertial.h>
#include "USSMathUtils.h"

#include "ReflectionFunc.h"
#include "PhenoModel.h"
#include <MathUtils.h>

#include "us_drv_def_vfa.h"
#include "vCUS_mfPlot.h"

#include "vcus_dist_lobe.h"

#ifndef max
#define max M_MAX
#endif

#ifndef min
#define min M_MIN
#endif

extern int nTC, nOS;
extern double null_vec_2D[2], null_vec_3D[3];

#ifdef vCUS_HIL_MODE
extern US_DRV::VFA28::PdcmRecorderModeFrame *localPDCMframe;
#endif

/**/
void
parametrzLine(double * params1, double * params2, int j) {
	/* Sensor to  Nearest Point line parametrization, see mathinsight.org/line_parametrization_examples
	nearest point coordinates - x, y z coord
		TrfObj[int_var->Detc_TO_ID[int_var->targ_ID]].NearestPointPos_Fr0[j][0,1,2] - x,y,z
	sensor corrdinates x,y,z;
		SensData[j].Fr0_position_Sens[0,1,2]  - x,y,z*/

		//x parameters:x = params1[0] + params2[0] * t, for -inf <t< inf
	params1[0] = SensData[j].Fr0_position_Sens[0];
	params2[0] = TrfObj[int_var->Detc_TO_ID[int_var->targ_ID]].NearestPointPos_Fr0[j][0] - params1[0];
	//y parameters
	params1[1] = SensData[j].Fr0_position_Sens[1];
	params2[1] = TrfObj[int_var->Detc_TO_ID[int_var->targ_ID]].NearestPointPos_Fr0[j][1] - params1[1];
	//z parameters		
	params1[2] = SensData[j].Fr0_position_Sens[2];
	params2[2] = TrfObj[int_var->Detc_TO_ID[int_var->targ_ID]].NearestPointPos_Fr0[j][2] - params1[2];
}


void
parametrzLine(double * params1, double * params2, int j, double *NPcoord) {
	//************overloaded line parametrization func
	/* Sensor to  Nearest Point line parametrization, see mathinsight.org/line_parametrization_examples
	nearest point coordinates - x, y z coord
		TrfObj[int_var->Detc_TO_ID[int_var->targ_ID]].NearestPointPos_Fr0[j][0,1,2] - x,y,z
	sensor corrdinates x,y,z;
		SensData[j].Fr0_position_Sens[0,1,2]  - x,y,z*/

	//x parameters:x = params1[0] + params2[0] * t, for -inf <t< inf
	params1[0] = SensData[j].Fr0_position_Sens[0];
	params2[0] = NPcoord[0] - params1[0];
	//y parameters
	params1[1] = SensData[j].Fr0_position_Sens[1];
	params2[1] = NPcoord[1] - params1[1];
	//z parameters		
	params1[2] = SensData[j].Fr0_position_Sens[2];
	params2[2] = NPcoord[2] - params1[2];
}


void
planeParams(double *params, int k, tTrafficObj *Obj) {
	/*TO plane parametrization; see math.stackexchange.com/questions/2686606*/
	double vec1[3], vec2[3], summsq, doubleCheck = 0;
	int i, n = 3;
	/*vector Edge[2][] - Edge[1][]*/
	vec1[0] = Obj->Envelope.Areas_0[k].Edge[2][0] - Obj->Envelope.Areas_0[k].Edge[1][0];
	vec1[1] = Obj->Envelope.Areas_0[k].Edge[2][1] - Obj->Envelope.Areas_0[k].Edge[1][1];
	vec1[2] = Obj->Envelope.Areas_0[k].Edge[2][2] - Obj->Envelope.Areas_0[k].Edge[1][2];
	/*vector Edge[3][] - Edge[1][]*/
	vec2[0] = Obj->Envelope.Areas_0[k].Edge[3][0] - Obj->Envelope.Areas_0[k].Edge[1][0];
	vec2[1] = Obj->Envelope.Areas_0[k].Edge[3][1] - Obj->Envelope.Areas_0[k].Edge[1][1];
	vec2[2] = Obj->Envelope.Areas_0[k].Edge[3][2] - Obj->Envelope.Areas_0[k].Edge[1][2];

	/*vector normal to plane: params[1-3],
	plane eq: params[0] * x + params[1] * y + params[2] * z + params[3] = 0 */
	params[0] = (vec1[1] * vec2[2]) - (vec1[2] * vec2[1]);
	params[1] = -((vec1[0] * vec2[2]) - (vec1[2] * vec2[0]));
	params[2] = (vec1[0] * vec2[1]) - (vec1[1] * vec2[0]);

	//normalize a*a + b*b +c*c = 1
	summsq = sqrt(pow(params[0], 2) + pow(params[1], 2) + pow(params[2], 2));
	for (i = 0; i < n; i++) {
		params[i] = params[i] / summsq;
	}

	params[3] = -(params[0] * Obj->Envelope.Areas_0[k].Edge[1][0] +
		params[1] * Obj->Envelope.Areas_0[k].Edge[1][1] +
		params[2] * Obj->Envelope.Areas_0[k].Edge[1][2]);

	//check that all four corners are in plane, should be zero
	//tbd> CarMaker Warning in case of doubleCheck != 0
	for (int m = 0; m < 4; m++)
	doubleCheck = doubleCheck + (params[0] * Obj->Envelope.Areas_0[k].Edge[m][0] +
		params[1] * Obj->Envelope.Areas_0[k].Edge[m][1] +
		params[2] * Obj->Envelope.Areas_0[k].Edge[m][2] +
		params[3]);
}

void
planeLineIntx(double *lnPara1, double *lnPara2, double *planePara, double *intxP, bool *noIntx) {
	/*calculates intersection point between line and plane
		(defined by lnPara1&2 and planePara)
		see math.stackexchange.com/questions/100439/*/

	double t, tnumerator, tdenominator, checkIntx;

	tnumerator = -(planePara[3] + planePara[0] * lnPara1[0] + planePara[1] * lnPara1[1] + planePara[2] * lnPara1[2]);
	tdenominator = planePara[0] * lnPara2[0] + planePara[1] * lnPara2[1] + planePara[2] * lnPara2[2];
	if (abs(tdenominator) <= ZERODIFF)
		*noIntx = true;
	else {
		t = tnumerator / tdenominator;
		intxP[0] = lnPara1[0] + lnPara2[0] * t;
		intxP[1] = lnPara1[1] + lnPara2[1] * t;
		intxP[2] = lnPara1[2] + lnPara2[2] * t;
		checkIntx = planePara[0] * intxP[0] + planePara[1] * intxP[1] + planePara[2] * intxP[2] + planePara[3];
		if (abs(checkIntx) <= ZERODIFF)
			*noIntx = false;
	}
}

void
vectModulus(double *proj, double *vecMod) {
	/*this could be a macro?; calculates modulus of a vector: Pythagoras'*/
	*vecMod = sqrt(pow(proj[0], 2) + pow(proj[1], 2) + pow(proj[2], 2));
}

void
intx2polygoAnglesSum(double *itx, int k, tTrafficObj *Obj, bool *isOnTO, double *pPrecis, bool *nearMiss) {
	/*calculates the sum of angles formed by the intersection between sens-NP direction and
	TO surface plane (itx[3]) with the points of the TO surface polygon;
	if the sum of angles equals 2*PI, intersection is on TO surface
	addapted from www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
	*pPrecis: angle precision -for TO shorter than z of sensor should be larger, about 0.1,
			due to small error in NP z position (z_NP > z_TO)*/

			//n- number of polygon corners, m- number of coordinates (x, y, z)
	int i, n = 4, j, m = 3;
	double proj1[3], proj2[3], modulu1, modulu2;
	double allAngleSum = 0, cosAngle;
	*nearMiss = false;
	//Obj->Envelope.Areas_0[int_var->Detc_TO_Surface_ID[int_var->TO_ID[l]]].Edge[0-3][0-2] -- polygon corners coordinates
	//itx[0-2] -- intersection point coordinates x-z

	*isOnTO = false;
	for (i = 0; i < n; i++) {
		//do all coordinates j for each pair of poygon corners i
		for (j = 0; j < m; j++) {
			proj1[j] = Obj->Envelope.Areas_0[k].Edge[i][j] - itx[j];
			proj2[j] = Obj->Envelope.Areas_0[k].Edge[(i + 1) % n][j] - itx[j];
		}
		vectModulus(proj1, &modulu1);
		vectModulus(proj2, &modulu2);

		if (modulu1 * modulu2 <= ZERODIFF) {
			//intx on corner, consider on TO surface
			*isOnTO = true;
			break;
		}
		else
			cosAngle = (proj1[0] * proj2[0] + proj1[1] * proj2[1] + proj1[2] * proj2[2]) / (modulu1 * modulu2);

		/*cosAngle can be i.e., -1.0000002 => acos = nan*/
		if (cosAngle < -1) cosAngle = -1;
		else if (cosAngle > 1) cosAngle = 1; 

		allAngleSum += acos(cosAngle);
	}

	if (abs(allAngleSum - 2 * M_PI) <= ZERODIFF) {
		*isOnTO = true;
		*nearMiss = false;
	}
	else if (abs(allAngleSum - 2 * M_PI) <= *pPrecis) {
		*isOnTO = true;
		*nearMiss = true;
	}

}

void
intx2polygoAnglesSum(double *itx, int k, double(*edges)[3], bool *isOnTO, double *pPrecis, bool *nearMiss) {
	/*overloaded function: argument is pointer to 4x3 array edges instead of tTrafficObj *Obj */
	/*calculates the sum of angles formed by the intersection between sens-NP direction and
	TO surface plane (itx[3]) with the points of the TO surface polygon;
	if the sum of angles equals 2*PI, intersection is on TO surface
	addapted from www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
	*pPrecis: angle precision -for TO shorter than z of sensor should be larger, about 0.1,
			due to small error in NP z position (z_NP > z_TO)*/

			//n- number of polygon corners, m- number of coordinates (x, y, z)
	int i, n = 4, j, m = 3;
	double proj1[3], proj2[3], modulu1, modulu2;
	double allAngleSum = 0, cosAngle;
	*nearMiss = false;
	//Obj->Envelope.Areas_0[int_var->Detc_TO_Surface_ID[int_var->TO_ID[l]]].Edge[0-3][0-2] -- polygon corners coordinates
	//itx[0-2] -- intersection point coordinates x-z

	*isOnTO = false;
	for (i = 0; i < n; i++) {
		//do all coordinates j for each pair of poygon corners i
		for (j = 0; j < m; j++) {
			proj1[j] = edges[i][j] - itx[j];
			proj2[j] = edges[(i + 1) % n][j] - itx[j];
		}
		vectModulus(proj1, &modulu1);
		vectModulus(proj2, &modulu2);

		if (modulu1 * modulu2 <= ZERODIFF) {
			//intx on corner, consider on TO surface
			*isOnTO = true;
			break;
		}
		else
			cosAngle = (proj1[0] * proj2[0] + proj1[1] * proj2[1] + proj1[2] * proj2[2]) / (modulu1 * modulu2);
		
		/*cosAngle can be i.e., -1.0000002 => acos = nan*/
		if (cosAngle < -1) cosAngle = -1;
		else if (cosAngle > 1) cosAngle = 1;

		allAngleSum += acos(cosAngle);
	}

//	if (abs(allAngleSum - 2 * M_PI) <= ZERODIFF) {
//		*isOnTO = true;
//		*nearMiss = false;
//	}
//else if
	if (abs(allAngleSum - 2 * M_PI) <= *pPrecis) {
		*isOnTO = true;
		*nearMiss = false;// true;
	}

}
void
dotProduct(double *dotProd, double *oneVec, double *twoVec) {
	/*calculates the dot product of 2 vectors in 3D*/
	*dotProd = 0;
	for (int i = 0; i < 3; i++)
		*dotProd = *dotProd + oneVec[i] * twoVec[i];
}

void
point2edgeDist(tTrafficObj *Obj, int k, int j, int virtualUSS, double *dist, double *pointCoord, double *vRx_coord) {
	/*calculates the distance from sensor to closest TO surface edge*/
	double	distances[4] = { 0 };
	double	vecsCoord[3] = { 0 };
	double	dotProd = 0;
	double	dotProdSelf = 0;
	int		onSegment;
	double	sens2NpVec[3] = { 0 };
	double	sensorVec[3], edgeVec[3], sensProj[3];
	int		targetCorner;
	double	distLine;
	*dist = HUGE_VAL;
	double sensorCoord[3];

	if (virtualUSS == 0)
		VEC_Assign(sensorCoord, SensData[j].Fr0_position_Sens);
	else if (virtualUSS == 1)
		VEC_Assign(sensorCoord, SensData[j].Fr0_position_vSens);
	else if (virtualUSS == 2)
		VEC_Assign(sensorCoord, vRx_coord);
	else
		LogErrF(EC_Init, "vCUS: Virtual sensor flag error");

	/*calculate distances: sensor to TO surface corners */
	for (int i = 0; i < 4; i++) {
		/*for each of 4 corners go through all coordinates: x y z*/
		for (int l = 0; l < 3; l++)
			vecsCoord[l] = sensorCoord[l] - Obj->Envelope.Areas_0[k].Edge[i][l];

		vectModulus(vecsCoord, &distances[i]);
	}

	/* check all TO surface edges find shortest distance to TO surface edge*/
	for (int i = 0; i < 4; i++) {
		int		corner1 = i;
		int		corner2 = (i + 1) % 4;

		/*see codereview.stackexchange.com/questions/162447/shortest-distance-from-point-to-line-segment-in-3d*/
		for (int l = 0; l < 3; l++) {
			sensorVec[l] = sensorCoord[l] - Obj->Envelope.Areas_0[k].Edge[corner1][l];
			edgeVec[l] = Obj->Envelope.Areas_0[k].Edge[corner2][l] - Obj->Envelope.Areas_0[k].Edge[corner1][l];
		}

		dotProduct(&dotProd, sensorVec, edgeVec);
		dotProduct(&dotProdSelf, edgeVec, edgeVec);
		/*sensor projection on the line defined by current corners*/
		onSegment = 0;
		for (int l = 0; l < 3; l++) {
			sensProj[l] = Obj->Envelope.Areas_0[k].Edge[corner1][l] + dotProd / dotProdSelf * edgeVec[l];
			/*check if projection is on segment*/
			if (INLIMITS(Obj->Envelope.Areas_0[k].Edge[corner2][l], Obj->Envelope.Areas_0[k].Edge[corner1][l], sensProj[l]))
				onSegment++;
		}

		for (int l = 0; l < 3; l++) {
			sens2NpVec[l] = sensProj[l] - sensorCoord[l];
		}
		vectModulus(sens2NpVec, &distLine);

		/*if is outside, take NP as the closest corner */
		if (onSegment != 3) {
			if (distances[corner1] < distances[corner2]) {
				targetCorner = corner1;
			}
			else {
				targetCorner = corner2;
			}

			VEC_Assign(sensProj, Obj->Envelope.Areas_0[k].Edge[targetCorner]);
			distLine = distances[targetCorner];
		}

		/*get smallest distance to edge and NP*/
		if (distLine < *dist) {
			*dist = distLine;
			VEC_Assign(pointCoord, sensProj);
		}

	}
}

void
point2edgeDist(double(*edges)[3], int k, int j, double *dist, double *pointCoord) {
	/*overloaded func argument is pointer to 4x3 array instead of tTrafficObj *Obj*/
	/*calculates the distance from sensor to closest TO surface edge*/
	double	distances[4] = { 0 };
	double	vecsCoord[3] = { 0 };
	double	dotProd = 0;
	double	dotProdSelf = 0;
	int		onSegment;
	double	sens2NpVec[3] = { 0 };
	double	sensorVec[3], edgeVec[3], sensProj[3];
	int		targetCorner;
	double	distLine;
	*dist = HUGE_VAL;

	/*calculate distances: sensor to TO surface corners */
	for (int i = 0; i < 4; i++) {
		/*for each of 4 corners go through all coordinates: x y z*/
		for (int l = 0; l < 3; l++)
			vecsCoord[l] = SensData[j].Fr0_position_Sens[l] - edges[i][l];

		vectModulus(vecsCoord, &distances[i]);
	}

	/* check all TO surface edges find shortest distance to TO surface edge*/
	for (int i = 0; i < 4; i++) {
		double testArr[3] = { edges[i][0], edges[i][1], edges[i][2] };
		int		corner1 = i;
		int		corner2 = (i + 1) % 4;

		/*see codereview.stackexchange.com/questions/162447/shortest-distance-from-point-to-line-segment-in-3d*/
		for (int l = 0; l < 3; l++) {
			sensorVec[l] = SensData[j].Fr0_position_Sens[l] - edges[corner1][l];
			edgeVec[l] = edges[corner2][l] - edges[corner1][l];
		}

		dotProduct(&dotProd, sensorVec, edgeVec);
		dotProduct(&dotProdSelf, edgeVec, edgeVec);
		/*sensor projection on the line defined by current corners*/
		onSegment = 0;
		for (int l = 0; l < 3; l++) {
			sensProj[l] = edges[corner1][l] + dotProd / dotProdSelf * edgeVec[l];
			/*check if projection is on segment*/
			if (INLIMITS(edges[corner1][l], edges[corner2][l], sensProj[l]))
				onSegment++;
		}

		for (int l = 0; l < 3; l++) {
			sens2NpVec[l] = sensProj[l] - SensData[j].Fr0_position_Sens[l];
		}
		vectModulus(sens2NpVec, &distLine);

		/*if is outside, take NP as the closest corner */
		if (onSegment != 3) {
			if (distances[corner1] < distances[corner2]) {
				targetCorner = corner1;
			}
			else {
				targetCorner = corner2;
			}

			VEC_Assign(sensProj, edges[targetCorner]);
			distLine = distances[targetCorner];
		}

		/*get smallest distance to edge and NP*/
		if (distLine < *dist) {
			*dist = distLine;
			VEC_Assign(pointCoord, sensProj);
		}

	}
}

void
getAngleQuadrant(double *pXVector, double *pYVector, double *pAngleOut) {
	/*calculates angle of vector based on tangent and quadrant orientation*/

		/* avoid division by zero*/
	if (*pXVector == 0) *pXVector = ZERODIFF;
	*pAngleOut = rad2deg * atan(*pYVector / *pXVector);

	/*check oquadrant
	(((*pYVector >= 0) && (*pXVector < 0)) ||   //Q2
		((*pYVector <= 0) && (*pXVector < 0))   ) //Q3*/
	if (*pXVector < 0)  //Q2 or Q3
		*pAngleOut += 180;
	else if ((*pYVector < 0) && (*pXVector > 0)) 	//Q4: angle < 0
		*pAngleOut += 360;
}
void
doAnglesConvention(double *arrSensorLimit, double *pAngleOut) {
	/*changes the sign of an angle depending on size and sensor FOV limits*/
	/*i.e., if sensor ortientation is -120, min FOV angle can be -210, in Q2, and TO also in Q2 */
	if (((arrSensorLimit[0] < 0 && arrSensorLimit[1] <= 0)
		&& ((*pAngleOut < 360 && *pAngleOut >= 90) || *pAngleOut == 360)) ||
		((arrSensorLimit[0] <= 0 && arrSensorLimit[1] > 0)
			&& ((*pAngleOut < 360 && *pAngleOut >= 180) || *pAngleOut == 360)))
		*pAngleOut -= 360;
}

void
getSensorFOV(int sensID, double vehYaw, double *fovAngleLims, double *ussDirection) {
	/*calculates fov angle limits for sensor sensID
	based on vehicle Yaw*/

	*ussDirection = vehYaw + SensParam[sensID].orientation;
	/*this happens when yaw is e.g. -170*/
	if (*ussDirection >= -360 && *ussDirection < -180) *ussDirection += 360;
	/*sensor angle +/- sensor azimuth range*/
	fovAngleLims[1] = *ussDirection + (SensParam[sensID].Azimuth / 2);
	fovAngleLims[0] = *ussDirection - (SensParam[sensID].Azimuth / 2);
	/*does > 360 happen ?*/
	if (fovAngleLims[1] > 360) fovAngleLims[1] -= 360;
	if (fovAngleLims[0] > 360) fovAngleLims[0] -= 360;

	/*check if min max FOV angles are on either side of the x axis*/
	if (((fovAngleLims[1] >= 0 && fovAngleLims[1] < 180) && (fovAngleLims[0] < 360 && fovAngleLims[0] >= 180)) ||
		(fovAngleLims[0] == 360))
		fovAngleLims[0] = fovAngleLims[0] - 360;
}

void
checkNPinFOV(double *NPCoord, double *sensorCoord, double *fovLims, bool *inFOV, double *sens2NPDirection){

/*calculate sensor-NP orientation in FR0, azimuth: 2D should be ok*/
double	xNpSens, yNpSens;
//double sens2NP_orient;
/*here use the NP position for FOV checking, i.e., the position of the emiting real sensor*/
xNpSens = NPCoord[0] - sensorCoord[0];
yNpSens = NPCoord[1] - sensorCoord[1];
getAngleQuadrant(&xNpSens, &yNpSens, sens2NPDirection);
doAnglesConvention(fovLims, sens2NPDirection);

/*if one of the lower limit angle is close to 0 and np orientation is close to 360*/
if (INLIMITS(0, _FOV_TOLERANCE_, fovLims[0]) && INLIMITS(360 - _FOV_TOLERANCE_, 360, *sens2NPDirection))
*sens2NPDirection -= 360;

/*if top limit is <= 360 and np orientation is >= 0*/
if (INLIMITS(360 - _FOV_TOLERANCE_, 360, fovLims[1]) && INLIMITS(0, _FOV_TOLERANCE_, *sens2NPDirection))
*sens2NPDirection += 360;
*inFOV = ((*sens2NPDirection > fovLims[0] && fovLims[1] > *sens2NPDirection) ||
(abs(*sens2NPDirection - fovLims[0]) < _FOV_TOLERANCE_) || (abs(*sens2NPDirection - fovLims[1]) < _FOV_TOLERANCE_));

}

double 
getNpElevation(double *npCoord, double *ussCoord) {
/*calculates and returns the elevation between sensor coordinates and nearest point*/
	double npAtZsensor[3];
	double elevation, zDiff, horizDist;

	/*get NP projection at uss height*/
	VEC_Assign(npAtZsensor, npCoord);
	npAtZsensor[2]	= ussCoord[2];
	/*distances for elevation*/
	horizDist		= VEC_Abs(ussCoord, npAtZsensor);
	zDiff			= npCoord[2] - ussCoord[2];//relative to USS
	/*elevation from dist, in degrees*/
	elevation		= rad2deg * atan(zDiff / horizDist);

	return elevation;
}

void
nearestP_custom(tTrafficObj *Obj, int j, int *surfNoOut, double *coordOut_NP_Fr0,
											double *pDist, double *direction, bool *notInFOV, int EchoType) {
	/*calculates the Nearest point for the current TO
		goes through all TO surfaces, determines the shortest distance from each and
		the coordinates where the line falls on surf (NP)
		and keeps the surface No and NP with the shortest distance of all surfaces*/

	/*set sensor position based on EchoType*/
	/* for virtual sensor case*/
	double	limAngleSensLeft[2],	limAngleSensRight[2];
	double	sensorCoordLeft[3],		sensorCoordRight[3];
	double	leftUssDirection, rightUssDirection;
	double	maxLeftAzm, minLeftAzm, maxRightAzm, minRightAzm;
	double	limAzmVirtUss[2], distUssLeft, distUssRight;
	
	/*for real sensorcase*/
	double	sensorCoordNP[3], sensorCoordFOV[3], limAngleSens[2];
	double	ussDirection;

	int		rightSensId = (j + 1) % nOS; 

	 if (EchoType == 1)
	{
		/*this is the virtual sensor case; to check if virtual NP is in FOV of real sensors left and right of 
		virtual sensor (counter-clockwise and clockwise, respectively)
		check FOV from the perspective of real sensors*/
		VEC_Assign(sensorCoordLeft, SensData[j].Fr0_position_Sens);  //real sensor left
		VEC_Assign(sensorCoordRight, SensData[rightSensId].Fr0_position_Sens); //real sensor right
		VEC_Assign(sensorCoordNP, SensData[j].Fr0_position_vSens); //virtual
	}
	else if (EchoType == 0){
		/*this is the real sensor case*/
		VEC_Assign(sensorCoordFOV, SensData[j].Fr0_position_Sens); //real sensor
		VEC_Assign(sensorCoordNP, SensData[j].Fr0_position_Sens); //real sensor
	}
	
	/*for tile FoV*/
	bool	isInTile;
	double	centerTile[3];
	double  tile2UssVec[2], tile2UssDirection, tileFOV[2];
	/**/
	
	double	tempCoord [3] = { 0 }, tempDist;
	double	tempParams[4] = { 0 };
	/*parameters of line passing through sensor and normal to plane
	see tutorial.math.lamar.edu/classes/calciii/gradientvectortangentplane.aspx
		math.stackexchange.com/questions/646420/find-the-vector-equation-of-a-line-perpendicular-to-the-plane*/
	double	linePara1[3];
	VEC_Assign(linePara1, sensorCoordNP);
	double	linePara2[3]	= { 0 };
	bool	noLPIntx		= true;//there is no intersection between line and plane (no NP)
	bool    isInPolig		= false;
	bool	nearPoly, intxState, nearNormal;
	double	err = 0.01;//ZERODIFF too strict; // 0.01;
	/*these are all in FR0, min/maxAngleSens: FOV azimuth range of sensor*/
	double	minInFovDist = HUGE_VAL;
	double	sens2NP_orient, vehYaw;
	double	xVecNormal[2];
	double	normTileAngle;
	*notInFOV = true;
	VEC_Assign(coordOut_NP_Fr0, null_vec_3D);
	*pDist = HUGE_VAL;

	/*calculate min max angles of sensor FOV*/
	vehYaw = rad2deg * Vehicle.Yaw;
	/* rad2deg * Vehicle.Yaw can be e.g. -224 , correct this*/
	if (vehYaw >= -360 && vehYaw < -180) vehYaw += 360;
	
	/*get fov limits for the correct sensor based on virtual sensor flag*/
	if (EchoType == 1) {
		/*if virtual get fov for both left and right real sensors
		virtual NP ok only if vNP is in FOV of both*/
		getSensorFOV(j, vehYaw, limAngleSensLeft, &leftUssDirection);
		getSensorFOV(rightSensId, vehYaw, limAngleSensRight, &rightUssDirection);
		
		/*calculate virtual sensor fov - to implement as elipse ?!*/
		
		/*get max & min of left- this could be skipped? limAngleSens***[0] should be min*/
		maxLeftAzm = (limAngleSensLeft[1] > leftUssDirection) ? limAngleSensLeft[1] : limAngleSensLeft[0];
		minLeftAzm = (limAngleSensLeft[1] < leftUssDirection) ? limAngleSensLeft[1] : limAngleSensLeft[0];
		/*get max & min of right*/
		maxRightAzm = (limAngleSensRight[1] > rightUssDirection) ? limAngleSensRight[1] : limAngleSensRight[0];
		minRightAzm = (limAngleSensRight[1] > rightUssDirection) ? limAngleSensRight[1] : limAngleSensRight[0];
		/*reset the sign of angles: make all 0 to 360*/
		CHECKAZIMUTH(maxLeftAzm);
		CHECKAZIMUTH(minLeftAzm);
		CHECKAZIMUTH(maxRightAzm);
		CHECKAZIMUTH(minRightAzm);
		/*take max of min and min of max*/
		limAzmVirtUss[0] = max(minLeftAzm, minRightAzm);
		limAzmVirtUss[1] = min(maxLeftAzm, maxRightAzm);

		/*calculate virtual sensor direction as (dirLeft+dirRight)/2*/
			/*check uss left right direction*/
		CHECKAZIMUTH(leftUssDirection);
		CHECKAZIMUTH(rightUssDirection);
		SensData[j].Fr0_vUssDirection = (leftUssDirection + rightUssDirection) / 2;

	}
	else if (EchoType == 0){
		getSensorFOV(j, vehYaw, limAngleSens, &ussDirection);
	}

	for (int k = 0; k < Obj->Cfg.Envelope.nArea; k++) {

		//find tile plane parameters; tempParams[0-2] are xyz coordinates of normal vec to plane
		planeParams(tempParams, k, Obj);

		/*skip horizontal tiles, top and bottom tiles: z coord of normal vector to plane  == +/-1
		but varries depending on roll and pitch of TO*/
		if (abs(abs(tempParams[2]) - 1) < 0.0001)
			continue;

		/*take only tiles for which USS is in tile FOV*/
		/*============================================*/
		env_vector->NormalDir[k] = rad2deg * acos(VEC_Scalar2D(env_vector->NormalDirDefault, TrfObj[Obj->Cfg.Id].normals[k])
			/ (VEC_Norm2D(env_vector->NormalDirDefault) * VEC_Norm2D(TrfObj[Obj->Cfg.Id].normals[k])));
		if (TrfObj[Obj->Cfg.Id].normals[k][1] < 0)
			env_vector->NormalDir[k] = env_vector->NormalDir[k] * -1;
		tileFOV[0] = env_vector->NormalDir[k] - 90;
		tileFOV[1] = env_vector->NormalDir[k] + 90;
		/*tile center: middle of the tile diagonal*/
		for (int i = 0; i < 3; i++)
			centerTile[i] = (Obj->Envelope.Areas_0[k].Edge[3][i] + Obj->Envelope.Areas_0[k].Edge[1][i]) / 2;
		/*direction from center of tile to USS*/
		tile2UssVec[0] = sensorCoordNP[0] - centerTile[0];
		tile2UssVec[1] = sensorCoordNP[1] - centerTile[1];
		getAngleQuadrant(&tile2UssVec[0], &tile2UssVec[1], &tile2UssDirection);
		doAnglesConvention(tileFOV, &tile2UssDirection);
		isInTile = ((tile2UssDirection > tileFOV[0] && tile2UssDirection <= env_vector->NormalDir[k]) ||
			(tile2UssDirection > env_vector->NormalDir[k] && tile2UssDirection < tileFOV[1]));

		if (!isInTile) {
			//Log("Skipping tile %d\n", k);
			continue;
		}
		/*============================================*/

		intxState = false;
		nearPoly = false;
		nearNormal = false;

		/*distance from P(xp,yp,zp) to plane ax+by+cz+d=0 is (a*xp+b*yp+c*zp+d)/sqrt(a*a+b*b+c*c)
		see mathinsight.org/distance_point_plane*/
		tempDist = (tempParams[0] * sensorCoordNP[0] +
			tempParams[1] * sensorCoordNP[1] +
			tempParams[2] * sensorCoordNP[2] + tempParams[3]) /
			sqrt(pow(tempParams[0], 2) + pow(tempParams[1], 2) + pow(tempParams[2], 2));

		VEC_Assign(linePara2, tempParams);
		/*find where normal to plane passing through Sensor meets the plane*/
		planeLineIntx(linePara1, linePara2, tempParams, tempCoord, &noLPIntx);
		
		/****/
		/*check if tile intersects edge of sensor FOV in 2D - assumes tiles are vertical rectangles
		- takes projection of tile in 2D, i.e. bottom edge: Edge[0][0,1] - Edge[1][0,1]
		- if sensor to tile corner angles are in FOV skip the intersection thing*/
		double	angleSens2Corners[2];
		bool 	offFOV[2] = { true, true }, fovXto[2] = { false, false };
		bool    fovXtop[2] = { false, false }, fovXbotm[2] = { false, false };
		double  xDiff, yDiff;
		double  newTOEdge[4][3];
		double  intxPntAngle[2];

		if (EchoType == 0) {
			/***only split tiles if sensor is real not virtual*/

			/*direction of normal to tile in 2D (azimuth)*/
			for (int p = 0; p < 2; p++) {
				xVecNormal[p] = tempCoord[p] - sensorCoordNP[p];
			}
			getAngleQuadrant(&xVecNormal[0], &xVecNormal[1], &normTileAngle);
			doAnglesConvention(limAngleSens, &normTileAngle);

			/*check if tile edges are outside sensor's FOV*/
			for (int m = 0; m < 2; m++) {
				xDiff = Obj->Envelope.Areas_0[k].Edge[m][0] - sensorCoordFOV[0];
				yDiff = Obj->Envelope.Areas_0[k].Edge[m][1] - sensorCoordFOV[1];
				getAngleQuadrant(&xDiff, &yDiff, &angleSens2Corners[m]);
				doAnglesConvention(limAngleSens, &angleSens2Corners[m]);

				if ((angleSens2Corners[m] > limAngleSens[0] && angleSens2Corners[m] < limAngleSens[1]) ||
					abs(angleSens2Corners[m] - limAngleSens[1]) < _FOV_TOLERANCE_ || abs(angleSens2Corners[m] - limAngleSens[0]) < _FOV_TOLERANCE_)
					offFOV[m] = false;

				/*use this m loop: if fov edge angle is almost the same direction as the normal to tile,
				skip tile splitting, take the normal to tile from sensor as NP*/
				if (abs(limAngleSens[m] - normTileAngle) <= _FOV_TOLERANCE_)
					nearNormal = true;
			}
		}
		/*offFOV == false : the corner(s) of the current TO tile (k) is(are) inside FOV*/

		/*if both are false, the tile is inside FOV completely (in 2D) so skip
		if at least one is true -tile (partly) outside of FOV- look for intersection of FOV edge and tile
		***only split tiles if sensor is real not virtual*/
		if (!nearNormal && EchoType == 0 &&
			((offFOV[0] || offFOV[1]) && (!noLPIntx))) {
			double	slopeEdge, bEdgeBotm, bEdgeTop;
			double	slopeFov[2], bFov[2], intx[2][3];
			double	intxPnt[4][3];
			double  intxPntVec[2][2];
			int		indxTop[2] = { 3, 2 };

			/* The following illustration shows orientation of one face with own x-y reference
			and the position of the intersection points intxPnt[0,1,2,3][3]

			3-----------2
			|           |
			|           |
			|           |
			|      x <--o Pos
			|           |
			|           v y
			|           |
			0-----------1
			*/

			/*calculate intersections for both the bottom and top edges of the tile;
			for tilted TOs (roll || ptich != 0) top tile has different x,y coordinates. Calculating only bottom makes the tile 
			vertical and intx2polygoAnglesSum places the NP projection outside the re-built tile (because it was calculated on the whole tilted tile)
			*/
			
			//equation of bottom edge of current TO tile; y = slope*x + b 
			slopeEdge	= (Obj->Envelope.Areas_0[k].Edge[1][1] - Obj->Envelope.Areas_0[k].Edge[0][1]) /
				(Obj->Envelope.Areas_0[k].Edge[1][0] - Obj->Envelope.Areas_0[k].Edge[0][0]);
			bEdgeBotm	= Obj->Envelope.Areas_0[k].Edge[1][1] - slopeEdge * Obj->Envelope.Areas_0[k].Edge[1][0];
			//equation of the top edge of tile: should be parallel, same slope differet b
			bEdgeTop	= Obj->Envelope.Areas_0[k].Edge[2][1] - slopeEdge * Obj->Envelope.Areas_0[k].Edge[2][0];

			for (int m = 0; m < 2; m++) {
				fovXto[m] = false;
				//equation of FOV line-border max & min  y = slopeFov*x + b;
				slopeFov[m] = tan(deg2rad * limAngleSens[m]);
				bFov[m] = sensorCoordFOV[1] - slopeFov[m] * sensorCoordFOV[0];

				/*check intersection: if FOV edge is not parallel to TO tile*/
				if ((Obj->Envelope.Areas_0[k].Edge[1][0] == Obj->Envelope.Areas_0[k].Edge[0][0]) && //vertical tile in 2D -undefined slope
					(abs(slopeFov[m] - slopeEdge) > ZERODIFF)) {
					/*intx FOV limit vs. bottom tile edge*/
					intxPnt[m][0] = Obj->Envelope.Areas_0[k].Edge[1][0]; // x of intersection is x of tile
					intxPnt[m][1] = bFov[m] + slopeFov[m] * intxPnt[m][0];
					intxPnt[m][2] = Obj->Envelope.Areas_0[k].Edge[1][2]; // same z
					/*intx FOV limit vs. top tile edge*/
					intxPnt[indxTop[m]][0] = Obj->Envelope.Areas_0[k].Edge[2][0]; // x of intersection is x of tile
					intxPnt[indxTop[m]][1] = bFov[m] + slopeFov[m] * intxPnt[indxTop[m]][0];
					intxPnt[indxTop[m]][2] = Obj->Envelope.Areas_0[k].Edge[2][2]; // same z
				}
				else if (abs(slopeFov[m] - slopeEdge) > ZERODIFF) {
					/*intx FOV limit vs. bottom tile edge*/
					intxPnt[m][0] = (bFov[m] - bEdgeBotm) / (slopeEdge - slopeFov[m]);
					intxPnt[m][1] = (bFov[m] * slopeEdge - bEdgeBotm * slopeFov[m]) / (slopeEdge - slopeFov[m]);
					intxPnt[m][2] = Obj->Envelope.Areas_0[k].Edge[1][2]; // same z
					/*intx FOV limit vs. top tile edge*/
					intxPnt[indxTop[m]][0] = (bFov[m] - bEdgeTop) / (slopeEdge - slopeFov[m]);
					intxPnt[indxTop[m]][1] = (bFov[m] * slopeEdge - bEdgeTop * slopeFov[m]) / (slopeEdge - slopeFov[m]);
					intxPnt[indxTop[m]][2] = Obj->Envelope.Areas_0[k].Edge[2][2]; // same z
				}

				/*check if intxPnt falls inside the bottom TO tile edge, checks on x&y axes */
				intxPntVec[m][0] = intxPnt[m][0] - sensorCoordFOV[0];
				intxPntVec[m][1] = intxPnt[m][1] - sensorCoordFOV[1];
				getAngleQuadrant(&intxPntVec[m][0], &intxPntVec[m][1], &intxPntAngle[m]);
				doAnglesConvention(limAngleSens, &intxPntAngle[m]);

				if ((INLIMITS(Obj->Envelope.Areas_0[k].Edge[0][0], Obj->Envelope.Areas_0[k].Edge[1][0], intxPnt[m][0]) &&  // on x
					INLIMITS(Obj->Envelope.Areas_0[k].Edge[0][1], Obj->Envelope.Areas_0[k].Edge[1][1], intxPnt[m][1])) // on y
					&& abs(limAngleSens[m] - intxPntAngle[m]) < 1) //intersection point is in the direction of the sensor (not 180)
					fovXbotm[m] = true;

				/*check if intxPnt falls inside the top TO tile edge, checks on x&y axes: recycle intxPntVec &  intxPntAngle*/
				intxPntVec[m][0] = intxPnt[indxTop[m]][0] - sensorCoordFOV[0];
				intxPntVec[m][1] = intxPnt[indxTop[m]][1] - sensorCoordFOV[1];
				getAngleQuadrant(&intxPntVec[m][0], &intxPntVec[m][1], &intxPntAngle[m]);
				doAnglesConvention(limAngleSens, &intxPntAngle[m]);

				if ((INLIMITS(Obj->Envelope.Areas_0[k].Edge[0][0], Obj->Envelope.Areas_0[k].Edge[2][0], intxPnt[indxTop[m]][0]) &&  // on x
					INLIMITS(Obj->Envelope.Areas_0[k].Edge[0][1], Obj->Envelope.Areas_0[k].Edge[2][1], intxPnt[indxTop[m]][1])) // on y
					&& abs(limAngleSens[m] - intxPntAngle[m]) < 1) //intersection point is in the direction of the sensor (not 180)
					fovXtop[m] = true;

				fovXto[m] = fovXtop[m] && fovXbotm[m];
			}
			/*fovXto == true : fov edge(s) intersect TO tile*/

			/*do next tile if tile outside FOV and no FOV edge intersection*/
			if ((!fovXto[0] && !fovXto[1]) && (offFOV[0] && offFOV[1]))
				continue;

			if (fovXto[0] && fovXto[1]) {
				/*all TO corners are outside FOV; FOV intersects TO tile at both FOV edges
				- easy, redo entire tile*/
				for (int p = 0; p < 4; p++)
					VEC_Assign(newTOEdge[p], intxPnt[p]);

				intxState = true;
			}
			else if ((fovXto[0] && !fovXto[1]) || (!fovXto[0] && fovXto[1])) {
				/*only one intersection, two tile corners are inside FOV*/
				//which intersection point:
				for (int m = 0; m < 2; m++) {
					if (fovXto[m]) {
						VEC_Assign(intx[0], intxPnt[m]);
						VEC_Assign(intx[1], intxPnt[indxTop[m]]);
					}
				}

				if (!offFOV[0]) {
					/*Edge[0][] is inside FOV:take Edge[0][] and Edge[3][]*/
					VEC_Assign(newTOEdge[0], Obj->Envelope.Areas_0[k].Edge[0]);
					VEC_Assign(newTOEdge[3], Obj->Envelope.Areas_0[k].Edge[3]);
					VEC_Assign(newTOEdge[1], intx[0]);
					VEC_Assign(newTOEdge[2], intx[1]);

					intxState = true;

				}
				else if (!offFOV[1]) {
					/*Edge[1][] is inside FOV:take Edge[1][] and Edge[2][]*/
					VEC_Assign(newTOEdge[1], Obj->Envelope.Areas_0[k].Edge[1]);
					VEC_Assign(newTOEdge[2], Obj->Envelope.Areas_0[k].Edge[2]);
					VEC_Assign(newTOEdge[0], intx[0]);
					VEC_Assign(newTOEdge[3], intx[1]);

					intxState = true;
				}
			}
			else if ((!offFOV[0] || !offFOV[1]) && (!fovXto[0] && !fovXto[1])) {
				/*one corner is in FOV but there is no intersection - due to fovTolerance:
				corner taken as "in fov" but outside of FOV by less than fovTolerance*/

				/*make the tile == to the edge closest to FOV*/
				if (!offFOV[0]) {
					/*Edge[0][] is close to FOV:take Edge[0][] and Edge[3][]*/
					VEC_Assign(newTOEdge[1], Obj->Envelope.Areas_0[k].Edge[0]);
					VEC_Assign(newTOEdge[0], Obj->Envelope.Areas_0[k].Edge[0]);
					VEC_Assign(newTOEdge[2], Obj->Envelope.Areas_0[k].Edge[3]);
					VEC_Assign(newTOEdge[3], Obj->Envelope.Areas_0[k].Edge[3]);

					intxState = true;
				}
				else if (!offFOV[1]) {
					/*Edge[1][] is inside FOV:take Edge[1][] and Edge[2][]*/
					VEC_Assign(newTOEdge[0], Obj->Envelope.Areas_0[k].Edge[1]);
					VEC_Assign(newTOEdge[1], Obj->Envelope.Areas_0[k].Edge[1]);
					VEC_Assign(newTOEdge[3], Obj->Envelope.Areas_0[k].Edge[2]);
					VEC_Assign(newTOEdge[2], Obj->Envelope.Areas_0[k].Edge[2]);

					intxState = true;
				}
			}
			/*check if normal from USS falls inside the modified TO surface*/
			if (intxState)
				intx2polygoAnglesSum(tempCoord, k, newTOEdge, &isInPolig, &err, &nearPoly);
		}
		else if (!noLPIntx) {
			/*check if normal from USS falls inside original tile - overloaded func */
			intx2polygoAnglesSum(tempCoord, k, Obj, &isInPolig, &err, &nearPoly);
		}
		else {
			continue;//this should not happen
		}
		/****/

		/*if itx is outside the TO surface & there is some intersection with FOV edge
		or partly in FOV || if is in polygon but slighlty out -- > find closest line*/
		if ((!isInPolig && intxState) || (isInPolig && nearPoly && intxState)) {
			/*find shortest distance to closest modified TO tile edge and NP on edge*/
			point2edgeDist(newTOEdge, k, j, &tempDist, tempCoord);

			/*store point of shortest distance (if NP falls on the fake tile edge) */
			if (VEC_Abs(tempCoord, sensorCoordNP) < minInFovDist) {
				minInFovDist = VEC_Abs(tempCoord, sensorCoordNP);
				/*closest in-FOV 3D point,
				can be different than NP if NP is on a corner and
				the closest tile is only partly in fov and not normal to sensor*/
				VEC_Assign(TrfObj[Obj->Cfg.Id].OccNearestPointPos_Fr0[j], tempCoord);
			}

			/*check if the NP was found on the fake tile edge,
			if yes, don't store data, not normal to tile and not a real corner*/
			if (!nearPoly && /*if it's not near the polygon*/
				!offFOV[1] && //this is the new edge: newTOEdge[0][] - newTOEdge[3][]:
				(INLIMITS(newTOEdge[0][0], newTOEdge[3][0], tempCoord[0]) &&
					INLIMITS(newTOEdge[0][1], newTOEdge[3][1], tempCoord[1]) &&
					INLIMITS(newTOEdge[0][2], (newTOEdge[3][2] - 2 * ZERODIFF), tempCoord[2]))) {
				/*(newTOEdge[3][2] - 2*ZERODIFF), make height smaller: NP could be on the top line of the fake tile;
				if height of TO smaller than USS height, top line is a valid reflection ;
				2*ZERODIFF because INLIMITS already has ZERODIFF*/
				continue;
			}
			else if (!nearPoly && /*if it's not near the polygon*/
				!offFOV[0] && //this is the new edge: newTOEdge[1][] to newTOEdge[2][]
				(INLIMITS(newTOEdge[1][0], newTOEdge[2][0], tempCoord[0]) &&
					INLIMITS(newTOEdge[1][1], newTOEdge[2][1], tempCoord[1]) &&
					INLIMITS(newTOEdge[1][2], (newTOEdge[2][2] - 2 * ZERODIFF), tempCoord[2]))) {
				continue;
			}

		}
		else if ((!isInPolig) || (isInPolig && nearPoly)) {
			/*shortest distance to original tile edge - overloaded func*/
			double fakeRx[3] = {0};
			point2edgeDist(Obj, k, j, EchoType, &tempDist, tempCoord, fakeRx);
		}

		tempDist = abs(tempDist);

		/*check if NP is the FOV of sensor*/
		bool	isInFov = false, inFovLeft = false, inFovRight = false;
		double	uss2NP_orientRight, uss2NP_orientLeft;
		if (EchoType == 0) {
			checkNPinFOV(tempCoord, sensorCoordFOV, limAngleSens, &isInFov, &sens2NP_orient);
			/*if dist to NP is larger than sensor range -- it's not in FOV*/
			isInFov = (tempDist > SensParam[j].Range) ? false : isInFov;
		}
		else if (EchoType == 1){
			checkNPinFOV(tempCoord, sensorCoordLeft, limAngleSensLeft, &inFovLeft, &uss2NP_orientLeft);
			checkNPinFOV(tempCoord, sensorCoordRight, limAngleSensRight, &inFovRight, &uss2NP_orientRight);
			isInFov			= inFovLeft && inFovRight;
			distUssLeft		= VEC_Abs(sensorCoordLeft, tempCoord);
			distUssRight	= VEC_Abs(sensorCoordRight, tempCoord);
			
			/*if dist to NP is larger than sensor range -- it's not in FOV*/
			isInFov = (distUssLeft > SensParam[j].Range || distUssRight > SensParam[rightSensId].Range) ? false : isInFov;
		}

		double xNpSens, yNpSens, ussToVirtUss;
		double sens2NP_ussFR, ussLeft2NP_ussFR, ussRight2NP_ussFR;

		if (EchoType == 0) {
			//NP distance to sensor	== distToNP
			//sensor to NP azimuth	== NPdirection

			// check if reflection points are in FoV based on nominal masurement lobe
			bool inRangeTx = false;
			//bool inRangeRx = false;

			uint8_t idx_dist_x = 0;
			uint8_t idx_dist_y = 40;

			sens2NP_ussFR		 = NP2USS_DIRECTION(limAngleSens, sens2NP_orient);
			double sin_azimuthTx = sin(sens2NP_ussFR * deg2rad);
			double cos_azimuthTx = cos(sens2NP_ussFR * deg2rad);

			idx_dist_x = M_BOUND(0, 80, (uint8_t)((tempDist * cos_azimuthTx) * 10));
			idx_dist_y = M_BOUND(0, 80, (uint8_t)((tempDist * sin_azimuthTx) * 10 + 40));

			inRangeTx = (tempDist < 0.10) ? true : cus_dist_lobe[idx_dist_x][idx_dist_y];

			isInFov = inRangeTx;
		}
		if (EchoType == 1) {
			//NP distance to sensor left == TrfObj[Obj->Cfg.Id].vNpDistLeft[j]
			//sensor left to NP azimuth	== TrfObj[Obj->Cfg.Id].vNpDirLeft[j]

			//NP distance to sensor right	== TrfObj[Obj->Cfg.Id].vNpDistRight[j]
			//sensor right to NP azimuth	== TrfObj[Obj->Cfg.Id].vNpDirRight[j]


			// check if reflection points are in FoV based on nominal masurement lobe
			bool inRangeTx = false;
			bool inRangeRx = false;

			uint8_t idx_dist_x = 0;
			uint8_t idx_dist_y = 40;

			ussLeft2NP_ussFR	 = NP2USS_DIRECTION(limAngleSensLeft, uss2NP_orientLeft);
			double sin_azimuthTx = sin(ussLeft2NP_ussFR * deg2rad);
			double cos_azimuthTx = cos(ussLeft2NP_ussFR * deg2rad);

			idx_dist_x = M_BOUND(0, 80, (uint8_t)((distUssLeft * cos_azimuthTx) * 10));
			idx_dist_y = M_BOUND(0, 80, (uint8_t)((distUssLeft * sin_azimuthTx) * 10 + 40));

			inRangeTx = (distUssLeft < 0.10) ? true : cus_dist_lobe[idx_dist_x][idx_dist_y];

			ussRight2NP_ussFR	 = NP2USS_DIRECTION(limAngleSensRight, uss2NP_orientRight);
			double sin_azimuthRx = sin(ussRight2NP_ussFR * deg2rad);
			double cos_azimuthRx = cos(ussRight2NP_ussFR * deg2rad);

			idx_dist_x = M_BOUND(0, 80, (uint8_t)((distUssRight * cos_azimuthRx) * 10));
			idx_dist_y = M_BOUND(0, 80, (uint8_t)((distUssRight * sin_azimuthRx) * 10 + 40));

			inRangeRx = (distUssRight < 0.10) ? true : cus_dist_lobe[idx_dist_x][idx_dist_y];

			isInFov = (inRangeTx && inRangeRx);
		}

		/*compare distance to previous one and keep NP if dist is smaller and NP is in sensor FOV*/
		if ((tempDist < *pDist) && isInFov)
		{
			*pDist		= tempDist;
			*surfNoOut	= k;
			if (EchoType == 0) {
				CHECKAZIMUTH(sens2NP_orient);
				*direction = sens2NP_orient;
				/*get and store elevation*/
				TrfObj[Obj->Cfg.Id].NearestPointElev_Fr0[j] = getNpElevation(tempCoord, SensData[j].Fr0_position_Sens);
			}
			else if (EchoType == 1) {
				/*get sensor to NP orientation
					-for now, independently of virtual sensor FOV*/
				xNpSens = tempCoord[0] - sensorCoordNP[0];
				yNpSens = tempCoord[1] - sensorCoordNP[1];
				getAngleQuadrant(&xNpSens, &yNpSens, &sens2NP_orient);
				/*macro - correct angles*/
				CHECKAZIMUTH(sens2NP_orient);
				*direction = sens2NP_orient;

				/*azimuth with real sensors*/
				CHECKAZIMUTH(uss2NP_orientLeft);
				TrfObj[Obj->Cfg.Id].vNpDirLeft[j]	= uss2NP_orientLeft;
				CHECKAZIMUTH(uss2NP_orientRight);
				TrfObj[Obj->Cfg.Id].vNpDirRight[j]	= uss2NP_orientRight;

				/*distance to real sensors*/
				TrfObj[Obj->Cfg.Id].vNpDistLeft[j]	= distUssLeft;
				TrfObj[Obj->Cfg.Id].vNpDistRight[j] = distUssRight;

				/*elevation of vNP with real sensors*/
				TrfObj[Obj->Cfg.Id].vNpElevLeft[j]	= getNpElevation(tempCoord, sensorCoordLeft);
				TrfObj[Obj->Cfg.Id].vNpElevRight[j] = getNpElevation(tempCoord, sensorCoordRight);

				TrfObj[Obj->Cfg.Id].vLoF[j] = distUssLeft + distUssRight;
				ussToVirtUss = VEC_Abs(sensorCoordLeft, sensorCoordNP);

				/*calculate vritual NP elipse parameters: 
							a = d/2, where d =  L1+L2 (distances from each adjacent real sensor to vNP)
							b = sqrt(a*a -  realToVirtUssDist*realToVirtUssDist)*/
				TrfObj[Obj->Cfg.Id].vNpA[j] = TrfObj[Obj->Cfg.Id].vLoF[j] / 2;
				TrfObj[Obj->Cfg.Id].vNpB[j] = sqrt(pow(TrfObj[Obj->Cfg.Id].vNpA[j], 2) - pow(ussToVirtUss, 2));
			}
			*notInFOV	= false;
			VEC_Assign(coordOut_NP_Fr0, tempCoord);

			/*store point of shortest distance (if it's the same with the NP) */
			if (*pDist < minInFovDist && EchoType == 0) {
				minInFovDist = *pDist;
				VEC_Assign(TrfObj[Obj->Cfg.Id].OccNearestPointPos_Fr0[j], tempCoord);
			}
		}
	}
}


void
getVirtualSensorPos(int iSens, int maxNSens) {
/*calculates virtual sensors position, situated in the middle of the line segment between
the current sensor and next one. i.e., for sensor 1, virtual sensor 1 is half way between 
sensor 1 and sensor 2*/
	int nextSens = (iSens + 1) % maxNSens;

	for (int j = 0; j < 3; j++){
		SensParam[iSens].vFr1_position[j] = (SensParam[iSens].Fr1_position[j] + SensParam[nextSens].Fr1_position[j]) / 2;
	}

}

void 
getGlobalSensorPosition(int j) {
	/*calculates gloabal positions for all sensors
	-used in Init_Clean to pre-calculate all positions 
	position of sensor j+1 is needed when j is called for cross-reflections*/

	/* Derive global sensor positions */
	SensData[j].Fr0_position_Sens[3] = { 0 };


	SensData[j].Fr0_position_Sens[0] = Vehicle.Fr1A.t_0[0] +
		(SensParam[j].Fr1_position[0] * cos(Vehicle.Fr1A.r_zyx[2]) - SensParam[j].Fr1_position[1] * sin(Vehicle.Fr1A.r_zyx[2]));
	// [xPos in Fr0] + [x = xdot*cos(phi) - ydot*sin(phi)]
	SensData[j].Fr0_position_Sens[1] = Vehicle.Fr1A.t_0[1] +
		(SensParam[j].Fr1_position[0] * sin(Vehicle.Fr1A.r_zyx[2]) + SensParam[j].Fr1_position[1] * cos(Vehicle.Fr1A.r_zyx[2]));
	// [yPos in Fr0] + [y = xdot*sin(phi) + ydot*cos(phi)]
	SensData[j].Fr0_position_Sens[2] = SensParam[j].Fr1_position[2];

	/* Generate Fr0_position_vSens for Cross Echoes ********** */
	SensData[j].Fr0_position_vSens[0] = Vehicle.Fr1A.t_0[0] +
		(SensParam[j].vFr1_position[0] * cos(Vehicle.Fr1A.r_zyx[2]) - SensParam[j].vFr1_position[1] * sin(Vehicle.Fr1A.r_zyx[2]));
	//[xPos in Fr0] + [x = xdot*cos(phi) - ydot*sin(phi)]
	SensData[j].Fr0_position_vSens[1] = Vehicle.Fr1A.t_0[1] +
		(SensParam[j].vFr1_position[0] * sin(Vehicle.Fr1A.r_zyx[2]) + SensParam[j].vFr1_position[1] * cos(Vehicle.Fr1A.r_zyx[2]));
	//[yPos in Fr0] + [y = xdot*sin(phi) + ydot*cos(phi)]
	SensData[j].Fr0_position_vSens[2] = SensParam[j].vFr1_position[2];
}

void 
checkCorner(tTrafficObj *Obj, int *npTile, double tempOrientation, int i, double *tolerance, bool *reflecting, double *ussPositionNow, double *nearestPointNow) {
	/*checks if current corner is a reflecting one:
	if the NP-sensor direction is in the "FOV" of both tiles forming the corner*/
	bool	corner0, corner1;
	bool	isInTile1, isInTile2;
	int		otherTile;
	double	xCornSens, yCornSens;
//	double	nearestPointNow[3], ussPositionNow[3];
	double	tileFOV1[2], tileFOV2[2], np2UssOrientation;
	bool isNotCuboid	= Obj->Cfg.Envelope.Mode == EnvelopeMode_2DContour;
	bool isCuboid		= Obj->Cfg.Envelope.Mode == EnvelopeMode_Cuboid;


	/*which tile corner - see 'Edge' in traffic.h*/
	if (abs(Obj->Envelope.Areas_0[*npTile].Edge[0][0] - nearestPointNow[0]) < *tolerance &&
		abs(Obj->Envelope.Areas_0[*npTile].Edge[0][1] - nearestPointNow[1]) < *tolerance) {
		corner0 = true;
		corner1 = false;
	}
	else if (abs(Obj->Envelope.Areas_0[*npTile].Edge[1][0] - nearestPointNow[0]) < *tolerance &&
		abs(Obj->Envelope.Areas_0[*npTile].Edge[1][1] - nearestPointNow[1]) < *tolerance) {
		corner1 = true;
		corner0 = false;
	}
	else {//this should not happen, corner was detected in parrent func
		corner0 = false;
		corner1 = false;
		*reflecting = false;
		return;
	}

	if (*npTile > 3 && (isCuboid)) {
		/*tiles 4 and 5 of cuboids are horizontal: this should not happen
		because horizontals are excluded in the corner eval loop from vCUS_ReflectionEval*/
		*reflecting = false;
		return;
	}

	if (corner0) {
		/*corner was assigned to NP (even if reflection is on tile +/-tolerance from corner),
		so set sensor-NP azimuth to corner*/
		xCornSens = Obj->Envelope.Areas_0[*npTile].Edge[0][0] - ussPositionNow[0];
		yCornSens = Obj->Envelope.Areas_0[*npTile].Edge[0][1] - ussPositionNow[1];
		getAngleQuadrant(&xCornSens, &yCornSens, &tempOrientation);
		//next tile: if cuboid  ? look only at tiles 0-3 : else look at all tiles for 2D shape
		otherTile = (isCuboid) ? ((*npTile + 1) % 4) : ((*npTile + 1) % Obj->Cfg.Envelope.nArea);

		/*reflective corners based on normals to tiles*/
		tileFOV1[0] = env_vector->NormalDir[otherTile] - 90;
		tileFOV1[1] = env_vector->NormalDir[otherTile] + 90;
		tileFOV2[0] = env_vector->NormalDir[otherTile] - 90;
		tileFOV2[1] = env_vector->NormalDir[otherTile] + 90;
	}
	else if (corner1) {
		/*corner was assigned to NP (even if reflection is on tile +/-tolerance from corner),
		so set sensor-NP azimuth to corner*/
		xCornSens = Obj->Envelope.Areas_0[*npTile].Edge[1][0] - ussPositionNow[0];
		yCornSens = Obj->Envelope.Areas_0[*npTile].Edge[1][1] - ussPositionNow[1];
		getAngleQuadrant(&xCornSens, &yCornSens, &tempOrientation);
		//previous tile
		if (isCuboid && (*npTile == 0))//cuboid
			otherTile = 3;
		else if (isNotCuboid && (*npTile == 0))//2D shape TO
			otherTile = Obj->Cfg.Envelope.nArea - 1;
		else
			otherTile = *npTile - 1;

		/*reflective corners based on normals to tiles*/
		tileFOV1[0] = env_vector->NormalDir[*npTile] - 90;
		tileFOV1[1] = env_vector->NormalDir[*npTile] + 90;
		tileFOV2[0] = env_vector->NormalDir[otherTile] - 90;
		tileFOV2[1] = env_vector->NormalDir[otherTile] + 90;
	}

	/*check corner based on tiles normals; tile fov defined as tile normal +/- 90
	sensor should be inside both tile FOVs*/
	np2UssOrientation = tempOrientation + 180;//take NP2USS not USS2NP
	CHECKAZIMUTH(np2UssOrientation);
	//facing tile 1
	doAnglesConvention(tileFOV1, &np2UssOrientation);
	isInTile1 = ((np2UssOrientation > tileFOV1[0] && np2UssOrientation <= env_vector->NormalDir[*npTile]) ||
						(np2UssOrientation > env_vector->NormalDir[*npTile] && np2UssOrientation < tileFOV1[1]));
	
	//facing tile 2
	doAnglesConvention(tileFOV2, &np2UssOrientation);
	isInTile2 = ((np2UssOrientation > tileFOV2[0] && np2UssOrientation <= env_vector->NormalDir[otherTile]) ||
							(np2UssOrientation > env_vector->NormalDir[otherTile] && np2UssOrientation < tileFOV2[1]));

	if (isInTile1 && isInTile2) 
		*reflecting = true;
	else
		*reflecting = false;

}

void getLineProjection(double *A, double *B, double *C, double *D) {
	double x1 = A[0], y1 = A[1], x2 = B[0], y2 = B[1], x3 = C[0], y3 = C[1];
	double px = x2 - x1, py = y2 - y1, dAB = px * px + py * py;
	double u = ((x3 - x1) * px + (y3 - y1) * py) / dAB;
	D[0] = x1 + u * px;
	D[1] = y1 + u * py;
}

void
checkTO_inRange(double *minXY, double *maxXY, int j, bool *inRange) {

	double	boundBox[4][2];//4 points, with x and y
	double	egoYaw, ussFoV_lims[2], ussAzim, angleUSS2point; //degrees
	double	dist2pnt, proj[2];
	double	cornerCrd[2];
	bool	cornerInFov;
	double  boxSlope, bBox;
	int		nextCorner;

	double	bFov[2], slopeFov[2], thisCorner[2], thatCorner[2];
	double	intxPnt[2], intxPntAngle, intxPntVec[2], ussProj[2];
	double	dist2interX;
	bool	FoVintersects;
	
	*inRange = false;

	/*make bounding box*/
	VEC_Assign2D(boundBox[0], minXY);
	boundBox[1][0] = minXY[0];
	boundBox[1][1] = maxXY[1];
	VEC_Assign2D(boundBox[2], maxXY);
	boundBox[3][0] = maxXY[0];
	boundBox[3][1] = minXY[1];
	/**/

	/*get sensor direction and FoV*/
	egoYaw = rad2deg * Vehicle.Yaw;
	/* rad2deg * Vehicle.Yaw can be e.g. -224 , correct this*/
	if (egoYaw >= -360 && egoYaw < -180) egoYaw += 360;
	getSensorFOV(j, egoYaw, ussFoV_lims, &ussAzim);

	/*check if any of the box corners are in USS FoV and in Range*/
	for (int p = 0; p < 4; p++) {
		cornerInFov = false;
		VEC_Assign2D(cornerCrd, boundBox[p]);
		/*get distance in 2d should be OK, unless the obj is very high*/
		proj[0] = cornerCrd[0] - SensData[j].Fr0_position_Sens[0];
		proj[1] = cornerCrd[1] - SensData[j].Fr0_position_Sens[1];
		dist2pnt = sqrt(pow(proj[0], 2) + pow(proj[1], 2));

		checkNPinFOV(cornerCrd, SensData[j].Fr0_position_Sens, ussFoV_lims, &cornerInFov, &angleUSS2point);
		/*if a corner is in FoV and in range, stop, TO is in range*/
		if ((cornerInFov) & (dist2pnt <= SensParam[j].Range)) {
			*inRange = true;
			return;
		}
	}

	/*for large TO, TO can be completely in FoV but corners not in range, i.e. > 8 m;
	check USS projection on box edges if it falls in FoV and in range*/
	for (int i = 0; i < 4; i++) {
		cornerInFov = false;
		nextCorner = (i + 1) % 4;
		VEC_Assign2D(thisCorner, boundBox[i]);
		VEC_Assign2D(thatCorner, boundBox[nextCorner]);
		getLineProjection(thisCorner, thatCorner, SensData[j].Fr0_position_Sens, ussProj);

		/*this should be a function ################*/
		/*check if intx is on the box line*/
		intxPntVec[0] = ussProj[0] - SensData[j].Fr0_position_Sens[0];
		intxPntVec[1] = ussProj[1] - SensData[j].Fr0_position_Sens[1];
		getAngleQuadrant(&intxPntVec[0], &intxPntVec[1], &intxPntAngle);
		doAnglesConvention(ussFoV_lims, &intxPntAngle);

		checkNPinFOV(ussProj, SensData[j].Fr0_position_Sens, ussFoV_lims, &cornerInFov, &angleUSS2point);

		/*distance USS to projection on box*/
		dist2interX = sqrt(pow(intxPntVec[0], 2) + pow(intxPntVec[1], 2));

		if ((INLIMITS(thisCorner[0], thatCorner[0], ussProj[0]) &&  // on x on segment
			INLIMITS(thisCorner[1], thatCorner[1], ussProj[1])) &&// on y on segment
			cornerInFov  &&//projection is in USS FoV
			(dist2interX <= (SensParam[j].Range + _IN_RANGE_TOLERANCE_))) {
			*inRange = true;
				return;
		}
		/*################*/
	}

		/*if corners are not in range, and no projection in range, check intersection of FoV edges with the box edges*/
		/*=============================================*/
	for (int i = 0; i < 4; i++) {
		//equation of current edge; y = slope*x + b 
		nextCorner	= (i+1) % 4; 
		boxSlope	= (boundBox[nextCorner][1] - boundBox[i][1]) /
									(boundBox[nextCorner][0] - boundBox[i][0]);
		bBox		= boundBox[nextCorner][1] - boxSlope * boundBox[nextCorner][0];

		for (int m = 0; m < 2; m++) {
			FoVintersects = false;
			//equation of FOV line-border max & min  y = slopeFov*x + b;
			slopeFov[m] = tan(deg2rad * ussFoV_lims[m]);
			bFov[m]		= SensData[j].Fr0_position_Sens[1] - slopeFov[m] * SensData[j].Fr0_position_Sens[0];

			/*check intersection if FOV edge is not parallel to the edge*/
			if ((boundBox[nextCorner][0] == boundBox[i][0]) && //vertical edge in 2D -undefined slope
				(abs(slopeFov[m] - boxSlope) > ZERODIFF)) {
				/*intx FOV limit vs. bottom tile edge*/
				intxPnt[0] = boundBox[i][0]; // x of intersection is x of edge
				intxPnt[1] = bFov[m] + slopeFov[m] * intxPnt[0];
			}
			else if (abs(slopeFov[m] - boxSlope) > ZERODIFF) {
				/*intx FOV limit vs. edge*/
				intxPnt[0] = (bFov[m] - bBox) / (boxSlope - slopeFov[m]);
				intxPnt[1] = (bFov[m] * boxSlope - bBox * slopeFov[m]) / (boxSlope - slopeFov[m]);
			}

			/*check if intx is on the box line*/
			intxPntVec[0] = intxPnt[0] - SensData[j].Fr0_position_Sens[0];
			intxPntVec[1] = intxPnt[1] - SensData[j].Fr0_position_Sens[1];
			getAngleQuadrant(&intxPntVec[0], &intxPntVec[1], &intxPntAngle);
			doAnglesConvention(ussFoV_lims, &intxPntAngle);

			if ((INLIMITS(boundBox[i][0], boundBox[nextCorner][0], intxPnt[0]) &&  // on x
				INLIMITS(boundBox[i][1], boundBox[nextCorner][1], intxPnt[1])) // on y
				&& abs(ussFoV_lims[m] - intxPntAngle) < 1) //intersection point is in the direction of the sensor (not 180)
				FoVintersects = true;

			/*distance USS to intersection*/
			dist2interX = sqrt(pow(intxPntVec[0], 2) + pow(intxPntVec[1], 2));

			if (FoVintersects & (dist2interX <= (SensParam[j].Range + _IN_RANGE_TOLERANCE_))) {
				*inRange = true;
				return;
			}

		}//for FoV edges
			   
	}// for 4 corners

}


void
shift_mfPlot_stru() {
	/*moves the structure one position: frees position 0*/

	int nPnts;

	for (int m = _MF_PLOT_HISTORY_ - 1; m > 0; m--) {
		
		if (MfPlotDataStru[m-1].vCUS_CycleId != -1) {
		/*if there is some data at this index, copy to m*/
			
			/*make all m entries zero*/
			MfPlotDataStru[m]				= { 0 }; 

			/*copy data from previous index*/
			nPnts							= MfPlotDataStru[m - 1].nReflPnts;
			MfPlotDataStru[m].mfp_CycleId	= MfPlotDataStru[m - 1].mfp_CycleId;
			MfPlotDataStru[m].vCUS_CycleId	= MfPlotDataStru[m - 1].vCUS_CycleId;
			MfPlotDataStru[m].nReflPnts		= MfPlotDataStru[m - 1].nReflPnts;
			
			/*copy coordinates*/
			for (int iPnt = 0; iPnt < nPnts; iPnt++){/*loop in loop: consider replacing with c++ array to use std::copy() function*/
				
				//VEC_Assign2D(MfPlotDataStru[m].CoordArrayFr1[iPnt], MfPlotDataStru[m - 1].CoordArrayFr1[iPnt]);
				
				/*don't convert/copy if coordonates are 0*/
				if ((MfPlotDataStru[m - 1].CoordArrayFr0[iPnt][0] == 0) &&
					(MfPlotDataStru[m - 1].CoordArrayFr0[iPnt][1] == 0))
					continue;
				
				/*not only copy the previous cycle data but convert the FR0 data in the previous cycle to FR1-- ego to NP dist might have changed*/
				convertFr0_toFr1(MfPlotDataStru[m].CoordArrayFr1[iPnt], MfPlotDataStru[m - 1].CoordArrayFr0[iPnt]);
				
				/*copy FR0 data*/
				VEC_Assign2D(MfPlotDataStru[m].CoordArrayFr0[iPnt], MfPlotDataStru[m - 1].CoordArrayFr0[iPnt]);
			}
		}
	}
}

void
convertFr0_toFr1(float *Fr1_coordinates, float *Fr0_coordinates) {
	/*converts coordinates in 2D from fr0 to vehicle inertial frame-- for mf_plot*/
	
	double sinCalc, cosCalc;
	sinCalc = sin(Vehicle.Yaw);
	cosCalc = cos(Vehicle.Yaw);

	/*actually inertial frame not FR1*/
	Fr1_coordinates[0] = Fr0_coordinates[0] * (float)cosCalc + Fr0_coordinates[1] * (float)sinCalc
							- (float)(InertialSensor[1].Pos_0[0] * cosCalc + InertialSensor[1].Pos_0[1] * sinCalc);
	Fr1_coordinates[1] = -Fr0_coordinates[0] * (float)sinCalc + Fr0_coordinates[1] * (float)cosCalc
							- (float)(-InertialSensor[1].Pos_0[0] * sinCalc + InertialSensor[1].Pos_0[1] * cosCalc);

}

void
convertFr0_toFr1_3D(double *Fr1_coordinates, double *Fr0_coordinates) {
	/*converts coordinates in 3D from fr0 to fr1*/

	double sinCalc, cosCalc;
	sinCalc = sin(Vehicle.Yaw);
	cosCalc = cos(Vehicle.Yaw);

	/*convert to FR1*/
	Fr1_coordinates[0] = Fr0_coordinates[0] * cosCalc + Fr0_coordinates[1] * sinCalc
		- (Vehicle.Fr1A.t_0[0] * cosCalc + Vehicle.Fr1A.t_0[1] * sinCalc);
	Fr1_coordinates[1] = -Fr0_coordinates[0] * sinCalc + Fr0_coordinates[1] * cosCalc
		- (-Vehicle.Fr1A.t_0[0] * sinCalc + Vehicle.Fr1A.t_0[1] * cosCalc);
	Fr1_coordinates[2] = Fr0_coordinates[2];

}

void
convertFr1toFr0(double *vehFr1, double *vehYaw, double *sensor_FrI, double* sensor_Fr0) {
	/*converts coordinates in 2D from Fr0 to fr1*/

	double conversion_FactX, conversion_FactY, conversion_yawFactor, conversion_sensorFactor;
	double sinCalc, cosCalc;
	sinCalc = sin(*vehYaw);
	cosCalc = cos(*vehYaw);

	conversion_FactX		=  vehFr1[0] * cosCalc + vehFr1[1] * sinCalc;
	conversion_FactY		= -vehFr1[0] * sinCalc + vehFr1[1] * cosCalc;
	conversion_yawFactor	= cosCalc / (cosCalc*cosCalc + sinCalc * sinCalc);
	conversion_sensorFactor = sensor_FrI[1] + (sensor_FrI[0] + conversion_FactX) * (sinCalc / cosCalc) + conversion_FactY;

	sensor_Fr0[1] = conversion_yawFactor * conversion_sensorFactor;
	sensor_Fr0[0] = (sensor_FrI[0] - sensor_Fr0[1] * sinCalc + conversion_FactX) / cosCalc;
	sensor_Fr0[2] = sensor_FrI[2];

}

void
recalculateNP_postToF(tRxState_tof *stateOfRx) {
/*calculates NP position for the position of Rx corrected for vehicle movement during ToF*/
	
	double		virtualRx[3], tempCoord[3], planeCoefs[4] = { 0 }, linePara1[3], linePara2[3] = { 0 };
	int			nextSurf, prevSurf, k;
	double		tempDist = HUGE_VAL, finalDist = HUGE_VAL, err = 0.01;
	double		sens2NP_orient, polygTolerance = 0.05;
	bool		noLPIntx, isInPolig, nearPoly;
	bool		onTile, nearPolyg;
	int			equalNormals, doCornerMapping, ReflState;
	tTrafficObj *Obj = NULL;

	stateOfRx->npIsUpdated = false;
	
	/*get the virtual Rx coordinates, halfway between Tx and the corrected Rx postion*/
	for (int iDim = 0; iDim < 3; iDim++)
		virtualRx[iDim] = (SensData[stateOfRx->txId].Fr0_position_Sens[iDim] + stateOfRx->coord_RxFr0_updated[iDim])/2;
	
	VEC_Assign(linePara1, virtualRx);

	/************ find new NP on the current tile or neighbouring*/
	
	/*check only current tile and adjacent ones, particular case for cuboid which has 6 tiles
	with 4 and 5 horizontal (excluded from NP generation)*/
	Obj = Traffic_GetByTrfId(stateOfRx->TOid);
	bool isNotCuboid = Obj->Cfg.Envelope.Mode == EnvelopeMode_2DContour;
	bool isCuboid = Obj->Cfg.Envelope.Mode == EnvelopeMode_Cuboid;

	if (isCuboid && stateOfRx->startNP_Tile == 0)
		prevSurf = 3;
	else if (isNotCuboid && stateOfRx->startNP_Tile == 0)
		prevSurf = Obj->Cfg.Envelope.nArea - 1;
	else
		prevSurf = stateOfRx->startNP_Tile - 1;

	/*if TO is cuboid, nArea == 6, do not look at top and bottom surfaces: 4 & 5*/
	nextSurf = (isCuboid && stateOfRx->startNP_Tile == 3) ? 0 :
				((stateOfRx->startNP_Tile + 1) % Obj->Cfg.Envelope.nArea);

	int checkTiles[] = { prevSurf, stateOfRx->startNP_Tile,  nextSurf };//list of the current and neighboring tiles
		
	/*check the tiles and edges*/
	for (int p = 0; p < 3; p++) {
		k				= checkTiles[p];
		equalNormals	= 0;
		doCornerMapping = 0;
		ReflState		= 0;
		onTile = false, nearPolyg = false;
		noLPIntx = true, isInPolig = false, nearPoly = false;

		//find tile plane parameters; planeParams[0-2] are xyz coordinates of normal vec to plane
		planeParams(planeCoefs, k, Obj);
		VEC_Assign(linePara2, planeCoefs);

		/*distance from P(xp,yp,zp) to plane ax+by+cz+d=0 is (a*xp+b*yp+c*zp+d)/sqrt(a*a+b*b+c*c)
		see mathinsight.org/distance_point_plane*/
		tempDist = (planeCoefs[0] * virtualRx[0] +
			planeCoefs[1] * virtualRx[1] +
			planeCoefs[2] * virtualRx[2] + planeCoefs[3]) /
			sqrt(pow(planeCoefs[0], 2) + pow(planeCoefs[1], 2) + pow(planeCoefs[2], 2));

		/*find where normal to plane passing through Sensor meets the plane*/
		planeLineIntx(linePara1, linePara2, planeCoefs, tempCoord, &noLPIntx);

		/*check if normal from USS falls inside original tile - overloaded func */
		intx2polygoAnglesSum(tempCoord, k, Obj, &isInPolig, &err, &nearPoly);

		if ((!isInPolig) || (isInPolig && nearPoly)) {
			/*shortest distance to original tile edge - overloaded func*/
			int EchoType = 2, fakeJ = 0;//flag for virtual Rx: function uses virtualRx, not sensor j
			point2edgeDist(Obj, k, fakeJ, EchoType, &tempDist, tempCoord, virtualRx);
		}
		tempDist = abs(tempDist);

		/*check if NP is the FOV of Rx and Tx at new position */
		bool	isInFov = false, isInFovRx = false, isInFovTx = false;
		double  limAngle_Tx[2], txDirection, txNp_orientation; 
		double  limAngle_Rx[2], rxDirection, rxNp_orientation;
		double	Rx_dist, Tx_dist;
		
		//check for Rx at updated position, yaw and new NP coordinates
		getSensorFOV(stateOfRx->rxId, rad2deg * stateOfRx->egoYaw_Final, limAngle_Rx, &rxDirection);
		checkNPinFOV(tempCoord, stateOfRx->coord_RxFr0_updated, limAngle_Rx, &isInFovRx, &rxNp_orientation);
		//check for Tx at new NP coordinates
		getSensorFOV(stateOfRx->txId, rad2deg * Vehicle.Yaw, limAngle_Tx, &txDirection);
		checkNPinFOV(tempCoord, SensData[RxState_tof->txId].Fr0_position_Sens, limAngle_Tx, &isInFovTx, &txNp_orientation);

		/*if dist to NP is larger than sensor range -- it's not in FOV*/
		Rx_dist		= VEC_Abs(RxState_tof->coord_RxFr0_updated, tempCoord);
		Tx_dist		= VEC_Abs(SensData[RxState_tof->txId].Fr0_position_Sens, tempCoord);
		isInFovRx	= (Rx_dist > SensParam[stateOfRx->rxId].Range) ? false : isInFovRx;
		isInFovTx	= (Tx_dist > SensParam[stateOfRx->txId].Range) ? false : isInFovTx;
		isInFov		= isInFovRx && isInFovTx;

		/*additional checks*/
		if (isInFov) {
			double xNpSens = tempCoord[0] - virtualRx[0];
			double yNpSens = tempCoord[1] - virtualRx[1];
			getAngleQuadrant(&xNpSens, &yNpSens, &sens2NP_orient);
			/*macro - correct angles*/
			CHECKAZIMUTH(sens2NP_orient);

			/*also check if it's normal to virt sensor, i.e. if tile is facing ego*/
			check_equal_normals(k, stateOfRx->TOid, &sens2NP_orient, &equalNormals);

			/*check if the NP is on current tile -- non NP tiles can be normal to USS- and generate reflection?*/
			intx2polygoAnglesSum(tempCoord, k, Obj, &onTile, &polygTolerance, &nearPolyg);

			ReflState = equalNormals && (onTile || nearPolyg);
			if (!ReflState)
			/*check if it's a coner and is reflecting in Rx direction*/
				check_for_corners(stateOfRx->TOid, rxNp_orientation, virtualRx,
									tempCoord, k, ReflState, &doCornerMapping);
		}

		/*is in FoV is true only if the tile is facing the sensor or the corner reflects towards Rx*/
		isInFov = isInFov && (ReflState || doCornerMapping);

		/*store data if distance to np is smallest*/
		if ((tempDist < finalDist) && isInFov) {
			finalDist						 = tempDist;
			stateOfRx->NpInFov_updated		 = isInFov;
			stateOfRx->RxDirection_corrected = rxDirection;
			stateOfRx->Rx2Np_orientation	 = rxNp_orientation;
			stateOfRx->Tx2Np_orientation	 = txNp_orientation;
			stateOfRx->npIsUpdated			 = true;
			CHECKAZIMUTH(stateOfRx->Rx2Np_orientation);
			VEC_Assign2D(stateOfRx->limFoVAngle_corrected, limAngle_Rx);
			VEC_Assign(stateOfRx->nearestPoint_Coord, tempCoord);
		}
	}
   /**********/
}

void
SensorRxState_afterToF(tRxState_tof *RxState){
/*calculates Rx corrected position after movement during ToF
calls NP correction function*/

	double tof1, tof2, lof1, lof2, egoFr1Moved[3];
	double vehYaw_moved_ToF1, vehYaw_moved_ToF2; 
	double dist_tof1, dist_tof2;
	double coordRx_ToF1_Fr0[3], coordRx_ToF2_Fr0[3];

	/************ do initial correction*/
	lof1 = VEC_Abs(SensData[RxState->txId].Fr0_position_Sens, RxState->nearestPoint_Coord) + 
							VEC_Abs(RxState->coord_RxFr0_FirstToF, RxState->nearestPoint_Coord);
	tof1 = 2 * (lof1 / (2 * DistanceToTofFactor(Env.Temperature))) / 1000; //convert ms to s

	//ego Fr1 origin position after tof1 
	VEH_COORD_MOVEINTOF(egoFr1Moved, Vehicle.Fr1A.t_0, Vehicle.Fr1A.v_0[0], Vehicle.Fr1A.v_0[1], tof1);
	vehYaw_moved_ToF1 = Vehicle.Yaw + Vehicle.YawRate * tof1;

	//calculate Rx postition in Fr0 after movement, from new veh Fr1 sensor coord in FR0 and old sensor Fr1 frame coordinates
	convertFr1toFr0(egoFr1Moved, &vehYaw_moved_ToF1, RxState->Fr1Rx_start, coordRx_ToF1_Fr0);
	
	//Rx distance traveled for tof1
	dist_tof1 = VEC_Abs(SensData[RxState->rxId].Fr0_position_Sens, coordRx_ToF1_Fr0);
	/************/
	
	/*output*/
	VEC_Assign(RxState->coord_RxFr0_updated, coordRx_ToF1_Fr0);
	RxState->egoYaw_Final = vehYaw_moved_ToF1;

	/*calculate new reflection point*/
	recalculateNP_postToF(RxState);

	/*********** get ToF2 and get difference
	check if the new ToF for new Rx position places Rx too far from the initial ToF*/
	//calculate updated ToF
	lof2 = VEC_Abs(SensData[RxState->txId].Fr0_position_Sens, RxState->nearestPoint_Coord) + VEC_Abs(coordRx_ToF1_Fr0, RxState->nearestPoint_Coord);
		
	tof2 = 2 * (lof2 / (2 * DistanceToTofFactor(Env.Temperature))) / 1000; //convert ms to s

	//ego Fr1 origin position after tof2
	VEH_COORD_MOVEINTOF(egoFr1Moved, Vehicle.Fr1A.t_0, Vehicle.Fr1A.v_0[0], Vehicle.Fr1A.v_0[1], tof2);
	vehYaw_moved_ToF2 = Vehicle.Yaw + Vehicle.YawRate * tof2;

	//calculate Rx postition in Fr0 after movement for ToF2
	convertFr1toFr0(egoFr1Moved, &vehYaw_moved_ToF2, RxState->Fr1Rx_start, coordRx_ToF2_Fr0);

	//ego distance traveled for tof2
	dist_tof2 = VEC_Abs(SensData[RxState->rxId].Fr0_position_Sens, coordRx_ToF2_Fr0);
	/***********/

	/*output*/
	RxState->distDiff		= dist_tof2 - dist_tof1;
	RxState->correctedToF	= tof2 * 1000; //in ms
	RxState->correctedLof	= lof2;
}

void 
check_equal_normals(int k, int i, double *npDirection, int *ReflState) {

	double MaxNPDirDeviation = 1; //in deg

	/* Get normal vector's direction relative to Fr0 */
	env_vector->NormalDir[k] = rad2deg * acos(VEC_Scalar2D(env_vector->NormalDirDefault, TrfObj[i].normals[k])
		/ (VEC_Norm2D(env_vector->NormalDirDefault) * VEC_Norm2D(TrfObj[i].normals[k])));

	if (TrfObj[i].normals[k][1] < 0)
		env_vector->NormalDir[k] = env_vector->NormalDir[k] * -1;

	/* Calculate result of normals + NearestPointDir (if 180 then 'ortho') */
	env_vector->NormalsCombined[k] = fabs(env_vector->NormalDir[k] - *npDirection);

	/* Check if normals are equal */
	if (env_vector->NormalsCombined[k] >= (180 - MaxNPDirDeviation)
		&& env_vector->NormalsCombined[k] <= (180 + MaxNPDirDeviation))
		*ReflState = 1;
}


void
check_for_corners(int i, double npOrientation, double *ussPosition,  double *npCoordTemp, int k, int ReflState, int *doCornerMapping) {
/*checks if corners should be mapped as reflection points: */
	double	MaxBoundaryPointDeviation = 0.02; // in m
	bool	reflectCorner = false;
	tTrafficObj *Obj = Traffic_GetByTrfId(i);
	
	*doCornerMapping = 0;

	/*check if it's a corner*/
	if ((abs(Obj->Envelope.Areas_0[k].z[2]) != 1) && // Check if normal vector points in z-direction for cubic TOs (surface No. 5 & 6)
		(npCoordTemp[0] >= TrfObj[i].boundary_points[k][0] - MaxBoundaryPointDeviation
			&& npCoordTemp[0] <= TrfObj[i].boundary_points[k][0] + MaxBoundaryPointDeviation)
		&& (npCoordTemp[1] >= TrfObj[i].boundary_points[k][1] - MaxBoundaryPointDeviation
			&& npCoordTemp[1] <= TrfObj[i].boundary_points[k][1] + MaxBoundaryPointDeviation)
		&& ReflState == 0) {

		/*check corner reflectivity*/
		//with &k should always find corner1, with &npSurfNumber can be corner0 or 1- see checkCorner function 
		checkCorner(Obj, &k, npOrientation, i, &MaxBoundaryPointDeviation, &reflectCorner, ussPosition, npCoordTemp);
		
		if (reflectCorner)
			*doCornerMapping = 1;
	}
}

void
rxCorrection_egoMotion(int TOid, int Tx_ID, int Rx_ID, int k) {

	int maxOptimSteps	= 20;
	bool EchoTypeDirect = (Tx_ID == Rx_ID) ? 1 : 0;
	bool NP_notUpdated = true;

	memset(RxState_tof, 0, sizeof(tRxState_tof));

	RxState_tof->rxId			= Rx_ID;
	RxState_tof->txId			= Tx_ID;
	RxState_tof->startNP_Tile	= k;
	RxState_tof->TOid			= TOid;
	
	/* Rx position in Fr1 frame - always the same*/
	convertFr0_toFr1_3D(RxState_tof->Fr1Rx_start, SensData[Rx_ID].Fr0_position_Sens);

	/*do Rx movement correction for direct echoes*/
	//if (EchoType == 0) {

	/*calculate initial movement correction*/
	//set initial state
	if (EchoTypeDirect)	
		VEC_Assign(RxState_tof->initialNP_coord, TrfObj[TOid].NearestPointPos_Fr0[Tx_ID]);
	else{				//or cross echoes
		int ussRight, ussLeft, vUssID;//real sensors to the right and left of Tx, and virtual sensor ID
		ussLeft		= (Tx_ID == 0) ? (nOS - 1) : (Tx_ID - 1);
		ussRight	= (Tx_ID == (nOS - 1)) ? 0 : (Tx_ID + 1);
		vUssID		= (Rx_ID == ussLeft) ? Rx_ID : Tx_ID;
		VEC_Assign(RxState_tof->initialNP_coord, TrfObj[TOid].vNearestPointPos_Fr0[vUssID]);
	}

	/*for debug only*/
	RxState_tof->initialLof = VEC_Abs(SensData[RxState_tof->txId].Fr0_position_Sens, RxState_tof->initialNP_coord) +
								VEC_Abs(SensData[RxState_tof->rxId].Fr0_position_Sens, RxState_tof->initialNP_coord);
	RxState_tof->initialToF = 2 * (RxState_tof->initialLof / (2 * DistanceToTofFactor(Env.Temperature))); //in ms
	
	//do first correction 
	VEC_Assign(RxState_tof->nearestPoint_Coord, RxState_tof->initialNP_coord);
	VEC_Assign(RxState_tof->coord_RxFr0_FirstToF, SensData[Rx_ID].Fr0_position_Sens);
	SensorRxState_afterToF(RxState_tof);

	/*if the correction is not within the expected range, iterate to optimize*/
	int iSteps = 0;
	while ((abs(RxState_tof->distDiff) >= _MOVEMENT_CORRECTION_ERROR_) && (iSteps < maxOptimSteps)) {
		/*recalculate tof1 and 2 until Rx position estimation is within  _MOVEMENT_CORRECTION_ERROR_ = 1cm*/
		VEC_Assign(RxState_tof->coord_RxFr0_FirstToF, RxState_tof->coord_RxFr0_updated);
		SensorRxState_afterToF(RxState_tof);
		++iSteps;
	}
		
	/*kepp the old NP if it was not updated
		i.e., if during ToF the NP position moves out of the FoV of Tx due to Rx movement
		alternative: remove the reflection if NP is not updated-- not implemented*/
	NP_notUpdated = !RxState_tof->npIsUpdated;
	if (NP_notUpdated) {
		/*check if NP is the FOV of Rx and Tx at new position */
		bool	isInFovRx = false, isInFovTx = false;
		double  limAngle_Tx[2], txDirection, txNp_orientation;
		double  limAngle_Rx[2], rxDirection, rxNp_orientation;
		double	Rx_dist, Tx_dist;

		//check for Rx at updated position, yaw and new NP coordinates
		getSensorFOV(RxState_tof->rxId, rad2deg * RxState_tof->egoYaw_Final, limAngle_Rx, &rxDirection);
		checkNPinFOV(RxState_tof->initialNP_coord, RxState_tof->coord_RxFr0_updated, limAngle_Rx, &isInFovRx, &rxNp_orientation);
		//check for Tx at new NP coordinates
		getSensorFOV(RxState_tof->txId, rad2deg * Vehicle.Yaw, limAngle_Tx, &txDirection);
		checkNPinFOV(RxState_tof->initialNP_coord, SensData[RxState_tof->txId].Fr0_position_Sens, limAngle_Tx, &isInFovTx, &txNp_orientation);

		/*if dist to NP is larger than sensor range -- it's not in FOV*/
		Rx_dist		= VEC_Abs(RxState_tof->coord_RxFr0_updated, RxState_tof->initialNP_coord);
		Tx_dist		= VEC_Abs(SensData[RxState_tof->txId].Fr0_position_Sens, RxState_tof->initialNP_coord);
		isInFovRx	= (Rx_dist > SensParam[RxState_tof->rxId].Range) ? false : isInFovRx;
		isInFovTx	= (Tx_dist > SensParam[RxState_tof->txId].Range) ? false : isInFovTx;
			  
		/*map data to structure for movement corection: new Rx but old NP position*/
		RxState_tof->NpInFov_updated		= isInFovRx && isInFovTx;
		RxState_tof->RxDirection_corrected	= rxDirection;
		RxState_tof->Rx2Np_orientation		= rxNp_orientation;
		RxState_tof->Tx2Np_orientation		= txNp_orientation;
		RxState_tof->npIsUpdated			= false;
		CHECKAZIMUTH(RxState_tof->Rx2Np_orientation);
		VEC_Assign2D(RxState_tof->limFoVAngle_corrected, limAngle_Rx);
		/*assign initial NP coordinates*/
		VEC_Assign	(RxState_tof->nearestPoint_Coord, RxState_tof->initialNP_coord);

	}

	/*for debug/validation*/
	//initialToF;								
	//correctedToF;
	//initialLof;
	//correctedLof;
	RxState_tof->isDirectEcho		= (int)EchoTypeDirect; 
	RxState_tof->egoVelocity		= sqrt(pow(Vehicle.Fr1A.v_0[0], 2) + pow(Vehicle.Fr1A.v_0[1], 2));
	RxState_tof->Rx_displacement	= VEC_Abs(RxState_tof->coord_RxFr0_updated, SensData[Rx_ID].Fr0_position_Sens);
	RxState_tof->NP_displacement	= VEC_Abs(RxState_tof->initialNP_coord, RxState_tof->nearestPoint_Coord);
	RxState_tof->vehYaw				= Vehicle.Yaw;

}

void
outputToOutline() {
	/*output traffic object outline for mapping to DAV- see Init_Clean*/
	tTrafficObj *Obj;
	bool isCuboid;

	if (Traffic.nObjs > 0) {

		//for all traffic objects
		for (int i = 0; i < Traffic.nObjs; i++) {
			Obj							= Traffic_GetByTrfId(i);
			isCuboid					= Obj->Cfg.Envelope.Mode == EnvelopeMode_Cuboid;
			ToDatacontour[i].ToID		= Obj->Cfg.Id;
			ToDatacontour[i].ToHeight	= Obj->Cfg.h;
			ToDatacontour[i].nDataP		= (isCuboid) ? 4 : (Obj->Cfg.Envelope.nArea);
			//strcpy(ToDatacontour[i].ToName, Obj->Cfg.Name);
			//strcpy(ToDatacontour[i].ToType, Obj->Cfg.Info);
			//for all tiles
			for (int k = 0; k < Obj->Cfg.Envelope.nArea; k++) {

				//for cuboids skip top or bottom tile  
				if (isCuboid && (k == 4 || k == 5)) continue;
				ToDatacontour[i].Point0xyEdge[k][0] = Obj->Envelope.Areas_0[k].Edge[0][0];
				ToDatacontour[i].Point0xyEdge[k][1] = Obj->Envelope.Areas_0[k].Edge[0][1];
			}//for k
		}//for i
	}//if
}