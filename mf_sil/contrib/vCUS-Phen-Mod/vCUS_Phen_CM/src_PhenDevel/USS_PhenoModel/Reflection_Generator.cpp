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
#include "ReflectionFunc.h"
#include "USSMathUtils.h"
#include <Vehicle\Sensor_Object.h>

extern int nTC, nOS;
extern double null_vec_2D[2], null_vec_3D[3];


int
CheckOcclusion(int n, int j, tTrafficObj *Obj, double *np2SensPara1, double *np2SensPara2, double *NPcoordinates)
{
	int		OccFlag = 0;
	double	angleAcc = 0.0001;//in polygon angle accuracy
	bool	nearTile = false;

	/* Walk through all TOs (n) that are located shorter than the current target (int_var->targ_ID) */
	for (int l = 0; l < n; l++) {
		// Looped TO-ID			-> int_var->TO_ID[l]
		// Current Target-ID	-> int_var->targ_ID
		// Current TO-ID		-> int_var->Detc_TO_ID[int_var->targ_ID]
		/* Target = TO that gives a reflection | TO = Any TO */

		/*plane parameters: ax+by+cz+d=0; see math.stackexchange.com/questions/2686606*/
		double planeVec[4] = { 0 }, lineOnPlane[2], intersectP[3];
		bool noIntPoint = true, isOnLine = false, isInPlane = false;

		Obj = Traffic_GetByTrfId(int_var->TO_ID[l]);

		//for each object, go through all surface areas 
		for (int k = 0; k < Obj->Cfg.Envelope.nArea; k++)
		{
			//TO surface plane parametrization, plane defined by Edge[1,2,3][];
			planeParams(planeVec, k, Obj);

			//check if line lies on the plane
			lineOnPlane[0] = planeVec[0] * SensData[j].Fr0_position_Sens[0]
				+ planeVec[1] * SensData[j].Fr0_position_Sens[1]
				+ planeVec[2] * SensData[j].Fr0_position_Sens[2] + planeVec[3];
			lineOnPlane[1] = planeVec[0] * NPcoordinates[0]
				+ planeVec[1] * NPcoordinates[1]
				+ planeVec[2] * NPcoordinates[2] + planeVec[3];
			
			/*check if both sensor and NP are on the TO surrface plane*/
			if ((abs(lineOnPlane[0]) + abs(lineOnPlane[1])) >= ZERODIFF)
				//find the point of intersection
				planeLineIntx(np2SensPara1, np2SensPara2, planeVec, intersectP, &noIntPoint);
			else {
				continue;
			}
			/*this is an exception, is there occlusion if sens2NP line is in TO surface plane !?
				should come up as intersection with the neighbouring TO surface plane in intx2polygoAnglesSum
				one angle is 180 and sum is 360
				*/

			if (noIntPoint)
				OccFlag = 0;
			else {
				/*check if intersection falls between sensor and NP*/
				if (INLIMITS(SensData[j].Fr0_position_Sens[0], NPcoordinates[0], intersectP[0]) &&
					INLIMITS(SensData[j].Fr0_position_Sens[1], NPcoordinates[1], intersectP[1]) &&
					INLIMITS(SensData[j].Fr0_position_Sens[2], NPcoordinates[2], intersectP[2]))
					isOnLine = true;
				else
					isOnLine = false;
			}

			if (isOnLine)
				/*check if point is on TO surface defined by ABCD */
				intx2polygoAnglesSum(intersectP, k, Obj, &isInPlane, &angleAcc, &nearTile);

			//isInPlane same with state 
			if (isInPlane) {
				OccFlag = 1;
			}

			if (OccFlag == 1)
				break;
		}
		if (OccFlag == 1)
			break;
	}


	return OccFlag;
}


void
vCUS_ExtractTrafInf(int j, tTrafficObj *Obj, int *ToIds_inRange, int *pnt_nTos)
{
	int leftUSS, rightUSS;
	
	/*for bounding box & in range test*/
	//double minXY_TO[2], maxXY_TO[2];
	bool inRange;
	
	leftUSS = (j == 0) ? (nOS - 1) : (j - 1);	/*counterclockwise sensor*/
	rightUSS = (j == (nOS - 1)) ? 0 : (j + 1);	/*clockwise sensor*/

	/*reset n TOs in range*/
	*pnt_nTos		= 0;

	/* Derive global sensor positions */
	getGlobalSensorPosition(j);
	/*get next and previous sensor position for virtual NP directions relative to neighboring sensors */
	getGlobalSensorPosition(leftUSS);
	getGlobalSensorPosition(rightUSS);

	/* Get information out of traffic object API */
	if (Traffic.nObjs > 0) {
		for (int i = 0; i < Traffic.nObjs; i++) {

			/*function called for once for real sensor: reset left and right virtual sensors*/
			TrfObj[i].vNpExists[j] = false;
			TrfObj[i].vNpExists[leftUSS] = false;
			
			/* Get Traffic object information */
			Obj = Traffic_GetByTrfId(i);
			/* Get information for sensor and TO */
			//tObjectSensor *pOS = ObjectSensor_GetByIndex(j);
			tObjectSensorObj *pOSO = ObjectSensor_GetObject(j, i);

			/* Check if TO within sensor range: the CM way*/
			if ((!SimParameters->SWSwitch_CustomInRange) && (pOSO->dtct == 0))
				continue;
			else if (SimParameters->SWSwitch_CustomInRange) {
				/*custom in-range detection: do this outside CM, here/**/
				double maxXY_TO[2] = { -HUGE_VAL, -HUGE_VAL };
				double minXY_TO[2] = { HUGE_VAL, HUGE_VAL };
				inRange = false;

				for (int k = 0; k < Obj->Cfg.Envelope.nArea; k++) {						// k = surface counting number
					/*get min max of TO x & y for bounding box, check only bottom right of tile*/
					//max
					if (Obj->Envelope.Areas_0[k].Edge[1][0] > maxXY_TO[0])
						maxXY_TO[0] = Obj->Envelope.Areas_0[k].Edge[1][0];// x coordinate
					if (Obj->Envelope.Areas_0[k].Edge[1][1] > maxXY_TO[1])
						maxXY_TO[1] = Obj->Envelope.Areas_0[k].Edge[1][1];// y coordinate

					//min
					if (Obj->Envelope.Areas_0[k].Edge[1][0] < minXY_TO[0])
						minXY_TO[0] = Obj->Envelope.Areas_0[k].Edge[1][0];// x coordinate
					if (Obj->Envelope.Areas_0[k].Edge[1][1] < minXY_TO[1])
						minXY_TO[1] = Obj->Envelope.Areas_0[k].Edge[1][1];// y coordinate
				}

				/*if TO is v low, skip the rest*/
				if (Obj->Cfg.h < SimParameters->min_height_TO)
					continue;

				checkTO_inRange(minXY_TO, maxXY_TO, j, &inRange);
				TrfObj[i].inRange[j] = inRange;
				if (!inRange)
					continue;

				/*this should not happen: false negative; false positive ok at this point*/
				if ((int(inRange) == 0) && (pOSO->dtct == 1))
					LogWarnF(EC_General, "TO is %d; USS is %d; Custom is: %d; CM is: %d, uss x is %2.5f\n", i, j, inRange, pOSO->dtct, SensData[j].Fr0_position_Sens[0]);
			}/*else if CustomInRange*/

			/* Extract 2D-information & safe into struct */
			for (int k = 0; k < Obj->Cfg.Envelope.nArea; k++) {										// k = surface counting number
				/* Assigne basic information */
				TrfObj[i].boundary_points[k][0] = Obj->Envelope.Areas_0[k].Edge[1][0];		// Assigne bottom-right TO-surface-point to global struct -> x
				TrfObj[i].boundary_points[k][1] = Obj->Envelope.Areas_0[k].Edge[1][1];		// Assigne bottom-right TO-surface-point to global struct -> y
				TrfObj[i].normals[k][0] = Obj->Envelope.Areas_0[k].z[0];			// Normal of TO-surface -> x-Compontent
				TrfObj[i].normals[k][1] = Obj->Envelope.Areas_0[k].z[1];			// Normal of TO-surface -> y-Compontent

				/* Position Vector between Sensor's Origin in Fr0 & TO boundary points in Fr0 [bottom-right] */
				env_vector->PosVector[0] = TrfObj[i].boundary_points[k][0]	// Surface x-elem
					- SensData[j].Fr0_position_Sens[0];						// Sensor x-elem
					// x-Coord of Vector to Bound_Point
				env_vector->PosVector[1] = TrfObj[i].boundary_points[k][1]	// Surface y-elem
					- SensData[j].Fr0_position_Sens[1];						// Sensor y-elem
					// y-Coord of Vector to Bound_Point
			}

			if ((!SimParameters->SWSwitch_CustomInRange) && (Obj->Cfg.h < SimParameters->min_height_TO))
				/*if custom in-range is used, this is skipped before the Extract 2D-information loop*/
				continue;/*skip if CM in-range is used and TO heighs is low*/

			/*make a list of in range TOs*/
			ToIds_inRange[*pnt_nTos] = i;
			*pnt_nTos = *pnt_nTos + 1;
		}
	}

	return;
}


void
vCUS_ReflectionEval(int j, tTrafficObj *Obj, int EchoType, int *pCnt, int *ToIds_inRange, int* nToInRange)
{
	int		ReflState = 0;
	double	Vec = 0;
	double	Vec2 = 0;
	double	r[2] = { 0 };
	int		r_stat[2] = { 0 };
	double	tol = 0.01;
	bool	isOnSurf = false;
	double	npCoord[3] = { 0 };
	/*should replace CM NPs*/
	double	npCoordTemp[3] = { 0 };
	int		inRange, i;

	double	npSurfPoly[4][3] = { 0 }; //3 coordinates for each of the 4 points of the NP surface
	/*precision of NP on surface computation - angle in radian min 0.05*/
	double	preciz = 0.1;//.1
	bool	outOfFOV = true;
	int		npSurfNumber; // which TO surface contains the NP
	double	NPdirection, distToNP;
	tObjectSensorObj *pOSO;

	/*set intvar once to zero when func is called first for direct echoes*/
	if (EchoType == 0) 
		memset(int_var, 0, sizeof(tInt_var));

	if (*nToInRange > 0) {
		for (int iTo = 0; iTo < *nToInRange; iTo++) {
			i = ToIds_inRange[iTo];

			/* Get Traffic object information */
			Obj = Traffic_GetByTrfId(i);

			/*obj sensor for in-range check*/
			/*if (EchoType == 0)
				pOSO = ObjectSensor_GetObject(j, i);*/
			
			/*reset NP distance for each TO*/
			distToNP = HUGE_VAL;
			/*and reflection state - for corner check*/
			ReflState = 0;

			/*calculates TO nearest point outside of CM*/
			nearestP_custom(Obj, j, &npSurfNumber, npCoordTemp, &distToNP, &NPdirection, &outOfFOV, EchoType);

			if (outOfFOV) {
				if (EchoType == 1)
					TrfObj[i].vNpExists[j] = !outOfFOV;//if out of FOV, vNP does not exist
				continue;
			}

			if (EchoType == 0) {
				if (SimParameters->SWSwitch_CustomInRange)
					inRange = TrfObj[i].inRange[j];
				else if (!SimParameters->SWSwitch_CustomInRange) {
					pOSO = ObjectSensor_GetObject(j, i);
					inRange = pOSO->dtct;
				}
			}
			else if (EchoType == 1)
				inRange = !outOfFOV;

			if (EchoType == 1) {
				VEC_Assign(TrfObj[i].vNearestPointPos_Fr0[j], npCoordTemp);
				TrfObj[i].vNpSurface[j] = npSurfNumber;
				TrfObj[i].vNpExists[j] = false;//becomes true in normality check, see below
				TrfObj[i].vNearestPointDir_Fr0[j] = NPdirection;

				/* mapping vNP to sensor in
				vCUS_ReflectionGenerator after vNP creation for adjacent virtual sensors*/
			}
			else if (EchoType == 0) {
				VEC_Assign(TrfObj[i].NearestPointPos_Fr0[j], npCoordTemp);
				TrfObj[i].NearestPointDir_Fr0[j] = NPdirection;
			}

			/* (1) Check for surface npSurfNumber*/
			//double	MaxNPDirDeviation = 1; //in deg
			bool	onTile, nearPolyg = false;
			double	polygDeviation = 0.05;
			double  npDirection;

			/*check only current tile and adjacent ones, particular case for cuboid which has 6 tiles
			with 4 and 5 horizontal (excluded from NP generation)*/
			int nextSurf, prevSurf;
			bool isNotCuboid	= Obj->Cfg.Envelope.Mode == EnvelopeMode_2DContour;
			bool isCuboid		= Obj->Cfg.Envelope.Mode == EnvelopeMode_Cuboid;

			if (isCuboid && npSurfNumber == 0)
				prevSurf = 3;
			else if (isNotCuboid && npSurfNumber == 0)
				prevSurf = Obj->Cfg.Envelope.nArea - 1;
			else
				prevSurf = npSurfNumber - 1; 

			/*if TO is cuboid, nArea == 6, do not look at top and bottom surfaces: 4 & 5*/
			nextSurf = (isCuboid && npSurfNumber == 3) ? 0 :
										((npSurfNumber + 1) % Obj->Cfg.Envelope.nArea);
			
			int k, checkTiles[] = { prevSurf, npSurfNumber,  nextSurf };
			int equalNormals;

			if (EchoType == 1)
				npDirection = TrfObj[i].vNearestPointDir_Fr0[j];
			else if (EchoType == 0)
				npDirection = TrfObj[i].NearestPointDir_Fr0[j];

			//for (int k = 0; k < Obj->Cfg.Envelope.nArea; k++) {// loop checking all tiles - not in use
			for (int p = 0; p < 3; p++) { /*only check current and adjacent tiles, in case NP is on corner with a tile normal to sensor*/
				k				= checkTiles[p];
				onTile			= false;
				equalNormals	= 0;

				/*check if normals are equal*/
				check_equal_normals(k, i, &npDirection, &equalNormals);

				/*check if the NP is on current tile -- non NP tiles can be normal to USS- and generate reflection?*/
				intx2polygoAnglesSum(npCoordTemp, k, Obj, &onTile, &polygDeviation, &nearPolyg);
				
				if (equalNormals && inRange == 1 && onTile) {
				/*map the data*/

					if (EchoType == 0) {
						TrfObj[i].NPisCorner[j] = false;

						/*here re-calculate Rx (not yet NP position) to correct for ego motion; direct echoes*/
						if (SimParameters->SWSwitch_RxCorection_ToF)
							rxCorrection_egoMotion(i, j, j, k);

						intvar_Mapping(i, j, j, k, *pCnt);
						if (*pCnt < _NO_OF_ECHOES_MAX_)//if larger the func exits before mapping -do not increase count
							(*pCnt)++;

						ReflState = 1;
					}
					else if (EchoType == 1) {
						TrfObj[i].virtNPisCorner[j] = false;
						TrfObj[i].vNpExists[j] = true;
						ReflState = 1; //check if this is ok
					}
				}
			}

			if (ReflState) continue;

			/* (2) Check for corners */
			double	MaxBoundaryPointDeviation = 0.02; // in m
			bool	reflectCorner = false;
			int		doCornerMapping;
			double	ussPosition[3];
			//double	tempOrientation = (EchoType == 0) ? TrfObj[i].NearestPointDir_Fr0[j] : TrfObj[i].vNearestPointDir_Fr0[j];
			if (EchoType == 0) {
				VEC_Assign(ussPosition, SensData[j].Fr0_position_Sens);
			}
			else if (EchoType == 1) {
				VEC_Assign(ussPosition, SensData[j].Fr0_position_vSens);
			}

			//for (int k = 0; k < Obj->Cfg.Envelope.nArea; k++) {
			for (int p = 0; p < 3; p++) { /*only check current and adjacent tiles*/
				k = checkTiles[p];
				doCornerMapping = 0;
				if (inRange)//check if it's a corner and if it is reflecting
					check_for_corners(i, npDirection, ussPosition, npCoordTemp, k, ReflState, &doCornerMapping);

				if ((doCornerMapping) && (EchoType == 0)) {
					TrfObj[i].NPisCorner[j] = true;
			
					/*here re-calculate Rx position to correct for for ego motion, keep NP on the corner; direct-echoes*/
					if (SimParameters->SWSwitch_RxCorection_ToF)
						rxCorrection_egoMotion(i, j, j, k);

					intvar_Mapping(i, j, j, k, *pCnt);
					if (*pCnt < _NO_OF_ECHOES_MAX_)//if larger the func exits before mapping-do not increase count
						(*pCnt)++;
					// abort loop after one refl point has been found 
					break;
				}
				else if ((doCornerMapping) && (EchoType == 1)) {
					TrfObj[i].virtNPisCorner[j] = true;
					TrfObj[i].vNpExists[j] = true;
					//if vNpExists, abort loop, mapping done in vCUS_ReflectionGenerator
					break;
				}
				
			}
		
		} // End of traffic-loop (nTC)
	} // if-case (nTC>0) [EVALUATION]

	return;
}

int
vCUS_OcclusionEval(int j, tTrafficObj *Obj, int *ToIds_inRange, int *nToClose)
{
	/* initialize everything with 0 */
	SensData[j].Target = {0};

	//tObjectSensor		*pOS = ObjectSensor_GetByIndex(j);
	tObjectSensorObj	*pOSO;
	double	min, NPregular[3];
	double	np2UssPara1[3], np2UssPara2[3];
	double	DistToCurrNP, z0Np[3];
	int		occCrsEcho, OccFlag, cnt, n, m;
	int		EchoType, OccFlagZ0, OccFlagCrsEchoZ0;
	bool	isCrsEcho, inRange;

	cnt = 0;
	occCrsEcho = 1;

	memset(int_var->TO_ID, -1, _NO_TOS_MAX_ * sizeof(int));

	/* Walk through found targets */
	/*including for cross-echoes (virtual NPs: Rx != Tx)*/
	for (int k = 0; k < int_var->noTargets; k++) {
		OccFlag = 0;
		min = HUGE_VAL;

		/* Determine next shortest (REFLECTING) target to sensor j */
		/*including cross-echoes (virtual NPs: Rx != Tx)*/
		for (int l = 0; l < int_var->noTargets; l++) {
			if (int_var->targ_distance[l] < min) {
				min = int_var->targ_distance[l];
				int_var->targ_ID = l;
			}
		}

		/*check if current targed is NP from cross-echo*/
		if (int_var->Tx[int_var->targ_ID] == int_var->Rx[int_var->targ_ID]) {
			isCrsEcho = false;
			EchoType = 0;
		}
		else if (int_var->Tx[int_var->targ_ID] != int_var->Rx[int_var->targ_ID]) {
			isCrsEcho = true;
			EchoType = 1;
		}

		/* Determine all TOs with shorter distance between sensor j & current target */
		/* Consider also TOs which DO NOT have any Refelction Point */
		/* All TOs -> TrfObj[TOid].NearestPointPos_Fr0[SensID] */
		/* Reflecting TO to check -> int_var->Fr0_position_NP[int_var->targ_ID] */
		n = 0;
		for (int p = 0; p < *nToClose; p++) {
			m = ToIds_inRange[p];
			if (!SimParameters->SWSwitch_CustomInRange) {
				pOSO	= ObjectSensor_GetObject(j, m); // j=SensorID m=TOID
				inRange = pOSO->dtct;
			}
			else if (SimParameters->SWSwitch_CustomInRange)
				inRange = TrfObj[m].inRange[j];/*m should always be in range, it's from the list of in range TOs */
			
			DistToCurrNP = VEC_Abs(SensData[j].Fr0_position_Sens, TrfObj[m].OccNearestPointPos_Fr0[j]);

			/*use a modified copy of this loop to check for occlusion if Rx != Tx from the coordinates of Tx - see below*/
			if (int_var->Detc_TO_ID[int_var->targ_ID] != m && //skip occlusion for TO == current target 
				DistToCurrNP < (int_var->targ_distance[int_var->targ_ID] + _IN_RANGE_TOLERANCE_) /*check also obj wich are slightly behind target*/
				&& DistToCurrNP != 0
				&& inRange == 1) {
				int_var->TO_ID[n] = m; // Save TOIDs of short located TOs into array
				n++;
			}
		}

		/* Call function for checking occulsion -> */
		VEC_Assign(np2UssPara1, null_vec_3D);
		VEC_Assign(np2UssPara2, null_vec_3D);

		if (isCrsEcho) {
			VEC_Assign(NPregular, int_var->Fr0_position_NP[int_var->targ_ID]);
		}
		else if (!isCrsEcho) {
			VEC_Assign(NPregular, TrfObj[int_var->Detc_TO_ID[int_var->targ_ID]].NearestPointPos_Fr0[j]);
		}

		//first: parametrization of line from sensor to NP
		parametrzLine(np2UssPara1, np2UssPara2, j, NPregular);
		//call occlusion
		OccFlag = CheckOcclusion(n, j, Obj, np2UssPara1, np2UssPara2, NPregular);

		/* #### check for occlusion from corner formed with the ground if TO contacts the ground: z = 0*/
		Obj = Traffic_GetByTrfId(int_var->Detc_TO_ID[int_var->targ_ID]); /* Assign additional reflection point for target in case of walls/curbstones (z0 = 0) */
		OccFlagZ0 = 1;
		if (Obj->Envelope.Areas_0[int_var->Detc_TO_Surface_ID[int_var->targ_ID]].Edge[1][2] == 0) {
			/*first check that the z=0 NP is not occluded*/
			z0Np[2] = Obj->Envelope.Areas_0[int_var->Detc_TO_Surface_ID[int_var->targ_ID]].Edge[1][2];
			z0Np[1] = int_var->Fr0_position_NP[int_var->targ_ID][1];
			z0Np[0] = int_var->Fr0_position_NP[int_var->targ_ID][0];

			//!!!!!!!!!!!!!!!!!!!!!!!  - use with one sensor and one TO only
			/*temp : to visualize diffs between corrected and ucorrected NPs*/
			/*
			z0Np[2] = Obj->Envelope.Areas_0[int_var->Detc_TO_Surface_ID[int_var->targ_ID]].Edge[1][2];
			z0Np[1] = int_var->NP_uncorrected[int_var->targ_ID][1];
			z0Np[0] = int_var->NP_uncorrected[int_var->targ_ID][0];
			*/
			//!!!!!!!!!!!!!!!!!!!!!!!

			/*do occlusion*/
			//sens to z0NP parametrization - overloaded func
			parametrzLine(np2UssPara1, np2UssPara2, j, z0Np);
			//call occlusion
			OccFlagZ0 = CheckOcclusion(n, j, Obj, np2UssPara1, np2UssPara2, z0Np);
		}
		/* ####*/

		/*if it's cross echo and there is no occlusion for the receivig sensor,
				check for occlussion on the direction of emitting sensor to NP*/
		if (isCrsEcho && OccFlag == 0) {
			/*make list with TOs other than current TO, brute force approach:
				 check all TOs, NPs for Rx not calculated*/

			n = 0;
			for (int p = 0; p < *nToClose; p++) {
				m = ToIds_inRange[p];//list of TOs in range to Tx
				if (!SimParameters->SWSwitch_CustomInRange) {
					pOSO = ObjectSensor_GetObject(int_var->Rx[int_var->targ_ID], m);//check if also in range for Rx
					inRange = pOSO->dtct;
				}
				else if (SimParameters->SWSwitch_CustomInRange)
					/*Rx in range TOs are not calculated if custom in-range func is used*/
					inRange = 1;/*if custom in range func is used, take all TOs in range for Tx*/

				if (int_var->Detc_TO_ID[int_var->targ_ID] != m && //skip occlusion for TO == current target 
					inRange == 1) {
					int_var->TO_ID[n] = m; // Save TOIDs of short located TOs into array
					n++;
				}
			}

			parametrzLine(np2UssPara1, np2UssPara2, int_var->Rx[int_var->targ_ID], NPregular);
			//call occlusion
			occCrsEcho = CheckOcclusion(n, int_var->Rx[int_var->targ_ID], Obj, np2UssPara1, np2UssPara2, NPregular);
			OccFlag = (occCrsEcho == 0) ? 0 : 1;
		}

		if (OccFlag != 1) { //&& isCrsEcho
			/* Map the non-occluded reflection point for this direct/CROSS-echo to the */
			/* global SensData struct  */

			/*update all reflection data if direct echoes are on & do not map > _NO_OF_ECHOES_MAX_ echoes*/
			if ((SimParameters->SWSwitch_DirectEchoes) && (cnt < _NO_OF_ECHOES_MAX_)){
				SensData_Mapping(int_var->Rx[int_var->targ_ID], j, EchoType, cnt);
				cnt++;
			}
			else if ((!SimParameters->SWSwitch_DirectEchoes && EchoType) && (cnt < _NO_OF_ECHOES_MAX_)){
				/*if direct echoes are off, update only cross echoes data*/
				SensData_Mapping(int_var->Rx[int_var->targ_ID], j, EchoType, cnt);
				cnt++;
			}
		}

		/*check ground angle cross-reflection for TOs on the ground from the direction of Rx (if Tx is non-ocluded)*/
		OccFlagCrsEchoZ0 = 1;
		if (isCrsEcho && OccFlagZ0 == 0) {

			if (Obj->Envelope.Areas_0[int_var->Detc_TO_Surface_ID[int_var->targ_ID]].Edge[1][2] == 0) {
				/*first check that the z=0 NP is not occluded*/
				/*do occlusion*/
				//Rx to z0NP parametrization - overloaded func
				parametrzLine(np2UssPara1, np2UssPara2, int_var->Rx[int_var->targ_ID], z0Np);
				//call occlusion for Rx & z0Np
				OccFlagCrsEchoZ0 = CheckOcclusion(n, int_var->Rx[int_var->targ_ID], Obj, np2UssPara1, np2UssPara2, z0Np);
				OccFlagZ0 = (OccFlagCrsEchoZ0 == 0) ? 0 : 1;
			}
		}

		if (OccFlagZ0 != 1) {
			VEC_Assign(int_var->Fr0_position_NP[int_var->targ_ID], z0Np);// int_var->Fr0_position_NP[int_var->targ_ID]);
			//recalculate elevations for NP on the ground, azimuth is the same as NP on TO
			int_var->NPElevRx_Fr0[int_var->targ_ID] = getNpElevation(z0Np, SensData[int_var->Rx[int_var->targ_ID]].Fr0_position_Sens);
			int_var->NPElevTx_Fr0[int_var->targ_ID] = getNpElevation(z0Np, SensData[int_var->Tx[int_var->targ_ID]].Fr0_position_Sens);
			
			/*update all reflection data if direct echoes are on & do not map > _NO_OF_ECHOES_MAX_ echoes*/
			if ((SimParameters->SWSwitch_DirectEchoes) && (cnt < _NO_OF_ECHOES_MAX_)) {
				SensData_Mapping(int_var->Rx[int_var->targ_ID], j, EchoType, cnt);
				cnt++;
			}
			else if ((!SimParameters->SWSwitch_DirectEchoes && EchoType) && (cnt < _NO_OF_ECHOES_MAX_)) {
				/*if direct echoes are off, update only cross echoes data*/
				SensData_Mapping(int_var->Rx[int_var->targ_ID], j, EchoType, cnt);
				cnt++;
			}
		}

		int_var->targ_distance[int_var->targ_ID] = HUGE_VAL;
	}

	return cnt;
}


int
vCUS_ReflectionGenerator(int SensID)
{
	int EchoType = 0; // 0->Dir | 1->Cross
	int cnt = 0, i;
	int vSensID = (SensID == 0) ? (nOS - 1) : (SensID-1);

	tTrafficObj *Obj = NULL;
	//tObjectSensor *pOS = NULL;
	//tObjectSensorObj *pOSO = NULL;

	int nToInRange = 0;

	/*get a list of TO ids which are in range */
	int ToIds_inRange[_NO_TOS_MAX_];
	for(int k = 0; k < nTC; k++) ToIds_inRange[k] = 0;
	//int* ToIds_inRange = new int[nTC];

	/* Run through Object Sensors - Main Routine - */
	if (ObjectSensorCount > 0 && (SimCore.State == SCState_StartLastCycle || SimCore.State == SCState_Simulate)) {
		
		/* Extraction of Traffic information */
		vCUS_ExtractTrafInf(SensID, Obj, ToIds_inRange, &nToInRange);

		/*output traffic object outline for mapping to DAV- see Init_Clean*/
		outputToOutline();

		/* This is a sensor burst msg -> 1 and it will send no matter what (also for no reflection) */
		ReflData_Mapping(0 /* not relevant */, SensID /* Transmitter ID */, 1 /* BurstFlag -> BURST*/);

		/* Direct Echoes */
		vCUS_ReflectionEval(SensID, Obj, EchoType, &cnt, ToIds_inRange, &nToInRange);			// Evaluation REFLECTION / ORTHOGONAL / EDGE for Direct Echoes ->0

		/* Cross Echoes */

		/*SensID - bursting sensor*/
		/*check cross-reflection from both the counterclockwise and the clockwise sensor
			- ReflectionEval\nearestP_custom - creates NP for virtual sensor
											 - checks if the virtual NP(vNP) is in FOV of SensID & SensID+1
											 - calculates the distance vNP-to-SensID & vNP-to-(SensID+1): see TrfObj[i].vLoF[j]

			 vCUS_OcclusionEval - checks occlusion between vNP of vSensID and SensID
								- assigns NP to SensID (with SensID as TX and vSensID as Rx)
								- checks occlusion between vNP of vSensID-1 and SensID
								- assigns NP to SensID (with SensID as TX and vSensID-1 as Rx)

				   SensID
	   SensID-1		       SensID+1
			 ^		*      ^
			  \	   / \    /
			   \  /   \  /
				\/     \/
			````````````````````
NPfor vSensID-1 *       * NP for vSensID

		*/

		if (SimParameters->SWSwitch_CrossEchoes && nOS < 3)
			/*should also work with nOS == 2 sensors, but mind the FOV of edge sensors
			i.e. for nOS == 2 sensors the clockwise and counterclock virtual sensors will have the same FOV, not oposing/mirrored fov*/
			Log("Less than 3 sensors, skipping cross-reflections");
		else if (SimParameters->SWSwitch_CrossEchoes && cnt > 0) {
			/*cnt > 0 :  there should be some direct echoes from sensor before cross-echoes (no matter if occluded)*/
			EchoType = 1;
			
			/*for virtual USSs, use in range TOs list from real SensID USS (i.e., the Tx sensor)*/
			vCUS_ReflectionEval(SensID, Obj, EchoType, &cnt, ToIds_inRange, &nToInRange);	/*get NP from virtual clockwise virtual sensor for real sensor SensID*/
			vCUS_ReflectionEval(vSensID, Obj, EchoType, &cnt, ToIds_inRange, &nToInRange);/*get NP from virtual counter-clockwise virtual sensor for real sensor SensID*/
			
			/*assign virtual NPs (cross-echoes) to the current real sensor: Tx always SensID
			Rx is once the sensor to the left and once the sensor to the right*/
			//if (Traffic.nObjs > 0) {
			if (nToInRange > 0) {
				int rightSensID = (SensID + 1) % nOS;
				for (int p = 0; p < nToInRange; p++) {
					i = ToIds_inRange[p];
					/*map cross-echoes from virtual NPs */

					/*Cross-echo from the sensor to the left
						- virtual sensor SensID-1, real sensors: Rx = SensID-1, Tx = SensID*/
					if (TrfObj[i].vNpExists[vSensID]) {
						/*here re-calculate Rx position for vehicle movement correction*/
						if (SimParameters->SWSwitch_RxCorection_ToF)
							rxCorrection_egoMotion(i, SensID, vSensID, TrfObj[i].vNpSurface[vSensID]);

						intvar_Mapping(i, SensID, vSensID, TrfObj[i].vNpSurface[vSensID], cnt, nOS);
						if (cnt < _NO_OF_ECHOES_MAX_)//if larger the func exits before mapping-do not increase count
							cnt++;
					}

					/*Cross-echo from the sensor to the right
						- virtual sensor SensID, real sensors Rx = SensID+1, Tx = SensID*/
					if (TrfObj[i].vNpExists[SensID]) {
						/*here re-calculate Rx position for vehicle movement correction*/
						if (SimParameters->SWSwitch_RxCorection_ToF)
							rxCorrection_egoMotion(i, SensID, rightSensID, TrfObj[i].vNpSurface[vSensID]);

						intvar_Mapping(i, SensID, rightSensID, TrfObj[i].vNpSurface[SensID], cnt, nOS);
						if (cnt < _NO_OF_ECHOES_MAX_)//if larger the func exits before mapping-do not increase count
							cnt++;
					}

				}
			}
		
		}
		if (cnt > _NO_TOS_MAX_) cnt = _NO_TOS_MAX_;//a max of _NO_TOS_MAX_ are mapped in intvar_Mapping
		int_var->noTargets = cnt;//store target number
		cnt = vCUS_OcclusionEval(SensID, Obj, ToIds_inRange, &nToInRange);		// Evaluation OCCLUSISON/SORTING for Direct&Cross Echoes 
		/* Write total number of non-occluded reflections for this SFSP burst */
		SensData[SensID].Target.nReflections = cnt;
	}

	return 0;
}