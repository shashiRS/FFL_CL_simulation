/*
******************************************************************************
**  CarMaker - Version 8.1.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Functions
** ---------
**
** int vCUS_InitData (void)				-> Initialization of Signals / Quantities
** int vCUS_Cleanup (void)				-> Cleanup / free memspace
**
******************************************************************************
*/

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
#include "User.h"
#include <Vehicle\Sensor_Object.h>


extern int nTC,nOS,debug;

void
vCUS_InitReflGen(void)
{
	for (int k = 0; k < 20; k++) {
		int_var->distance[k] = 0;
		int_var->Detc_TO_ID[k] = 0;
		int_var->Detc_TO_Surface_ID[k] = 0;
		int_var->Fr0_position_NP[k][0] = 0;
		int_var->Fr0_position_NP[k][1] = 0;
		int_var->NPDir_Fr0[k] = 0;
	}
	int_var->noTargets = 0;
	int_var->ID = -1;

	env_vector->DirVector[0] = 0;
	env_vector->DirVector[1] = 0;
	env_vector->NormalDirDefault[0] = 1;
	env_vector->NormalDirDefault[1] = 0;
	for (int i = 0; i < 4; i++)
		env_vector->NormalDir[i] = 0;
	for (int i = 0; i < 4; i++)
		env_vector->NormalsCombined[i] = 0;

	return;
}

void
vCUS_ResetVarTREnd(void)
{
	for (int j = 0; j < ObjectSensorCount; j++) {
		/* tSensorData SensData */
		for (int k = 0; k < 3; k++)
			SensData[j].Fr0_position_Sens[k] = 0;
		for (int k = 0; k < 20; k++) {
			SensData[j].Fr0_position_NP[k][0] = 0;
			SensData[j].Fr0_position_NP[k][1] = 0;
			SensData[j].NPDir_Fr0[k] = 0;
			SensData[j].distance[k] = 0;
			SensData[j].Detc_TO_ID[k] = 0;
			SensData[j].Detc_TO_Surface_ID[k] = 0;
		}
		/* tSensorParam SensParam */
		SensParam[j].orientation = 0;
		for (int k = 0; k < 3; k++)
			SensParam[j].Fr1_position[k] = 0;
	}

	for (int i = 0; i < Traffic.nObjs; i++) {
		/* tTrafficObject TrfObj */
		for (int k = 0; k < 4; k++) {
			TrfObj[i].boundary_points[k][0] = 0;
			TrfObj[i].boundary_points[k][1] = 0;
			for (int l = 0; l < _NO_OF_SENSORS_MAX_; l++) {
				TrfObj[i].boundary_pointsAngles[l][k] = 0;
			}
			TrfObj[i].normals[k][0] = 0;
			TrfObj[i].normals[k][1] = 0;
		}
		for (int k = 0; k < _NO_OF_SENSORS_MAX_; k++) {
			TrfObj[i].NearestPointDir_Fr0[k] = 0;
			TrfObj[i].boundary_pointsAngles_min[k] = 0;
			TrfObj[i].boundary_pointsAngles_max[k] = 0;
		}
	}

	for (int k = 0; k < 20; k++) {
		int_var->distance[k] = 0;
		int_var->Detc_TO_ID[k] = 0;
		int_var->Detc_TO_Surface_ID[k] = 0;
		int_var->Fr0_position_NP[k][0] = 0;
		int_var->Fr0_position_NP[k][1] = 0;
		int_var->NPDir_Fr0[k] = 0;
	}
	int_var->noTargets = 0;
	int_var->ID = -1;

	env_vector->DirVector[0] = 0;
	env_vector->DirVector[1] = 0;
	env_vector->NormalDirDefault[0] = 1;
	env_vector->NormalDirDefault[1] = 0;
	for (int i = 0; i < 4; i++)
		env_vector->NormalDir[i] = 0;
	for (int i = 0; i < 4; i++)
		env_vector->NormalsCombined[i] = 0;

	return;
}

void
vCUS_ExtractTrafInf(int j, tTrafficObj *Obj)
{
	/* Derive global sensor position */
	for (int k = 0; k < 3; k++)
		SensData[j].Fr0_position_Sens[k] = Vehicle.Fr1A.t_0[k] + SensParam[j].Fr1_position[k];

	/* Reset struct to '0' for each loop */
	for (int k = 0; k < 20; k++) {
		SensData[j].distance[k] = 0;
		SensData[j].Detc_TO_ID[k] = -1;
		SensData[j].Detc_TO_Surface_ID[k] = 0;
		SensData[j].Fr0_position_NP[k][0] = 0;
		SensData[j].Fr0_position_NP[k][1] = 0;
		SensData[j].NPDir_Fr0[k] = 0;
	}

	/* Get information out of traffic object API */
	if (Traffic.nObjs > 0) {
		for (int i = 0; i < Traffic.nObjs; i++) {

			/* Get Traffic object information */
			Obj = Traffic_GetByTrfId(i);
			if (Obj->Cfg.Envelope.nArea == 6) {
				/* Get information for sensor and TO */
				tObjectSensor *pOS = ObjectSensor_GetByIndex(j);
				tObjectSensorObj *pOSO = ObjectSensor_GetObject(j, i);

				/* Extract 2D-information & safe into struct */
				TrfObj[i].boundary_points[0][0] = Obj->Envelope.Areas_0[0].Edge[3][0];		// Point A -> x-Coordinate
				TrfObj[i].boundary_points[0][1] = Obj->Envelope.Areas_0[0].Edge[3][1];		// Point A -> y-Coordinate
				TrfObj[i].normals[0][0] = Obj->Envelope.Areas_0[0].z[0];			// Normal of line A-B -> x-Compontent
				TrfObj[i].normals[0][1] = Obj->Envelope.Areas_0[0].z[1];			// Normal of line A-B -> y-Compontent
				env_vector->DirVector[0] = TrfObj[i].boundary_points[0][0]// A_x
					- SensData[j].Fr0_position_Sens[0];//Sens_x
					// x-Coord of Vector to Bound_Point
				env_vector->DirVector[1] = TrfObj[i].boundary_points[0][1]// Ay
					- SensData[j].Fr0_position_Sens[1];//Sens_Y
					// y-Coord of Vector to Bound_Point
				TrfObj[i].boundary_pointsAngles[j][0] = rad2deg *
					acos(VEC_Scalar2D(env_vector->NormalDirDefault, env_vector->DirVector)
						/ (VEC_Norm2D(env_vector->NormalDirDefault) * VEC_Norm2D(env_vector->DirVector)));

				TrfObj[i].boundary_points[1][0] = Obj->Envelope.Areas_0[0].Edge[2][0];		// Point B -> x-Coordinate
				TrfObj[i].boundary_points[1][1] = Obj->Envelope.Areas_0[0].Edge[2][1];		// Point B -> y-Coordinate
				TrfObj[i].normals[1][0] = Obj->Envelope.Areas_0[1].z[0];			// Normal of line B-C -> x-Compontent
				TrfObj[i].normals[1][1] = Obj->Envelope.Areas_0[1].z[1];			// Normal of line B-C -> y-Compontent
				env_vector->DirVector[0] = TrfObj[i].boundary_points[1][0]// A_x
					- (Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]);//Sens_x
					// x-Coord of Vector to Bound_Point
				env_vector->DirVector[1] = TrfObj[i].boundary_points[1][1]// Ay
					- (Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]);//Sens_Y
					// y-Coord of Vector to Bound_Point
				TrfObj[i].boundary_pointsAngles[j][1] = rad2deg *
					acos(VEC_Scalar2D(env_vector->NormalDirDefault, env_vector->DirVector)
						/ (VEC_Norm2D(env_vector->NormalDirDefault) * VEC_Norm2D(env_vector->DirVector)));

				TrfObj[i].boundary_points[2][0] = Obj->Envelope.Areas_0[2].Edge[3][0];		// Point C -> x-Coordinate
				TrfObj[i].boundary_points[2][1] = Obj->Envelope.Areas_0[2].Edge[3][1];		// Point C -> y-Coordinate
				TrfObj[i].normals[2][0] = Obj->Envelope.Areas_0[2].z[0];			// Normal of line C-D -> x-Compontent
				TrfObj[i].normals[2][1] = Obj->Envelope.Areas_0[2].z[1];			// Normal of line C-D -> y-Compontent
				env_vector->DirVector[0] = TrfObj[i].boundary_points[2][0]// A_x
					- (Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]);//Sens_x
					// x-Coord of Vector to Bound_Point
				env_vector->DirVector[1] = TrfObj[i].boundary_points[2][1]// Ay
					- (Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]);//Sens_Y
					// y-Coord of Vector to Bound_Point
				TrfObj[i].boundary_pointsAngles[j][2] = rad2deg *
					acos(VEC_Scalar2D(env_vector->NormalDirDefault, env_vector->DirVector)
						/ (VEC_Norm2D(env_vector->NormalDirDefault) * VEC_Norm2D(env_vector->DirVector)));

				TrfObj[i].boundary_points[3][0] = Obj->Envelope.Areas_0[2].Edge[2][0];		// Point D -> x-Coordinate
				TrfObj[i].boundary_points[3][1] = Obj->Envelope.Areas_0[2].Edge[2][1];		// Point D -> y-Coordinate
				TrfObj[i].normals[3][0] = Obj->Envelope.Areas_0[3].z[0];			// Normal of line D-A -> x-Compontent
				TrfObj[i].normals[3][1] = Obj->Envelope.Areas_0[3].z[1];			// Normal of line D-A -> y-Compontent
				env_vector->DirVector[0] = TrfObj[i].boundary_points[3][0]// A_x
					- (Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]);//Sens_x
					// x-Coord of Vector to Bound_Point
				env_vector->DirVector[1] = TrfObj[i].boundary_points[3][1]// Ay
					- (Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]);//Sens_Y
					// y-Coord of Vector to Bound_Point
				TrfObj[i].boundary_pointsAngles[j][3] = rad2deg *
					acos(VEC_Scalar2D(env_vector->NormalDirDefault, env_vector->DirVector)
						/ (VEC_Norm2D(env_vector->NormalDirDefault) * VEC_Norm2D(env_vector->DirVector)));

				// Debug Output
				if (SimCore.State == SCState_StartLastCycle && debug == 1) {
					Log("Traffic No: %d\n", i);
					for (int j = 0; j < 4; j++) {
						Log("Point %d (x/y)->(%f/%f)\n", j, TrfObj[i].boundary_points[j][0], TrfObj[i].boundary_points[j][1]);
						Log("Point %d-%d (x/y)->(%f/%f)\n", j, (j + 1), TrfObj[i].normals[j][0], TrfObj[i].normals[j][1]);
					}
					Log("------------------\n");
				}
			}
		}
	}

	return;
}

void
vCUS_OrthoEval(int j, tTrafficObj *Obj)
{
	int cnt = 0;

	if (Traffic.nObjs > 0) {
		for (int i = 0; i < Traffic.nObjs; i++) {

			/* Get Traffic object information */
			Obj = Traffic_GetByTrfId(i);
			if (Obj->Cfg.Envelope.nArea == 6) {
				/* Get information for sensor and TO */
				tObjectSensor *pOS = ObjectSensor_GetByIndex(j);
				tObjectSensorObj *pOSO = ObjectSensor_GetObject(j, i);

				/* Calculate global angles */
				TrfObj[i].NearestPointDir_Fr0[j] = rad2deg * (Vehicle.Yaw + pOSO->NearPnt.alpha_p/*NearesPoint rotation*/) + SensParam[j].orientation/*Sensors rotation*/;

				/* For all 4 surfaces */
				for (int k = 0; k < 4; k++)
					/* Get normal vector's direction relative to Fr0 */
					env_vector->NormalDir[k] = rad2deg * acos(VEC_Scalar2D(env_vector->NormalDirDefault, TrfObj[i].normals[k])
						/ (VEC_Norm2D(env_vector->NormalDirDefault) * VEC_Norm2D(TrfObj[i].normals[k])));

				if ((rad2deg * Obj->r_zyx[2]) <= 90 && (rad2deg * Obj->r_zyx[2]) >= 0) {
					env_vector->NormalDir[0] = env_vector->NormalDir[0] * (-1);
					env_vector->NormalDir[3] = env_vector->NormalDir[3] * (-1);
				}
				else if ((rad2deg * Obj->r_zyx[2]) > 90 && (rad2deg * Obj->r_zyx[2]) <= 180) {
					env_vector->NormalDir[0] = env_vector->NormalDir[0] * (-1);
					env_vector->NormalDir[1] = env_vector->NormalDir[1] * (-1);
				}
				else if ((rad2deg * Obj->r_zyx[2]) < 0 && (rad2deg * Obj->r_zyx[2]) >= -90) {
					env_vector->NormalDir[2] = env_vector->NormalDir[0] * (-1);
					env_vector->NormalDir[3] = env_vector->NormalDir[1] * (-1);
				}
				else if ((rad2deg * Obj->r_zyx[2]) < -90 && (rad2deg * Obj->r_zyx[2]) >= -180) {
					env_vector->NormalDir[1] = env_vector->NormalDir[0] * (-1);
					env_vector->NormalDir[2] = env_vector->NormalDir[3] * (-1);
				}

				/* Check if normals are orthogonal to NearestPointDir */
				for (int k = 0; k < 4; k++) {
					/* Calculate result of normals + NearestPointDir (if 180 then 'ortho') */
					env_vector->NormalsCombined[k] = env_vector->NormalDir[k] - TrfObj[i].NearestPointDir_Fr0[j];

					// Debug Output
					if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1) {
						Log("TO_Point(%f|%f) , Sensor_Point(%f|%f), Angle between %f\n",
							TrfObj[i].boundary_points[k][0],
							TrfObj[i].boundary_points[k][1],
							(Vehicle.Fr1A.t_0[0] + SensParam[j].Fr1_position[0]),
							(Vehicle.Fr1A.t_0[1] + SensParam[j].Fr1_position[1]),
							TrfObj[i].boundary_pointsAngles[j][k]);
						Log("DirNo. %d: NormalDirection %f  -  NearestPointDirection %f  =  CombinedDirection %f\n",
							k, env_vector->NormalDir[k], TrfObj[i].NearestPointDir_Fr0[j], env_vector->NormalsCombined[k]);
					}

					if (env_vector->NormalsCombined[k] >= (180 - 0.001) && env_vector->NormalsCombined[k] <= (180 + 0.001)) {
						//Debug Output
						if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
							Log("Reflection point @ %f\n", pOSO->NearPnt.ds_p);

						/* Increase counter in case of orthogonal match of one sensor */
						cnt++;

						/* Mapping Data of one sensor */
						int_var->distance[cnt - 1] = pOSO->NearPnt.ds_p;
						int_var->Detc_TO_ID[cnt - 1] = i;
						int_var->Detc_TO_Surface_ID[cnt - 1] = k;
						int_var->Fr0_position_NP[cnt - 1][0] = SensData[j].Fr0_position_Sens[0] + pOSO->NearPnt.ds[0];
						int_var->Fr0_position_NP[cnt - 1][1] = SensData[j].Fr0_position_Sens[1] + pOSO->NearPnt.ds[1];
						int_var->NPDir_Fr0[cnt - 1] = TrfObj[i].NearestPointDir_Fr0[j];
					}
					else {
						// Debug Output
						if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
							Log("No reflection point\n");
					}
				} // End of k-loop (4)

				int_var->noTargets = cnt;
				/* Set rest of cnt's distances to '0' */
				for (int k = cnt; k < 20; k++) {
					int_var->distance[k] = 0;
					int_var->Detc_TO_ID[k] = 0;
					int_var->Detc_TO_Surface_ID[k] = 0;
					int_var->Fr0_position_NP[k][0] = 0;
					int_var->Fr0_position_NP[k][1] = 0;
					int_var->NPDir_Fr0[k] = 0;
				}

				/* Calculate min/max angles for each TO */
				TrfObj[i].boundary_pointsAngles_min[j] = TrfObj[i].boundary_pointsAngles[j][0];
				TrfObj[i].boundary_pointsAngles_max[j] = TrfObj[i].boundary_pointsAngles[j][0];
				for (int k = 1; k < 4; k++) {
					if (TrfObj[i].boundary_pointsAngles[j][k] < TrfObj[i].boundary_pointsAngles_min[j])
						TrfObj[i].boundary_pointsAngles_min[j] = TrfObj[i].boundary_pointsAngles[j][k];
					if (TrfObj[i].boundary_pointsAngles[j][k] > TrfObj[i].boundary_pointsAngles_max[j])
						TrfObj[i].boundary_pointsAngles_max[j] = TrfObj[i].boundary_pointsAngles[j][k];
				}

				if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
					Log("min=%f | max=%f\n", TrfObj[i].boundary_pointsAngles_min[j], TrfObj[i].boundary_pointsAngles_max[j]);
			} // if-case (running)
		} // End of traffic-loop (nTC)
	} // if-case (nTC>0) [EVALUATION]

	return;
}

void
vCUS_OcclusionEvalSorting(int j, tTrafficObj *Obj)
{
	int cnt = 1;

	for (int k = 0; k < int_var->noTargets; k++) {
		double min = 9999;

		/* Determine next target entry */
		for (int l = 0; l < int_var->noTargets; l++) {
			if (int_var->distance[l] < min) {
				min = int_var->distance[l];
				int_var->ID = l;
			}
		}

		if (k == 0) {
			SensData[j].distance[k] = int_var->distance[int_var->ID];
			SensData[j].Detc_TO_ID[k] = int_var->Detc_TO_ID[int_var->ID];
			SensData[j].Detc_TO_Surface_ID[k] = int_var->Detc_TO_Surface_ID[int_var->ID];
			SensData[j].Fr0_position_NP[k][0] = int_var->Fr0_position_NP[int_var->ID][0];
			SensData[j].Fr0_position_NP[k][1] = int_var->Fr0_position_NP[int_var->ID][1];
			SensData[j].NPDir_Fr0[k] = int_var->NPDir_Fr0[int_var->ID];

			int_var->distance[int_var->ID] = 9999;
		}
		else {
			/* Starting one after least filled target in list 'k' */
			for (int l = 0; l < k; l++) {
				if (TrfObj[SensData[j].Detc_TO_ID[l]].boundary_pointsAngles_min[j] > TrfObj[int_var->ID].NearestPointDir_Fr0[j] * (-1)
					&& TrfObj[SensData[j].Detc_TO_ID[l]].NearestPointDir_Fr0[j] * (-1) > TrfObj[int_var->ID].boundary_pointsAngles_max[j]) {

					SensData[j].distance[cnt] = int_var->distance[int_var->ID];
					SensData[j].Detc_TO_ID[cnt] = int_var->Detc_TO_ID[int_var->ID];
					SensData[j].Detc_TO_Surface_ID[cnt] = int_var->Detc_TO_Surface_ID[int_var->ID];
					SensData[j].Fr0_position_NP[cnt][0] = int_var->Fr0_position_NP[int_var->ID][0];
					SensData[j].Fr0_position_NP[cnt][1] = int_var->Fr0_position_NP[int_var->ID][1];
					SensData[j].NPDir_Fr0[cnt] = int_var->NPDir_Fr0[int_var->ID];
				}
				else {
					int_var->distance[int_var->ID] = 9999;
					break; // Go out of loop
				}
			}

			if (int_var->distance[int_var->ID] != 9999)
				cnt++;
		}
	}

	for (int k = cnt; k < 20; k++) {
		SensData[j].distance[k] = 0;
		SensData[j].Detc_TO_ID[k] = -1;
		SensData[j].Detc_TO_Surface_ID[k] = 0;
		SensData[j].Fr0_position_NP[k][0] = 0;
		SensData[j].Fr0_position_NP[k][1] = 0;
		SensData[j].NPDir_Fr0[k] = 0;
	}

	return;
}

int
vCUS_ReflectionGenerator(void)
{
	tTrafficObj *Obj = NULL;
	tObjectSensor *pOS = NULL;
	tObjectSensorObj *pOSO = NULL;

	/* Initialize structs */
	vCUS_InitReflGen();

	/* Reset variables at TestRun end */
	if (SimCore.State >= SCState_End && SimCore.State != SCState_Pause) {
		vCUS_ResetVarTREnd();
		return 0;
	}

	/* Run through Object Sensors - Main Routine - */
	if (ObjectSensorCount > 0 && (SimCore.State == SCState_StartLastCycle || SimCore.State == SCState_Simulate)) {
		for (int j = 0; j < ObjectSensorCount; j++) {

			/* Extraction of Traffic information */
			vCUS_ExtractTrafInf(j, Obj);

			/* Evaluation ORTHOGONAL */
			vCUS_OrthoEval(j, Obj);

			/* Evaluation OCCLUCISON / SORTING */
			vCUS_OcclusionEvalSorting(j, Obj);
		}

		// Debug Output
		if (SimCore.State == SCState_Simulate && (SimCore.CycleNo % 1000 == 0) && debug == 1)
			Log("--\n");
	}

	return 0;
}