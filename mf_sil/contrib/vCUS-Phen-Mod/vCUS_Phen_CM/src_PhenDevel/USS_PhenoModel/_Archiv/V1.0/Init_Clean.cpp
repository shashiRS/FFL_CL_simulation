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

/* Define Trfobj handle & initiallize handle */
tTrafficObject	*TrfObj = NULL;
tSensorParam	*SensParam = NULL;
tSensorData		*SensData = NULL;
tVector_calc	*env_vector = NULL;
tInt_var		*int_var = NULL;

int nTC, nOS;
int debug = 1;

/* Initialization of Signals / Quantities */
int
vCUS_InitData(void)
{
	float  *sensor_orientation = NULL, *sensor_position = NULL;
	int nCols;
	char pre[64], sbuf[256];

	nTC = iGetIntOpt(SimCore.TestRun.Inf, "Traffic.N", 0);
	nOS = iGetIntOpt(SimCore.Vhcl.Inf, "Sensor.Object.N", 0);

	// Obj = (tTrafficObj *)calloc(nTC, sizeof(tTrafficObj));
	TrfObj		= (tTrafficObject *)	calloc(nTC, sizeof(tTrafficObject));
	SensParam	= (tSensorParam *)		calloc(nOS, sizeof(tSensorParam));
	SensData	= (tSensorData *)		calloc(nOS, sizeof(tSensorData));
	env_vector	= (tVector_calc *)		calloc(1, sizeof(tVector_calc));
	int_var		= (tInt_var *)			calloc(1, sizeof(tInt_var));

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

	/* DDict -> Traffic */
	for (int i = 0; i < nTC; i++) {
		/* Generate DDict Entries for all edges */
		for (int j = 0; j < 4; j++) {
			TrfObj[i].boundary_points[j][0] = 0;
			TrfObj[i].boundary_points[j][1] = 0;

			sprintf(sbuf, "TrafficData.%d.Edge.%d.PosX", i, j);
			DDefDouble4(NULL, sbuf, "m", &TrfObj[i].boundary_points[j][0], DVA_None);
			sprintf(sbuf, "TrafficData.%d.Edge.%d.PosY", i, j);
			DDefDouble4(NULL, sbuf, "m", &TrfObj[i].boundary_points[j][1], DVA_None);
		}
	}

	/* DDict/IFile -> Sensor */
	for (int i = 0; i < nOS; i++) {

		sprintf(pre, "Sensor.Object.%d.rot", i);
		if ((sensor_orientation = iGetTableFltOpt2(SimCore.Vhcl.Inf, pre, NULL, 1, &nCols)) == NULL)
			LogErrF(EC_Init, "Can't read ''Sensor.Object.%d.rot''\n", i);
		SensParam[i].orientation = sensor_orientation[2];

		sprintf(pre, "Sensor.Object.%d.pos", i);
		if ((sensor_position = iGetTableFltOpt2(SimCore.Vhcl.Inf, pre, NULL, 1, &nCols)) == NULL)
			LogErrF(EC_Init, "Can't read ''Sensor.Object.%d.pos''\n", i);
		for (int j = 0; j < 3; j++) {
			SensParam[i].Fr1_position[j] = sensor_position[j];
			SensData[i].Fr0_position_Sens[j] = 0;
			if (debug == 1)
				Log("Position for sensor No %d | Element %d = %f\n\n", i, j, SensParam[i].Fr1_position[j]);
		}

		if (debug == 1)
			Log("Orientation for sensor No %d = %f\n\n", i, SensParam[i].orientation);

		for (int k = 0; k < 3; k++)
			SensData[i].Fr0_position_Sens[k] = 0;

		/* Generate DDict Entries for all Sensors */
		sprintf(sbuf, "SensData.%d.SensPos.x", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[0], DVA_None);
		sprintf(sbuf, "SensData.%d.SensPos.y", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[1], DVA_None);
		sprintf(sbuf, "SensData.%d.SensPos.z", i);
		DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_Sens[2], DVA_None);

		for (int k = 0; k < 20; k++) {
			SensData[i].Fr0_position_NP[k][0] = 0;
			SensData[i].Fr0_position_NP[k][1] = 0;
			SensData[i].distance[k] = 0;
			SensData[i].Detc_TO_ID[k] = 0;
			SensData[i].Detc_TO_Surface_ID[k] = 0;
			SensData[i].NPDir_Fr0[k] = 0;

			sprintf(sbuf, "SensData.%d.Target.NPPos.x.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_NP[k][0], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.NPPos.y.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].Fr0_position_NP[k][1], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.NPDir_Fr0.%d", i, k);
			DDefDouble(NULL, sbuf, "-", &SensData[i].NPDir_Fr0[k], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.distance.%d", i, k);
			DDefDouble4(NULL, sbuf, "m", &SensData[i].distance[k], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.Detected_TOid.%d", i, k);
			DDefInt(NULL, sbuf, "-", &SensData[i].Detc_TO_ID[k], DVA_None);
			sprintf(sbuf, "SensData.%d.Target.Detected_TO_SurfaceID.%d", i, k);
			DDefInt(NULL, sbuf, "-", &SensData[i].Detc_TO_Surface_ID[k], DVA_None);
		}
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

	if (env_vector != NULL)
		free(env_vector);

	if (int_var != NULL)
		free(int_var);
}