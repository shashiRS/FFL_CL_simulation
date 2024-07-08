
#include <stdlib.h>
#include <string.h>
#include <math.h>

#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include "CarMaker.h"
#pragma warning( pop )
#include "Car/Vehicle_Car.h"
#include "VehTurnCntr.h"
#include "Vehicle\Sensor_Inertial.h"

struct tVhclTurnCntr {
	double c_2, h_radius, c_2_fr1, h_radius_ackermann, h_radius_yawrate;
	int sens1, sens2; 
};

static const char ThisModelClass[] = "VehicleControl";
static const char ThisModelKind[]  = "VhclTurnCntr";
static const int  ThisVersionId    = 1;

/******************************************************************************/


static void
VhclTurnCntr_DeclQuants_dyn (struct tVhclTurnCntr *mp, int park)
{
	static struct tVhclTurnCntr VhclTurnCntr_Dummy = { 0 };
	if (park)
		mp = &VhclTurnCntr_Dummy;

	int CladdingFL_inertial_ID = InertialSensor_FindIndexForName("CladdingFL");
	int CladdingFR_inertial_ID = InertialSensor_FindIndexForName("CladdingFR");
	int CladdingRL_inertial_ID = InertialSensor_FindIndexForName("CladdingRL");
	int CladdingRR_inertial_ID = InertialSensor_FindIndexForName("CladdingRR");
    /* Define here dict entries for dynamically allocated variables. */

	DDefDouble4(NULL, "NeutralSteerPos", "m", &mp->c_2, DVA_None);
	DDefDouble4(NULL, "NeutralSteerPosFr1", "m", &mp->c_2_fr1, DVA_None);
	DDefDouble4(NULL, "TurningRadius", "m", &mp->h_radius, DVA_None);
	DDefDouble4(NULL, "SA1.pos", "m", &InertialSensor[mp->sens1].OBO_B[0], DVA_None);
	DDefDouble4(NULL, "SA2.pos", "m", &InertialSensor[mp->sens2].OBO_B[0], DVA_None);
	//DDefDouble4(NULL, "SA3.pos", "m", &InertialSensor[mp->sens3].OBO_B[0], DVA_None);  //need to check
	DDefDouble4(NULL, "TurningRadiusAckermann", "m", &mp->h_radius_ackermann, DVA_None);
    DDefDouble4(NULL, "TurningRadiusYawRate", "m", &mp->h_radius_yawrate, DVA_None);
	DDefDouble4(NULL, "CladdingFL.pos", "m", &InertialSensor[CladdingFL_inertial_ID].OBO_0[2], DVA_None);
	DDefDouble4(NULL, "CladdingFR.pos", "m", &InertialSensor[CladdingFR_inertial_ID].OBO_0[2], DVA_None);
	DDefDouble4(NULL, "CladdingRL.pos", "m", &InertialSensor[CladdingRL_inertial_ID].OBO_0[2], DVA_None);
	DDefDouble4(NULL, "CladdingRR.pos", "m", &InertialSensor[CladdingRR_inertial_ID].OBO_0[2], DVA_None);
}


static void
VhclTurnCntr_DeclQuants (void *MP)
{
    struct tVhclTurnCntr *mp = (struct tVhclTurnCntr *)MP;

    if (mp == NULL) {
	/* Define here dict entries for non-dynamically allocated (static) variables. */

    } else {
	    VhclTurnCntr_DeclQuants_dyn (mp, 0);
    }

}


static int
VhclTurnCntr_Calc(void *MP, double dt)
{
    dt; /*silence unreferenced warning*/
	struct tVhclTurnCntr *mp = (struct tVhclTurnCntr *)MP;

	/* Internes Koordinatensystem */
	double v_x_f, v_y_f, v_x_r, v_y_r, v_r, c, alpha_f, alpha_r;

	v_x_f = fabs(InertialSensor[mp->sens2].Vel_B[0]);
	v_y_f = fabs(InertialSensor[mp->sens2].Vel_B[1]);

	v_x_r = fabs(InertialSensor[mp->sens1].Vel_B[0]);
	v_y_r = fabs(InertialSensor[mp->sens1].Vel_B[1]);

    v_r = sqrt(v_x_r*v_x_r + v_y_r*v_y_r);

	c = InertialSensor[mp->sens2].OBO_B[0] - InertialSensor[mp->sens1].OBO_B[0];

	if (fabs(v_x_f) <= 0.000001) {
		alpha_f = M_PI_2;
	}
	else {
		alpha_f = M_PI_2 - atan(v_y_f / v_x_f);
	}

	if (fabs(v_x_r) <= 0.000001) {
		alpha_r = M_PI_2;
	}
	else {
		alpha_r = M_PI_2 - atan(v_y_r / v_x_r);
	}

	// Ackermann:
    mp->h_radius_ackermann = fabs(tan(alpha_f)*c*M_SGN(Car.YawRate));
    if (fabs(mp->h_radius_ackermann) > 100 || v_r < 0.05) {
        mp->h_radius_ackermann = 0;
    }
    if ((Steering.IF.Ang) < 0.0f) mp->h_radius_ackermann *= -1.0;
	//////

 	if (fabs(v_x_r) > 0.1) {
	mp->c_2 = c / ((tan(alpha_r) / (tan(alpha_f))) + 1);
	mp->c_2_fr1 = c / ((tan(alpha_r) / (tan(alpha_f))) + 1) + InertialSensor[mp->sens1].OBO_B[0];
	}
	else {
	 mp->c_2 = 0;
	 mp->c_2_fr1 = 0;
	}

	mp->h_radius = fabs(tan(alpha_r) * mp->c_2*M_SGN(Car.YawRate));
	if (fabs(mp->h_radius) > 100 || v_r < 0.05) {
		mp->h_radius = 0;
	}
    if ((Steering.IF.Ang) < 0.0f) mp->h_radius *= -1.0;
	
    mp->h_radius_yawrate = v_x_r/Car.YawRate;
	
    if (fabs(mp->h_radius_yawrate) > 20000) {
        mp->h_radius_yawrate = 0;
    }

    return 0;
}


static void *
VhclTurnCntr_New (struct tInfos *Inf, const char *KindKey)
{
    struct tVhclTurnCntr *mp = NULL;
    const char *ModelKind;
    char MsgPre[64];
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_VehicleControl, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tVhclTurnCntr *)calloc(1, sizeof(*mp));
	mp->sens1 = InertialSensor_FindIndexForName("SA1");
	mp->sens2 = InertialSensor_FindIndexForName("SA2");
	//mp->sens3 = InertialSensor_FindIndexForName("SA3");  //need to check
	if (mp->sens1 == -1) {LogErrF(EC_Init, "InertialSensor SA1 does not exist! Please place at middle of rear axle.\n"); }
	if (mp->sens2 == -1) {LogErrF(EC_Init, "InertialSensor SA2 does not exist! Please place at middle of front axle.\n"); }
	if (InertialSensor[mp->sens1].OBO_B[1] != 0) {LogErrF(EC_Init, "InertialSensor SA1 y-position is not zero!\n"); }
	if (InertialSensor[mp->sens2].OBO_B[1] != 0) {LogErrF(EC_Init, "InertialSensor SA2 y-position is not zero!\n"); }
	if (InertialSensor[mp->sens2].OBO_B[0] < InertialSensor[mp->sens1].OBO_B[0]) { LogErrF(EC_Init, "InertialSensor SA2 must be place in front of SA1!\n"); }
    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);
    return mp;

}


static void
VhclTurnCntr_Delete (void *MP)
{
    struct tVhclTurnCntr *mp = (struct tVhclTurnCntr *)MP;

    /* Park the dict entries for dynamically allocated variables before deleting */
	VhclTurnCntr_DeclQuants_dyn (mp, 1);
    free (mp);
}


int
VehicleControl_Register_VhclTurnCntr (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.VehicleControl.VersionId =	ThisVersionId;	
    m.VehicleControl.New    =		VhclTurnCntr_New;
    m.VehicleControl.Calc   =       VhclTurnCntr_Calc;
    m.VehicleControl.DeclQuants   = VhclTurnCntr_DeclQuants;
    m.VehicleControl.Delete = VhclTurnCntr_Delete;
    /* Should only be used if the model doesn't read params from extra files */
    m.VehicleControl.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_VehicleControl, ThisModelKind, &m);
}
