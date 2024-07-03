/*
******************************************************************************
**  CarMaker - Version 8.1.1
** 
******************************************************************************
**
** Simple 'steer by torque'/'steer by angle' Model
**
** Author : Soumen Bhowmick (uib03394)
**
** Following the conventional CarMaker method to identify current rack position 
** User input for speed range and front to rear ratio is received and applied on rear wheels
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include "CarMaker.h"
#pragma warning( pop )
#include "Car/Vehicle_Car.h"
#include "Mdl_RearSteering.h"

static const char ThisModelClass[] = "Steering";
static const char ThisModelKind[]  = "RearAxleSteering";
static const int  ThisVersionId    = 1;

#define SEC2MSEC 0.001

struct tMyModel {
	
	int	EnableRearSteer;			/* Parameter to ENABLE/DISABLE rear axle steering [nu] */
	double  RearSteerSpdRange;		/* Ego Vehicle Speed till what the rear steering should be supported [km/h] */
	double RearSteerFactor;         /* Factor to convert rear axle steering amount [nu] */

	int IndependentMoveEnabled;     /* ENABLE/DISABLE independent translation movement of rear axle */
	double RearTranslation;         /* Placeholder for user value to define rear axle translation amount [m] */

    int		disabled;	/* error occured, steering system disabled */
    double	iSteer2Rack;	/* ratio			      [0.01]  */
    double	iRack2Steer;	/* ratio			      [100]   */
    double	RackRange[2];	/* rack translation range   [+0.1 ... -0.1] m */
    double	mass;		/* generalized mass of steering system        *
				 * without (suspension parts)		      *
				 *                              [Irot = 0.03] */
    double	TrqAmplify;	/* torque amplify                       [3.0] */
    double	d;		/* rack damping coefficient		[0.1] */

    /* Current signals */
    double	q, qp, qpp;	/* rack DOF				      */
    double	RackFrc;	/* rack force 				  [N] */
    double	FrcL, FrcR;	/* Left/right rack force		  [N] */

    /* Input signals: storing in own variables, for example if using a
       steering in-the-loop test bench with different signals for current
       and input state variables */
    struct {
	double	WhlAng;		/* Steering wheel angle			[rad] */
	double	WhlVel;		/* Steering wheel angle velocity      [rad/s] */
	double	WhlAcc;		/* Steering wheel angle acceleration[rad/s^2] */
	double	WhlTrq;		/* Steering wheel torque                 [Nm] */
	double	RackFrc;	/* Total rack force                       [N] */
	double	FrcL;		/* Left/right rack force		  [N] */
	double	FrcR;
	
    } In;
};


static void
MyModel_DeclQuants_dyn (struct tMyModel *mp, int park)
{
    static struct tMyModel MyModel_Dummy = {0};
    tDDefault *df = DDefaultCreate("Steer.");

    if (park)
	mp = &MyModel_Dummy;

    /* Define here dict entries for dynamically allocated variables. */
    DDefDouble4 (df, "Rack.Frc",	"N",		&mp->RackFrc,	DVA_None);
    DDefDouble4 (df, "L.Frc",		"N",		&mp->FrcL,	DVA_None);
    DDefDouble4 (df, "R.Frc",		"N",		&mp->FrcR,	DVA_None);

    DDefDouble4 (df, "In.WhlAng",	"rad",		&mp->In.WhlAng, DVA_None);
    DDefDouble4 (df, "In.WhlVel",	"rad/s",	&mp->In.WhlVel, DVA_None);
    DDefDouble4 (df, "In.WhlAcc",	"rad/s^2",	&mp->In.WhlAcc,	DVA_None);
    DDefDouble4 (df, "In.WhlTrq",	"Nm",		&mp->In.WhlTrq, DVA_None);
    DDefDouble4 (df, "In.Rack.Frc",	"N",		&mp->In.RackFrc,DVA_None);
    DDefDouble4 (df, "In.L.Frc",	"N",		&mp->In.FrcL,	DVA_None);
    DDefDouble4 (df, "In.R.Frc",	"N",		&mp->In.FrcR,	DVA_None);
    
    DDefaultDelete(df);
}


static void
MyModel_DeclQuants (void *MP)
{
    struct tMyModel *mp = (struct tMyModel *)MP;

	if (mp != NULL)
	{
		DDefDouble(NULL, "RearAxleTranslation", "m", &mp->RearTranslation, DVA_IO_In);
	}
	
    
	if (mp == NULL) {
	/* Define here dict entries for non-dynamically allocated (static) variables. */

    } else {
	MyModel_DeclQuants_dyn (mp, 0);
    }
}


static void *
MyModel_New (tInfos *Inf, struct tSteeringCfgIF *CfgIF, const char *KindKey)
{
    struct tMyModel *mp = NULL;
    double	val, dvec[3];
    const char *ModelKind, *key;
    char 	MsgPre[64];
    unsigned	nError =   GetInfoErrorCount() + Log_nError;
    int nRows, VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_Steering, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tMyModel*)calloc(1,sizeof(*mp));

    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    /* get CfgIF parameters */
    if (Steering_GetCfgOutIF (Inf, CfgIF, ModelKind) != 0)
	goto ErrorReturn;

    key = "Steering.Rack2StWhl";
    if ((val=iGetDblOpt(Inf, key, 0.0)) != 0.0) {
	mp->iSteer2Rack = 1.0 / val;
    } else {
	LogErrF(EC_Init, "%s: Steering ratio rack/wheel '%s' not defined or zero.",
	    MsgPre, key);
	mp->iSteer2Rack = 0.01;
    }

    /* Consider if the rack is behind the steering axle or not */
    mp->iSteer2Rack *= CfgIF->PosSign;

    dvec[0] = dvec[1] = 0.0;
    iGetTable(Inf, "Steering.RackRange", dvec, 2, 1, &nRows);
    mp->RackRange[0] = dvec[0];
    mp->RackRange[1] = dvec[1];

    if (mp->RackRange[0] > mp->RackRange[1]) {
	val = mp->RackRange[0];
	mp->RackRange[0] = mp->RackRange[1];
	mp->RackRange[1] = val;
    }

	val = 0.001;// iGetDbl(Inf, "Steering.Wheel.Irot"); //Currently hardcoded
    mp->mass = val / (mp->iSteer2Rack*mp->iSteer2Rack);

	mp->TrqAmplify = 3.0;// iGetDbl(Inf, "Steering.TrqAmplify"); //Currently hardcoded
    mp->d = 0.1/ mp->iSteer2Rack;		//iGetDbl(Inf, "Steering.d") / mp->iSteer2Rack; //Currently hardcoded

	mp->EnableRearSteer = iGetInt(Inf, "Steering.EnableRearSteer");
	mp->RearSteerSpdRange = iGetDbl(Inf, "Steering.RearSteerSpdRange");
	mp->RearSteerFactor = iGetDbl(Inf, "Steering.RearSteerFactor");

	mp->IndependentMoveEnabled = iGetInt(Inf, "Steering.RearSteerIndependentEn");
	//mp->RearTranslation = iGetDbl(Inf, "Steering.RearTranslation"); /* Placeholder, currently not defined */

    if (nError != GetInfoErrorCount() + Log_nError)
	goto ErrorReturn;

    return mp;

    ErrorReturn:
	free (mp);
	return NULL;
}


static int
MyModel_Calc (void *MP, tSteeringIF *IF, double dt)
{
    struct tMyModel *mp = (struct tMyModel *)MP;
    double mass;
    const double kRackBuf = 1e6;
    const double dRackBuf = 1e4;

    mp->iRack2Steer = 1.0 / mp->iSteer2Rack;

    if (IF->SteerBy == SteerBy_Unknown)
	return 0;


    /*** Total mass */
    mass = mp->mass + IF->L.Inert + IF->R.Inert;

    /*** Total rack force (input and current rack force) */
    mp->In.FrcL    = IF->L.Frc;
    mp->In.FrcR    = IF->R.Frc;
    mp->In.RackFrc = mp->In.FrcL + mp->In.FrcR;

    /* current rack force signals can differ from
       the input signals:
       mp->FrcL = mp->In.FrcL;
       mp->FrcR = mp->In.FrcR;
    */
    mp->FrcL    = mp->In.FrcL;
    mp->FrcR    = mp->In.FrcR;
    mp->RackFrc = mp->FrcL + mp->FrcR;


    /*** Limitation of rack buffers */
    if (mp->q < mp->RackRange[0]) {
	double val = mp->q - mp->RackRange[0];
	mp->RackFrc += - kRackBuf * val - dRackBuf * mp->qp;

    } else if (mp->q > mp->RackRange[1]) {
	double val = mp->q - mp->RackRange[1];
	mp->RackFrc += - kRackBuf * val - dRackBuf * mp->qp;
    }

    /*** Kinetics */
    if (IF->SteerBy == SteerBy_Trq) {		/* Steering by torque */

	mp->In.WhlTrq = IF->Trq;

	/*** DOF rack position */
	mp->qpp = (mp->TrqAmplify * IF->Trq * mp->iRack2Steer 
	        +  mp->RackFrc - mp->d * mp->qp) / mass;

	/*** Integration: rack position */
	mp->qp += mp->qpp * dt;
	mp->q  += mp->qp  * dt;

	/* Assignment: current steering wheel angle */
	IF->Ang    = mp->q   * mp->iRack2Steer;
	IF->AngVel = mp->qp  * mp->iRack2Steer;
	IF->AngAcc = mp->qpp * mp->iRack2Steer;

    } else {					/* Steering by angle */

	mp->In.WhlAng = IF->Ang;
	mp->In.WhlVel = IF->AngVel;
	mp->In.WhlAcc = IF->AngAcc;

	mp->q   = IF->Ang    * mp->iSteer2Rack;
	mp->qp  = IF->AngVel * mp->iSteer2Rack;
	mp->qpp = IF->AngAcc * mp->iSteer2Rack;

	/* Assignment: current steering wheel torque */
	IF->Trq = (mp->qpp * mass + mp->d * mp->qp - mp->RackFrc) * mp->iSteer2Rack
	        / mp->TrqAmplify;
    }

    /*** Assignment: current rack position */
    IF->L.q   = IF->R.q   = mp->q;
    IF->L.qp  = IF->R.qp  = mp->qp;
    IF->L.qpp = IF->R.qpp = mp->qpp;

    IF->L.iSteer2q = IF->R.iSteer2q = mp->iSteer2Rack;
    
    /* REAR STEERING */
	static double RL_q, RR_q;

	if (mp->EnableRearSteer == 1)
	{
		double SpeedRange = mp->RearSteerSpdRange;

		if (Car.ConBdy1.v <= (SpeedRange /3.6))
		{
			if (mp->IndependentMoveEnabled == 0) /* If the independent movement is disabled */
			{
				/* Relative to the front axle translation */
				Steering.RL.q = mp->q * mp->RearSteerFactor; 
				Steering.RR.q = mp->q * mp->RearSteerFactor;
				
				Steering.RL.qp = mp->qp * mp->RearSteerFactor;
				Steering.RR.qp = mp->qp * mp->RearSteerFactor;

			}
			else /* Otherwise provide the independent rear axle translation value */
			{
				/* Set as per maximum rack range */
				if (mp->RearTranslation < mp->RackRange[0])
				{
					Steering.RL.q = mp->RackRange[0];
					Steering.RR.q = mp->RackRange[0];
				}
				else if (mp->RearTranslation > mp->RackRange[1])
				{
					Steering.RL.q = mp->RackRange[1];
					Steering.RR.q = mp->RackRange[1];
				}
				else
				{
					Steering.RL.q = mp->RearTranslation;
					Steering.RR.q = mp->RearTranslation;
				}

				/* Calculate the time derivative manually */
				Steering.RL.qp = (Steering.RL.q - RL_q) / SEC2MSEC;
				Steering.RR.qp = (Steering.RR.q - RR_q) / SEC2MSEC;

				RL_q = Steering.RL.q;
				RR_q = Steering.RR.q;
			}

		}
	}

    /*
     * The signal TrqStatic is only an output signal or
     * an additional information!
     *
     * steering wheel torque, to keep the wheel in its position
     * under static conditions
     */
    IF->TrqStatic = IF->L.iSteer2q * IF->L.Frc + IF->R.iSteer2q * IF->R.Frc;

    return 0;
}


static void
MyModel_Delete (void *MP)
{
    struct tMyModel *mp = (struct tMyModel *)MP;

    /* Park the dict entries for dynamically allocated variables before deleting */
    MyModel_DeclQuants_dyn (mp, 1);
    free (mp);
}


int 
Steering_Register_MyModel (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.Steering.VersionId =		ThisVersionId;
    m.Steering.New =			MyModel_New;
    m.Steering.Calc =			MyModel_Calc;
    m.Steering.Delete =			MyModel_Delete;
    m.Steering.DeclQuants =		MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.Steering.ParamsChanged = 		ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_Steering, ThisModelKind, &m);
}
