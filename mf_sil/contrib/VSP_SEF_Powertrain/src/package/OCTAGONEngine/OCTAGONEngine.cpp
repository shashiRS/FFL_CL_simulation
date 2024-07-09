/******** OCTAGONEngine Model - Continental AG ***************
 *
 * Author: Ragunathan, Kaustub (uib55286)
 * Call the function 
 * Engine_Register_OCTAGONEngine();
 * in User.c(pp) after including the OCTAGONEngine.h header file
 *
******************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <queue>
#include "OCTAGONEngine.h"
#include "CarMaker.h"
#include "Car/Vehicle_Car.h"

static const char ThisModelClass[] = "PowerTrain.Engine";
static const char ThisModelKind[]  = "OCTAGONEngine";
static const int  ThisVersionId    = 1;

static std::queue<double> torque_queue;

struct tOCTAGONEngine {

    double	TrqKl15Off;	 /* Torque for Kl15=0 [Nm] */
    double	rotvIdle;	 /* Engine idle speed [rad/s] */
    double	rotvOff;	 /* Engine off speed	[rad/s] */
    double	rotvMin;	 /* Engine minimum speed	[rad/s] */
    double	rotvMax;	 /* Engine maximum speed	[rad/s] */
	double  rotvOpt;     /*Engine speed for optimal fuel consumption [rad/s]*/
	double  DeadTime;	 /* transport delay for engine torque [s]*/
	double	BuildUpTime; /*PT1 build up for engine torque [s]*/
	double Trq_Full;	 /* Full torque value for a given rpm [Nm]*/
	double Trq_Drag;	 /* Drag torque value for a given rpm [Nm]*/
    tLM		*TrqFull;	 /* Look-Up Table 1D for full-load torque [Nm] */
    tLM		*TrqDrag;	 /* Look-Up Table 1D for drag torque [Nm] */
    double	I_out;		 /* Inertia outshaft  [kgm^2] */
};

static void OCTAGONEngine_Delete (void *MP);


static void
OCTAGONEngine_DeclQuants_dyn (struct tOCTAGONEngine *mp, int park)
{
    static struct tOCTAGONEngine OCTAGONEngine_Dummy;
    memset (&OCTAGONEngine_Dummy, 0, sizeof(struct tOCTAGONEngine));
    if (park)
	mp = &OCTAGONEngine_Dummy;

	DDefDouble(NULL, "PT.Engine.MinTrq", "Nm", &mp->Trq_Drag, DVA_None);
	DDefDouble(NULL, "PT.Engine.MaxTrq", "Nm", &mp->Trq_Full, DVA_None);
    /* Define here dict entries for dynamically allocated variables. */
}


static void
OCTAGONEngine_DeclQuants (void *MP)
{
    struct tOCTAGONEngine *mp = (struct tOCTAGONEngine *)MP;

    if (mp == NULL) {
	/* Define here dict entries for non-dynamically allocated (static) variables. */

    } else {
	OCTAGONEngine_DeclQuants_dyn (mp, 0);
    }
}

static void *
OCTAGONEngine_New (
    struct tInfos	  *Inf,
    struct tPTEngineCfgIF *CfgIF,
    const char		  *KindKey,
    const char		  *Ident)
{
    struct tOCTAGONEngine *mp = NULL;
    char 	MsgPre[64];
    int		VersionId = 0;
    const char *ModelKind;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_PTEngine, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tOCTAGONEngine*)calloc(1,sizeof(*mp));

    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    /* get CfgIF parameters */
    if (Engine_GetCfgOutIF (Inf, CfgIF, ModelKind) != 0)
	goto ErrorReturn;

    /* Get parameters from file */
    mp->TrqKl15Off   = iGetDbl(Inf, "PowerTrain.Engine.TrqKl15Off");
	mp->rotvIdle     = iGetDbl(Inf, "PowerTrain.Engine.rotv_idle");
	mp->rotvMax      = iGetDbl(Inf, "PowerTrain.Engine.rotv_max");
	mp->rotvMin      = iGetDbl(Inf, "PowerTrain.Engine.rotv_min");
	mp->rotvOff      = iGetDbl(Inf, "PowerTrain.Engine.rotv_off");
	mp->rotvOpt		 = iGetDbl(Inf, "PowerTrain.Engine.rotv_opt");
	mp->DeadTime     = iGetDbl(Inf, "PowerTrain.Engine.DeadTime");
	mp->BuildUpTime  = iGetDbl(Inf, "PowerTrain.Engine.BuildUpTime");

    /* Inertia */
    mp->I_out = iGetDbl (Inf, "PowerTrain.Engine.I");

    /* CfgIF output -> Model */
    mp->TrqFull = CfgIF->TrqFull;
    mp->TrqDrag = CfgIF->TrqDrag;

    /* CfgIF output: verification if the parametrization corresponds to the model */
    if (mp->TrqFull==NULL || mp->TrqDrag==NULL) {
	LogErrF (EC_Init, "%s: missing engine torque characteristic", MsgPre);
	goto ErrorReturn;
    }

	std::queue<double>().swap(torque_queue);

    return mp;

  ErrorReturn:
    OCTAGONEngine_Delete (mp);
    return NULL;
}


/* Torque zero for rotation under reference speed */
static double
Fac4VelZero (double vel, double vel_ref)
{
    double absvel = fabs(vel);

    if (absvel >= vel_ref)
	return 1.0;
    else
	return 0.5 * (1.0 - cos(M_PI * absvel / vel_ref));
}


static int
OCTAGONEngine_Calc (void *MP, struct tPTEngineIF *IF, double dt)
{
    struct tOCTAGONEngine *mp = (struct tOCTAGONEngine *)MP;
    double TrqFull, TrqDrag, Load = M_BOUND(0.0, 1.0, IF->Load);
	double final_trq_delayed = 0;
	double final_trq = 0;
    if (IF->FuelLevel==0.0)
	Load = 0.0;

    if (IF->Ignition) {
	double rotv = M_MAX(IF->rotv, mp->rotvMin*rpm2radsec);
    rotv = M_MIN(rotv, mp->rotvMax*rpm2radsec);

	/* Use the torque from the Look-Up tables = f(rotv, Load) */
	TrqFull = mp->Trq_Full = LMEval(mp->TrqFull, rotv);
	TrqDrag = mp->Trq_Drag = LMEval(mp->TrqDrag, rotv);
	final_trq = TrqDrag + Load * (TrqFull-TrqDrag);
    } else {
	/* Use the torque for Kl15=0 */
	final_trq = mp->TrqKl15Off;
    }
	final_trq *= M_SGN(IF->rotv) * Fac4VelZero(IF->rotv, 2.0*rpm2radsec);

	/*Transport delay for engine output torque*/
	if (PowerTrain.ControlIF.EngineOut.Trq_trg >= 0) // transport delay with negative target torque leads to oscillations
	{
		if (mp->DeadTime > 0) {
			torque_queue.push(final_trq);
			if (mp->DeadTime / dt < torque_queue.size()) { /*queue is "full" --> copy delayed value*/
				final_trq_delayed = torque_queue.front(); /*read value*/
				torque_queue.pop(); /*remove last value and shift entries*/
			}
			else if (torque_queue.size() == 1) { /*first cycle--> copy valid initial values*/
				final_trq_delayed = torque_queue.front();
			}
		}
		else {
			final_trq_delayed = final_trq; /* use engine final torque directly */
		}

	}
	else {
		final_trq_delayed = final_trq; /* use engine final torque directly */
	}


	/* equation to generate final output torque through a Pt1 Delay: y(n) = Tc*(K*xn – yn-1) + yn-1). - Analytical solution for first order time delay (PT1 element) */
	IF->Trq = 1 / (mp->BuildUpTime / dt + 1) * (1 * final_trq_delayed - IF->Trq) + IF->Trq;
    IF->Inert = mp->I_out;

	if (IF->rotv < mp->rotvIdle*rpm2radsec)
		mp->Trq_Drag = IF->Trq;

    return 0;
}


static int
OCTAGONEngine_ModelCheck (void *MP, struct tInfos *Inf)
{
    struct tOCTAGONEngine *mp = (struct tOCTAGONEngine *)MP;
    FILE *fp;

    if ((fp = ModelCheck_GetDesignFile()) == NULL)
	return 0;

    fprintf (fp, "### Engine.Kind = %s\n", ThisModelKind);
    fprintf (fp, "Engine.I =  %10.7f\n", mp->I_out);
    fprintf (fp, "\n");

    return 0;
}


static void
OCTAGONEngine_Delete (void *MP)
{
    struct tOCTAGONEngine *mp = (struct tOCTAGONEngine *)MP;

    /* Park the dict entries for dynamically allocated variables before deleting */
    OCTAGONEngine_DeclQuants_dyn (mp, 1);

    free(mp);
}


int 
Engine_Register_OCTAGONEngine (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.PTEngine.VersionId =	ThisVersionId;
    m.PTEngine.New =		OCTAGONEngine_New;
    m.PTEngine.Calc =		OCTAGONEngine_Calc;
    m.PTEngine.Delete =		OCTAGONEngine_Delete;
    m.PTEngine.DeclQuants =	OCTAGONEngine_DeclQuants;
    m.PTEngine.ModelCheck =	OCTAGONEngine_ModelCheck;
    /* Should only be used if the model doesn't read params from extra files */
    m.PTEngine.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_PTEngine, ThisModelKind, &m);
}
