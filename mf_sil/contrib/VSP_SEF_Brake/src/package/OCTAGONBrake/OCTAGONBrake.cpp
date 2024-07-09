/******** VSP Generic Brake Model - Continental AG ***************
 * 
 * Author: Ragunathan, Kaustub (uib55286); Meissner, Thomas (uid33443)
 * Date:  16.08.2019
 * Update with ramp: 23.04.2020
 * Brake_Register_OCTAGONBrake ();
 * 
******************************************************************
*/

#include <queue>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "OCTAGONBrake.h"
#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#include <stdbool.h>
// minmax.h is not available in Linux. Use the std::min/max function instead.
#if defined(WIN32)
#include <minmax.h>
#else
using namespace std;
#endif

#define NWHEEL 4
static const char ThisModelClass[] = "Brake";
static const char ThisModelKind[]  = "OCTAGONBrake";
static const int  ThisVersionId    = 2;

double brakeRequestInput = 0.0;
double brakeRequestInput_delayed = 0.0;
double brakeRequestAccel = 0.0;
double brakeRequestAccel_delayed = 0.0;
double brakeRequestTrq = 0.0;
double brakeRequestTrq_delayed = 0.0;
double brakeRequestPmc = 0.0;
double brakeRequestPmc_delayed = 0.0;

double timeSinceBrakeRequest = 0.0;
double rTireFA = 0.0;
double rTireRA = 0.0;
double driver_BrakeMoment = 0.0;
double timeSinceBrakeRequest_driver = 0.0;
double EBA_brakemoment = 0.0;
double finalmoment = 0.0;
double req = 0.0;
double req_delayed = 0.0;
double aReq = 0.0;
int ABS_active_front = 0;
int ABS_active_rear = 0;
static std::queue<double> q_req;

bool rFrozen = false;
bool freezeRampedTime = false;
bool freezeRampedTime_driver = false;

typedef struct  {
	double TrqDistrib[NWHEEL]; /* Model Parameter */
	double Trq_WB[NWHEEL]; /* Brake torque [Nm] */
	double deadTime_init; /*[s]*/
	double deadTime_CAN; /*[s]*/
	double TimeConstPT1_Driver; /*[s]*/ // ramp time for driver is typically more than EBA*/
	double TimeConstPT1_EBA; /*[s]*/
	double RampGradient_Driver; /*[Nm/s]*/ // ramp time for driver is typically more than EBA* /
	double RampGradient_EBA; /*unit depends on InputKind*/
	double PedalAct2pMC; /*[bar]*/
	double a_max_decel;
	double pWB2Trq; /*[Nm/bar] Factor for Master cylinder pressure to torque - final value obtained by multiplying with individual TrqDistrib*/
	int UseABS; /*Use ABS torque limitation to physical possible value 1:=yes 0:=no*/
	int UseMode; /*UseMode 2:=PT2 transient behavior, 1:=PT1 transient behavior, 0:=Ramp behavior*/
	int InputKind; /*InputKind 0:=Acceleration, 1:=Pressure of Master Cylinder, 2:=Torque*/
	double ABS_factor; /*Factor for limiting brake torque output based on vertical force, mue and tire radius (Based on physical limits)*/
	double PT2_Coeff_b0;	/* -, PT2 system parameter: Brake torque request -> brake final torque */
	double PT2_Coeff_a0;    /* -, PT2 system parameter: Brake torque request -> brake final torque */
	double PT2_Coeff_a1;    /* -, PT2 system parameter */
	double PT2_Coeff_a2;	/* -, PT2 system parameter */
} sOCTAGONBrake_t;

enum { acceleration, pmc, torque };

static double b0 = 1;	  // -, PT2 system parameter: Brake torque request -> brake final torque
static double a0 = 1;    // -, PT2 system parameter: Brake torque request -> brake final torque
static double a1 = 0.1211;  // -, PT2 system parameter
static double a2 = 0.01;  // -, PT2 system parameter
static double y_PT2_kTime_1 = 0;  // PT2 system output value of the last time stample
static double y_PT2_kTime_2 = 0;  // PT2 system output value of the last last time stample
static double u_PT2_kTime_1 = 0;  // PT2 system input value of the last time stample
static double u_PT2_kTime_2 = 0;  // PT2 system input value of the last last time stample

/*double min(double lhs, double rhs)
{
	return ((lhs < rhs) ? lhs : rhs);
}*/


sOCTAGONBrake_t *sOCTAGONBrakeParameters = NULL;
void setBrakeRequest(double input)
{
	/*Negative input for acceleration, positive input for pmc (pressure master cylinder), negative for axle torque input*/
	brakeRequestInput = input;
}

/*
** OCTAGONBrake_New ()
**
** Initializing the model
**
** Call:
** - one times at the beginning of every TestRun
*/

static void resetGlobalsToZero() {
	brakeRequestInput = 0.0;
	brakeRequestInput_delayed = 0.0;
	brakeRequestAccel = 0.0;
	brakeRequestAccel_delayed = 0.0;
	brakeRequestTrq = 0.0;
	brakeRequestTrq_delayed = 0.0;
	brakeRequestPmc = 0.0;
	brakeRequestPmc_delayed = 0.0;
	timeSinceBrakeRequest = 0.0;
	rTireFA = 0.0;
	rTireRA = 0.0;
	driver_BrakeMoment = 0.0;
	timeSinceBrakeRequest_driver = 0.0;
	EBA_brakemoment = 0.0;
	finalmoment = 0.0;
	req = 0;
	req_delayed = 0;
	ABS_active_front = 0;
	ABS_active_rear = 0;
	aReq = 0;
	
	rFrozen = false;
	freezeRampedTime = false;
	freezeRampedTime_driver = false;
	std::queue<double>().swap(q_req);

	y_PT2_kTime_1 = 0.0;
	y_PT2_kTime_2 = 0.0;
	u_PT2_kTime_1 = 0.0;
	u_PT2_kTime_2 = 0.0;


}

static void *
  OCTAGONBrake_New (tInfos *Inf, tBrakeCfgIF *CfgIF, const char *KindKey)
{
	sOCTAGONBrakeParameters= (sOCTAGONBrake_t *) malloc (sizeof (sOCTAGONBrake_t));

	sOCTAGONBrakeParameters->TrqDistrib[0] = iGetDblOpt(Inf,"OCTAGONBrake.TrqDistrib.FL", 0.35);    /*brake torque distribution FL*/
	sOCTAGONBrakeParameters->TrqDistrib[1] = iGetDblOpt(Inf,"OCTAGONBrake.TrqDistrib.FR", 0.35);    /*brake torque distribution FR*/
	sOCTAGONBrakeParameters->TrqDistrib[2] = iGetDblOpt(Inf,"OCTAGONBrake.TrqDistrib.RL", 0.15);    /*brake torque distribution RL*/
	sOCTAGONBrakeParameters->TrqDistrib[3] = iGetDblOpt(Inf,"OCTAGONBrake.TrqDistrib.RR", 0.15);    /*brake torque distribution RR*/
	sOCTAGONBrakeParameters->deadTime_init = iGetDblOpt(Inf,"OCTAGONBrake.DeadTime_init", 0.04);    /*Dead time before torque build up*/
	sOCTAGONBrakeParameters->deadTime_CAN = iGetDblOpt(Inf, "OCTAGONBrake.DeadTime_CAN", 0.13);    /*Dead time before torque build up*/
	sOCTAGONBrakeParameters->a_max_decel = iGetDblOpt(Inf, "OCTAGONBrake.max_decel_mps2", 10.0);    /*Dead time before torque build up*/

	/*PT1 brake*/
	sOCTAGONBrakeParameters->TimeConstPT1_Driver = iGetDblOpt(Inf,"OCTAGONBrake.TimeConstPT1_Driver", 0.015);    /*Torque PT1 delay time for braking through driver*/
	sOCTAGONBrakeParameters->TimeConstPT1_EBA = iGetDblOpt(Inf,"OCTAGONBrake.TimeConstPT1_EBA", 0.013);  /*Torque PT1 delay time for algp braking*/
	/*Ramped brake*/
	sOCTAGONBrakeParameters->RampGradient_Driver = iGetDblOpt(Inf, "OCTAGONBrake.RampGradient_Driver_Nm/s", 60000);    /*Torque PT1 delay time for braking through driver*/
	sOCTAGONBrakeParameters->RampGradient_EBA = iGetDblOpt(Inf, "OCTAGONBrake.RampGradient_EBA", 100);  /*Torque PT1 delay time for algo braking*/

	sOCTAGONBrakeParameters->PedalAct2pMC = iGetDblOpt(Inf, "OCTAGONBrake.PedalAct2pMC", 150);  /*Conversion - brake pedal actuation to master cylinder pressure*/
	sOCTAGONBrakeParameters->pWB2Trq = iGetDblOpt(Inf, "OCTAGONBrake.pWB2Trq", 40);  /*Conversion -  master cylinder pressure to brake torque*/

	sOCTAGONBrakeParameters->Trq_WB[0] = 0; 
	sOCTAGONBrakeParameters->Trq_WB[1] = 0;
	sOCTAGONBrakeParameters->Trq_WB[2] = 0;
	sOCTAGONBrakeParameters->Trq_WB[3] = 0;

	resetGlobalsToZero();

	sOCTAGONBrakeParameters->UseABS = iGetIntOpt(Inf, "OCTAGONBrake.UseABS", 1);		/*Use ABS torque limitation to physical possible value 1:=yes 0:=no*/
	sOCTAGONBrakeParameters->UseMode = iGetIntOpt(Inf, "OCTAGONBrake.UseMode", 0);		/*UseMode 1:=PT1 transient behavior, 0:=Ramp behavior*/
	sOCTAGONBrakeParameters->InputKind = iGetIntOpt(Inf, "OCTAGONBrake.InputKind", 0);  /*InputKind 0:=Acceleration, 1:=Pressure of Master Cylinder, 2:=Torque*/
	sOCTAGONBrakeParameters->ABS_factor = iGetDblOpt(Inf, "OCTAGONBrake.ABS_factor", 0.99);  /*Factor for limiting brake torque output based on vertical force, mue and tire radius (Based on physical limits)*/

	if (sOCTAGONBrakeParameters->RampGradient_Driver < sOCTAGONBrakeParameters->RampGradient_EBA)
	LogErrF (EC_Init, "Parameter OCTAGONBrake.RampGradient_Driver cannot be lesser than parameter OCTAGONBrake.RampGradient_EBA");

	if (sOCTAGONBrakeParameters->TimeConstPT1_Driver < sOCTAGONBrakeParameters->TimeConstPT1_EBA)
		LogErrF(EC_Init, "Parameter OCTAGONBrake.TimeConstPT1_Driver cannot be lesser than parameter OCTAGONBrake.TimeConstPT1_EBA");

	if (sOCTAGONBrakeParameters->TrqDistrib[0]+sOCTAGONBrakeParameters->TrqDistrib[1]+sOCTAGONBrakeParameters->TrqDistrib[2]+sOCTAGONBrakeParameters->TrqDistrib[3] != 1)
	LogErrF (EC_Init, "Brake torque distribution does not sum-up to 1.0");

	sOCTAGONBrakeParameters->PT2_Coeff_b0 = iGetDblOpt(Inf, "OCTAGONBrake.PT2_Coeff_b0", 1);
	sOCTAGONBrakeParameters->PT2_Coeff_a0 = iGetDblOpt(Inf, "OCTAGONBrake.PT2_Coeff_a0", 1);
	sOCTAGONBrakeParameters->PT2_Coeff_a1 = iGetDblOpt(Inf, "OCTAGONBrake.PT2_Coeff_a1", 0.1211);
	sOCTAGONBrakeParameters->PT2_Coeff_a2 = iGetDblOpt(Inf, "OCTAGONBrake.PT2_Coeff_a2", 0.01);

	b0 = sOCTAGONBrakeParameters->PT2_Coeff_b0;
	a0 = sOCTAGONBrakeParameters->PT2_Coeff_a0;
	a1 = sOCTAGONBrakeParameters->PT2_Coeff_a1;
	a2 = sOCTAGONBrakeParameters->PT2_Coeff_a2;

	return sOCTAGONBrakeParameters;
}

/*
** OCTAGONBrake_DeclQuants ()
**
** Defining DataDictionary Quantities
**
** Call:
** - at the beginning of every TestRun, after the call for the _New function
*/

static void OCTAGONBrake_DeclQuants (void *MP)
{
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequestInput", "-", &brakeRequestInput, DVA_DM);
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequest_Ramped", "-", &req, DVA_DM);
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequestAccel", "m/s^2", &brakeRequestAccel, DVA_DM);
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequestAccel_CANdelayed", "m/s^2", &brakeRequestAccel_delayed, DVA_None);
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequestTrq", "Nm", &brakeRequestTrq, DVA_DM);
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequestTrq_CANdelayed", "Nm", &brakeRequestTrq_delayed, DVA_DM);
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequestPmc", "bar", &brakeRequestPmc, DVA_DM);
	DDefDouble(NULL, "OCTAGONBrake.BrakeRequestPmc_CANdelayed", "bar", &brakeRequestPmc_delayed, DVA_DM);
	DDefDouble(NULL, "OCTAGONBrake.TimeSinceBrakeReq_Driver", "s", &timeSinceBrakeRequest_driver, DVA_None);
	DDefDouble(NULL, "OCTAGONBrake.TimeSinceBrakeReq", "s", &timeSinceBrakeRequest, DVA_None);
	DDefDouble(NULL, "OCTAGONBrake.Driver_BrakeTorque", "Nm", &driver_BrakeMoment, DVA_None);
	DDefDouble(NULL, "OCTAGONBrake.EBA_BrakeTorque", "Nm", &EBA_brakemoment, DVA_None);
	DDefDouble(NULL, "OCTAGONBrake.Final_BrakeTorque", "Nm", &finalmoment, DVA_None);
	DDefInt   (NULL, "OCTAGONBrake.ABS_Active_front", "-", &ABS_active_front, DVA_None);
	DDefInt	  (NULL, "OCTAGONBrake.ABS_Active_rear", "-", &ABS_active_rear, DVA_None);
}

/*
** OCTAGONBrake_Calc ()
**
** Calculation of the model
**
** Call:
** - every cycle
*/
static int OCTAGONBrake_Calc(void	           *MP, /* model parameter handle	*/
	struct tBrakeIF *IF, /* brake interface structure	*/
	double           dt) /* time step */
{
	sOCTAGONBrake_t *sOCTAGONBrakeParameters = (sOCTAGONBrake_t *)MP;
	int i;
	double sumTrqDistr = 0.0;
	double driver_BrakeMoment_tot = 0.0;
	double EBA_brakemoment_tmp = 0.0;

	for (i = 0; i < NWHEEL; i++)
	{
		sumTrqDistr += sOCTAGONBrakeParameters->TrqDistrib[i]; /*should sum up to one*/
	}

	////////////////// account for driver braking ////////////////
	if (IF->Pedal > 0.0f)
	{
		/*driver brake torque based on CarMaker Pressure Distibution model: pedal actuation to master cylinder pressure*/
		for (i = 0; i < NWHEEL; i++)
		{
			driver_BrakeMoment_tot += IF->Pedal * sOCTAGONBrakeParameters->PedalAct2pMC * sOCTAGONBrakeParameters->pWB2Trq * sOCTAGONBrakeParameters->TrqDistrib[i];
		}

		if (freezeRampedTime_driver == false)
		{ /*only increment timeSinceBrakeRequest if change in brake request detected*/
			timeSinceBrakeRequest_driver += dt;
		}
		if (timeSinceBrakeRequest_driver > sOCTAGONBrakeParameters->deadTime_init)
		{// y(n) = Tc*(K*xn – yn-1) + yn-1). - Analytical solution for first order time delay (PT1 element)
			if (sOCTAGONBrakeParameters->UseMode == 1) { // PT1
				driver_BrakeMoment = 1/(sOCTAGONBrakeParameters->TimeConstPT1_Driver / dt + 1) * (1 * driver_BrakeMoment_tot - driver_BrakeMoment) + driver_BrakeMoment;
				// Freeze time if torque reached 99% of set value
				if (driver_BrakeMoment >= 0.99 * driver_BrakeMoment_tot)  freezeRampedTime_driver = true;
			}
			else if (sOCTAGONBrakeParameters->UseMode == 2) // PT2 -> for the toqre increase
			{
				//b0 = b0_default * Vehicle.Cfg.MassTotal * Vehicle.Cfg.WhlRadius;
				driver_BrakeMoment = 2 * y_PT2_kTime_1 - y_PT2_kTime_2 + (u_PT2_kTime_2 * b0 / a2 - y_PT2_kTime_2 * a0 / a2) * dt * dt + (y_PT2_kTime_2 - y_PT2_kTime_1) * a1 / a2 * dt;
				y_PT2_kTime_2 = y_PT2_kTime_1;
				y_PT2_kTime_1 = driver_BrakeMoment;
				u_PT2_kTime_2 = u_PT2_kTime_1;
				u_PT2_kTime_1 = driver_BrakeMoment_tot;
			}
			else
			{ // Ramp
				if (( sOCTAGONBrakeParameters->RampGradient_Driver * (timeSinceBrakeRequest_driver - sOCTAGONBrakeParameters->deadTime_init)) <= driver_BrakeMoment_tot) {
					driver_BrakeMoment = sOCTAGONBrakeParameters->RampGradient_Driver *(timeSinceBrakeRequest_driver - sOCTAGONBrakeParameters->deadTime_init);
					freezeRampedTime_driver = false;
				}
				else {
					driver_BrakeMoment = driver_BrakeMoment_tot;
					freezeRampedTime_driver = true;
				}
			}
		}

		/*Set brake light*/
		if (IF->Pedal >= 0.8f) {
			/* Brake with blinking lights (2.5Hz) for high EBA requests. Duration set to 100ms if cycle time is not 1ms */
			if (((int)(SimCore.Time * 5) % 2) == 0) {
				DVA_WriteRequest("VC.Lights.Brake", OWMode_Abs, 100, 1, 1.0, 1.0, NULL);
			}
			else {
				DVA_WriteRequest("VC.Lights.Brake", OWMode_Abs, 100, 1, 0.0, 0.0, NULL);
			}
			if (((int)(SimCore.Time * 2) % 2) == 0) {
				DVA_WriteRequest("VC.Lights.IndL", OWMode_Abs, 100, 1, 2.0, 2.0, NULL);
				DVA_WriteRequest("VC.Lights.IndR", OWMode_Abs, 100, 1, 2.0, 2.0, NULL);
			}
			else {
				DVA_WriteRequest("VC.Lights.IndL", OWMode_Abs, 100, 1, 1.0, 1.0, NULL);
				DVA_WriteRequest("VC.Lights.IndR", OWMode_Abs, 100, 1, 1.0, 1.0, NULL);
			}
		}
		/* if the driver brake is zero, the time counting for PT1 is set to zero. */
	}
	else
	{
		timeSinceBrakeRequest_driver = 0;
		driver_BrakeMoment = 0;
	}

	/*Queue for brake request through CAN and gateway*/
	if (sOCTAGONBrakeParameters->deadTime_CAN > 0) {
		q_req.push(brakeRequestInput);
		if (sOCTAGONBrakeParameters->deadTime_CAN / dt < q_req.size()) { /*queue is "full" --> copy delayed value*/
			brakeRequestInput_delayed = q_req.front(); /*read value*/
			q_req.pop(); /*remove last value and shift entries*/
		}
		else if (q_req.size() == 1) { /*first cycle--> copy valid initial values*/
			brakeRequestInput_delayed = q_req.front();
		}
	}
	else {
		brakeRequestInput_delayed = brakeRequestInput; /* use brakeRequestInput directly */
	}
	
	////////////////// account for algo braking ////////////////
	/*InputKind acceleration & torque have negative input, pmc has positive input */
	if (brakeRequestInput_delayed < 0 && (sOCTAGONBrakeParameters->InputKind == acceleration || sOCTAGONBrakeParameters->InputKind == torque) || (brakeRequestInput_delayed > 0 && sOCTAGONBrakeParameters->InputKind == pmc))
	{
		if (sOCTAGONBrakeParameters->UseMode == 1) //PT1
		{/*Brake Request by Input Interface*/
			if (freezeRampedTime == false)
			{ /*only increment timeSinceBrakeRequest if change in brake request detected*/
				timeSinceBrakeRequest += dt;
			}

			// Max allowed deceleration to be fed by the algo for every simulation cycle
			if (timeSinceBrakeRequest > sOCTAGONBrakeParameters->deadTime_init)
			{
				/* equation to generate req through a Pt1 Delay: y(n) = Tc*(K*xn – yn-1) + yn-1). - Analytical solution for first order time delay (PT1 element) */
				req = 1 / (sOCTAGONBrakeParameters->TimeConstPT1_EBA / dt + 1) * (1 * brakeRequestInput_delayed - req) + req;
				// Freeze time if torque reached 99% of set value
				if (req >= 0.99 * brakeRequestInput_delayed) freezeRampedTime = true;
			}
		}
		else if (sOCTAGONBrakeParameters->UseMode == 2) // PT2 -> for the toqre increase
		{/*Brake Request by Input Interface*/
			if (freezeRampedTime == false)
			{ /*only increment timeSinceBrakeRequest if change in brake request detected*/
				timeSinceBrakeRequest += dt;
			}

			// Max allowed deceleration to be fed by the algo for every simulation cycle
			if (timeSinceBrakeRequest > sOCTAGONBrakeParameters->deadTime_init)
			{
				req = 2 * y_PT2_kTime_1 - y_PT2_kTime_2 + (u_PT2_kTime_2 * b0 / a2 - y_PT2_kTime_2 * a0 / a2) * dt * dt + (y_PT2_kTime_2 - y_PT2_kTime_1) * a1 / a2 * dt;
				y_PT2_kTime_2 = y_PT2_kTime_1;
				y_PT2_kTime_1 = req;
				u_PT2_kTime_2 = u_PT2_kTime_1;
				u_PT2_kTime_1 = brakeRequestInput_delayed;
		
				// Freeze time if torque reached 99% of set value
				if (abs(req) >= abs(0.99 * brakeRequestInput_delayed)) freezeRampedTime = true;
			}

		}
		else // Ramp
		{
			{/*Brake Request by Input Interface*/
				if (freezeRampedTime == false)
				{ /*only increment timeSinceBrakeRequest if change in brake request detected*/
					timeSinceBrakeRequest += dt;
				}
				if (timeSinceBrakeRequest > sOCTAGONBrakeParameters->deadTime_init)
				{
					switch (sOCTAGONBrakeParameters->InputKind) 
					{
					case acceleration: /*Acceleration-Input < 0*/
						if (-1.0 * (sOCTAGONBrakeParameters->RampGradient_EBA * (timeSinceBrakeRequest - sOCTAGONBrakeParameters->deadTime_init)) >= brakeRequestInput_delayed)
						{/*if brakeRequestInput_delayed changes then incrementally ramp req to brakeRequestInput_delayed*/
							req = -1.0 * sOCTAGONBrakeParameters->RampGradient_EBA *(timeSinceBrakeRequest - sOCTAGONBrakeParameters->deadTime_init);
							freezeRampedTime = false;
						}
						else
						{/*otherwise save value and do not increment timeSinceBrakeRequest until brakeRequestInput_delayed changes again*/
							req = brakeRequestInput_delayed;
							freezeRampedTime = true;
						} break;

					case pmc: /*PMC-Input*/
						if ((sOCTAGONBrakeParameters->RampGradient_EBA * (timeSinceBrakeRequest - sOCTAGONBrakeParameters->deadTime_init)) >= brakeRequestInput_delayed)
						{/*if brakeRequestInput_delayed changes then incrementally ramp req to brakeRequestInput_delayed*/
							req = sOCTAGONBrakeParameters->RampGradient_EBA *(timeSinceBrakeRequest - sOCTAGONBrakeParameters->deadTime_init);
							freezeRampedTime = false;
						}
						else
						{/*otherwise save value and do not increment timeSinceBrakeRequest until brakeRequestInput_delayed changes again*/
							req = brakeRequestInput_delayed;
							freezeRampedTime = true;
						} break;
					case torque:	/*Torque-Input*/
						if (-1.0 * (sOCTAGONBrakeParameters->RampGradient_EBA * (timeSinceBrakeRequest - sOCTAGONBrakeParameters->deadTime_init)) >= brakeRequestInput_delayed)
						{/*if brakeRequestInput_delayed changes then incrementally ramp req to brakeRequestInput_delayed*/
							req = -1.0 * sOCTAGONBrakeParameters->RampGradient_EBA *(timeSinceBrakeRequest - sOCTAGONBrakeParameters->deadTime_init);
							freezeRampedTime = false;
						}
						else
						{/*otherwise save value and do not increment timeSinceBrakeRequest until brakeRequestInput_delayed changes again*/
							req = brakeRequestInput_delayed;
							freezeRampedTime = true;
						} break;
					default: break;
					}
				}
			}
		}
	}
	// pending - add condition for parking brakes

	if (rFrozen == false)
	{/*remember and freeze tire radius at FA & RA for rest of EBA braking to avoid oscillation*/
		rTireFA = (Car.Tire[0].WRadius + Car.Tire[1].WRadius) / 2.0; /*FL & FR*/
		rTireRA = (Car.Tire[2].WRadius + Car.Tire[3].WRadius) / 2.0; /*RL & RR*/
		rFrozen = true; /*remember and freeze value only once at beginning of EBA braking*/
	}

	switch (sOCTAGONBrakeParameters->InputKind) {
	case acceleration:		/*Acceleration-Input*/
		if (req <= 0.0)    /*Equal zero to be able to reset to zero once no request is active*/
		{

		/*a --> accel (<0)
		T --> Total Torque
		m --> Vehicle mass
		r --> wheel radius
		a = T/(r*m) --> T = a*m*r --> T_W = T*T_Distrib*/
		
			EBA_brakemoment = min(-1.0 * req * Vehicle.Cfg.MassTotal *rTireFA, 1.0 * sOCTAGONBrakeParameters->a_max_decel * Vehicle.Cfg.MassTotal *rTireFA);
			brakeRequestAccel = brakeRequestInput;
			brakeRequestAccel_delayed = brakeRequestInput_delayed;
			aReq = req;		
		}	break;

	case pmc:		/*PMC-Input*/
		if (req >= 0.0)    /*Equal zero to be able to reset to zero once no request is active*/
		{
			for (i = 0; i < NWHEEL; i++)
			{
				EBA_brakemoment_tmp += req * sOCTAGONBrakeParameters->pWB2Trq * sOCTAGONBrakeParameters->TrqDistrib[i]; /*Convert pressure input to turque using factor pWB2Trq*/
			}

			EBA_brakemoment = EBA_brakemoment_tmp;
			brakeRequestPmc = brakeRequestInput;
			brakeRequestPmc_delayed = brakeRequestInput_delayed;
			aReq = -1 * (EBA_brakemoment / (Vehicle.Cfg.MassTotal * rTireFA));
		}	break;

	case torque:		/*Torque-Input*/
		if (req <= 0.0)    /*Equal zero to be able to reset to zero once no request is active*/
		{
			EBA_brakemoment = -1.0 * req;    /*Torque input is negative, but Cm wheel brake torque IF->Trq_WB[i] is positive*/
			brakeRequestTrq = brakeRequestInput;
			brakeRequestTrq_delayed = brakeRequestInput_delayed;
			aReq = EBA_brakemoment / (Vehicle.Cfg.MassTotal * rTireFA);    /*Torque input is negative, but Cm wheel brake torque IF->Trq_WB[i] is positive*/
		}	break;

	default: EBA_brakemoment = 0.0; break;	/*No allowed Input-Kind selected*/
	}
	/*check which moment is lowest and assign that to the CarMaker*/
	if (abs(driver_BrakeMoment) > abs(EBA_brakemoment)) 
	{
		finalmoment =  driver_BrakeMoment;
	} else
	{
		finalmoment =  EBA_brakemoment;
	}
  
	if (sOCTAGONBrakeParameters->UseABS == 1) {
		for (i = 0; i < NWHEEL; i++)
		{ /* Limit torque by maximum applicable torque the tire can physically do (Vertical tire force * radius * mue road). Limit to 90% to allow lat. forces & take ABS unperfect behavior into account.*/
			if (i < 2) { /*FL & FR*/
				sOCTAGONBrakeParameters->Trq_WB[0] = min(sOCTAGONBrakeParameters->TrqDistrib[0] / sumTrqDistr * finalmoment, sOCTAGONBrakeParameters->ABS_factor * (Vehicle.FL.Fz * Car.Tire[0].muRoad * Car.Tire[0].WRadius));
				sOCTAGONBrakeParameters->Trq_WB[1] = min(sOCTAGONBrakeParameters->TrqDistrib[1] / sumTrqDistr * finalmoment, sOCTAGONBrakeParameters->ABS_factor * (Vehicle.FR.Fz * Car.Tire[1].muRoad * Car.Tire[1].WRadius));
				if (sOCTAGONBrakeParameters->ABS_factor * (Vehicle.FL.Fz * Car.Tire[0].muRoad * Car.Tire[0].WRadius) <= sOCTAGONBrakeParameters->TrqDistrib[0] / sumTrqDistr * finalmoment || sOCTAGONBrakeParameters->ABS_factor * (Vehicle.FR.Fz * Car.Tire[1].muRoad * Car.Tire[1].WRadius) <= sOCTAGONBrakeParameters->TrqDistrib[1] / sumTrqDistr * finalmoment) {
					sOCTAGONBrakeParameters->Trq_WB[0] = sOCTAGONBrakeParameters->Trq_WB[1] = min(sOCTAGONBrakeParameters->Trq_WB[0], sOCTAGONBrakeParameters->Trq_WB[1]); // select-low
					ABS_active_front = 1;
				}
				else { ABS_active_front = 0; }
			}
			else { /*RL & RR*/
				sOCTAGONBrakeParameters->Trq_WB[2] = min(sOCTAGONBrakeParameters->TrqDistrib[2] / sumTrqDistr * finalmoment, sOCTAGONBrakeParameters->ABS_factor * (Vehicle.RL.Fz * Car.Tire[2].muRoad * Car.Tire[2].WRadius));
				sOCTAGONBrakeParameters->Trq_WB[3] = min(sOCTAGONBrakeParameters->TrqDistrib[3] / sumTrqDistr * finalmoment, sOCTAGONBrakeParameters->ABS_factor * (Vehicle.RR.Fz * Car.Tire[3].muRoad * Car.Tire[3].WRadius));
				if (sOCTAGONBrakeParameters->ABS_factor * (Vehicle.RL.Fz * Car.Tire[2].muRoad * Car.Tire[2].WRadius) <= sOCTAGONBrakeParameters->TrqDistrib[2] / sumTrqDistr * finalmoment || sOCTAGONBrakeParameters->ABS_factor * (Vehicle.RR.Fz * Car.Tire[3].muRoad * Car.Tire[3].WRadius) <= sOCTAGONBrakeParameters->TrqDistrib[3] / sumTrqDistr * finalmoment) {
					sOCTAGONBrakeParameters->Trq_WB[2] = sOCTAGONBrakeParameters->Trq_WB[3] = min(sOCTAGONBrakeParameters->Trq_WB[2], sOCTAGONBrakeParameters->Trq_WB[3]); // select-low
					ABS_active_rear = 1;
				}
				else { ABS_active_rear = 0; }
			}
			IF->Trq_WB[i] = sOCTAGONBrakeParameters->Trq_WB[i];
		}
	}
	else
		{
		for (i = 0; i < NWHEEL; i++)
		{ /* Limit torque by maximum applicable torque the tire can physically do (Vertical tire force * radius * mue road). Limit to 90% to allow lat. forces & take ABS unperfect behavior into account.*/
			if (i < 2) { /*FL & FR*/
				sOCTAGONBrakeParameters->Trq_WB[0] = sOCTAGONBrakeParameters->TrqDistrib[0] / sumTrqDistr * finalmoment;
				sOCTAGONBrakeParameters->Trq_WB[1] = sOCTAGONBrakeParameters->TrqDistrib[1] / sumTrqDistr * finalmoment;
			}
			else { /*RL & RR*/
				sOCTAGONBrakeParameters->Trq_WB[2] = sOCTAGONBrakeParameters->TrqDistrib[2] / sumTrqDistr * finalmoment;
				sOCTAGONBrakeParameters->Trq_WB[3] = sOCTAGONBrakeParameters->TrqDistrib[3] / sumTrqDistr * finalmoment;
			}
			IF->Trq_WB[i] = sOCTAGONBrakeParameters->Trq_WB[i];
		}
	}

	/*Reset*/
	if (IF->Pedal == 0.0f)
	{
		freezeRampedTime_driver = false;
		timeSinceBrakeRequest_driver = 0;
	}
	if (brakeRequestInput_delayed >= 0.0f && (sOCTAGONBrakeParameters->InputKind == acceleration || sOCTAGONBrakeParameters->InputKind == torque)) // fo acceleration & torque request (negative input value for braking)
	{
		freezeRampedTime = false;
		timeSinceBrakeRequest = 0;
		req = brakeRequestInput_delayed;
	}
	if (brakeRequestInput_delayed <= 0.0f && sOCTAGONBrakeParameters->InputKind == pmc) // fo pressure positive input value for braking
	{
		freezeRampedTime = false;
		timeSinceBrakeRequest = 0;
		req = brakeRequestInput_delayed;
	}

	/*Set brake light*/
	if (aReq < 0.0 && aReq > -7.0)
	{
		DVA_WriteRequest("VC.Lights.Brake", OWMode_Abs, 1, 1, 1.0, 1.0, NULL);
		//DVA_WriteRequest("Driver.Long.passive", OWMode_Abs, 1.0, 0, 0, 1.0, NULL);
		//DrivMan.Clutch = 1; // TODO: Interferes with requests send to Powertrain
		//DrivMan.Gas = 0; // TODO: Interferes with requests send to Powertrain
		//DrivMan.Brake = 0;
	}
	else if (aReq <= -7.0)
	{
		/*Brake with blinking lights (2.5Hz) for high EBA requests. Duration set to 100ms if cycle time is not 1ms*/
		if (((int)(SimCore.Time * 5) % 2) == 0) {
			DVA_WriteRequest("VC.Lights.Brake", OWMode_Abs, 100, 1, 1.0, 1.0, NULL);
		}
		else {
			DVA_WriteRequest("VC.Lights.Brake", OWMode_Abs, 100, 1, 0.0, 0.0, NULL);
		}
		if (((int)(SimCore.Time * 2) % 2) == 0) {
			DVA_WriteRequest("VC.Lights.IndL", OWMode_Abs, 100, 1, 2.0, 2.0, NULL);
			DVA_WriteRequest("VC.Lights.IndR", OWMode_Abs, 100, 1, 2.0, 2.0, NULL);
		}
		else {
			DVA_WriteRequest("VC.Lights.IndL", OWMode_Abs, 100, 1, 1.0, 1.0, NULL);
			DVA_WriteRequest("VC.Lights.IndR", OWMode_Abs, 100, 1, 1.0, 1.0, NULL);
		}
		// If EBA braking torques go through, then the longitudinal driver controller is set to passive and the clutch pedal is pressed
		//DVA_WriteRequest("Driver.Long.passive", OWMode_Abs, 1.0, 0, 0, 1.0, NULL);
		DrivMan.Clutch = 1;
		DrivMan.Gas = 0;
		//DrivMan.Brake = 0;
	}
	return 0;
}


/*
** OCTAGONBrake_Delete ()
**
** Uninitializing the model
**
** Call:
** - at the end of every TestRun
*/
static void OCTAGONBrake_Delete (void *MP)
{
	sOCTAGONBrake_t *sOCTAGONBrakeParameters = (sOCTAGONBrake_t *)MP;
	// reset the long. passive to zero
	DVA_ReleaseQuant("Driver.Long.passive");
	DVA_ReleaseQuant("VC.Lights.Brake");
	DVA_ReleaseQuant("VC.Lights.IndL");
	DVA_ReleaseQuant("VC.Lights.IndR");
	resetGlobalsToZero();

	/* Park the dict entries for dynamically allocated variables before deleting */
	free (sOCTAGONBrakeParameters);
}


/* 
** Brake_Register_OCTAGONBrake
**
*/
int Brake_Register_OCTAGONBrake (void)
{
	tModelClassDescr mp;

	memset (&mp, 0, sizeof(mp));
	mp.Brake.New        = OCTAGONBrake_New;
	mp.Brake.Calc       = OCTAGONBrake_Calc;
	mp.Brake.Delete     = OCTAGONBrake_Delete;
	mp.Brake.DeclQuants = OCTAGONBrake_DeclQuants;
	mp.Brake.VersionId  = ThisVersionId;

	return Model_Register(ModelClass_Brake, ThisModelKind, &mp);
}