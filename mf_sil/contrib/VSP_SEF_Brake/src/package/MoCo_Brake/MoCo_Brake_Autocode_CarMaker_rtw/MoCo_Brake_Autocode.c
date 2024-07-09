/*
 * MoCo_Brake_Autocode.c
 *
 * Code generation for model "MoCo_Brake_Autocode".
 *
 * Model version              : 1.52
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Jan 12 13:51:41 2022
 *
 * Target selection: CarMaker.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "MoCo_Brake_Autocode_capi.h"
#include <infoc.h>
#include <Log.h>
#include <InfoParam.h>
#include <DataDict.h>
#include <MatSupp.h>
#include "MoCo_Brake_Autocode.h"
#include "MoCo_Brake_Autocode_private.h"
#include "MoCo_Brake_Autocode_wrap.h"

/* CarMaker: Compile- and link-time checks for the right Matlab version. */
#if MATSUPP_NUMVER == 90700

/* The following statement will cause an "unresolved symbol" error when
   linking with a MatSupp library built for the wrong Matlab version. */
extern int MATSUPP_VARNAME(MatSupp, MATSUPP_NUMVER);
void *MATSUPP_VARNAME(MODEL, MATSUPP_NUMVER) = &MATSUPP_VARNAME(MatSupp,
  MATSUPP_NUMVER);

#else
# error Compiler options unsuitable for C code created with Matlab 9.7
#endif

/* Block parameters (default storage) */
P_MoCo_Brake_Autocode_T MoCo_Brake_Autocode_P = {
  /* Expression: -1
   * Referenced by: '<Root>/brake_torque_defined_positive1'
   */
  -1.0,

  /* Expression: -1
   * Referenced by: '<Root>/brake_torque_defined_positive'
   */
  -1.0
};

/*
 * This function updates continuous states using the ODE1 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si ,
  RT_MODEL_MoCo_Brake_Autocode_T *const MoCo_Brake_Autocode_M)
{
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE1_IntgData *id = (ODE1_IntgData *)rtsiGetSolverData(si);
  real_T *f0 = id->f[0];
  int_T i;
  int_T nXc = 7;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);
  rtsiSetdX(si, f0);
  MoCo_Brake_Autocode_derivatives(MoCo_Brake_Autocode_M);
  rtsiSetT(si, tnew);
  for (i = 0; i < nXc; ++i) {
    x[i] += h * f0[i];
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void MoCo_Brake_Autocode_step(RT_MODEL_MoCo_Brake_Autocode_T *const
  MoCo_Brake_Autocode_M)
{
  B_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_B = ((B_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->blockIO);
  DW_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_DW = ((DW_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->dwork);
  X_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_X = ((X_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->contStates);
  if (rtmIsMajorTimeStep(MoCo_Brake_Autocode_M)) {
    /* set solver stop time */
    if (!(MoCo_Brake_Autocode_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(MoCo_Brake_Autocode_M->solverInfo,
                            ((MoCo_Brake_Autocode_M->Timing.clockTickH0 + 1) *
        MoCo_Brake_Autocode_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(MoCo_Brake_Autocode_M->solverInfo,
                            ((MoCo_Brake_Autocode_M->Timing.clockTick0 + 1) *
        MoCo_Brake_Autocode_M->Timing.stepSize0 +
        MoCo_Brake_Autocode_M->Timing.clockTickH0 *
        MoCo_Brake_Autocode_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(MoCo_Brake_Autocode_M)) {
    MoCo_Brake_Autocode_M->Timing.t[0] = rtsiGetT
      (MoCo_Brake_Autocode_M->solverInfo);
  }

  if (rtmIsMajorTimeStep(MoCo_Brake_Autocode_M)) {
    /* S-Function (read_dict): '<Root>/Read CM Dict' */
    {
      tDDictEntry *e;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->ReadCMDict_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->ReadCMDict_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->ReadCMDict_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      MoCo_Brake_Autocode_B->SET_TRQ = e->GetFunc(e->Var);
    }

    /* Gain: '<Root>/brake_torque_defined_positive1' */
    MoCo_Brake_Autocode_B->brake_torque_defined_positive1 =
      MoCo_Brake_Autocode_P.brake_torque_defined_positive1_Gain *
      MoCo_Brake_Autocode_B->SET_TRQ;

    /* S-Function (read_dict): '<Root>/Read CM Dict1' */
    {
      tDDictEntry *e;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->ReadCMDict1_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->ReadCMDict1_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->ReadCMDict1_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      MoCo_Brake_Autocode_B->HOLD_REQ = e->GetFunc(e->Var);
    }

    /* S-Function (read_dict): '<Root>/Read CM Dict2' */
    {
      tDDictEntry *e;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->ReadCMDict2_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->ReadCMDict2_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->ReadCMDict2_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      MoCo_Brake_Autocode_B->GO_REQ = e->GetFunc(e->Var);
    }
  }

  /* ModelReference: '<Root>/Model' */
  mdl_MoCo_Brake(&(MoCo_Brake_Autocode_DW->Model_InstanceData.rtm),
                 &MoCo_Brake_Autocode_B->brake_torque_defined_positive1,
                 &MoCo_Brake_Autocode_B->HOLD_REQ,
                 &MoCo_Brake_Autocode_B->GO_REQ,
                 &MoCo_Brake_Autocode_B->Model_o1,
                 &MoCo_Brake_Autocode_B->Model_o2,
                 &MoCo_Brake_Autocode_B->Model_o3,
                 &MoCo_Brake_Autocode_B->Model_o4,
                 &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtb),
                 &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtdw),
                 &(MoCo_Brake_Autocode_X->Model_CSTATE),
                 &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtzce));
  if (rtmIsMajorTimeStep(MoCo_Brake_Autocode_M)) {
    /* Gain: '<Root>/brake_torque_defined_positive' */
    MoCo_Brake_Autocode_B->brake_torque_defined_positive =
      MoCo_Brake_Autocode_P.brake_torque_defined_positive_Gain *
      MoCo_Brake_Autocode_B->Model_o2;

    /* Product: '<Root>/Multiply' incorporates:
     *  Constant: '<Root>/Constant'
     */
    MoCo_Brake_Autocode_B->Multiply =
      MoCo_Brake_Autocode_B->brake_torque_defined_positive * rtP_Trq_Distrib_FL;

    /* S-Function (write_dict): '<Root>/Write CM Dict' */
    {
      tDDictEntry *e;
      real_T value;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->WriteCMDict_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->WriteCMDict_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->WriteCMDict_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      value = MoCo_Brake_Autocode_B->Multiply;
      DVA_PokeSL(e, value);
    }

    /* Product: '<Root>/Multiply1' incorporates:
     *  Constant: '<Root>/Constant1'
     */
    MoCo_Brake_Autocode_B->Multiply1 =
      MoCo_Brake_Autocode_B->brake_torque_defined_positive * rtP_Trq_Distrib_FR;

    /* S-Function (write_dict): '<Root>/Write CM Dict1' */
    {
      tDDictEntry *e;
      real_T value;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->WriteCMDict1_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->WriteCMDict1_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->WriteCMDict1_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      value = MoCo_Brake_Autocode_B->Multiply1;
      DVA_PokeSL(e, value);
    }

    /* Product: '<Root>/Multiply2' incorporates:
     *  Constant: '<Root>/Constant2'
     */
    MoCo_Brake_Autocode_B->Multiply2 =
      MoCo_Brake_Autocode_B->brake_torque_defined_positive * rtP_Trq_Distrib_RL;

    /* S-Function (write_dict): '<Root>/Write CM Dict2' */
    {
      tDDictEntry *e;
      real_T value;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->WriteCMDict2_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->WriteCMDict2_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->WriteCMDict2_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      value = MoCo_Brake_Autocode_B->Multiply2;
      DVA_PokeSL(e, value);
    }

    /* Product: '<Root>/Multiply3' incorporates:
     *  Constant: '<Root>/Constant3'
     */
    MoCo_Brake_Autocode_B->Multiply3 = rtP_Trq_Distrib_RR *
      MoCo_Brake_Autocode_B->brake_torque_defined_positive;

    /* S-Function (write_dict): '<Root>/Write CM Dict3' */
    {
      tDDictEntry *e;
      real_T value;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->WriteCMDict3_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->WriteCMDict3_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->WriteCMDict3_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      value = MoCo_Brake_Autocode_B->Multiply3;
      DVA_PokeSL(e, value);
    }

    /* S-Function (write_dict): '<Root>/Write CM Dict4' */
    {
      tDDictEntry *e;
      real_T value;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->WriteCMDict4_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->WriteCMDict4_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->WriteCMDict4_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      value = MoCo_Brake_Autocode_B->brake_torque_defined_positive;
      DVA_PokeSL(e, value);
    }

    /* S-Function (write_dict): '<Root>/Write CM Dict5' */
    {
      tDDictEntry *e;
      real_T value;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->WriteCMDict5_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->WriteCMDict5_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->WriteCMDict5_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      value = MoCo_Brake_Autocode_B->Model_o3;
      DVA_PokeSL(e, value);
    }

    /* S-Function (write_dict): '<Root>/Write CM Dict6' */
    {
      tDDictEntry *e;
      real_T value;
      e = (tDDictEntry *)MoCo_Brake_Autocode_DW->WriteCMDict6_PWORK.Entry;
      if (!MoCo_Brake_Autocode_DW->WriteCMDict6_IWORK.Checked) {
        MoCo_Brake_Autocode_DW->WriteCMDict6_IWORK.Checked = 1;
        DDictErrorUponFakedEntry(e, "Model 'MoCo_Brake_Autocode'");
      }

      value = MoCo_Brake_Autocode_B->Model_o4;
      DVA_PokeSL(e, value);
    }
  }

  if (rtmIsMajorTimeStep(MoCo_Brake_Autocode_M)) {
    /* Update for ModelReference: '<Root>/Model' */
    mdl_MoCo_Brake_Update(&(MoCo_Brake_Autocode_DW->Model_InstanceData.rtm),
                          &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtb),
                          &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtdw));
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(MoCo_Brake_Autocode_M)) {
    rt_ertODEUpdateContinuousStates(MoCo_Brake_Autocode_M->solverInfo,
      MoCo_Brake_Autocode_M);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++MoCo_Brake_Autocode_M->Timing.clockTick0)) {
      ++MoCo_Brake_Autocode_M->Timing.clockTickH0;
    }

    MoCo_Brake_Autocode_M->Timing.t[0] = rtsiGetSolverStopTime
      (MoCo_Brake_Autocode_M->solverInfo);

    {
      /* Update absolute timer for sample time: [0.001s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.001, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      MoCo_Brake_Autocode_M->Timing.clockTick1++;
      if (!MoCo_Brake_Autocode_M->Timing.clockTick1) {
        MoCo_Brake_Autocode_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void MoCo_Brake_Autocode_derivatives(RT_MODEL_MoCo_Brake_Autocode_T *const
  MoCo_Brake_Autocode_M)
{
  DW_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_DW = ((DW_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->dwork);
  X_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_X = ((X_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->contStates);

  /* Derivatives for ModelReference: '<Root>/Model' */
  mdl_MoCo_Brake_Deriv(&(MoCo_Brake_Autocode_DW->Model_InstanceData.rtb),
                       &(MoCo_Brake_Autocode_X->Model_CSTATE),
                       &(((XDot_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->derivs)->Model_CSTATE));
}

/* Model initialize function */
void MoCo_Brake_Autocode_initialize(RT_MODEL_MoCo_Brake_Autocode_T *const
  MoCo_Brake_Autocode_M)
{
  DW_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_DW = ((DW_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->dwork);
  X_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_X = ((X_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->contStates);

  /* Start for S-Function (read_dict): '<Root>/Read CM Dict' */
  {
    char **namevec = CM_Names2StrVec("MoCoBrake.Target_Trq", NULL);
    MoCo_Brake_Autocode_DW->ReadCMDict_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->ReadCMDict_PWORK.Entry = (void *)DDictGetEntryOrFake
      (namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (read_dict): '<Root>/Read CM Dict1' */
  {
    char **namevec = CM_Names2StrVec("MoCoBrake.Ssm_Hold_Req", NULL);
    MoCo_Brake_Autocode_DW->ReadCMDict1_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->ReadCMDict1_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (read_dict): '<Root>/Read CM Dict2' */
  {
    char **namevec = CM_Names2StrVec("MoCoBrake.Ssm_Go_Req", NULL);
    MoCo_Brake_Autocode_DW->ReadCMDict2_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->ReadCMDict2_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for ModelReference: '<Root>/Model' */
  mdl_MoCo_Brake_Start(&(MoCo_Brake_Autocode_DW->Model_InstanceData.rtm),
                       &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtdw));

  /* Start for S-Function (write_dict): '<Root>/Write CM Dict' */
  {
    char **namevec = CM_Names2StrVec("Brake.Trq_FL_ext", NULL);
    MoCo_Brake_Autocode_DW->WriteCMDict_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->WriteCMDict_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (write_dict): '<Root>/Write CM Dict1' */
  {
    char **namevec = CM_Names2StrVec("Brake.Trq_FR_ext", NULL);
    MoCo_Brake_Autocode_DW->WriteCMDict1_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->WriteCMDict1_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (write_dict): '<Root>/Write CM Dict2' */
  {
    char **namevec = CM_Names2StrVec("Brake.Trq_RL_ext", NULL);
    MoCo_Brake_Autocode_DW->WriteCMDict2_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->WriteCMDict2_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (write_dict): '<Root>/Write CM Dict3' */
  {
    char **namevec = CM_Names2StrVec("Brake.Trq_RR_ext", NULL);
    MoCo_Brake_Autocode_DW->WriteCMDict3_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->WriteCMDict3_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (write_dict): '<Root>/Write CM Dict4' */
  {
    char **namevec = CM_Names2StrVec("MoCoBrake.Veh_Trq", NULL);
    MoCo_Brake_Autocode_DW->WriteCMDict4_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->WriteCMDict4_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (write_dict): '<Root>/Write CM Dict5' */
  {
    char **namevec = CM_Names2StrVec("MoCoBrake.Ssm_Status", NULL);
    MoCo_Brake_Autocode_DW->WriteCMDict5_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->WriteCMDict5_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* Start for S-Function (write_dict): '<Root>/Write CM Dict6' */
  {
    char **namevec = CM_Names2StrVec("MoCoBrake.Ssm_Torque", NULL);
    MoCo_Brake_Autocode_DW->WriteCMDict6_IWORK.Checked = 0;
    MoCo_Brake_Autocode_DW->WriteCMDict6_PWORK.Entry = (void *)
      DDictGetEntryOrFake(namevec[0]);
    CM_FreeStrVec(namevec);
  }

  /* SystemInitialize for ModelReference: '<Root>/Model' */
  mdl_MoCo_Brake_Init(&(MoCo_Brake_Autocode_DW->Model_InstanceData.rtm),
                      &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtdw),
                      &(MoCo_Brake_Autocode_X->Model_CSTATE));

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond(MoCo_Brake_Autocode_M)) {
    rtmSetFirstInitCond(MoCo_Brake_Autocode_M, 0);
  }
}

/* Model terminate function */
void MoCo_Brake_Autocode_terminate(RT_MODEL_MoCo_Brake_Autocode_T
  * MoCo_Brake_Autocode_M)
{
  rt_FREE(MoCo_Brake_Autocode_M->solverInfo);

  /* model code */
  rt_FREE(MoCo_Brake_Autocode_M->blockIO);
  rt_FREE(MoCo_Brake_Autocode_M->contStates);
  rt_FREE(MoCo_Brake_Autocode_M->dwork);
  rt_FREE(MoCo_Brake_Autocode_M);
}

/* Model data allocation function */
RT_MODEL_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode(struct tInfos *inf)
{
  RT_MODEL_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_M;
  MoCo_Brake_Autocode_M = (RT_MODEL_MoCo_Brake_Autocode_T *) malloc(sizeof
    (RT_MODEL_MoCo_Brake_Autocode_T));
  if (MoCo_Brake_Autocode_M == NULL) {
    return NULL;
  }

  (void) memset((char *)MoCo_Brake_Autocode_M, 0,
                sizeof(RT_MODEL_MoCo_Brake_Autocode_T));
  MatSupp_Init();

  {
    /* Setup solver object */
    RTWSolverInfo *rt_SolverInfo = (RTWSolverInfo *) malloc(sizeof(RTWSolverInfo));
    rt_VALIDATE_MEMORY(MoCo_Brake_Autocode_M,rt_SolverInfo);
    MoCo_Brake_Autocode_M->solverInfo = (rt_SolverInfo);
    rtsiSetSimTimeStepPtr(MoCo_Brake_Autocode_M->solverInfo,
                          &MoCo_Brake_Autocode_M->Timing.simTimeStep);
    rtsiSetTPtr(MoCo_Brake_Autocode_M->solverInfo, &rtmGetTPtr
                (MoCo_Brake_Autocode_M));
    rtsiSetStepSizePtr(MoCo_Brake_Autocode_M->solverInfo,
                       &MoCo_Brake_Autocode_M->Timing.stepSize0);
    rtsiSetdXPtr(MoCo_Brake_Autocode_M->solverInfo,
                 &MoCo_Brake_Autocode_M->derivs);
    rtsiSetContStatesPtr(MoCo_Brake_Autocode_M->solverInfo, (real_T **)
                         &MoCo_Brake_Autocode_M->contStates);
    rtsiSetNumContStatesPtr(MoCo_Brake_Autocode_M->solverInfo,
      &MoCo_Brake_Autocode_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(MoCo_Brake_Autocode_M->solverInfo,
      &MoCo_Brake_Autocode_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(MoCo_Brake_Autocode_M->solverInfo,
      &MoCo_Brake_Autocode_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(MoCo_Brake_Autocode_M->solverInfo,
      &MoCo_Brake_Autocode_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(MoCo_Brake_Autocode_M->solverInfo, (&rtmGetErrorStatus
      (MoCo_Brake_Autocode_M)));
    rtsiSetRTModelPtr(MoCo_Brake_Autocode_M->solverInfo, MoCo_Brake_Autocode_M);
  }

  rtsiSetSolverName(MoCo_Brake_Autocode_M->solverInfo,"ode1");

  /* block I/O */
  {
    B_MoCo_Brake_Autocode_T *b = (B_MoCo_Brake_Autocode_T *) malloc(sizeof
      (B_MoCo_Brake_Autocode_T));
    rt_VALIDATE_MEMORY(MoCo_Brake_Autocode_M,b);
    MoCo_Brake_Autocode_M->blockIO = (b);
  }

  /* states (continuous) */
  {
    real_T *x = (real_T *) malloc(sizeof(X_MoCo_Brake_Autocode_T));
    rt_VALIDATE_MEMORY(MoCo_Brake_Autocode_M,x);
    MoCo_Brake_Autocode_M->contStates = (x);
  }

  /* states (dwork) */
  {
    DW_MoCo_Brake_Autocode_T *dwork = (DW_MoCo_Brake_Autocode_T *) malloc(sizeof
      (DW_MoCo_Brake_Autocode_T));
    rt_VALIDATE_MEMORY(MoCo_Brake_Autocode_M,dwork);
    MoCo_Brake_Autocode_M->dwork = (dwork);
  }

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  MoCo_Brake_Autocode_InitializeDataMapInfo(MoCo_Brake_Autocode_M);

  /* CarMaker parameter tuning */
  {
    const struct tMatSuppMMI *mmi = &(rtmGetDataMapInfo(MoCo_Brake_Autocode_M).
      mmi);
    tMatSuppTunables *tuns = MatSupp_TunBegin("MoCo_Brake_Autocode", mmi);
    MoCo_Brake_Autocode_SetParams(MoCo_Brake_Autocode_M, tuns, inf);
    MatSupp_TunEnd(tuns);
  }

  {
    B_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_B = ((B_MoCo_Brake_Autocode_T *)
      MoCo_Brake_Autocode_M->blockIO);
    DW_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_DW =
      ((DW_MoCo_Brake_Autocode_T *) MoCo_Brake_Autocode_M->dwork);
    X_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_X = ((X_MoCo_Brake_Autocode_T *)
      MoCo_Brake_Autocode_M->contStates);

    /* initialize non-finites */
    rt_InitInfAndNaN(sizeof(real_T));
    rtsiSetSimTimeStep(MoCo_Brake_Autocode_M->solverInfo, MAJOR_TIME_STEP);
    MoCo_Brake_Autocode_M->intgData.f[0] = MoCo_Brake_Autocode_M->odeF[0];
    MoCo_Brake_Autocode_M->contStates = ((real_T *) MoCo_Brake_Autocode_X);
    rtsiSetSolverData(MoCo_Brake_Autocode_M->solverInfo, (void *)
                      &MoCo_Brake_Autocode_M->intgData);
    rtmSetTPtr(MoCo_Brake_Autocode_M, &MoCo_Brake_Autocode_M->Timing.tArray[0]);
    MoCo_Brake_Autocode_M->Timing.stepSize0 = 0.001;
    rtmSetFirstInitCond(MoCo_Brake_Autocode_M, 1);

    /* block I/O */
    (void) memset(((void *) MoCo_Brake_Autocode_B), 0,
                  sizeof(B_MoCo_Brake_Autocode_T));

    /* states (continuous) */
    {
      (void) memset((void *)MoCo_Brake_Autocode_X, 0,
                    sizeof(X_MoCo_Brake_Autocode_T));
    }

    /* states (dwork) */
    (void) memset((void *)MoCo_Brake_Autocode_DW, 0,
                  sizeof(DW_MoCo_Brake_Autocode_T));

    {
      static uint32_T *clockTickPtrs[2];
      static uint32_T *clockTickHPtrs[2];
      static real_T *taskTimePtrs[2];
      MoCo_Brake_Autocode_M->timingBridge.nTasks = 2;
      clockTickPtrs[0] = &(MoCo_Brake_Autocode_M->Timing.clockTick0);
      clockTickHPtrs[0] = &(MoCo_Brake_Autocode_M->Timing.clockTickH0);
      clockTickPtrs[1] = &(MoCo_Brake_Autocode_M->Timing.clockTick1);
      clockTickHPtrs[1] = &(MoCo_Brake_Autocode_M->Timing.clockTickH1);
      MoCo_Brake_Autocode_M->timingBridge.clockTick = clockTickPtrs;
      MoCo_Brake_Autocode_M->timingBridge.clockTickH = clockTickHPtrs;
      taskTimePtrs[0] = &(MoCo_Brake_Autocode_M->Timing.t[0]);
      taskTimePtrs[1] = (NULL);
      MoCo_Brake_Autocode_M->timingBridge.taskTime = taskTimePtrs;
      MoCo_Brake_Autocode_M->timingBridge.firstInitCond = &rtmIsFirstInitCond
        (MoCo_Brake_Autocode_M);
    }

    /* Model Initialize function for ModelReference Block: '<Root>/Model' */
    mdl_MoCo_Brake_initialize(rtmGetErrorStatusPointer(MoCo_Brake_Autocode_M),
      rtmGetStopRequestedPtr(MoCo_Brake_Autocode_M),
      MoCo_Brake_Autocode_M->solverInfo, &MoCo_Brake_Autocode_M->timingBridge, 0,
      1, &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtm),
      &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtb),
      &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtdw),
      &(MoCo_Brake_Autocode_X->Model_CSTATE),
      &(MoCo_Brake_Autocode_DW->Model_InstanceData.rtzce),
      &(MoCo_Brake_Autocode_M->DataMapInfo.mmi), "MoCo_Brake_Autocode/Model", 0,
      0);
  }

  return MoCo_Brake_Autocode_M;
}

/* CarMaker dictionary definitions. */
extern tQuantEntry *MoCo_Brake_Autocode_main_DictDefines[];
extern tQuantEntry mdl_MoCo_Brake_DictDefines[];
static tQuantEntry DictDefines[] = {
  { (void*)0x01234567, (void*)0x89ABCDEF, NULL, 0, 0, 0, "", 0.0, 0.0 },

  { (void*)MoCo_Brake_Autocode_main_DictDefines, NULL, NULL, 0, 0, 0, "", 0.0,
    0.0 },

  { "MoCoBrake.Target_Trq", "Nm", "Double4", 0, 0.0, 0.0, "VC", 0.0, 0.0 },

  { "MoCoBrake.Veh_Trq", "Nm", "Double4", 0, 0.0, 0.0, "None", 0.0, 0.0 },

  { "MoCoBrake.Ssm_Status", "", "Int", 0, 0.0, 0.0, "None", 0.0, 0.0 },

  { "MoCoBrake.Ssm_Hold_Req", "", "Int", 0, 0.0, 0.0, "VC", 0.0, 0.0 },

  { "MoCoBrake.Ssm_Go_Req", "", "Int", 0, 0.0, 0.0, "VC", 0.0, 0.0 },

  { "MoCoBrake.Ssm_Torque", "Nm", "Int", 0, 0.0, 0.0, "None", 0.0, 0.0 },

  { NULL, NULL, NULL, 0, 0, 0, "", 0.0, 0.0 }
};

tQuantEntry *MoCo_Brake_Autocode_DictDefines = DictDefines;
tQuantEntry *MoCo_Brake_Autocode_main_DictDefines[] = {
  DictDefines,
  mdl_MoCo_Brake_DictDefines,
  NULL
};

/* CarMaker bodyframe definitions. */
#ifndef RTMAKER

extern tMdlBdyFrame *MoCo_Brake_Autocode_main_BdyFrameDefines[];
extern tMdlBdyFrame mdl_MoCo_Brake_BdyFrameDefines[];
static tMdlBdyFrame BdyFrameDefines[] = {
  { (void*)0x01234567 },

  { (void*)0x89ABCDEF },

  { (void*)MoCo_Brake_Autocode_main_BdyFrameDefines },

  { NULL }
};

tMdlBdyFrame *MoCo_Brake_Autocode_BdyFrameDefines = BdyFrameDefines;
tMdlBdyFrame *MoCo_Brake_Autocode_main_BdyFrameDefines[] = {
  BdyFrameDefines,
  mdl_MoCo_Brake_BdyFrameDefines,
  NULL
};

#endif
