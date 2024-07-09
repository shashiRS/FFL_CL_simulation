/*
 * MoCo_Brake_Autocode_data.c
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

#include "MoCo_Brake_Autocode.h"
#include "MoCo_Brake_Autocode_private.h"

/* Model block global parameters (default storage) */
real_T rtP_Brk_SSM_Decel_Lim = -2.0;   /* Variable: Brk_SSM_Decel_Lim
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_D_brk = 0.6;                /* Variable: D_brk
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_K_brk = 0.0019538188277087036;/* Variable: K_brk
                                          * Referenced by: '<Root>/Model'
                                          */
real_T rtP_K_brk_accel_dec = 0.0019467;/* Variable: K_brk_accel_dec
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_K_brk_trq_dec = 1.0;        /* Variable: K_brk_trq_dec
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_T_brk_accel_dec = 0.13;     /* Variable: T_brk_accel_dec
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_T_brk_trq_dec = 0.17;       /* Variable: T_brk_trq_dec
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_Trq_Distrib_FL = 0.3;       /* Variable: Trq_Distrib_FL
                                        * Referenced by: '<Root>/Constant'
                                        */
real_T rtP_Trq_Distrib_FR = 0.3;       /* Variable: Trq_Distrib_FR
                                        * Referenced by: '<Root>/Constant1'
                                        */
real_T rtP_Trq_Distrib_RL = 0.2;       /* Variable: Trq_Distrib_RL
                                        * Referenced by: '<Root>/Constant2'
                                        */
real_T rtP_Trq_Distrib_RR = 0.2;       /* Variable: Trq_Distrib_RR
                                        * Referenced by: '<Root>/Constant3'
                                        */
real_T rtP_accel_to_trq = 510.0;       /* Variable: accel_to_trq
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_brake_pres_exist_threshold = -0.1;/* Variable: brake_pres_exist_threshold
                                              * Referenced by: '<Root>/Model'
                                              */
real_T rtP_delay_brake_inc = 0.08;     /* Variable: delay_brake_inc
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_delay_brake_inc_init = 0.3; /* Variable: delay_brake_inc_init
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_w0_brk = 10.0;              /* Variable: w0_brk
                                        * Referenced by: '<Root>/Model'
                                        */
real_T rtP_w0_brkinit = 5.0;           /* Variable: w0_brkinit
                                        * Referenced by: '<Root>/Model'
                                        */
