/***************************************************** target specific file ***/
/*  Wrapper module for Simulink models                                        */
/*  ------------------------------------------------------------------------  */
/*  (c) IPG Automotive GmbH    www.ipg-automotive.com   Fon: +49.721.98520-0  */
/*  Bannwaldallee 60      D-76185 Karlsruhe   Germany   Fax: +49.721.98520-99 */
/******************************************************************************/

#ifndef __MOCO_BRAKE_AUTOCODE_WRAP_H__
#define __MOCO_BRAKE_AUTOCODE_WRAP_H__

#ifndef IS_CAR
# define IS_CAR
#endif

#ifdef __cplusplus
extern "C" {
#endif


struct tInfos;
struct tMdlBdyFrame;
struct tMatSuppDictDef;
struct tMatSuppTunables;


#ifdef CLASSIC_INTERFACE
# define rtModel_MoCo_Brake_Autocode          RT_MODEL_MoCo_Brake_Autocode_T
#else
# define rtModel_MoCo_Brake_Autocode          tag_RTM_MoCo_Brake_Autocode_T
#endif //CLASSIC_INTERFACE

#define ExternalInputs_MoCo_Brake_Autocode   ExtU_MoCo_Brake_Autocode_T
#define ExternalOutputs_MoCo_Brake_Autocode  ExtY_MoCo_Brake_Autocode_T

#ifndef MoCo_Brake_Autocode_rtModel
typedef struct rtModel_MoCo_Brake_Autocode rtModel_MoCo_Brake_Autocode;
#endif

/* Model registration function */
rtModel_MoCo_Brake_Autocode *MoCo_Brake_Autocode (struct tInfos *Inf);

#if defined(CLASSIC_INTERFACE) && !defined(CM4SLDS)
void rt_ODECreateIntegrationData (RTWSolverInfo *si);
void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
void rt_ODEDestroyIntegrationData(RTWSolverInfo *si);
#endif


/* Dictionary variables and other items of the model */
extern struct tMatSuppDictDef *MoCo_Brake_Autocode_DictDefines;
extern struct tMdlBdyFrame *MoCo_Brake_Autocode_BdyFrameDefines;


/* Wrapper functions */
void MoCo_Brake_Autocode_SetParams (rtModel_MoCo_Brake_Autocode *rtm,
			struct tMatSuppTunables *tuns,
			struct tInfos *Inf);
int MoCo_Brake_Autocode_Register (void);


#ifdef __cplusplus
}
#endif

#endif /* __MOCO_BRAKE_AUTOCODE_WRAP_H__ */

