/* $Id$ */
/*
******************************************************************************
**
**  PRIVATE_CAN User header file.  Make your changes here.
**
******************************************************************************
**  PRIVATE_CAN_User.h
**  This file was automatically generated by CANiogen 2.4.3 for:
**
**  CarMaker - Version 6.0
**  Vehicle Dynamic Simulation Toolkit
**
**  IPG Automotive GmbH
**  Bannwaldallee 60             Phone  +49.721.98520.0
**  76185 Karlsruhe              Fax    +49.721.98520.99
**  Germany                      WWW    http://www.ipg.de
**
**  Created: 2022/04/26 08:44:58 (uib41902@IALN981W)
******************************************************************************
*/

#if !defined(__PRIVATE_CAN_User_H__)
#define __PRIVATE_CAN_User_H__

#if defined(DSPACE)
# include <ipgrt.h>
# include <dsio.h>
#endif

#if defined(XENO) || defined(LVRT)
# define _ISOC99_SOURCE 1
# include <math.h>
# include <string.h>
# include <mio.h>
#endif
#if defined(LVRT)
# include <nio.h>
#endif

#include <MathUtils.h>

#include "VectorCMInterface.h"

#ifdef __cplusplus
extern "C" {
#endif


struct CAN_Msg;
struct CANFD_Msg;

#if defined(_DS1005)
/* DataDictionary / APO do not support long long on DS1005 */
#   define DDefLLong(d, n, u, v, p)
#   define DDefULLong(d, n, u, v, p)
#endif
#define LONG_LONG			long long
#define UNSIGNED_LONG_LONG		unsigned long long

#define PRIVATE_CAN_SIGSTATE_UNKNOWN_CONVERT	1
#define NUM_SLOT_SIGNALS_PER_SIDE               4

/* Function definitions */
int  PRIVATE_CAN_User_Init_First (void);
int  PRIVATE_CAN_User_Init       (void);
void PRIVATE_CAN_User_In         (const unsigned int CycleNo);
void PRIVATE_CAN_User_Out        (const unsigned int CycleNo);

unsigned char PRIVATE_CAN_GetUserActionHU(void);
unsigned char PRIVATE_CAN_GetUserActionHUCounter(void);
unsigned char PRIVATE_CAN_GetUserActionRem(void);
unsigned char PRIVATE_CAN_GetUserActionRemCounter(void);
float PRIVATE_CAN_GetBatteryLevelRem(void);
unsigned char PRIVATE_CAN_GetAliveCounterRem(void);
unsigned char PRIVATE_CAN_GetRemDeviceConnected(void);
unsigned char PRIVATE_CAN_GetRemDevicePaired(void);
unsigned char PRIVATE_CAN_GetDeadManSwitchBtn(void);
unsigned short PRIVATE_CAN_GetFingerPositionX(void);
unsigned short PRIVATE_CAN_GetFingerPositionY(void);
unsigned short PRIVATE_CAN_GetScreenResolutionX(void);
unsigned short PRIVATE_CAN_GetScreenResolutionY(void);

void PRIVATE_CAN_SetBtnFullyAutomParkPoss(unsigned char btnFullyAutomParkingPoss);
void PRIVATE_CAN_SetBtnSemiAutomParkPoss(unsigned char btnSemiAutomParkingPoss);
void PRIVATE_CAN_SetRemoteModeActive(unsigned char remModeActive);
void PRIVATE_CAN_SetRemoteAppActive(unsigned char remAppActive);
void PRIVATE_CAN_SetRemoteAppAuthorized(unsigned char remoteAppAuthorized);
void PRIVATE_CAN_SetRemoteAppCoded(unsigned char remoteAppCoded);
void PRIVATE_CAN_SetContinuePoss(unsigned char continuePoss);
void PRIVATE_CAN_SetParkInPoss(unsigned char parkInPoss);
void PRIVATE_CAN_SetParkOutPoss(unsigned char parkOutPoss);
void PRIVATE_CAN_SetRemManPoss(unsigned char remManPoss);
void PRIVATE_CAN_SetUndoPoss(unsigned char undoPoss);
void PRIVATE_CAN_SetSvPoss(unsigned char svPoss);
void PRIVATE_CAN_SetBtnForwardPoss(unsigned char btnForwardPoss);
void PRIVATE_CAN_SetBtnBackwardPoss(unsigned char btnBackwardPoss);
void PRIVATE_CAN_SetDistanceToStop(float distanceToStop);
void PRIVATE_CAN_SetDrivingDirection(unsigned char drivingDirection);
void PRIVATE_CAN_SetAVGType(unsigned char avgType);
void PRIVATE_CAN_SetCurrentGear(unsigned char currentGear);
void PRIVATE_CAN_SetParkInOutFinished(unsigned char parkInOutFinished);
void PRIVATE_CAN_SetRemoteKeyPoss(unsigned char remoteKeyPoss);
void PRIVATE_CAN_SetStateVarPPC(unsigned char stateVarPPC);
void PRIVATE_CAN_SetHeadUnitScreen(unsigned char headUnitScreen);
void PRIVATE_CAN_SetHeadUnitMessage(unsigned char headUnitMessage);
void PRIVATE_CAN_SetRemoteScreen(unsigned char remoteScreen);
void PRIVATE_CAN_SetRemoteMessage(unsigned char remoteMessage);
void PRIVATE_CAN_SetGarageParking(unsigned char garageParking);
void PRIVATE_CAN_SetReverseAssistPoss(unsigned char reverseAssist);
void PRIVATE_CAN_SetMemParkingPoss(unsigned char memParkingPoss);
void PRIVATE_CAN_SetSVScreenResponse(unsigned char svScreenResponse);
void PRIVATE_CAN_SetStreamAvailable(unsigned char streamAvailable);
void PRIVATE_CAN_SetSlotUnreachReason(unsigned char slotUnreachReason);
void PRIVATE_CAN_SetMemorySlotsStatus(unsigned char memorySlotsStatus);
void PRIVATE_CAN_SetMemorizedPoseSlotId(unsigned char memorizedPoseSlotId);
void PRIVATE_CAN_SetDisplayBackButton(unsigned char displayBackButton);
void PRIVATE_CAN_SetAdjustmentButtons(unsigned char adjustmentButtons);

void PRIVATE_CAN_SetParkingSpacesScanned(unsigned char scannedFront[NUM_SLOT_SIGNALS_PER_SIDE],
                                         unsigned char scannedRight[NUM_SLOT_SIGNALS_PER_SIDE],
                                         unsigned char scannedRear[NUM_SLOT_SIGNALS_PER_SIDE],
                                         unsigned char scannedLeft[NUM_SLOT_SIGNALS_PER_SIDE]);
void PRIVATE_CAN_SetParkingSpacesFree(unsigned char freeFront[NUM_SLOT_SIGNALS_PER_SIDE],
                                      unsigned char freeRight[NUM_SLOT_SIGNALS_PER_SIDE],
                                      unsigned char freeRear[NUM_SLOT_SIGNALS_PER_SIDE],
                                      unsigned char freeLeft[NUM_SLOT_SIGNALS_PER_SIDE]);
void PRIVATE_CAN_SetParkingSpacesSelected(unsigned char selectedFront[NUM_SLOT_SIGNALS_PER_SIDE],
                                          unsigned char selectedRight[NUM_SLOT_SIGNALS_PER_SIDE],
                                          unsigned char selectedRear[NUM_SLOT_SIGNALS_PER_SIDE],
                                          unsigned char selectedLeft[NUM_SLOT_SIGNALS_PER_SIDE]);
void PRIVATE_CAN_SetParkingSpacesPossOrientation(unsigned char possOrientationFront[NUM_SLOT_SIGNALS_PER_SIDE],
                                                 unsigned char possOrientationRight[NUM_SLOT_SIGNALS_PER_SIDE],
                                                 unsigned char possOrientationRear[NUM_SLOT_SIGNALS_PER_SIDE],
                                                 unsigned char possOrientationLeft[NUM_SLOT_SIGNALS_PER_SIDE]);
void PRIVATE_CAN_SetParkingSpacesSelOrientation(unsigned char selOrientationFront[NUM_SLOT_SIGNALS_PER_SIDE],
                                                unsigned char selOrientationRight[NUM_SLOT_SIGNALS_PER_SIDE],
                                                unsigned char selOrientationRear[NUM_SLOT_SIGNALS_PER_SIDE],
                                                unsigned char selOrientationLeft[NUM_SLOT_SIGNALS_PER_SIDE]);
void PRIVATE_CAN_SetParkingSpacesPossDirection(unsigned char possDirectionFront[NUM_SLOT_SIGNALS_PER_SIDE],
                                               unsigned char possDirectionRight[NUM_SLOT_SIGNALS_PER_SIDE],
                                               unsigned char possDirectionRear[NUM_SLOT_SIGNALS_PER_SIDE],
                                               unsigned char possDirectionLeft[NUM_SLOT_SIGNALS_PER_SIDE]);
void PRIVATE_CAN_SetParkingSpacesSelDirection(unsigned char selDirectionFront[NUM_SLOT_SIGNALS_PER_SIDE],
                                              unsigned char selDirectionRight[NUM_SLOT_SIGNALS_PER_SIDE],
                                              unsigned char selDirectionRear[NUM_SLOT_SIGNALS_PER_SIDE],
                                              unsigned char selDirectionLeft[NUM_SLOT_SIGNALS_PER_SIDE]);
void PRIVATE_CAN_SetParkingSpaceMemorizedPoseYaw(float memorizedPoseYaw);
void PRIVATE_CAN_SetParkingSituationFront(unsigned char notAvailable, unsigned char perpendicularParkingOut);
void PRIVATE_CAN_SetParkingSituationRear(unsigned char notAvailable, unsigned char perpendicularParkingOut);
void PRIVATE_CAN_SetParkingSituationLeft(unsigned char notAvailable, unsigned char parallelParkingOut, unsigned char uncertainSituation, unsigned char street,
                                         unsigned char angledStandardSpaces, unsigned char angledReverseSpaces, unsigned char parallelParkingSpaces, unsigned char perpendicularParkingSpaces);
void PRIVATE_CAN_SetParkingSituationRight(unsigned char notAvailable, unsigned char parallelParkingOut, unsigned char uncertainSituation, unsigned char street,
                                          unsigned char angledStandardSpaces, unsigned char angledReverseSpaces, unsigned char parallelParkingSpaces, unsigned char perpendicularParkingSpaces);
void PRIVATE_CAN_SetParkEgoRelativePosLeft(unsigned char egoRelativePos);
void PRIVATE_CAN_SetParkEgoRelativePosRight(unsigned char egoRelativePos);

void PRIVATE_CAN_SetPdcStatus(unsigned char pdcSystemState, unsigned char pdcShutdownCause);
void PRIVATE_CAN_SetPdcDrvTube(unsigned char drvTubeDirection, unsigned char drvTubeDisplay, unsigned short drvTubeFrontRadius, unsigned short drvTubeRearRadius);
void PRIVATE_CAN_SetPdcFrontSector(unsigned char sectorIdx, unsigned char criticalLevel, unsigned char intersectDrvTube, unsigned char slice);
void PRIVATE_CAN_SetPdcLeftSector(unsigned char sectorIdx, unsigned char criticalLevel, unsigned char intersectDrvTube, unsigned char slice);
void PRIVATE_CAN_SetPdcRearSector(unsigned char sectorIdx, unsigned char criticalLevel, unsigned char intersectDrvTube, unsigned char slice);
void PRIVATE_CAN_SetPdcRightSector(unsigned char sectorIdx, unsigned char criticalLevel, unsigned char intersectDrvTube, unsigned char slice);

void PRIVATE_CAN_SetFrontSpeaker(unsigned char pitch, unsigned char volume, unsigned char soundOn);
void PRIVATE_CAN_SetRearSpeaker(unsigned char pitch, unsigned char volume, unsigned char soundOn);

void PRIVATE_CAN_SetWhlWarnFL(unsigned char wheelWarnLevel, unsigned char wheelAbsAngle, unsigned char wheelDirection);
void PRIVATE_CAN_SetWhlWarnFR(unsigned char wheelWarnLevel, unsigned char wheelAbsAngle, unsigned char wheelDirection);
void PRIVATE_CAN_SetWhlWarnRL(unsigned char wheelWarnLevel, unsigned char wheelAbsAngle, unsigned char wheelDirection);
void PRIVATE_CAN_SetWhlWarnRR(unsigned char wheelWarnLevel, unsigned char wheelAbsAngle, unsigned char wheelDirection);
void PRIVATE_CAN_SetWhpStatus(unsigned char whpState, unsigned char whpDisplayReq);

#ifdef __cplusplus
}
#endif

#endif	/* #if !defined(__PRIVATE_CAN_User_H__) */
