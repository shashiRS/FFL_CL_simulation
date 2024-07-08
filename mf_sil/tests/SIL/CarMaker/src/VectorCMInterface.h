/*─────────────────────────────────────────────────────────────────────────────┐
│                                                                              │
│  COPYRIGHT (C) 2019 A.D.C. Automotive Distance Control Systems GmbH          │
│  ALLE RECHTE VORBEHALTEN. ALL RIGHTS RESERVED.                               │
│                                                                              │
│  The reproduction, transmission or use of this document or its contents is   │
│  not permitted without express written authority. Offenders will be liable   │
│  for damages.  All rights,  including  rights  created  by patent grant or   │
│  registration of a utility model or design, are reserved.                    │
│                                                                              │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  Author: Mihai Baneu                             Last modified: 03.Jun.2019  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────*/

#pragma once

#include "IOCMInterface.h"

#ifdef __cplusplus
extern "C" {
#endif



int Initialize_Vector(void);
void Uninitialize_Vector(void);

int Vector_RecvFD(
  int slot,
  int channel,
  struct CANFD_Msg *msg);

int Vector_SendFD(
  int slot,
  int channel,
  struct CANFD_Msg *msg);

int Vector_Recv(
  int slot,
  int channel,
  struct CAN_Msg *msg);

int Vector_Send(
  int slot,
  int channel,
  struct CAN_Msg *msg);

int Vector_Send_normal_for_FD(
    int slot,
    int channel,
    struct CAN_Msg *msg);

#ifdef __cplusplus
}
#endif
