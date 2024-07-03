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
#pragma warning(push)
#pragma warning (disable: 4214 )
typedef struct CAN_Msg {
  unsigned int  MsgId;
  unsigned char FrameFmt:1;
  unsigned char RTR     :1;
  unsigned char FrameLen:4;
  unsigned char Data[8];
} CAN_Msg;
#pragma warning(pop)

typedef struct CANFD_Msg {
  unsigned int  MsgId         : 29;   // Message-ID
  unsigned int  IDE           : 1;    // 0: Standard Frame ID, 1: Extended Frame ID
  unsigned int  RTR           : 1;    // Request to Transmit, StandardCAN only
  unsigned int  DLC   : 4;  // Data Length Code
  unsigned int  FDF     : 1;  // EDL-Flag of CANFD: 0: Standard CAN, 1: CANFD
  unsigned int  BRS     : 1;  // 0: No Baudrate Switch, 1: Baudrate Switch with CANFD
  unsigned int  ESI     : 1;  // Error State Indicator with CANFD (Rx only)
  unsigned int  reserved  : 26;
  unsigned char Data[64];   // Data: Standard-CAN uses only Data[7..0]
} CANFD_Msg;
