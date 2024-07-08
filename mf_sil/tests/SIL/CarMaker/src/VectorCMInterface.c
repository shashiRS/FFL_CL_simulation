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
│  Author: Mihai Baneu                             Last modified: 10.Sep.2019  │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────*/
#undef UNICODE
#include <Windows.h>
#include "vxlapi.h"
#pragma warning( push )
#pragma warning ( disable: 4201 ) // disable  nameless struct/union warning in road.h
#include <CarMaker.h>
#pragma warning( pop )

#include "IOCMInterface.h"
#include "VectorCMInterface.h"

// xlDriver internal data and parameters
typedef struct t_xlChannelInfo
{
  XLportHandle  portHandle;
  long          channel;
  long          isCanFD;
  XLcanFdConf   paramFD;
  long          baudrate;
} t_xlChannelInfo;

// external variables
extern tAppStartInfo	AppStartInfo;

// configuration file
static const char *c_configFilePath = "./Data/Config/Vector";

// module internal data
static XLdriverConfig     m_drvConfig;
static t_xlChannelInfo   *m_channelConfig      = 0;
static long               m_channelConfigCount = 0;
static tInfos            *m_configFile         = 0;
static HINSTANCE          m_hDll               = 0;

// imported functions
static XLOPENDRIVER            xlOpenDriver            = 0;
static XLCLOSEDRIVER           xlCloseDriver           = 0;
static XLGETDRIVERCONFIG       xlGetDriverConfig       = 0;
static XLGETERRORSTRING        xlGetErrorString        = 0;
static XLOPENPORT              xlOpenPort              = 0;
static XLCLOSEPORT             xlClosePort             = 0;
static XLACTIVATECHANNEL       xlActivateChannel       = 0;
static XLDEACTIVATECHANNEL     xlDeactivateChannel     = 0;
static XLCANSETCHANNELBITRATE  xlCanSetChannelBitrate  = 0;
static XLCANFDSETCONFIGURATION xlCanFdSetConfiguration = 0;
static XLCANTRANSMITEX         xlCanTransmitEx         = 0;
static XLCANTRANSMIT           xlCanTransmit           = 0;
static XLCANRECEIVE            xlCanReceive            = 0;
static XLRECEIVE               xlReceive               = 0;

static void printConfig(void) {

  unsigned int i;
  char         str[100];

  Log("----------------------------------------------------------\n");
  Log("- %02d channels       Hardware Configuration               -\n", m_drvConfig.channelCount);
  Log("----------------------------------------------------------\n");

  for (i=0; i < m_drvConfig.channelCount; i++) {

    Log("- Ch:%02d, CM:0x%03I64x,", m_drvConfig.channel[i].channelIndex, m_drvConfig.channel[i].channelMask);

    strncpy_s(str, 100, m_drvConfig.channel[i].name, 23);
    Log(" %23s,", str);

    memset(str, 0, sizeof(str));

    if (m_drvConfig.channel[i].transceiverType != XL_TRANSCEIVER_TYPE_NONE) {
      strncpy_s( str, 100, m_drvConfig.channel[i].transceiverName, 13);
      Log("%13s -\n", str);
    } else {
      Log("    no Cab!   -\n");
    }
  }

  Log("----------------------------------------------------------\n\n");
}

static void convertBitrate(t_xlChannelInfo *channelInfo, char *bitrate)
{
  char *part, *next;

  part = strtok_s(bitrate, ", ", &next);
  while (part != 0) {
    char *entry = part;
    char *value = strchr(part, '=') + 1;

    part = strtok_s(0, ", ", &next);

    if (strncmp(entry, "arbitrationBitRate", strlen("arbitrationBitRate")) == 0) {
      channelInfo->paramFD.arbitrationBitRate = (unsigned int)atoi(value); continue;
    }
    if (strncmp(entry, "sjwAbr", strlen("sjwAbr")) == 0) {
      channelInfo->paramFD.sjwAbr = (unsigned int)atoi(value); continue;
    }
    if (strncmp(entry, "tseg1Abr", strlen("tseg1Abr")) == 0) {
      channelInfo->paramFD.tseg1Abr = (unsigned int)atoi(value); continue;
    }
    if (strncmp(entry, "tseg2Abr", strlen("tseg2Abr")) == 0) {
      channelInfo->paramFD.tseg2Abr = (unsigned int)atoi(value); continue;
    }
    if (strncmp(entry, "dataBitRate", strlen("dataBitRate")) == 0) {
      channelInfo->paramFD.dataBitRate = (unsigned int)atoi(value); continue;
    }
    if (strncmp(entry, "sjwDbr", strlen("sjwDbr")) == 0) {
      channelInfo->paramFD.sjwDbr = (unsigned int)atoi(value); continue;
    }
    if (strncmp(entry, "tseg1Dbr", strlen("tseg1Dbr")) == 0) {
      channelInfo->paramFD.tseg1Dbr = (unsigned int)atoi(value); continue;
    }
    if (strncmp(entry, "tseg2Dbr", strlen("tseg2Dbr")) == 0) {
      channelInfo->paramFD.tseg2Dbr = (unsigned int)atoi(value); continue;
    }
  }
}

int Initialize_Vector()
{
  tErrorMsg *perrors;
  XLstatus   xlStatus;
  char      *xlDllPath = 0;
  int        ret;

  // try to open the configuration file
  m_configFile = InfoNew();
  ret = InfoRead(&perrors, m_configFile, c_configFilePath);
  if (ret < 0) {
    LogErrF(EC_Init, "Failed to read the configuration file %s\n%s", c_configFilePath, perrors->Msg);
    return -1;
  } else if (ret > 0) {
    LogErrF(EC_Init, "Failed to parse the configuration file %s\n%s", c_configFilePath, perrors->Msg);
    return -1;
  }

  InfoGetStrDef(&xlDllPath, SimCore.TestRig.SimParam.Inf, "VECTOR.XLDriver.dll", "./bin/vxlapi.dll");
  InfoGetLongDef(&m_channelConfigCount, m_configFile, "VECTOR.Channel.Count", -1);
  m_channelConfig = (t_xlChannelInfo *)malloc(sizeof(t_xlChannelInfo) * (unsigned long)m_channelConfigCount);
  for (int i = 0; i < m_channelConfigCount; i++) {
    char entry[64];
    char *bitrateFD = 0;

    memset(&m_channelConfig[i], 0, sizeof(t_xlChannelInfo));

    sprintf(entry, "VECTOR.Channel.%d.Channel", i+1);
    ret = InfoGetLongDef(&m_channelConfig[i].channel, m_configFile, entry, -1);

    sprintf(entry, "VECTOR.Channel.%d.IsCanFD", i+1);
    ret = InfoGetLongDef(&m_channelConfig[i].isCanFD, m_configFile, entry, -1);

    sprintf(entry, "VECTOR.Channel.%d.Baudrate", i+1);
    ret = InfoGetLongDef(&m_channelConfig[i].baudrate, m_configFile, entry, -1);

    sprintf(entry, "VECTOR.Channel.%d.BitrateFD", i+1);
    ret = InfoGetStrDef(&bitrateFD, m_configFile, entry, "");

    if (!m_channelConfig[i].isCanFD && (m_channelConfig[i].baudrate == -1)) {
      LogErrF(EC_Init, "Invalid boudrate on channel %d", i + 1);
      return -1;
    }

    if (m_channelConfig[i].isCanFD && strcmp(bitrateFD, "") == 0) {
      LogErrF(EC_Init, "Invalid bitrate on channel %d", i + 1);
      return -1;
    }

    convertBitrate(&m_channelConfig[i], bitrateFD);
  }

  m_hDll = LoadLibrary(xlDllPath);
  if (m_hDll == 0) {
    LogErrF(EC_Init, "LoadLibrary for %s failed!\n", xlDllPath);
    return -1;
  }

  xlOpenDriver             = (XLOPENDRIVER)            GetProcAddress(m_hDll, "xlOpenDriver");
  xlCloseDriver            = (XLCLOSEDRIVER)           GetProcAddress(m_hDll, "xlCloseDriver");
  xlGetDriverConfig        = (XLGETDRIVERCONFIG)       GetProcAddress(m_hDll, "xlGetDriverConfig");
  xlGetErrorString         = (XLGETERRORSTRING)        GetProcAddress(m_hDll, "xlGetErrorString");
  xlOpenPort               = (XLOPENPORT)              GetProcAddress(m_hDll, "xlOpenPort");
  xlClosePort              = (XLCLOSEPORT)             GetProcAddress(m_hDll, "xlClosePort");
  xlActivateChannel        = (XLACTIVATECHANNEL)       GetProcAddress(m_hDll, "xlActivateChannel");
  xlDeactivateChannel      = (XLDEACTIVATECHANNEL)     GetProcAddress(m_hDll, "xlDeactivateChannel");
  xlCanSetChannelBitrate   = (XLCANSETCHANNELBITRATE)  GetProcAddress(m_hDll, "xlCanSetChannelBitrate");
  xlCanFdSetConfiguration  = (XLCANFDSETCONFIGURATION) GetProcAddress(m_hDll, "xlCanFdSetConfiguration");
  xlCanTransmitEx          = (XLCANTRANSMITEX)         GetProcAddress(m_hDll, "xlCanTransmitEx");
  xlCanTransmit            = (XLCANTRANSMIT)           GetProcAddress(m_hDll, "xlCanTransmit");
  xlCanReceive             = (XLCANRECEIVE)            GetProcAddress(m_hDll, "xlCanReceive");
  xlReceive                = (XLRECEIVE)               GetProcAddress(m_hDll, "xlReceive");

  xlStatus = xlOpenDriver();
  if(xlStatus != XL_SUCCESS) {
    LogErrF(EC_Init, "xlOpenDriver failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
    return -1;
  }

  xlStatus = xlGetDriverConfig(&m_drvConfig);
  if(xlStatus != XL_SUCCESS) {
    LogErrF(EC_Init, "xlGetDriverConfig failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
    return -1;
  }
  printConfig();

  for (int i = 0; i < m_channelConfigCount; i++) {
    XLaccess xlChannelMask, xlPermissionMask;

    xlChannelMask = xlPermissionMask = (unsigned __int64)(1 << m_channelConfig[i].channel);
    if (m_channelConfig[i].isCanFD) {
      xlStatus = xlOpenPort(&m_channelConfig[i].portHandle, (char *)AppStartInfo.App_Version, xlChannelMask, &xlPermissionMask, 16384, XL_INTERFACE_VERSION_V4, XL_BUS_TYPE_CAN);
      if(xlStatus != XL_SUCCESS) {
        LogErrF(EC_Init, "xlOpenPort failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
        return -1;
      }
      Log("- OpenPort         : CM=0x%I64x, PH=0x%02X, PM=0x%I64x, %s\n", xlChannelMask, m_channelConfig[i].portHandle, xlPermissionMask, xlGetErrorString(xlStatus));

      xlStatus = xlCanFdSetConfiguration(m_channelConfig[i].portHandle, xlChannelMask, &m_channelConfig[i].paramFD);
      if(xlStatus != XL_SUCCESS) {
        LogErrF(EC_Init, "xlCanFdSetConfiguration failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
        return -1;
      }
      Log("- SetFdConfig.     : ABaudr.=%u, DBaudr.=%u, %s\n", m_channelConfig[i].paramFD.arbitrationBitRate, m_channelConfig[i].paramFD.dataBitRate, xlGetErrorString(xlStatus));

      xlStatus = xlActivateChannel(m_channelConfig[i].portHandle, xlChannelMask, XL_BUS_TYPE_CAN, XL_ACTIVATE_RESET_CLOCK);
      if(xlStatus != XL_SUCCESS) {
        LogErrF(EC_Init, "xlActivateChannel failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
        return -1;
      }
      Log("- ActivateChannel  : CM=0x%I64x, %s\n", xlChannelMask, xlGetErrorString(xlStatus));
    } else {
      xlStatus = xlOpenPort(&m_channelConfig[i].portHandle, (char *)AppStartInfo.App_Version, xlChannelMask, &xlPermissionMask, 256, XL_INTERFACE_VERSION, XL_BUS_TYPE_CAN);
      if(xlStatus != XL_SUCCESS) {
        LogErrF(EC_Init, "xlOpenPort failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
        return -1;
      }
      Log("- OpenPort         : CM=0x%I64x, PH=0x%02X, PM=0x%I64x, %s\n", xlChannelMask, m_channelConfig[i].portHandle, xlPermissionMask, xlGetErrorString(xlStatus));

      xlStatus = xlCanSetChannelBitrate(m_channelConfig[i].portHandle, xlChannelMask, m_channelConfig[i].baudrate);
      if(xlStatus != XL_SUCCESS) {
        LogErrF(EC_Init, "xlCanSetChannelBitrate failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
        return -1;
      }
      Log("- SetBoudrate      : Baudrate.=%u, %s\n", m_channelConfig[i].baudrate, xlGetErrorString(xlStatus));

      xlStatus = xlActivateChannel(m_channelConfig[i].portHandle, xlChannelMask, XL_BUS_TYPE_CAN, XL_ACTIVATE_RESET_CLOCK);
      if(xlStatus != XL_SUCCESS) {
        LogErrF(EC_Init, "xlActivateChannel failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
        return -1;
      }
      Log("- ActivateChannel  : CM=0x%I64x, %s\n", xlChannelMask, xlGetErrorString(xlStatus));
    }
  }

  return 0;
}

void Uninitialize_Vector()
{
  for (int i=0; i<m_channelConfigCount; i++) {
    xlClosePort(m_channelConfig[i].portHandle);
  }

  // delete module data
  free(m_channelConfig);
  m_channelConfig = 0;

  // delete the config here - all strings alocated will be deleted
  InfoDelete(m_configFile);
  m_configFile = 0;

  // free the library
  xlCloseDriver();
  FreeLibrary(m_hDll);
  m_hDll = 0;
}

int Vector_RecvFD(int slot, int channel, CANFD_Msg *msg)
{
    slot; /*silence unreferenced warning*/
  XLstatus xlStatus = XL_SUCCESS;
  XLcanRxEvent xlCanRxEvt;

  memset(&xlCanRxEvt, 0, sizeof(xlCanRxEvt));

  xlStatus = xlCanReceive(m_channelConfig[channel].portHandle, &xlCanRxEvt);
  if((xlStatus != XL_SUCCESS) && (xlStatus != XL_ERR_QUEUE_IS_EMPTY)) {
    LogErrF(EC_Init, "xlCanReceive failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
    return -1;
  }

  if (xlStatus == XL_ERR_QUEUE_IS_EMPTY) {
    return -1;
  }

  memset(msg, 0, sizeof(CANFD_Msg));
  msg->MsgId = xlCanRxEvt.tagData.canRxOkMsg.canId &~ XL_CAN_EXT_MSG_ID;
  msg->DLC   = xlCanRxEvt.tagData.canRxOkMsg.dlc;
  msg->BRS   = (xlCanRxEvt.tagData.canRxOkMsg.msgFlags & XL_CAN_TXMSG_FLAG_BRS) ? 1 : 0;
  msg->IDE   = (xlCanRxEvt.tagData.canRxOkMsg.canId & XL_CAN_EXT_MSG_ID) ? 1 : 0;
  msg->FDF   = (xlCanRxEvt.tagData.canRxOkMsg.msgFlags & XL_CAN_TXMSG_FLAG_EDL) ? 1 : 0;

  memcpy(msg->Data, xlCanRxEvt.tagData.canRxOkMsg.data, XL_CAN_MAX_DATA_LEN);
  return 0;
}

int Vector_SendFD(int slot, int channel, CANFD_Msg *msg)
{
    slot; /*silence unreferenced warning*/
  XLcanTxEvent canTxEvt;
  XLaccess xlChanMaskTx;
  XLstatus xlStatus;
  unsigned int cntSent;

  memset(&canTxEvt, 0, sizeof(canTxEvt));

  canTxEvt.tag = XL_CAN_EV_TAG_TX_MSG;
  canTxEvt.tagData.canMsg.canId     = (msg->IDE ? XL_CAN_EXT_MSG_ID : 0) | msg->MsgId;
  canTxEvt.tagData.canMsg.msgFlags  = (msg->RTR ? XL_CAN_TXMSG_FLAG_RTR : 0) |
                                      (msg->FDF ? XL_CAN_TXMSG_FLAG_EDL : 0) |
                                      (msg->BRS ? XL_CAN_TXMSG_FLAG_BRS : 0);
  canTxEvt.tagData.canMsg.dlc = (unsigned char) msg->DLC;
  memcpy(canTxEvt.tagData.canMsg.data, msg->Data, XL_CAN_MAX_DATA_LEN);

  xlChanMaskTx = (m_channelConfig[channel].channel != -1) ? (1 << m_channelConfig[channel].channel) : 0;
  xlStatus = xlCanTransmitEx(m_channelConfig[channel].portHandle, xlChanMaskTx, 1, &cntSent, &canTxEvt);
  if(xlStatus != XL_SUCCESS) {
    LogErrF(EC_Init, "xlCanTransmitEx failed with code %#010x: %s\n", xlStatus, xlGetErrorString(xlStatus));
    return -1;
  }

  return 0;
}

int Vector_Recv(int slot, int channel, CAN_Msg *msg)
{
    slot; /*silence unreferenced warning*/
  XLstatus xlStatus;
  XLevent  xlEvent;
  unsigned int msgsrx = 1;

  xlEvent.chanIndex = (unsigned char)m_channelConfig[channel].channel;
  for (;;) {
    xlStatus = xlReceive(m_channelConfig[channel].portHandle, &msgsrx, &xlEvent);
    if (XL_SUCCESS != xlStatus) {
      return xlStatus;
    }

    if (xlEvent.tag != XL_RECEIVE_MSG || xlEvent.portHandle != m_channelConfig[channel].portHandle) {
      continue;
    }
    else {
      msg->MsgId    = xlEvent.tagData.msg.id &~ XL_CAN_EXT_MSG_ID;
      msg->FrameFmt = (xlEvent.tagData.msg.id & XL_CAN_EXT_MSG_ID) ? 1 : 0;
      msg->FrameLen = (unsigned char)xlEvent.tagData.msg.dlc;
      memcpy(msg->Data, xlEvent.tagData.msg.data, 8);
      return 0;
    }
  }
}

int Vector_Send(int slot, int channel, CAN_Msg *msg)
{
    slot; /*silence unreferenced warning*/
  XLstatus xlStatus;
  XLevent  xlEvent;
  XLaccess xlChanMaskTx;
  unsigned int msgstx = 1;

  memset(&xlEvent, 0, sizeof(xlEvent));
  xlEvent.chanIndex = (unsigned char)m_channelConfig[channel].channel;
  xlEvent.tag = XL_TRANSMIT_MSG;
  xlEvent.tagData.msg.id  = (msg->FrameFmt ? XL_CAN_EXT_MSG_ID : 0) | msg->MsgId;
  xlEvent.tagData.msg.dlc = msg->FrameLen;
  memcpy(xlEvent.tagData.msg.data, msg->Data, 8);

  xlChanMaskTx = (m_channelConfig[channel].channel != -1) ? (1 << m_channelConfig[channel].channel) : 0;
  xlStatus = xlCanTransmit(m_channelConfig[channel].portHandle, xlChanMaskTx, &msgstx, &xlEvent);
  if(XL_SUCCESS != xlStatus) {
    return xlStatus;
  }

  return 0;
}

int Vector_Send_normal_for_FD(int slot, int channel, CAN_Msg *msg)
{
  CANFD_Msg msgFD;

  memset(&msgFD, 0, sizeof(CANFD_Msg));
  msgFD.MsgId = msg->MsgId;
  msgFD.DLC   = msg->FrameLen;
  msgFD.IDE   = msg->FrameFmt;
  memcpy(msgFD.Data, msg->Data, 8);

  return Vector_SendFD(slot, channel, &msgFD);
}

