#ifndef _MODEL_FCT_DLLRAMPEDBRAKE_H__
#define _MODEL_FCT_DLLRAMPEDBRAKE_H__
#if FCT_CFG_SENSOR_TYPE_RADAR
#include "Platform_Types.h"
#endif



#ifdef __cplusplus
extern "C" {
#endif

  int Brake_Register_OCTAGONBrake (void);
  
  /*Negative input for acceleration, positive input for pmc (pressure master cylinder), negative for axle torque input*/
  void setBrakeRequest( double input ); 

#ifdef __cplusplus
}
#endif



#endif	/* #ifndef _MODEL_FCT_DLLBRAKE_H__ */