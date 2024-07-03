#pragma once
#include "stdint.h"

#include "lsm_vedodo/odo_estimation_output_port.h"

/*****************************************************************************
FILL FUNCTIONS
*****************************************************************************/

void iniOdoEstimationPort(lsm_vedodo::OdoEstimationOutputPort *port);
void fillOdoEstimationPortT(lsm_vedodo::OdoEstimationOutputPort& lsm_vedodo_outputs_CEM, uint64_t timestamp, uint16_t cycle_number);