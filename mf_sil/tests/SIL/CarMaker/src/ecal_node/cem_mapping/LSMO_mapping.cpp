/*****************************************************************************
Description			: LSMO Mapping for cem200 integration
Author				: Zhizhe02 Jia
Co-Author			: Hannes Brauckmann
Company             : Continental Autonomous Mobility Germany GmbH
Version				: 1.0
Revision History	:
******************************************************************************/

#include "LSMO_mapping.h"

static lsm_vedodo::OdoEstimationOutputPort *OdoEstimationOT_CM;
void iniOdoEstimationPort(lsm_vedodo::OdoEstimationOutputPort *port) {
    OdoEstimationOT_CM = port;
}

void fillOdoEstimationPortT(lsm_vedodo::OdoEstimationOutputPort& lsm_vedodo_outputs_CEM, uint64_t time_stamp, uint16_t cycle_number)
{
    lsm_vedodo_outputs_CEM = *OdoEstimationOT_CM;
    lsm_vedodo_outputs_CEM.sSigHeader = OdoEstimationOT_CM->sSigHeader;
} //template