
#include "USS_mapping.h"

static us_processing::UsProcessingPointList *ussPointList_usp;
static us_processing::UsProcessingDistanceList *ussDistList_usp;
static us_em::UsEnvModelPort *usEmEnvModelPort;

void iniUssPointListPort(us_processing::UsProcessingPointList *points, us_processing::UsProcessingDistanceList *dist, us_em::UsEnvModelPort *usEnvModel) {
    ussPointList_usp = points;
    ussDistList_usp = dist;
    usEmEnvModelPort = usEnvModel;
}

void fillUssPointListPortT(us_processing::UsProcessingPointList& ussPointListPort_CEM, uint64_t time_stamp, uint16_t cycle_number)
{
    ussPointListPort_CEM = *ussPointList_usp;
} //template

void fillUssDistanceListT(us_processing::UsProcessingDistanceList& ussDistanceList_CEM, uint64_t time_stamp, uint16_t cycle_number)
{
    ussDistanceList_CEM = *ussDistList_usp;
} //template

void fillUsEmEnvModelT(us_em::UsEnvModelPort& envModel_usEm, uint64_t time_stamp, uint16_t cycle_number)
{
    envModel_usEm = *usEmEnvModelPort;
} //template