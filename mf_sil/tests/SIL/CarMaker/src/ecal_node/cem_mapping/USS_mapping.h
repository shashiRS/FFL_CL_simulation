#pragma once
#include "stdint.h"

#include "us_processing/us_processing_point_list.h"
#include "us_processing/us_processing_distance_list.h"
#include "us_em/us_env_model_port.h"

void iniUssPointListPort(us_processing::UsProcessingPointList *port, us_processing::UsProcessingDistanceList *dist, us_em::UsEnvModelPort *usEnvModel);

void fillUssPointListPortT(us_processing::UsProcessingPointList& ussPort_CEM, uint64_t timestamp, uint16_t cycle_number);

void fillUssDistanceListT(us_processing::UsProcessingDistanceList& ussDistanceList_CEM, uint64_t time_stamp, uint16_t cycle_number);

void fillUsEmEnvModelT(us_em::UsEnvModelPort& envModel_usEm, uint64_t time_stamp, uint16_t cycle_number);