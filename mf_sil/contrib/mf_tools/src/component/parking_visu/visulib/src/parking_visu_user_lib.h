#ifndef PARKING_VISU_USER_LIB_H
#define PARKING_VISU_USER_LIB_H

#include <mf_taposd/TAPOSD_Interface.h>
#include <ap_common/vehicle_params.h>

#include <functional>

class QPainter;

namespace parking_visu {

void setPossibleTargetPoses(const ap_tp::TargetPosesPort& TargetPosesPort);
void setEnvModelData(const si::ApEnvModelPort& envModelPort);
void setParkingBoxData(const si::ApParkingBoxPort& parkingBoxPort);
void setVehicleParams(const ap_common::Vehicle_Params& vehicleParams);

void showData();

void setPaintingHandler(const std::function<void(QPainter*)>& handler);

} // namespace parking_visu

#endif // PARKING_VISU_USER_LIB_H
