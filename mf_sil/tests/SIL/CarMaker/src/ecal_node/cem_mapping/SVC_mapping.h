#pragma once

#include "stdint.h"

#include "CemInputData.hpp"
#include "gdr/gdr_point_list.h"
#include "pmsd/parking_line_list.h"
#include "pmsd/wheel_stopper_list.h"
#include "pmsd/wheel_locker_list.h"
#include "pmsd/parking_slot_list.h"
#include "pmsd/stop_line_list.h"
#include "pmsd/pedestrian_crossing_list.h"
#include "tpp/dynamic_object_list_t.h"
#include "spp/sem_point_list_t.h"
#include "spp/spp_polyline_list_t.h"



void iniCemInputDataPort(svc_model_processing::CemInputData *port);

void fillGdrPointListFrontT(gdr::GdrPointList &m_pGdrPointListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillGdrPointListRearT(gdr::GdrPointList &m_pGdrPointListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillGdrPointListLeftT(gdr::GdrPointList &m_pGdrPointListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillGdrPointListRightT(gdr::GdrPointList &m_pGdrPointListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingLineListFrontT(pmsd::ParkingLineList &m_pParkingLineListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingLineListRearT(pmsd::ParkingLineList &m_pParkingLineListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingLineListLeftT(pmsd::ParkingLineList &m_pParkingLineListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingLineListRightT(pmsd::ParkingLineList &m_pParkingLineListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelStopperListFrontT(pmsd::WheelStopperList &m_pWheelStopperListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelStopperListRearT(pmsd::WheelStopperList &m_pWheelStopperListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelStopperListLeftT(pmsd::WheelStopperList &m_pWheelStopperListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelStopperListRightT(pmsd::WheelStopperList &m_pWheelStopperListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelLockerListFrontT(pmsd::WheelLockerList &m_pWheelLockerListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelLockerListRearT(pmsd::WheelLockerList &m_pWheelLockerListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelLockerListLeftT(pmsd::WheelLockerList &m_pWheelLockerListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillWheelLockerListRightT(pmsd::WheelLockerList &m_pWheelLockerListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingSlotListFrontT(pmsd::ParkingSlotList &m_pParkingSlotListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingSlotListRearT(pmsd::ParkingSlotList &m_pParkingSlotListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingSlotListLeftT(pmsd::ParkingSlotList &m_pParkingSlotListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillParkingSlotListRightT(pmsd::ParkingSlotList &m_pParkingSlotListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillStopLineListFrontT(pmsd::StopLineList &m_pStopLineListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillStopLineListRearT(pmsd::StopLineList &m_pStopLineListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillStopLineListLeftT(pmsd::StopLineList &m_pStopLineListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillStopLineListRightT(pmsd::StopLineList &m_pStopLineListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillPedestrianCrossingListFrontT(pmsd::PedestrianCrossingList &m_pPedestrianCrossingListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillPedestrianCrossingListRearT(pmsd::PedestrianCrossingList &m_pPedestrianCrossingListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillPedestrianCrossingListLeftT(pmsd::PedestrianCrossingList &m_pPedestrianCrossingListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillPedestrianCrossingListRightT(pmsd::PedestrianCrossingList &m_pPedestrianCrossingListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillDynamicObjectListFrontT(tpp::DynamicObjectList_t &m_pDynamicObjectListFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillDynamicObjectListRearT(tpp::DynamicObjectList_t &m_pDynamicObjectListRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillDynamicObjectListLeftT(tpp::DynamicObjectList_t &m_pDynamicObjectListLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillDynamicObjectListRightT(tpp::DynamicObjectList_t &m_pDynamicObjectListRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSemPointListFrontT(spp::SemPointList_t &m_pSemPointFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSemPointListRearT(spp::SemPointList_t &m_pSemPointRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSemPointListLeftT(spp::SemPointList_t &m_pSemPointLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSemPointListRightT(spp::SemPointList_t &m_pSemPointRight_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSppPolylineListFrontT(spp::SppPolylineList_t &m_pSppPolylineFront_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSppPolylineListRearT(spp::SppPolylineList_t &m_pSppPolylineRear_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSppPolylineListLeftT(spp::SppPolylineList_t &m_pSppPolylineLeft_att, uint64_t time_stamp, uint16_t cycle_number);
void fillSppPolylineListRightT(spp::SppPolylineList_t &m_pSppPolylineRight_att, uint64_t time_stamp, uint16_t cycle_number);
