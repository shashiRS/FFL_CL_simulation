#include "SVC_mapping.h"


static svc_model_processing::CemInputData *CemInputData_CM;

void iniCemInputDataPort(svc_model_processing::CemInputData *port)
{
	CemInputData_CM = port;
}

void fillGdrPointListFrontT(gdr::GdrPointList &m_pGdrPointListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pGdrPointListFront_att = CemInputData_CM->gdrPointsListFront;
}

void fillGdrPointListRearT(gdr::GdrPointList &m_pGdrPointListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pGdrPointListRear_att = CemInputData_CM->gdrPointsListRear;
}

void fillGdrPointListLeftT(gdr::GdrPointList &m_pGdrPointListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pGdrPointListLeft_att = CemInputData_CM->gdrPointsListLeft;
}

void fillGdrPointListRightT(gdr::GdrPointList &m_pGdrPointListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pGdrPointListRight_att = CemInputData_CM->gdrPointsListRight;
}

void fillParkingLineListFrontT(pmsd::ParkingLineList &m_pParkingLineListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingLineListFront_att = CemInputData_CM->parkingLineListFront;
}

void fillParkingLineListRearT(pmsd::ParkingLineList &m_pParkingLineListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingLineListRear_att = CemInputData_CM->parkingLineListRear;
}

void fillParkingLineListLeftT(pmsd::ParkingLineList &m_pParkingLineListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingLineListLeft_att = CemInputData_CM->parkingLineListLeft;
}

void fillParkingLineListRightT(pmsd::ParkingLineList &m_pParkingLineListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingLineListRight_att = CemInputData_CM->parkingLineListRight;
}

void fillWheelStopperListFrontT(pmsd::WheelStopperList &m_pWheelStopperListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pWheelStopperListFront_att = CemInputData_CM->wheelStopperListFront;
}

void fillWheelStopperListRearT(pmsd::WheelStopperList &m_pWheelStopperListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pWheelStopperListRear_att = CemInputData_CM->wheelStopperListRear;
}

void fillWheelStopperListLeftT(pmsd::WheelStopperList &m_pWheelStopperListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pWheelStopperListLeft_att = CemInputData_CM->wheelStopperListLeft;
}

void fillWheelStopperListRightT(pmsd::WheelStopperList &m_pWheelStopperListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pWheelStopperListRight_att = CemInputData_CM->wheelStopperListRight;
}

void fillWheelLockerListFrontT(pmsd::WheelLockerList & m_pWheelLockerListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pWheelLockerListFront_att = CemInputData_CM->wheelLockerListFront;
}

void fillWheelLockerListRearT(pmsd::WheelLockerList & m_pWheelLockerListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pWheelLockerListRear_att = CemInputData_CM->wheelLockerListRear;
}

void fillWheelLockerListLeftT(pmsd::WheelLockerList & m_pWheelLockerListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pWheelLockerListLeft_att = CemInputData_CM->wheelLockerListLeft;
}

void fillWheelLockerListRightT(pmsd::WheelLockerList & m_pWheelLockerListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pWheelLockerListRight_att = CemInputData_CM->wheelLockerListRight;
}

void fillParkingSlotListFrontT(pmsd::ParkingSlotList &m_pParkingSlotListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingSlotListFront_att = CemInputData_CM->parkingSlotListFront;
}

void fillParkingSlotListRearT(pmsd::ParkingSlotList &m_pParkingSlotListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingSlotListRear_att = CemInputData_CM->parkingSlotListRear;
}

void fillParkingSlotListLeftT(pmsd::ParkingSlotList &m_pParkingSlotListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingSlotListLeft_att = CemInputData_CM->parkingSlotListLeft;
}

void fillParkingSlotListRightT(pmsd::ParkingSlotList &m_pParkingSlotListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pParkingSlotListRight_att = CemInputData_CM->parkingSlotListRight;
}

void fillStopLineListFrontT(pmsd::StopLineList & m_pStopLineListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pStopLineListFront_att = CemInputData_CM->stopLineListFront;
}

void fillStopLineListRearT(pmsd::StopLineList & m_pStopLineListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pStopLineListRear_att = CemInputData_CM->stopLineListRear;
}

void fillStopLineListLeftT(pmsd::StopLineList & m_pStopLineListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pStopLineListLeft_att = CemInputData_CM->stopLineListLeft;
}

void fillStopLineListRightT(pmsd::StopLineList & m_pStopLineListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pStopLineListRight_att = CemInputData_CM->stopLineListRight;
}

void fillPedestrianCrossingListFrontT(pmsd::PedestrianCrossingList & m_pPedestrianCrossingListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pPedestrianCrossingListFront_att = CemInputData_CM->pedestrCrossListFront;
}

void fillPedestrianCrossingListRearT(pmsd::PedestrianCrossingList & m_pPedestrianCrossingListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pPedestrianCrossingListRear_att = CemInputData_CM->pedestrCrossListRear;
}

void fillPedestrianCrossingListLeftT(pmsd::PedestrianCrossingList & m_pPedestrianCrossingListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pPedestrianCrossingListLeft_att = CemInputData_CM->pedestrCrossListLeft;
}

void fillPedestrianCrossingListRightT(pmsd::PedestrianCrossingList & m_pPedestrianCrossingListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{ 
    m_pPedestrianCrossingListRight_att = CemInputData_CM->pedestrCrossListRight;
}

void fillDynamicObjectListFrontT(tpp::DynamicObjectList_t &m_pDynamicObjectListFront_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pDynamicObjectListFront_att = CemInputData_CM->dynamicObjectsListFront;
}

void fillDynamicObjectListRearT(tpp::DynamicObjectList_t &m_pDynamicObjectListRear_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pDynamicObjectListRear_att = CemInputData_CM->dynamicObjectsListRear;
}

void fillDynamicObjectListLeftT(tpp::DynamicObjectList_t &m_pDynamicObjectListLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pDynamicObjectListLeft_att = CemInputData_CM->dynamicObjectsListLeft;
}

void fillDynamicObjectListRightT(tpp::DynamicObjectList_t &m_pDynamicObjectListRight_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pDynamicObjectListRight_att = CemInputData_CM->dynamicObjectsListRight;
}

void fillSemPointListFrontT(spp::SemPointList_t &m_pSemPointFront_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSemPointFront_att = CemInputData_CM->semPointListFront;
}

void fillSemPointListRearT(spp::SemPointList_t &m_pSemPointRear_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSemPointRear_att = CemInputData_CM->semPointListRear;
}

void fillSemPointListLeftT(spp::SemPointList_t &m_pSemPointLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSemPointLeft_att = CemInputData_CM->semPointListLeft;
}

void fillSemPointListRightT(spp::SemPointList_t &m_pSemPointRight_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSemPointRight_att = CemInputData_CM->semPointListRight;
}

void fillSppPolylineListFrontT(spp::SppPolylineList_t &m_pSppPolylineFront_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSppPolylineFront_att = CemInputData_CM->sppPolylineListFront;
}

void fillSppPolylineListRearT(spp::SppPolylineList_t &m_pSppPolylineRear_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSppPolylineRear_att = CemInputData_CM->sppPolylineListRear;
}

void fillSppPolylineListLeftT(spp::SppPolylineList_t &m_pSppPolylineLeft_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSppPolylineLeft_att = CemInputData_CM->sppPolylineListLeft;
}

void fillSppPolylineListRightT(spp::SppPolylineList_t &m_pSppPolylineRight_att, uint64_t time_stamp, uint16_t cycle_number)
{
    m_pSppPolylineRight_att = CemInputData_CM->sppPolylineListRight;
}