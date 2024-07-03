#include "carMakerAP_DLL_common.h"
#include "MF_Publishers.h"


bool MF_Publishers::isPublishingActive{ false };

/// Brief Fucntion to create the publishers for the respective sensors
void MF_Publishers::create_publisher(void)
{
    publisher_lsmo_odoEst_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::lsm_vedodo::OdoEstimationOutputPort>>("mf_vedodo_0_OdoEstimationOutputPort", fillOdoEstimationPortT, LONG_SAMPLE_TIME_MS, LONG_SAMPLE_TIME_MS); //template
    publisher_uss_point_list_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::us_processing::UsProcessingPointList>>("us_processing", fillUssPointListPortT, LONG_SAMPLE_TIME_MS, LONG_SAMPLE_TIME_MS); //template
    publisher_uss_distance_list_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::us_processing::UsProcessingDistanceList>>("CEM200_USS.m_UssDistanceList", fillUssDistanceListT, LONG_SAMPLE_TIME_MS, LONG_SAMPLE_TIME_MS); //template
    publisher_usem_env_model_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::us_em::UsEnvModelPort>>("CEM200_USS.m_UssEnvModel", fillUsEmEnvModelT, LONG_SAMPLE_TIME_MS, LONG_SAMPLE_TIME_MS); //template
    publisher_gdr_PointListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::gdr::GdrPointList>>("SFMPointCloud_Front", fillGdrPointListFrontT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS); //template 
    publisher_gdr_PointListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::gdr::GdrPointList>>("SFMPointCloud_Rear", fillGdrPointListRearT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS * 2); //template 
    publisher_gdr_PointListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::gdr::GdrPointList>>("SFMPointCloud_Left", fillGdrPointListLeftT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS); //template 
    publisher_gdr_PointListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::gdr::GdrPointList>>("SFMPointCloud_Right", fillGdrPointListRightT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_pmsd_ParkingLineListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingLineList>>("PMDLines_Front", fillParkingLineListFrontT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS); //template
    publisher_pmsd_ParkingLineListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingLineList>>("PMDLines_Rear", fillParkingLineListRearT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 4); //template
    publisher_pmsd_ParkingLineListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingLineList>>("PMDLines_Left", fillParkingLineListLeftT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_pmsd_ParkingLineListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingLineList>>("PMDLines_Right", fillParkingLineListRightT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 3); //template
    publisher_pmsd_WheelStopperListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelStopperList>>("WheelStopperLines_Front", fillWheelStopperListFrontT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS); //template
    publisher_pmsd_WheelStopperListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelStopperList>>("WheelStopperLines_Rear", fillWheelStopperListRearT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 4); //template
    publisher_pmsd_WheelStopperListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelStopperList>>("WheelStopperLines_Left", fillWheelStopperListLeftT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_pmsd_WheelStopperListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelStopperList>>("WheelStopperLines_Right", fillWheelStopperListRightT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 3); //template
    publisher_pmsd_WheelLockerListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelLockerList>>("WheelLockerList_Front", fillWheelLockerListFrontT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS); //template
    publisher_pmsd_WheelLockerListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelLockerList>>("WheelLockerList_Rear", fillWheelLockerListRearT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 4); //template
    publisher_pmsd_WheelLockerListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelLockerList>>("WheelLockerList_Left", fillWheelLockerListLeftT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_pmsd_WheelLockerListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::WheelLockerList>>("WheelLockerList_Right", fillWheelLockerListRightT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 3); //template
    publisher_pmsd_ParkingSlotListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingSlotList>>("ParkingSlots_Front", fillParkingSlotListFrontT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS); //template
    publisher_pmsd_ParkingSlotListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingSlotList>>("ParkingSlots_Rear", fillParkingSlotListRearT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 4); //template
    publisher_pmsd_ParkingSlotListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingSlotList>>("ParkingSlots_Left", fillParkingSlotListLeftT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_pmsd_ParkingSlotListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::ParkingSlotList>>("ParkingSlots_Right", fillParkingSlotListRightT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 3); //template
    publisher_pmsd_StopLineListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::StopLineList>>("StopLineList_Front", fillStopLineListFrontT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS); //template
    publisher_pmsd_StopLineListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::StopLineList>>("StopLineList_Rear", fillStopLineListRearT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 4); //template
    publisher_pmsd_StopLineListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::StopLineList>>("StopLineList_Left", fillStopLineListLeftT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_pmsd_StopLineListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::StopLineList>>("StopLineList_Right", fillStopLineListRightT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 3); //template
    publisher_pmsd_PedestrianCrossingListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::PedestrianCrossingList>>("PedestrianCrossingList_Front", fillPedestrianCrossingListFrontT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS); //template
    publisher_pmsd_PedestrianCrossingListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::PedestrianCrossingList>>("PedestrianCrossingList_Rear", fillPedestrianCrossingListRearT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 4); //template
    publisher_pmsd_PedestrianCrossingListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::PedestrianCrossingList>>("PedestrianCrossingList_Left", fillPedestrianCrossingListLeftT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_pmsd_PedestrianCrossingListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::pmsd::PedestrianCrossingList>>("PedestrianCrossingList_Right", fillPedestrianCrossingListRightT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 3); //template
    publisher_tpp_DynamicObjectListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::tpp::DynamicObjectList_t>>("PalincaFront", fillDynamicObjectListFrontT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS); //template
    publisher_tpp_DynamicObjectListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::tpp::DynamicObjectList_t>>("PalincaRear", fillDynamicObjectListRearT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 4); //template
    publisher_tpp_DynamicObjectListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::tpp::DynamicObjectList_t>>("PalincaLeft", fillDynamicObjectListLeftT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_tpp_DynamicObjectListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::tpp::DynamicObjectList_t>>("PalincaRight", fillDynamicObjectListRightT, LONG_SAMPLE_TIME_MS * 4, LONG_SAMPLE_TIME_MS * 3); //template
    publisher_spp_SemPointListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SemPointList_t>>("SPPSemPts_Front", fillSemPointListFrontT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS); //template
    publisher_spp_SemPointListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SemPointList_t>>("SPPSemPts_Rear", fillSemPointListRearT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_spp_SemPointListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SemPointList_t>>("SPPSemPts_Left", fillSemPointListLeftT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS); //template
    publisher_spp_SemPointListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SemPointList_t>>("SPPSemPts_Right", fillSemPointListRightT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_spp_PolylineListFront_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SppPolylineList_t>>("SPPLines_Front", fillSppPolylineListFrontT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS); //template
    publisher_spp_PolylineListRear_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SppPolylineList_t>>("SPPLines_Rear", fillSppPolylineListRearT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS * 2); //template
    publisher_spp_PolylineListLeft_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SppPolylineList_t>>("SPPLines_Left", fillSppPolylineListLeftT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS); //template
    publisher_spp_PolylineListRight_tp = std::make_shared<CarMakerNode_Publisher_eCAL<::spp::SppPolylineList_t>>("SPPLines_Right", fillSppPolylineListRightT, LONG_SAMPLE_TIME_MS * 2, LONG_SAMPLE_TIME_MS * 2); //template
}

///Function to fill the messages for all the CEM components
void MF_Publishers::Fill_Messages_to_CEM()
{
    if (isPublishingActive) {
        publisher_lsmo_odoEst_tp->fill_message(); //template
        publisher_uss_point_list_tp->fill_message(); //template
        publisher_uss_distance_list_tp->fill_message(); //template
        publisher_usem_env_model_tp->fill_message(); //template
        publisher_gdr_PointListFront_tp->fill_message(); //template
        publisher_gdr_PointListRear_tp->fill_message(); //template
        publisher_gdr_PointListLeft_tp->fill_message(); //template
        publisher_gdr_PointListRight_tp->fill_message(); //template
        publisher_pmsd_ParkingLineListFront_tp->fill_message(); //template
        publisher_pmsd_ParkingLineListRear_tp->fill_message(); //template
        publisher_pmsd_ParkingLineListLeft_tp->fill_message(); //template
        publisher_pmsd_ParkingLineListRight_tp->fill_message(); //template
        publisher_pmsd_WheelStopperListFront_tp->fill_message(); //template
        publisher_pmsd_WheelStopperListRear_tp->fill_message(); //template
        publisher_pmsd_WheelStopperListLeft_tp->fill_message(); //template
        publisher_pmsd_WheelStopperListRight_tp->fill_message(); //template
        publisher_pmsd_WheelLockerListFront_tp->fill_message(); //template
        publisher_pmsd_WheelLockerListRear_tp->fill_message(); //template
        publisher_pmsd_WheelLockerListLeft_tp->fill_message(); //template
        publisher_pmsd_WheelLockerListRight_tp->fill_message(); //template
        publisher_pmsd_ParkingSlotListFront_tp->fill_message(); //template
        publisher_pmsd_ParkingSlotListRear_tp->fill_message(); //template
        publisher_pmsd_ParkingSlotListLeft_tp->fill_message(); //template
        publisher_pmsd_ParkingSlotListRight_tp->fill_message(); //template
        publisher_pmsd_StopLineListFront_tp->fill_message(); //template
        publisher_pmsd_StopLineListRear_tp->fill_message(); //template
        publisher_pmsd_StopLineListLeft_tp->fill_message(); //template
        publisher_pmsd_StopLineListRight_tp->fill_message(); //template
        publisher_pmsd_PedestrianCrossingListFront_tp->fill_message(); //template
        publisher_pmsd_PedestrianCrossingListRear_tp->fill_message(); //template
        publisher_pmsd_PedestrianCrossingListLeft_tp->fill_message(); //template
        publisher_pmsd_PedestrianCrossingListRight_tp->fill_message(); //template
        publisher_tpp_DynamicObjectListFront_tp->fill_message(); //template
        publisher_tpp_DynamicObjectListRear_tp->fill_message(); //template
        publisher_tpp_DynamicObjectListLeft_tp->fill_message(); //template
        publisher_tpp_DynamicObjectListRight_tp->fill_message(); //template
        publisher_spp_SemPointListFront_tp->fill_message(); //template
        publisher_spp_SemPointListRear_tp->fill_message(); //template
        publisher_spp_SemPointListLeft_tp->fill_message(); //template
        publisher_spp_SemPointListRight_tp->fill_message(); //template
        publisher_spp_PolylineListFront_tp->fill_message(); //template
        publisher_spp_PolylineListRear_tp->fill_message(); //template
        publisher_spp_PolylineListLeft_tp->fill_message(); //template
        publisher_spp_PolylineListRight_tp->fill_message(); //template
    }
}

/// Function to publish the data of all the CEM components
void MF_Publishers::publish_to_CEM(void)
{
    if (isPublishingActive) {
        publisher_lsmo_odoEst_tp->publish_data(); //template
        publisher_uss_point_list_tp->publish_data(); //template
        publisher_uss_distance_list_tp->publish_data(); //template
        publisher_usem_env_model_tp->publish_data(); //template
        publisher_gdr_PointListFront_tp->publish_data(); //template
        publisher_gdr_PointListRear_tp->publish_data(); //template
        publisher_gdr_PointListLeft_tp->publish_data(); //template
        publisher_gdr_PointListRight_tp->publish_data(); //template
        publisher_pmsd_ParkingLineListFront_tp->publish_data();//template
        publisher_pmsd_ParkingLineListRear_tp->publish_data(); //template
        publisher_pmsd_ParkingLineListLeft_tp->publish_data(); //template
        publisher_pmsd_ParkingLineListRight_tp->publish_data(); //template
        publisher_pmsd_WheelStopperListFront_tp->publish_data(); //template
        publisher_pmsd_WheelStopperListRear_tp->publish_data(); //template
        publisher_pmsd_WheelStopperListLeft_tp->publish_data(); //template
        publisher_pmsd_WheelStopperListRight_tp->publish_data(); //template
        publisher_pmsd_WheelLockerListFront_tp->publish_data(); //template
        publisher_pmsd_WheelLockerListRear_tp->publish_data(); //template
        publisher_pmsd_WheelLockerListLeft_tp->publish_data(); //template
        publisher_pmsd_WheelLockerListRight_tp->publish_data(); //template
        publisher_pmsd_ParkingSlotListFront_tp->publish_data(); //template
        publisher_pmsd_ParkingSlotListRear_tp->publish_data(); //template
        publisher_pmsd_ParkingSlotListLeft_tp->publish_data(); //template
        publisher_pmsd_ParkingSlotListRight_tp->publish_data(); //template
        publisher_pmsd_StopLineListFront_tp->publish_data(); //template
        publisher_pmsd_StopLineListRear_tp->publish_data(); //template
        publisher_pmsd_StopLineListLeft_tp->publish_data(); //template
        publisher_pmsd_StopLineListRight_tp->publish_data(); //template
        publisher_pmsd_PedestrianCrossingListFront_tp->publish_data(); //template
        publisher_pmsd_PedestrianCrossingListRear_tp->publish_data(); //template
        publisher_pmsd_PedestrianCrossingListLeft_tp->publish_data(); //template
        publisher_pmsd_PedestrianCrossingListRight_tp->publish_data(); //template
        publisher_tpp_DynamicObjectListFront_tp->publish_data(); //template
        publisher_tpp_DynamicObjectListRear_tp->publish_data(); //template
        publisher_tpp_DynamicObjectListLeft_tp->publish_data(); //template
        publisher_tpp_DynamicObjectListRight_tp->publish_data(); //template
        publisher_spp_SemPointListFront_tp->publish_data(); //template
        publisher_spp_SemPointListRear_tp->publish_data(); //template
        publisher_spp_SemPointListLeft_tp->publish_data(); //template
        publisher_spp_SemPointListRight_tp->publish_data(); //template
        publisher_spp_PolylineListFront_tp->publish_data(); //template
        publisher_spp_PolylineListRear_tp->publish_data(); //template
        publisher_spp_PolylineListLeft_tp->publish_data(); //template
        publisher_spp_PolylineListRight_tp->publish_data(); //template
    }
}

/// Function to reset the publisher messages and reset the publisher queues
void MF_Publishers::reset_publishers(void)
{ 
    publisher_lsmo_odoEst_tp->reset_messages(); //template
    publisher_uss_point_list_tp->reset_messages(); //template
    publisher_uss_distance_list_tp->reset_messages(); //template
    publisher_usem_env_model_tp->reset_messages(); //template
	publisher_gdr_PointListFront_tp->reset_messages(); //template
	publisher_gdr_PointListRear_tp->reset_messages(); //template
	publisher_gdr_PointListLeft_tp->reset_messages(); //template
	publisher_gdr_PointListRight_tp->reset_messages(); //template
	publisher_pmsd_ParkingLineListFront_tp->reset_messages(); //template
	publisher_pmsd_ParkingLineListRear_tp->reset_messages(); //template
	publisher_pmsd_ParkingLineListLeft_tp->reset_messages(); //template
	publisher_pmsd_ParkingLineListRight_tp->reset_messages(); //template
	publisher_pmsd_WheelStopperListFront_tp->reset_messages(); //template
	publisher_pmsd_WheelStopperListRear_tp->reset_messages(); //template
	publisher_pmsd_WheelStopperListLeft_tp->reset_messages(); //template
	publisher_pmsd_WheelStopperListRight_tp->reset_messages(); //template
    publisher_pmsd_WheelLockerListFront_tp->reset_messages(); //template
    publisher_pmsd_WheelLockerListRear_tp->reset_messages(); //template
    publisher_pmsd_WheelLockerListLeft_tp->reset_messages(); //template
    publisher_pmsd_WheelLockerListRight_tp->reset_messages(); //template
	publisher_pmsd_ParkingSlotListFront_tp->reset_messages(); //template
	publisher_pmsd_ParkingSlotListRear_tp->reset_messages(); //template
	publisher_pmsd_ParkingSlotListLeft_tp->reset_messages(); //template
	publisher_pmsd_ParkingSlotListRight_tp->reset_messages(); //template
    publisher_pmsd_StopLineListFront_tp->reset_messages(); //template
    publisher_pmsd_StopLineListRear_tp->reset_messages(); //template
    publisher_pmsd_StopLineListLeft_tp->reset_messages(); //template
    publisher_pmsd_StopLineListRight_tp->reset_messages(); //template
    publisher_pmsd_PedestrianCrossingListFront_tp->reset_messages(); //template
    publisher_pmsd_PedestrianCrossingListRear_tp->reset_messages(); //template
    publisher_pmsd_PedestrianCrossingListLeft_tp->reset_messages(); //template
    publisher_pmsd_PedestrianCrossingListRight_tp->reset_messages(); //template
    publisher_tpp_DynamicObjectListFront_tp->reset_messages(); //template
    publisher_tpp_DynamicObjectListRear_tp->reset_messages(); //template
    publisher_tpp_DynamicObjectListLeft_tp->reset_messages(); //template
    publisher_tpp_DynamicObjectListRight_tp->reset_messages(); //template
    publisher_spp_SemPointListFront_tp->reset_messages(); //template
    publisher_spp_SemPointListRear_tp->reset_messages(); //template
    publisher_spp_SemPointListLeft_tp->reset_messages(); //template
    publisher_spp_SemPointListRight_tp->reset_messages(); //template
    publisher_spp_PolylineListFront_tp->reset_messages(); //template
    publisher_spp_PolylineListRear_tp->reset_messages(); //template
    publisher_spp_PolylineListLeft_tp->reset_messages(); //template
    publisher_spp_PolylineListRight_tp->reset_messages(); //template
}
