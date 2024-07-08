#pragma once
#include <memory>

#include "CarmakerNode_Control.h"

#include "ecal/msg/string/publisher.h"

#include "LSMO_mapping.h"
#include "USS_mapping.h"
#include "SVC_mapping.h"

/*!
 * @brief MF_Publishers class with functionalities that includes creation, fill messages, 
 * publish and reset the messages
*/
class MF_Publishers {
public:
	/*!
	 * @brief Fucntion to create the publishers for the respective sensors
	 * @param
	*/
	void create_publisher(void);
	/*!
	* @brief Function to fill the messages for all the CEM components
	*/
	void Fill_Messages_to_CEM(void);
	/*!
	* @brief Function to publish the data of all the CEM components
	* @param
	*/
	void publish_to_CEM(void);
	/*!
	 * @brief Function to reset the publisher messages and reset the publisher queues
	* @param
	*/
	void reset_publishers(void);
	/// Constructor for the Pad Publishers class
	MF_Publishers() {}

    static bool isPublishingActive;
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::lsm_vedodo::OdoEstimationOutputPort>> publisher_lsmo_odoEst_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::us_processing::UsProcessingPointList>> publisher_uss_point_list_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::us_processing::UsProcessingDistanceList>> publisher_uss_distance_list_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::us_em::UsEnvModelPort>> publisher_usem_env_model_tp; //template

    std::shared_ptr <CarMakerNode_Publisher_eCAL <::gdr::GdrPointList>> publisher_gdr_PointListFront_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::gdr::GdrPointList>> publisher_gdr_PointListRear_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::gdr::GdrPointList>> publisher_gdr_PointListLeft_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::gdr::GdrPointList>> publisher_gdr_PointListRight_tp; //template

	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingLineList>> publisher_pmsd_ParkingLineListFront_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingLineList>> publisher_pmsd_ParkingLineListRear_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingLineList>> publisher_pmsd_ParkingLineListLeft_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingLineList>> publisher_pmsd_ParkingLineListRight_tp; //template

	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelStopperList>> publisher_pmsd_WheelStopperListFront_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelStopperList>> publisher_pmsd_WheelStopperListRear_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelStopperList>> publisher_pmsd_WheelStopperListLeft_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelStopperList>> publisher_pmsd_WheelStopperListRight_tp; //template

    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelLockerList>> publisher_pmsd_WheelLockerListFront_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelLockerList>> publisher_pmsd_WheelLockerListRear_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelLockerList>> publisher_pmsd_WheelLockerListLeft_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::WheelLockerList>> publisher_pmsd_WheelLockerListRight_tp; //template

	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingSlotList>> publisher_pmsd_ParkingSlotListFront_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingSlotList>> publisher_pmsd_ParkingSlotListRear_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingSlotList>> publisher_pmsd_ParkingSlotListLeft_tp; //template
	std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::ParkingSlotList>> publisher_pmsd_ParkingSlotListRight_tp; //template

    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::StopLineList>> publisher_pmsd_StopLineListFront_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::StopLineList>> publisher_pmsd_StopLineListRear_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::StopLineList>> publisher_pmsd_StopLineListLeft_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::StopLineList>> publisher_pmsd_StopLineListRight_tp; //template

    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::PedestrianCrossingList>> publisher_pmsd_PedestrianCrossingListFront_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::PedestrianCrossingList>> publisher_pmsd_PedestrianCrossingListRear_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::PedestrianCrossingList>> publisher_pmsd_PedestrianCrossingListLeft_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::pmsd::PedestrianCrossingList>> publisher_pmsd_PedestrianCrossingListRight_tp; //template

    std::shared_ptr <CarMakerNode_Publisher_eCAL <::tpp::DynamicObjectList_t>> publisher_tpp_DynamicObjectListFront_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::tpp::DynamicObjectList_t>> publisher_tpp_DynamicObjectListRear_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::tpp::DynamicObjectList_t>> publisher_tpp_DynamicObjectListLeft_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::tpp::DynamicObjectList_t>> publisher_tpp_DynamicObjectListRight_tp; //template

    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SemPointList_t>> publisher_spp_SemPointListFront_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SemPointList_t>> publisher_spp_SemPointListRear_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SemPointList_t>> publisher_spp_SemPointListLeft_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SemPointList_t>> publisher_spp_SemPointListRight_tp; //template

    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SppPolylineList_t>> publisher_spp_PolylineListFront_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SppPolylineList_t>> publisher_spp_PolylineListRear_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SppPolylineList_t>> publisher_spp_PolylineListLeft_tp; //template
    std::shared_ptr <CarMakerNode_Publisher_eCAL <::spp::SppPolylineList_t>> publisher_spp_PolylineListRight_tp; //template
};