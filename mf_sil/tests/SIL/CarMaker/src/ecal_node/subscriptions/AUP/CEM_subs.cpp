#include <chrono>
#include <queue>
#include <string.h>
//#include "sim_logger.h"
#include "SimCore.h"

#include "Subscribe_Share.h"
#include "CEM_subs.hpp"
#include "CEM_disp.hpp"
#include "Log.h"

//extern SimLogger* simLogger;
extern uint16_t cyclenumber;

using namespace std::chrono_literals;
using namespace std::chrono;

#include "aupdf/aupdf_generated_types.h"

eCAL::CSubscriber subscriber_staticObjects_bin; //SefOutput
eCAL::CSubscriber subscriber_parkingDelimiters_bin; // PclOutput
eCAL::CSubscriber subscriber_parkingSlots_bin; // ParkingSlotDetectionOutput
eCAL::CSubscriber subscriber_dynamicObjects_bin; // DynamicEnvironment
eCAL::CSubscriber subscriber_egoMotionOutput_bin; // EgoMotionAtCemOutput
eCAL::CSubscriber subscriber_stopLineOutput_bin; // StopLineOutput
eCAL::CSubscriber subscriber_pedestrianCrossingOutput_bin; //PedestrianCrossingOutput
aupdf::DynamicEnvironment *dynamic_environment_subscriber = new ::aupdf::DynamicEnvironment{ 0 };
aupdf::EgoMotionAtCemOutput *ego_motion_at_cem_output_subscriber = new::aupdf::EgoMotionAtCemOutput{ 0 };
aupdf::ParkingSlotDetectionOutput *parking_slot_detection_output_subscriber = new::aupdf::ParkingSlotDetectionOutput{ 0 };
aupdf::PclOutput *pcl_output_subscriber = new::aupdf::PclOutput{ 0 };
aupdf::SgfOutput *sgf_output_subscriber = new::aupdf::SgfOutput{ 0 };
aupdf::StopLineOutput *stop_Line_output_subscriber = new::aupdf::StopLineOutput{ 0 };
aupdf::PedestrianCrossingOutput *pedes_cross_output_subscriber = new::aupdf::PedestrianCrossingOutput{ 0 };

void setSgf_Outputs(const struct eCAL::SReceiveCallbackData* msg)
{
    *sgf_output_subscriber = *static_cast<aupdf::SgfOutput*>(msg->buf);
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
}

void setPcl_Outputs(const struct eCAL::SReceiveCallbackData* msg)
{
    *pcl_output_subscriber = *static_cast<aupdf::PclOutput*>(msg->buf);
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
}

void setParkingSlot_Outputs(const struct eCAL::SReceiveCallbackData* msg)
{
    *parking_slot_detection_output_subscriber = *static_cast<aupdf::ParkingSlotDetectionOutput*>(msg->buf);
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
}

void setDynObj_Outputs(const struct eCAL::SReceiveCallbackData* msg)
{
    *dynamic_environment_subscriber = *static_cast<aupdf::DynamicEnvironment*>(msg->buf);
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
}

void setEgoMotion_Outputs(const struct eCAL::SReceiveCallbackData* msg)
{
    *ego_motion_at_cem_output_subscriber = *static_cast<aupdf::EgoMotionAtCemOutput*>(msg->buf);
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
}

void setStopLine_Outputs(const struct eCAL::SReceiveCallbackData* msg)
{
    *stop_Line_output_subscriber = *static_cast<aupdf::StopLineOutput*>(msg->buf);
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
}

void setPedestrianCrossing_Outputs(const struct eCAL::SReceiveCallbackData* msg)
{
    *pedes_cross_output_subscriber = *static_cast<aupdf::PedestrianCrossingOutput*>(msg->buf);
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
}

void CEMSubscription::subscribe_CEM()
{
    subscriber_staticObjects_bin.Create("AUPDF_0_StaticObjects");
    subscriber_staticObjects_bin.AddReceiveCallback([this]
    (const char* /*topic_*/,
        const struct eCAL::SReceiveCallbackData* msg) {
        return setSgf_Outputs(msg);
    });

    subscriber_parkingDelimiters_bin.Create("AUPDF_0_ParkingDelimiters");
    subscriber_parkingDelimiters_bin.AddReceiveCallback([this]
    (const char* /*topic_*/,
        const struct eCAL::SReceiveCallbackData* msg) {
        return setPcl_Outputs(msg);
    });

    subscriber_parkingSlots_bin.Create("AUPDF_0_ParkingSlots");
    subscriber_parkingSlots_bin.AddReceiveCallback([this]
    (const char* /*topic_*/,
        const struct eCAL::SReceiveCallbackData* msg) {
        return setParkingSlot_Outputs(msg);
    });

    subscriber_dynamicObjects_bin.Create("AUPDF_0_DynamicObjects");
    subscriber_dynamicObjects_bin.AddReceiveCallback([this]
    (const char* /*topic_*/,
        const struct eCAL::SReceiveCallbackData* msg) {
        return setDynObj_Outputs(msg);
    });

    subscriber_egoMotionOutput_bin.Create("AUPDF_0_EgoMotionOutput");
    subscriber_egoMotionOutput_bin.AddReceiveCallback([this]
    (const char* /*topic_*/,
        const struct eCAL::SReceiveCallbackData* msg) {
        return setEgoMotion_Outputs(msg);
    });

    subscriber_stopLineOutput_bin.Create("AUPDF_0_Stoplines");
    subscriber_stopLineOutput_bin.AddReceiveCallback([this]
    (const char* /*topic_*/,
        const struct eCAL::SReceiveCallbackData* msg) {
        return setStopLine_Outputs(msg);
    });

    subscriber_pedestrianCrossingOutput_bin.Create("AUPDF_0_PedestrianCrossings");
    subscriber_pedestrianCrossingOutput_bin.AddReceiveCallback([this]
    (const char* /*topic_*/,
        const struct eCAL::SReceiveCallbackData* msg) {
        return setPedestrianCrossing_Outputs(msg);
    });

}

void CEMSubscription::reset_subscription_containers_CEM()
{
    memset(dynamic_environment_subscriber, 0, sizeof(*dynamic_environment_subscriber));
    memset(ego_motion_at_cem_output_subscriber, 0, sizeof(*ego_motion_at_cem_output_subscriber));
    memset(parking_slot_detection_output_subscriber, 0, sizeof(*parking_slot_detection_output_subscriber));
    memset(pcl_output_subscriber, 0, sizeof(pcl_output_subscriber));
    memset(sgf_output_subscriber, 0, sizeof(sgf_output_subscriber));
    memset(stop_Line_output_subscriber, 0, sizeof(stop_Line_output_subscriber));
    memset(pedes_cross_output_subscriber, 0, sizeof(pedes_cross_output_subscriber));
    //eCAL::Finalize();
}

