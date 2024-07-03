/*****************************************************************************
Description			: eCAL services calls, publish action, and subscription
                      of nodes for cem200 integration
Author				: Zhizhe02 Jia
Co-Author			: Hannes Brauckmann
Company             : Continental Autonomous Mobility Germany GmbH
Version				: 1.0
Revision History	: 
******************************************************************************/
#include "CEM_Call.h"
#include <chrono>
#include <ecal/ecal.h>
#include <ecal/ecal_publisher.h>
#include "Subscribe_Share.h"
#include "MF_publishers.h"
#include "Log.h"

#include "carMakerAP_DLL_common.h"
static constexpr uint64_t CEM_CYCLE_TIME_MS{ LONG_SAMPLE_TIME_MS };
/*****************************************************************************
GENERAL TIMESTAMP, CYCLE NUMBER AND COUNTER FOR CEM COMPONENTS
*****************************************************************************/
uint64_t timestamp ;
uint16_t cyclenumber = 0u;
uint32_t start_system_time;

/// Pad Publisher instance
MF_Publishers publisher_;

/// eCal Service Clients to trigger CEM_LSM nodes
enum CemNode {LSMO, USS, TPF2, SGF, PFS, AUPDF, MAX_NUM_CEM_NODES};
const std::unordered_map<CemNode, std::string> initTriggerNameMap{
    {CemNode::LSMO, "CEM200_LSMO_m_fwpStageTrigger"},
    {CemNode::USS, "CEM200_USS_m_fwpStageTrigger"},
    {CemNode::TPF2, "CEM200_TPF2_m_fwpStageTrigger"},
    {CemNode::SGF, "CEM200_SGF_m_fwpStageTrigger"},
    {CemNode::PFS, "CEM200_PFS_m_fwpStageTrigger"},
    {CemNode::AUPDF, "CEM200_AUPDF_m_fwpStageTrigger"} };
const std::unordered_map<CemNode, std::string> execTriggerNameMap{ 
    {CemNode::LSMO, "CEM200_LSMO_m_fwpRunnableTrigger_0"},
    {CemNode::USS, "CEM200_USS_m_fwpRunnableTrigger_0"},
    {CemNode::TPF2, "CEM200_TPF2_m_fwpRunnableTrigger_0"},
    {CemNode::SGF, "CEM200_SGF_m_fwpRunnableTrigger_0"},
    {CemNode::PFS, "CEM200_PFS_m_fwpRunnableTrigger_0"},
    {CemNode::AUPDF, "CEM200_AUPDF_m_fwpRunnableTrigger_0"} };
std::unordered_map<CemNode, std::shared_ptr<eCAL::CServiceClient>> initTriggerMap;
std::unordered_map<CemNode, std::shared_ptr<eCAL::CServiceClient>> execTriggerMap;

/*****************************************************************************
                                CODE START
*****************************************************************************/

/// Initialization of the eCAL Publishers and Subscribers
void Initialize_Publishers_Subscribers()
{
    /// Initialize Carmaker publishers
    publisher_.create_publisher();

    /// Create CarMaker subscriber and read CEM interfaces for display and sharing for mapping to algo
    subscribe_share();

    /// Initialize eCal Service Clients to trigger CEM_LSM nodes
    for (const auto& initTriggerName : initTriggerNameMap) {
        initTriggerMap.insert(std::pair<CemNode, std::shared_ptr<eCAL::CServiceClient>>(
            initTriggerName.first, std::make_shared<eCAL::CServiceClient>(initTriggerName.second)));
    }
    for (const auto& execTriggerName : execTriggerNameMap) {
        execTriggerMap.insert(std::pair<CemNode, std::shared_ptr<eCAL::CServiceClient>>(
            execTriggerName.first, std::make_shared<eCAL::CServiceClient>(execTriggerName.second)));
    }

    /// Time initialization for logging
    auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
    start_system_time = system_time.count();

}

/// Update the Timestamp for SIP
void Update_Timestamp()
{
    /// Increment timestamps
    if (SimCore.Time > 0) {
        timestamp = uint64_t(std::round(SimCore.Time * 1000u));
        /// Increment cycle
        if (static_cast<uint64_t>(std::round(SimCore.Time * 1000)) % CEM_CYCLE_TIME_MS == 0)
        {
            cyclenumber++;
        }
    }
}

/// Function to fill the CEM messages for all the SIP Components
void fill_cem_msg()
{
    publisher_.Fill_Messages_to_CEM();
}

/// Function to publish the CEM messages and trigger the execution of CEM nodes
void Run_PSS_CEM()
{
	//Publish filled publisher interfaces and send
    publisher_.publish_to_CEM();
	//auto system_time_run_pss = std::chrono::duration_cast<std::chrono::milliseconds>(system_clock::now().time_since_epoch()) % 1000;

    if (static_cast<uint64_t>(std::round(SimCore.Time * 1000)) % CEM_CYCLE_TIME_MS == 0) {
        for (int cemNodeInt{ CemNode::LSMO }; cemNodeInt != CemNode::MAX_NUM_CEM_NODES; cemNodeInt++) {
            const CemNode cemNode{ static_cast<CemNode>(cemNodeInt) };
            const std::string execTriggerName{ execTriggerNameMap.at(cemNode) };
            std::shared_ptr<eCAL::CServiceClient> execTrigger{ execTriggerMap.at(cemNode) };
            if (execTrigger->IsConnected()) {
                execTrigger->Call(execTriggerName, "");
            }
            else {
                LogWarnStr(EC_Sim, (execTriggerName + " trigger not connected.").c_str());
            }
        }
    }
}

/// Function to trigger the CEM node reset
void Reset_CEM() {
    for (int cemNodeInt{ CemNode::LSMO }; cemNodeInt != CemNode::MAX_NUM_CEM_NODES; cemNodeInt++) {
        const CemNode cemNode{ static_cast<CemNode>(cemNodeInt) };
        const std::string initTriggerName{ initTriggerNameMap.at(cemNode) };
        std::shared_ptr<eCAL::CServiceClient> initTrigger{ initTriggerMap.at(cemNode) };
        if (initTrigger->IsConnected()) {
            initTrigger->Call(initTriggerName, "reset_request");
        }
        else {
            LogWarnStr(EC_Sim, (initTriggerName + " trigger not connected.").c_str());
        }
    }
}

void Response_Alloc_Declquants() 
{
	// services_disp();
}

void free_Alloc()
{
    //SEF_Scheduler_API::free_disp_alloc();
}

/// Function to reset the publishers and the timestamp
void End_PSS_CEM()
{
    timestamp = 0;
    cyclenumber = 0;
    /// reseting data containers over pointer
    publisher_.reset_publishers();
    reset_subscription_containers();
}
