#include "SimCore.h"

#include "CEM_Call.h"
#include "CarmakerNode_Control.h"

SimLogger* simLogger;

/// Init function to the publishers of all the CEM components 
void CarMakerNode::Init(void)
{

    Initialize_Publishers_Subscribers();
	///scheduler initialization
	//SchdlCfg_Init();
}

void CarMakerNode::TestRun_Start(void)
{
    if (mCemSimulationActive) {
        simLogger = new SimLogger("CM");
        simLogger->init();
        //Initialize_Services();
        Reset_CEM();
    }
}

/// Calc function to start the CEM components to publish the data 
void CarMakerNode::Calc(void)
{
    if (mCemSimulationActive && SimCore.Time > 0) {
		Run_PSS_CEM();
    }
}

/// TestRun_End function to reset the timestamp and also reset the 
/// publishers of all the CEM components
void CarMakerNode::TestRun_End(void)
{
    if (mCemSimulationActive) {
        End_PSS_CEM();
        simLogger->done();
    }
}

void CarMakerNode::MemDeclquant(void)
{
    //Response_Alloc_Declquants(*this);
}

void CarMakerNode::Shutdown(void)
{
    //free_Alloc();
}

void  CarMakerNode::SetCemSimulationActive(const unsigned char cemSimulationActive)
{
    mCemSimulationActive = (cemSimulationActive != 0U);
}