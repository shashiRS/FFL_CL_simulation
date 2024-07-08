/// eCAL CEM Headers
#include "CEM_Call.h"
#include "CarmakerNode_Control.h"
/// eCAL core Library
#include <ecal/ecal.h>
/// cls visualization Header 
//#include "cem_carmaker_user.h"

/*!
 * @brief Get Carmaker Reference Node information
*/
auto Carmakernode = CarMakerNode::getReferenceCarmakerNode();

/// Initialization of eCAL and Carmaker Node
void CarMaker_CEM_Init(void)
{
	eCAL::Initialize();
	Carmakernode->Init();
}

/// Shutdown of eCAL and the Carmaker Node
void CarMaker_CEM_Shutdown(void)
{
	Carmakernode->Shutdown();
	eCAL::Finalize();
}

/// Carmaker test run start
void CarMaker_CEM_TestRun_Start(void)
{
	Carmakernode->TestRun_Start();
}

/// Carmaker start to publish the data in eCAL
void CarMaker_CEM_Calc(void)
{
	Carmakernode->Calc();
}

/// Carmaker test run end
void CarMaker_CEM_TestRun_End(void)
{
	Carmakernode->TestRun_End();
}

void Memory_CEM_Reponse_Alloc(void)
{
	//CarMakerNode::getInstance().MemDeclquant();
}

void CEM200_DeclQuants(void)
{
	//call_display_func();
}

/// Update the timstamp and fill the CEM messages
void fill_cem200(void)
{
	Update_Timestamp();
	fill_cem_msg();
}
