#pragma once
/*!
 * @brief Initialization of eCAL and Carmaker Node
 * @param
*/
void CarMaker_CEM_Init(void);
/*!
 * @brief Shutdown of eCAL and the Carmaker Node
 * @param
*/
void CarMaker_CEM_Shutdown(void);
/*!
 * @brief Carmaker test run start
 * @param
*/
void CarMaker_CEM_TestRun_Start(void);
/*!
 * @brief Carmaker start to publish the data in eCAL
 * @param
*/
void CarMaker_CEM_Calc(void);
/*!
 * @brief Carmaker test run end
 * @param
*/
void CarMaker_CEM_TestRun_End(void);
/*!
 * @brief 
 * @param  
*/
void Memory_CEM_Reponse_Alloc(void); 
/*!
 * @brief 
 * @param  
*/
void CEM200_DeclQuants(void);
/*!
 * @brief Update the timstamp and fill the CEM messages
 * @param
*/
void fill_cem200(void);
