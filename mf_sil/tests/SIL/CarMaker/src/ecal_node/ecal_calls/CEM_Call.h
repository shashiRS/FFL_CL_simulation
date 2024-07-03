
#ifndef _CEM_CALL_H__
#define _CEM_CALL_H__

/*!
 * @brief Initialization of the eCAL Publishers and Subscribers
*/
void Initialize_Publishers_Subscribers(void);
/*!
 * @brief Update the Timestamp for SIP
*/
void Update_Timestamp(void);
/*!
 * @brief Function to publish the CEM messages and trigger the execution of CEM nodes
*/
void Run_PSS_CEM(void);
/*!
 * @brief Function to trigger the CEM node reset
*/
void Reset_CEM(void);
/*!
 * @brief Function to reset the publishers and the timestamp
*/
void End_PSS_CEM(void);
void free_Alloc(void);
/*!
 * @brief Function to fill the CEM messages for all the SIP Components
*/
void fill_cem_msg(void);

#endif	/* #ifndef _CEM_CALL_H__ */
