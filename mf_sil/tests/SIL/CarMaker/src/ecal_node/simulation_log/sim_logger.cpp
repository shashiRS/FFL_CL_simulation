/*****************************************************************************
Description            : Simulation events and time logger
Original Author        : Marcel Brand
Company             : Continental Teves AG & Co. oHG
Version                : 2.0
Revision History    : Second version
First version URL   : https://github.conti.de/uidj9089/SimulationEngine/blob/sandbox/simengine-pub-clock/simulationengine/src/sim_logger.cpp
******************************************************************************/

/******************************************************************************
* @file  sim_logger.cpp
*
* @author: Mohamed Jmari
* @date: 25.05.2020
*
* @brief: Integration in CEM project and correspondingly changes in logger
*
* @subversion_tags
*   $LastChangedBy:  Mohamed Jmari $
*   $LastChangedRevision:  $
*   $LastChangedDate: 25.05.2020 $
*   $URL:  $
******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <direct.h>
#include <log.h>
#include "SimCore.h"

#include "sim_logger.h"

#if defined(SERVICE_DEBUG)
bool SimLogger::loggerEnabled = true;
#else
bool SimLogger::loggerEnabled = false;
#endif

ofstream SimLogger::logFile = ofstream();
ofstream SimLogger::executionLogFile = ofstream();
const std::string SimLogger::message_types_str[] = { "INFO", "WARNING", "ERROR", "DEBUG" };

void SimLogger::log(uint32_t simTime, MESSAGE_TYPE type, string message) {
    if (loggerEnabled) {
        logFile << setw(fieldWidth) << left << to_string(simTime) << setw(fieldWidth) << left << message_types_str[type] << setw(fieldWidth) << left << message << std::endl;
    }
}



void SimLogger::logExecution(uint32_t systemTime, uint32_t simTime, uint32_t CEM_Cycle,string event_name, string event_category, uint32_t event_duration) {
    if (loggerEnabled) {

		executionLogFile << setw(fieldWidth) << left << to_string(systemTime) << setw(fieldWidth) << left << to_string(simTime) << setw(fieldWidth) << left << to_string(CEM_Cycle) << setw(fieldWidth) << left << event_name << setw(fieldWidth) << left << event_category << setw(fieldWidth) << left << event_duration << std::endl;
	}
}

//void SimLogger::logExecution(uint32_t systemTime, uint32_t simTime, string eventType, string eventName) {
//    if (loggerEnabled) {
//        executionLogFile << setw(fieldWidth) << left << to_string(systemTime) << setw(fieldWidth) << left << to_string(simTime) << setw(fieldWidth) << left << eventType << setw(fieldWidth) << left << eventName << std::endl;
//    }
//}


void SimLogger::init(void){
    if (SimLogger::loggerEnabled)
    {
        // get current working directory for log file location
        char buff[FILENAME_MAX];
        getcwd( buff, FILENAME_MAX );
        std::string current_working_dir(buff);
        std::cout << "cwd: " << current_working_dir << endl;
        // create new log files
        executionLogFile.open("execution_log_" + identifierName + ".txt");
        logFile.open("log_" + identifierName + ".txt");

        // write headings to log files
        logFile << setw(fieldWidth) << left << "Simulation Time (ms)" << setw(fieldWidth) << left << "Type" << setw(fieldWidth) << left << "Message" << std::endl;
#if defined (P_HAD20) || defined (P_PAD21) || defined(P_BMW) 
        executionLogFile << setw(fieldWidth) << left << "System Time since init(us)" << setw(fieldWidth) << left << "Simulation Time (ms)" << setw(fieldWidth) << left << "CEM Cycle" << setw(fieldWidth) << left << "Event" << setw(fieldWidth) << left << "Event Category" << setw(fieldWidth) << left << "Event Duration (us)" << std::endl;
#else
        executionLogFile << setw(fieldWidth) << left << "System Time" << setw(fieldWidth) << left << "Simulation Time" << setw(fieldWidth) << left << "Event Type" << setw(fieldWidth) << left << "Event Name" << std::endl;
#endif
    }
}

void SimLogger::done(void){
    if (SimLogger::loggerEnabled)
    {
        logFile.close();
        executionLogFile.close();
    }
}