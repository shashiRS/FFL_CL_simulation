#pragma once

#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <iostream>
#include <string>
using namespace std;

class SimLogger
{
public:
    SimLogger(string identifierName = "unknown")
    {
        this->identifierName = identifierName;
    }

    enum MESSAGE_TYPE{
        INFO = 0,
        WARNING,
        ERROR_SIM,
        DEBUGGING
    };
    
    const static std::string message_types_str[];

    static const uint8_t fieldWidth = 60;

    static bool loggerEnabled;

    string identifierName;

    static ofstream executionLogFile;
    static ofstream logFile;

    static void log(uint32_t simTime, MESSAGE_TYPE type, string message);
    

    static void logExecution(uint32_t systemTime, uint32_t simTime, uint32_t CEM_Cycle, string event, string event_category, uint32_t event_duration);

    //static void logExecution(uint32_t systemTime, uint32_t simTime, string eventType, string eventName);

    void init(void);
    void done(void);
};

constexpr auto SimLog = &SimLogger::log;
constexpr auto SimExecLog = &SimLogger::logExecution;