#include "CarMakerLogger.h"
#include <Log.h>

namespace logger {

    // return the singleton instance of the CarMakerLogger;
    CarMakerLogger& CarMakerLogger::getInstance() {
        static CarMakerLogger instance;
        return instance;
    }

    void CarMakerLogger::add(LogLevel level, const char* message) {
        switch (level) {
        case LogLevel::LOGGER_INFO:
            Log("%s\n", message);
            break;
        case LogLevel::LOGGER_WARNING:
            LogWarnStr(EC_Sim, message);
            break;
        case LogLevel::LOGGER_ERROR:
        case LogLevel::LOGGER_FATAL:
            LogErrStr(EC_Sim, message);
            break;
        default:
            Log("%s\n", message);
        }
    }

}
