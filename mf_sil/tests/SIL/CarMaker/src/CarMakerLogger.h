#pragma once

#include <log_global.h>
#include <log_ilogadapter.h>

namespace logger {

    // Log adapter to route messages from plp_log logging system to CarMaker's session log
    class CarMakerLogger : public ILogAdapter
    {
        CarMakerLogger() {}
        ~CarMakerLogger() override {};

    public:
        // return the singleton instance of the CarMakerLogger
        static CarMakerLogger& getInstance();

        CarMakerLogger(CarMakerLogger const&) = delete; // prevent copying the singleton
        CarMakerLogger& operator= (CarMakerLogger const&) = delete;

        /**
        * @brief This function is called when a message is added to the logger.
        */
        void add(LogLevel level, const char* message);

        /**
        * @brief Satisfies ILogAdapter. No implementation required for this adapter
        */
        void init() override {};
        /**
         * @brief Satisfies ILogAdapter. No implementation required for this adapter
         */
        void clear() override {};
        /**
         * @brief Satisfies ILogAdapter. No implementation required for this adapter
         */
        void shutdown() override {};
    };

}


