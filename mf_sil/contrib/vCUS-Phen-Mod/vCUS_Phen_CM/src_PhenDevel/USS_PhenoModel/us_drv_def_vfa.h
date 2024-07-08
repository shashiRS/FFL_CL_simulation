/**
 * @brief       Definitions related to the VFA Ultrasonic Sensor
 *
 * @details     This file contains definitions describing the Vodafone
 *              automotive ultrasonic sensor protocol.
 *
 * @author      Robin Adams <robin.adams@continental-corporation.com>
 * @copyright   2019 Continental Teves AG & Co. oHG / Continental Corporation
 *
 * @ingroup     US_DRV
 *
 * @file
 *
 */

#ifndef US_DRV_DEF_VFA
#define US_DRV_DEF_VFA

#include <cstdint>

namespace US_DRV {

	/**
	 * @brief       Definitions for GEN2.8 Sensor
	 *
	 * @details     This namespace contains the definitions for the GEN 2.8
	 *              ultrasonic sensor.
	 *
	 * @namespace
	 *
	 */
	namespace VFA28 {

#pragma pack(push, 1)

		/**
		 * @brief       GEN 2.8 Command Codes Enumeration.
		 *
		 * @details     Definition of the Commands supported by the GEN2.8 ultrasonic
		 *              sensor.
		 *
		 */
		enum SENSOR_COMMAND : uint8_t {
			SENSOR_COMMAND_REGISTER_READ = 0U,  /**< Data Read */
			SENSOR_COMMAND_READ_MODULE_ID_SELECTION = 1U,  /**< Module ID Selection */
			SENSOR_COMMAND_READ_MODULE_ID_ADDRESS = 2U,  /**< Module ID Address */
			SENSOR_COMMAND_START_FIRING = 3U,  /**< Start Firing (broadcasting, sync on PDCM) */
			SENSOR_COMMAND_STOP_FIRING = 4U,  /**< Stop Firing */
			SENSOR_COMMAND_SLEEP_MODE_SELECTION = 5U,  /**< Sleep Mode Selection */
			SENSOR_COMMAND_SENSOR_RELEASE_VERSION = 6U,  /**< Sensor Release Version */
			SENSOR_COMMAND_SELF_CHECK = 7U,  /**< Self-Check */
			SENSOR_COMMAND_REGISTER_WRITE = 8U,  /**< Data Write */
			SENSOR_COMMAND_PREPARE_START_FIRING = 9U,  /**< Prepare Start Firing */
			SENSOR_COMMAND_DIAGNOSTIC_RX_MODE = 10U, /**< Diagnostic Rx Mode */
			SENSOR_COMMAND_DIAGNOSTIC_TX_MODE = 11U, /**< Diagnostic Tx Mode */
			SENSOR_COMMAND_NA1 = 12U, /**< NA */
			SENSOR_COMMAND_NA2 = 13U, /**< NA */
			SENSOR_COMMAND_UPDATE_MEASURE_MODE = 14U, /**< Update Measure Mode */
			SENSOR_COMMAND_RELOAD_DEFAULT_PARAMETERS = 15U  /**< Reload Default Parameters */
		};

		/**
		 * @brief       GEN 2.8 Status Codes Enumeration.
		 *
		 * @details     Definition of the Status Codes returned by the
		 *              GEN2.8 ultrasonic sensor.
		 *
		 */
		enum SENSOR_STATUS : uint8_t {
			SENSOR_STATUS_COMMAND_PERFORMED = 0U, /**< Command performed */
			SENSOR_STATUS_COMMAND_IN_PROGRESS = 1U, /**< Command in progress */
			SENSOR_STATUS_FAILED = 2U, /**< Command Failed */
			SENSOR_STATUS_RESERVED = 3U, /**< Reserved */
			SENSOR_STATUS_COMMAND_NOT_AVAILABLE = 4U, /**< Command Not Available */
			SENSOR_STATUS_PARAMETER_OUT_OF_RANGE = 5U, /**< Parameters Out of Range */
			SENSOR_STATUS_CONDITION_NOT_CORRECT = 6U, /**< Condition Not Correct */
			SENSOR_STATUS_COMMAND_CRC_ERROR = 7U  /**< Command CRC Error */
		};

		constexpr uint8_t PDCM_HEADER_SIZE = 6U;       /**< PDCM header size in bytes */
		constexpr uint8_t PDCM_MEASUREMENT_SIZE = 6U;  /**< PDCM measurement size in bytes */
		constexpr uint8_t PDCM_MEASUREMENT_COUNT = 5U; /**< PDCM measurement count */
		constexpr uint8_t PDCM_FRAME_SIZE = PDCM_HEADER_SIZE + PDCM_MEASUREMENT_SIZE * PDCM_MEASUREMENT_COUNT + 1U; /**< PDCM Frame Size in bytes */

		/**
		 * @brief       GEN 2.8 PDCM Frame Header
		 *
		 * @details     Header at the start of every PDCM frame.
		 *
		 */
		union PdcmHeader {
			uint8_t rawData[PDCM_HEADER_SIZE];
			struct {
				uint8_t sensorTemperatureHigh : 1; /**< hi 2 bits temp         Byte 0 bit 0  */
				uint8_t subFramePosition : 1; /**<  lo => Mearurement 0..3   hi => Measurement 4..7   BYte 0 bit 1 */
				uint8_t newData : 1; /**<  see UssPdcmNewData_e   Byte 0 bit 2    */
				uint8_t echoType : 1; /**<  if set, this frame contains measurements from the cross echo path, direct echo path measurements otherwise   */
				uint8_t physicalAddress : 4; /**< DSI3 physical address of the sensor */
				uint8_t transducerFail : 1; /**<  reserved               Byte 1 bit 1..0 */
				uint8_t noise : 1; /**<  reserved               Byte 1 bit 1..0 */
				uint8_t status : 2; /**<  status                 Byte 1 bit 3..4 */
				uint8_t sensorTemperatureLow : 4; /**<  sensor temperature     Byte 1 bit 7..4 */
				uint8_t syncCntCurrent;            /**< Tag                    Byte 2 Bit 0..7 */
			} rec;
		};

		/**
		 * @brief       GEN 2.8 PDCM Measurement
		 *
		 * @details     A single measurement in the PDCM Frame.
		 *
		 */
		union PdcmMeasurement {
			uint8_t rawData[PDCM_MEASUREMENT_SIZE];
			struct {
				uint8_t syncCntMeasurement;            /*!< Indicate the sync number when the computation cycle is completed related with its ToF Byte 0*/
				uint8_t timeOfFlightHigh : 6;          /*!< 8usec pro bit 0..0xFFFF    */
				uint8_t nfd : 1;          /*!< if the     */
				uint8_t virtualisation : 1;          /*!< the virtualisation bit */
				uint8_t timeOfFlightLow;               /*!< 8usec pro bit 0..0xFFFF    */
				uint8_t codingConfidenceLevel : 4; /*!< US Coding Confidence Level 0..15 */
				uint8_t timeDomainConfidenceLevel : 4; /*!< Confidence Level Time Domain 0..15 */
				int8_t  dopplerFrequency;              /*!< Doppler frequency due to the obstacle or vehicle movement with 100Hz    */
				uint8_t amplitude;                     /*!< Amplitude */
			} rec;
		};               /*!< Measure Response from -Sensor */

		static_assert(sizeof(PdcmMeasurement) == PDCM_MEASUREMENT_SIZE, "PdcmMeasurement wrong size");

		/**
		 * @brief       GEN 2.8 PDCM Frame
		 *
		 * @details     When measuring the sensors sends PDCM (periodic data collection mode) Frames
		 *              consisting of a header and several measurements.
		 *
		 */
		union PdcmFrame {
			uint8_t rawData[PDCM_FRAME_SIZE];
			struct {
				PdcmHeader      header;
				PdcmMeasurement measurement[PDCM_MEASUREMENT_COUNT];
				uint8_t         crc;
			} rec;
		};

		constexpr uint8_t PDCM_RX_DIAGNOSTIC_HEADER_SIZE = 3U;     /*!>  header size in byte */
		constexpr uint8_t PDCM_RX_DIAGNOSTIC_MEASUREMENT_COUNT = 15U;     /*!>  header size in byte */

		union PdcmRxDiagnosticHeader {
			uint8_t rawData[PDCM_RX_DIAGNOSTIC_HEADER_SIZE];
			struct {                  /* R7 DSI3 interface over SPI */
				uint8_t reserved1 : 4; /*!<     Byte 0 bit 3    */
				uint8_t physicalAddress : 4; /*!< see UssPhyAdr_e        Byte 0 bit 7..4 */
				uint8_t reserved2 : 4;
				uint8_t format : 1;
				uint8_t indirect : 1;
				uint8_t valid : 1;
				uint8_t shoot : 1;
				uint8_t syncCntCurrent;      /*!< Tag                      Byte 2 Bit 0..7 */
			} rec;
		};

		union PdcmRxDiagnosticFrame {
			uint8_t rawData[PDCM_FRAME_SIZE];
			struct {
				PdcmRxDiagnosticHeader header;
				uint16_t measurements[PDCM_RX_DIAGNOSTIC_MEASUREMENT_COUNT];
				uint8_t  crc;
			} rec;
		};

		constexpr uint8_t PDCM_RECORDER_MODE_ECHO_SIZE = 6U;  /**< PDCM echo size in recorder mode in bytes */
		constexpr uint8_t PDCM_RECORDER_MODE_ECHO_COUNT = 5U; /**< PDCM echo count in recorder mode */

		/**
		 * @brief       GEN 2.8 PDCM Frame Header - Recorder Mode
		 *
		 * @details     Header at the start of every PDCM frame in recorder mode.
		 *
		 */
		union PdcmRecorderModeHeader {
			uint8_t rawData[PDCM_HEADER_SIZE];
			struct {
				//Byte 0
				uint8_t echosLost : 1;          /**<  if set, echo is not reported due to reception time of > reception_time_max or one of the sensor receiving buffers has an overflow   Byte 0 bit 0   */
				uint8_t rxTransDelay : 1;       /**<  indicate if sensor can not send all received echoes in the current PDCM frame   Byte 0 bit 1    */
				uint8_t pdcmMode : 1;           /**<  specify the system PDCM status		   Byte 0 bit 2 */
				uint8_t burstFlag : 1;          /**<  indicate if a burst is started         Byte 0 bit 3  */
				uint8_t physicalAddress : 4;    /**<  DSI3 physical address of the sensor    Byte 0 bit 4..7 */
				//Byte 1
				uint8_t multiplexDataCnt : 3;   /**<  indicate the received multiplexed data frame number	   Byte 1 bit 0..2 */
				uint8_t notUsed : 2;   /**< Not used */
				uint8_t sumErrorFlag : 1;       /**<  indicate if the sensor has detected an internal failure	   Byte 1 bit 5 */
				uint8_t burstErrorFlag : 1;     /**<  indicate if the sensor is not able to transmit a burst	   Byte 1 bit 6 */
				uint8_t blankingFlag : 1;       /**<  indicate if the sensor is not able to receive an echo during the PDCM sync    Byte 1 bit 7 */
				//Byte 2
				uint8_t syncCntCurrent;         /**< Tag									   Byte 2 Bit 0..7 */
				//Byte 3
				uint8_t AATG1_TH;               /**<  filt1 of the advanced coding path of the ELMOS ASSP	   Byte 3 bit 0..7 */
				//Byte 4
				uint8_t AATG2_TH;               /**<  filt2 of the advanced coding path of the ELMOS ASSP	   Byte 4 bit 0..7 */
				//Byte 5
				uint8_t multiplexedData         /**< contain 8 frames of error details & info		   Byte 5 bit 0..7 */;
			} rec;
		};

		/**
		 * @brief       GEN 2.8 PDCM Recoder Mode Echo Data
		 *
		 * @details     A single echo in Recorder Mode.
		 *
		 */
		union PdcmRecorderModeEcho {
			uint8_t rawData[PDCM_RECORDER_MODE_ECHO_SIZE];
			struct {
				uint16_t receptionTime;               /*!< 1usec/LSB 0..0xFFF    */
				uint16_t amplitude;                   /*!< Amplitude */
				int8_t phaseDerivative;               /*!< phase derivative of the standard path for the echoes received range from -128 to 127 */
				uint8_t selectedSignal : 3;           /*!< Indicate by which reception path the echo was received */
				uint8_t notUsed : 1;                  /*!< Not used */
				uint8_t codingConfidenceLevel : 4;    /*!< US Coding Confidence Level 0..15 */
			} rec;
			struct {
				uint16_t ringingTime;            /*!< ringing time RTM    */
				uint16_t ringingFreq;            /*!< ringing frequency RFM */
				uint8_t burstDelay : 4;          /*!< burst flag delay count */
				uint8_t notUsed1 : 4;            /*!< Not used */
				uint8_t selectedSignal : 3;      /*!< Indicate by which reception path the echo was received */
				uint8_t notUsed2 : 5;            /*!< Not used */
			} burstInfo;
		};

		static_assert(sizeof(PdcmRecorderModeEcho) == PDCM_RECORDER_MODE_ECHO_SIZE, "PdcmRecorderEcho wrong size");

		/**
		 * @brief       GEN 2.8 PDCM Recoder Mode PDCM Frame
		 *
		 * @details     The PDCM frame in recorder mode.
		 *
		 */
		union PdcmRecorderModeFrame {
			uint8_t rawData[PDCM_FRAME_SIZE];
			struct {
				PdcmRecorderModeHeader header;
				PdcmRecorderModeEcho   echoes[PDCM_RECORDER_MODE_ECHO_COUNT];
				uint8_t            crc;
			} rec;
		};

		static_assert(sizeof(PdcmRecorderModeFrame) == PDCM_FRAME_SIZE, "PdcmRecorderFrame wrong size");

		constexpr uint16_t SENSOR_EEPROM_SIZE = 256U; /*!< array size EEPROM in sensor */

		constexpr uint16_t SENSOR_CONFIGURATION_ADDRESS = 0x98U; /*!< address of sensor configuration */
		constexpr uint16_t SENSOR_CONFIGURATION_RECORDER_ENABLED = 0x08U; /*!< recorder enabled bit */

#pragma pack(pop)

	} // namepsace VFA28

} // namespace US_DRV

#endif /* US_DRV_DEF_VFA */