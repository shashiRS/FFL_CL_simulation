#pragma once

#include "eco/system_services.h"
#include <cmath>
#include<time.h>

namespace eco {

  //No specific definition. Just ensure non abstract class based on VehicleParameterService hence needed to make CarMakerSystemServices non abstract.
  class MyVehicleParameterService : public VehicleParameterService
  {
  public:
    MyVehicleParameterService() {}

    ~MyVehicleParameterService() {}

    bool getVehicleMass(float32& paramVehicleMass) const override { return true; }

    bool getAxisLoadDistr(float32& paramAxisLoadDistr) const override { return true; }

    bool getVehicleLength(float32& paramVehicleLength) const override { return true; }

    bool getVehicleWidth(float32& paramVehicleWidth) const override { return true; }

    bool getWheelBase(float32& paramWheelBase) const override { return true; }

    bool getTrackWidthFront(float32& paramTrackWidthFront) const override { return true; }

    bool getTrackWidthRear(float32& paramTrackWidthRear) const override { return true; }

    bool getFrontOverhang(float32& paramFrontOverhang) const override { return true; }

    bool getRearOverhang(float32& paramRearOverhang) const override { return true; }

    bool getCenterOfGravityX(float32& paramCenterOfGravityX) const override { return true; }

    bool getCenterOfGravityY(float32& paramCenterOfGravityY) const override { return true; }

    bool getCenterOfGravityZ(float32& paramCenterOfGravityZ) const override { return true; }

    bool getMountingPosition(const SensorID& sensorID, SensorMountingPositionDetailed& paramSensorPosition) const override { return true; }
  };

  //No specific definition. Just ensure non abstract class based on TimeMeasurementService hence needed to make CarMakerSystemServices non abstract.
  class MyTimeMeasurementService : public TimeMeasurementService
  {
  public:
    MyTimeMeasurementService() {}

    ~MyTimeMeasurementService() {}

    bool start(const uint8 componentTaskId, const uint8 optionalData) override { return true; }

    bool stop(const uint8 componentTaskId, const uint8 optionalData) override { return true; }

    void addTimeMarker(const uint8 markerId, const uint8 optionalData) override { return; }
  };

  // System services to provide system time, receive DEM events and to send to output DEM events to CarMaker's session log
  class CarMakerSystemServices : public SystemServices
  {
  public:
    CarMakerSystemServices() {}
    ~CarMakerSystemServices() override {}
  
    /// @brief Returns the current system time as AlgoDataTimeStamp in microseconds [us]. (HINT: [ms] wrong in base class definition. [us] is correct.)
    /// @return The current system time as AlgoDataTimeStamp.
    uint64 getSystemTime() const override;

    /// @brief Returns the initialization timestamp of the cycle as AlgoDataTimeStamp in microseconds [us]. (HINT: [ms] wrong in base class definition. [us] is correct.)
    /// @return The initialization timestamp of the cycle as AlgoDataTimeStamp.
    uint64 getCemCycleInitTimeStamp() const override;
      
    /// @brief Returns the end timestamp of the cycle as AlgoDataTimeStamp in microseconds [us]. (HINT: [ms] wrong in base class definition. [us] is correct.)
    /// @return The end timestamp of the cycle as AlgoDataTimeStamp.
    uint64 getEndCycleEstimatedTimeStamp() const override;
      
    //TODO: Check whether needed. If so connect to available vehicle parameters. No specific definition. Just ensure non abstract class.
    ::eco::VehicleParameterService& getVehicleParameterService() const override { return eco::MyVehicleParameterService(); }  
      
    //TODO: Check whether needed. No specific definition. Just ensure non abstract class.
    bool writeDebugData(const void * payload,
                        const uint32 nrOfBytes,
                        const uint32 virtualAddress,
                        const uint8 funcChannelID) override {return true;}  //TODO: Check whether needed.
                                
    /// @brief Returns the current measurement counter. If the platform does not provide a measurement
    /// counter 0 will always be returned.
    /// @return The current measurement counter or 0 if this is not provided.
    uint16 getMeasurementCounter() const override { return 0U; }
      
    //TODO: Check whether needed. No specific definition. Just ensure non abstract class.
    ::eco::TimeMeasurementService& getTimeMeasurementService() { return eco::MyTimeMeasurementService(); }     
      
    /// @brief This method provides the possibility to report errors (DEMs) to platform
    /// @param[in] diagnosisEventID diagnostic trouble code identifier
    /// @param[in] diagnosisEventStatus - Error status, such as passed/failed/pre-passed/pre-failed
    /// @param[in] extendedData  - reason for error occurrence
    /// @param[in] extendedDataSize - size of extended data
    bool writeDiagnosisEvent (const uint32 diagnosisEventID,
                              const ::eco::DiagnosisEventStatus diagnosisEventStatus,
                              const void* extendedData,
                              const uint64 extendedDataSize) override;                                 
  };
}


