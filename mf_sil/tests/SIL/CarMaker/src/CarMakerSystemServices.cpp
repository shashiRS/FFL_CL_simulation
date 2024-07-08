#include "CarMakerSystemServices.h"
#include <Log.h>
#include <string>
#include "SimCore.h"

namespace eco {

  uint64 CarMakerSystemServices::getSystemTime() const { return static_cast<uint64>(std::round(1e6 * SimCore.Time)); } //SimCore.Time in seconds. getSystemTime() in microseconds. Therefore factor 1e6.

  uint64 CarMakerSystemServices::getCemCycleInitTimeStamp() const { return static_cast<uint64>(std::round(1e6 * SimCore.Time)); } //SimCore.Time in seconds. getCemCycleInitTimeStamp() in microseconds. Therefore factor 1e6.

  uint64 CarMakerSystemServices::getEndCycleEstimatedTimeStamp() const { return static_cast<uint64>(std::round(1e6 * SimCore.Time)); }  //SimCore.Time in seconds. getEndCycleEstimatedTimeStamp() in microseconds. Therefore factor 1e6.
  
  bool CarMakerSystemServices::writeDiagnosisEvent(
    const uint32 diagnosisEventID,
    const ::eco::DiagnosisEventStatus diagnosisEventStatus,
    const void* extendedData,
    const uint64 extendedDataSize)
  {
    if (eco::DiagnosisEventStatus::DIAGNOSIS_EVENT_STATUS_FAILED == diagnosisEventStatus) {
      std::string message = "DIAGNOSIS_EVENT_STATUS_FAILED by ID: " + std::to_string(diagnosisEventID);
      LogErrStr(EC_Sim, message.c_str());
    }
    else if (eco::DiagnosisEventStatus::DIAGNOSIS_EVENT_STATUS_PREFAILED == diagnosisEventStatus) {
      std::string message = "DIAGNOSIS_EVENT_STATUS_PREFAILED by ID: " + std::to_string(diagnosisEventID);
      LogErrStr(EC_Sim, message.c_str());
    }
    else if (eco::DiagnosisEventStatus::DIAGNOSIS_EVENT_STATUS_PASSED == diagnosisEventStatus) {
      std::string message = "DIAGNOSIS_EVENT_STATUS_PASSED by ID: " + std::to_string(diagnosisEventID);
      //Logging deactivated for PASSED:
      //Log("%s\n", message.c_str());    
    }
    else if (eco::DiagnosisEventStatus::DIAGNOSIS_EVENT_STATUS_PREPASSED == diagnosisEventStatus) {
      std::string message = "DIAGNOSIS_EVENT_STATUS_PREPASSED by ID: " + std::to_string(diagnosisEventID);
      //Logging deactivated for PREPASSED:
      //Log("%s\n", message.c_str());
    }
    return true;
  }

}
