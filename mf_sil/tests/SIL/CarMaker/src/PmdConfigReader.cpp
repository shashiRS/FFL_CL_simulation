#include"PmdConfigReader.h"
#include <Log.h> // CarMaker logging
#include "MfSilTypes.h"

#pragma warning( push )
#pragma warning ( disable: 4996 ) // disable deprecation warnings

inline float32_t convertMMtoM(const float32_t x_mm) { return static_cast<float32_t>(x_mm / 1000); }


// Fill the given configuration structure with information loaded from file (PMD_ALGORITHM)cfg
bool PmdConfigReader::loadPepDemoConfiguration(pmd::PepDemoConf& cfg, char const * const jsonFileName)
{
    std::ifstream jsonPepDemoCfgFile(jsonFileName);
    ::Json::Reader readerCfg;
    ::Json::Value jsonPepDemoCfgValues;
    if (jsonPepDemoCfgFile.is_open()) {
        bool isOk = readerCfg.parse(jsonPepDemoCfgFile, jsonPepDemoCfgValues, false);
        // Parse the json config file with the reader and load it into the json value variable
        if (isOk) {
            cfg.pmdmr = jsonPepDemoCfgValues["enableAlgo"]["pmd_mr"].asUInt();
            cfg.pmdml = jsonPepDemoCfgValues["enableAlgo"]["pmd_ml"].asUInt();
            cfg.pmdfv = jsonPepDemoCfgValues["enableAlgo"]["pmd_fv"].asUInt();
            cfg.pmdrv = jsonPepDemoCfgValues["enableAlgo"]["pmd_rv"].asUInt();
            cfg.sfm = jsonPepDemoCfgValues["enableAlgo"]["sfm"].asUInt();
            cfg.uss = jsonPepDemoCfgValues["enableAlgo"]["uss"].asUInt();
            cfg.cem = jsonPepDemoCfgValues["enableAlgo"]["cem"].asUInt();
            cfg.si = jsonPepDemoCfgValues["enableAlgo"]["si"].asUInt();
            
            return isOk;
        }
        else {
            LogWarnStr(EC_General, "PMDSurrogate: PepDemoCfg: Error file read (no cameras available)");
            return isOk;
        }
    }
    else {
        LogWarnStr(EC_General, "PMDSurrogate: PepDemoCfg: Error file open (no cameras available)");
        return false;
    }
}

// Fill the given configuration structure with information loaded from file (PMD_ALGORITHM)cfg
bool PmdConfigReader::loadPMDCamConfiguration(pmd::PmdCameraConfig& cfg, char const * const jsonFileName)
{
    static Json::Reader reader;
    static Json::Value  jsonValues;
    std::ifstream jsonFile(jsonFileName);

    if (jsonFile.is_open()) {
        // parse the json config file with the reader and load it into the json value variable
        if (reader.parse(jsonFile, jsonValues, false)) {
            const uint32_t maxCameraNameLength = 100U;
            char cameraName[maxCameraNameLength];
            switch (cfg.camId) {
            case pmd::CAMERAIDX::CAM_FRONT: strcpy(cameraName, "PMDConfig_frontCamera");
                break;
            case pmd::CAMERAIDX::CAM_REAR: strcpy(cameraName, "PMDConfig_rearCamera");
                break;
            case pmd::CAMERAIDX::CAM_LEFT: strcpy(cameraName, "PMDConfig_leftCamera");
                break;
            case pmd::CAMERAIDX::CAM_RIGHT: strcpy(cameraName, "PMDConfig_rightCamera");
                break;
            default: strcpy(cameraName, "PMDConfig_rightCamera");
                break;
            }
            //set camera availability
            cfg.cameraAvailability = true;
            // Topdown image transform
            cfg.topDownViewHeight_m = convertMMtoM(static_cast<float32_t>(jsonValues[cameraName]["topDownView"]["topDownViewHeight_mm"].asInt()));
            cfg.topDownViewWidth_m = convertMMtoM(static_cast<float32_t>(jsonValues[cameraName]["topDownView"]["topDownViewWidth_mm"].asInt()));

            cfg.lineSegmentConfig.roiThresholdingX_m = convertMMtoM(static_cast<float32_t>(jsonValues[cameraName]["topDownView"]["roiThresholdingX_mm"].asInt()));
            cfg.lineSegmentConfig.roiThresholdingY_m = convertMMtoM(static_cast<float32_t>(jsonValues[cameraName]["topDownView"]["roiThresholdingY_mm"].asInt()));
            cfg.lineSegmentConfig.roiThresholdingWidth_m = convertMMtoM(static_cast<float32_t>(jsonValues[cameraName]["topDownView"]["roiThresholdingWidth_mm"].asInt()));
            cfg.lineSegmentConfig.roiThresholdingHeight_m = convertMMtoM(static_cast<float32_t>(jsonValues[cameraName]["topDownView"]["roiThresholdingHeight_mm"].asInt()));

            return true;
        }
        else {
            LogWarnStr(EC_General, "PMDSurrogate: PmdCamCfg: Error file read");
            return false;
        }
    }
    else {
        LogWarnStr(EC_General, "PMDSurrogate: PmdCamCfg: Error file open");
        return false;
    }
}

bool PmdConfigReader::resetPMDCamConfigurationTo0(pmd::PmdCameraConfig& cfg) {
    cfg.cameraAvailability = false;
    cfg.topDownViewHeight_m = convertMMtoM(0);
    cfg.topDownViewWidth_m = convertMMtoM(0);
    cfg.lineSegmentConfig.roiThresholdingX_m = convertMMtoM(0);
    cfg.lineSegmentConfig.roiThresholdingY_m = convertMMtoM(0);
    cfg.lineSegmentConfig.roiThresholdingWidth_m = convertMMtoM(0);
    cfg.lineSegmentConfig.roiThresholdingHeight_m = convertMMtoM(0);
    return true;
}

void PmdConfigReader::setDefaultValuesPMDConfiguration(pmd::PmdConfig& cfg) {
    cfg.frontPmdCamera.camId = pmd::CAMERAIDX::CAM_FRONT;
    cfg.frontPmdCamera.cameraAvailability = true;
    cfg.frontPmdCamera.topDownViewHeight_m = convertMMtoM(6000);
    cfg.frontPmdCamera.topDownViewWidth_m = convertMMtoM(4000);
    cfg.frontPmdCamera.lineSegmentConfig.roiThresholdingX_m = convertMMtoM(0);
    cfg.frontPmdCamera.lineSegmentConfig.roiThresholdingY_m = convertMMtoM(2000);
    cfg.frontPmdCamera.lineSegmentConfig.roiThresholdingWidth_m = convertMMtoM(4000);
    cfg.frontPmdCamera.lineSegmentConfig.roiThresholdingHeight_m = convertMMtoM(4000);

    cfg.rearPmdCamera.camId = pmd::CAMERAIDX::CAM_REAR;
    cfg.rearPmdCamera.cameraAvailability = true;
    cfg.rearPmdCamera.topDownViewHeight_m = convertMMtoM(7000);
    cfg.rearPmdCamera.topDownViewWidth_m = convertMMtoM(6000);
    cfg.rearPmdCamera.lineSegmentConfig.roiThresholdingX_m = convertMMtoM(1000);
    cfg.rearPmdCamera.lineSegmentConfig.roiThresholdingY_m = convertMMtoM(0);
    cfg.rearPmdCamera.lineSegmentConfig.roiThresholdingWidth_m = convertMMtoM(4000);
    cfg.rearPmdCamera.lineSegmentConfig.roiThresholdingHeight_m = convertMMtoM(4000);

    cfg.leftPmdCamera.camId = pmd::CAMERAIDX::CAM_LEFT;
    cfg.leftPmdCamera.cameraAvailability = true;
    cfg.leftPmdCamera.topDownViewHeight_m = convertMMtoM(6200);
    cfg.leftPmdCamera.topDownViewWidth_m = convertMMtoM(7200);
    cfg.leftPmdCamera.lineSegmentConfig.roiThresholdingX_m = convertMMtoM(4250);
    cfg.leftPmdCamera.lineSegmentConfig.roiThresholdingY_m = convertMMtoM(250);
    cfg.leftPmdCamera.lineSegmentConfig.roiThresholdingWidth_m = convertMMtoM(4000);
    cfg.leftPmdCamera.lineSegmentConfig.roiThresholdingHeight_m = convertMMtoM(5000);

    cfg.rightPmdCamera.camId = pmd::CAMERAIDX::CAM_RIGHT;
    cfg.rightPmdCamera.cameraAvailability = true;
    cfg.rightPmdCamera.topDownViewHeight_m = convertMMtoM(6200);
    cfg.rightPmdCamera.topDownViewWidth_m = convertMMtoM(7200);
    cfg.rightPmdCamera.lineSegmentConfig.roiThresholdingX_m = convertMMtoM(250);
    cfg.rightPmdCamera.lineSegmentConfig.roiThresholdingY_m = convertMMtoM(250);
    cfg.rightPmdCamera.lineSegmentConfig.roiThresholdingWidth_m = convertMMtoM(4000);
    cfg.rightPmdCamera.lineSegmentConfig.roiThresholdingHeight_m = convertMMtoM(5000);
}

bool logWarningPmdDefaultValuesUsed()
{
    LogWarnStr(EC_General, "Load PMD Camera configuration error \n"
        "Default values are used");
    return false;
}

bool PmdConfigReader::loadPmdAlgorithmConfig(pmd::PmdConfig& PMDcfg, char const * const jsonPMDFileConfigName, char const * const jsonPepDemoConfigFileName)
{
    PMDcfg.init();
    pmd::PepDemoConf pepDemoConf;
    memset(&pepDemoConf, 0, sizeof(pepDemoConf));
    bool isPepDemoConfOk = loadPepDemoConfiguration(pepDemoConf, jsonPepDemoConfigFileName);

    if (isPepDemoConfOk) {
        bool isPmdConfigRead = (pepDemoConf.pmdml == 1U) ? loadPMDCamConfiguration(PMDcfg.leftPmdCamera, jsonPMDFileConfigName) :
            resetPMDCamConfigurationTo0(PMDcfg.leftPmdCamera);
        //if the PMD Configuration read is not succeded, exit from function
        if (!isPmdConfigRead) { 
            return logWarningPmdDefaultValuesUsed();
        }

        isPmdConfigRead = (pepDemoConf.pmdfv == 1U) ? loadPMDCamConfiguration(PMDcfg.frontPmdCamera, jsonPMDFileConfigName) :
            resetPMDCamConfigurationTo0(PMDcfg.frontPmdCamera);

        //do for the all cases to cover the case when the previous camera is not available for pmd algoritm
        if (!isPmdConfigRead) {
            return logWarningPmdDefaultValuesUsed();
        }

        //do for the all cases to cover the case when the previous cameras are not available for pmd algoritm
        isPmdConfigRead = (pepDemoConf.pmdrv == 1U) ? loadPMDCamConfiguration(PMDcfg.rearPmdCamera, jsonPMDFileConfigName) :
            resetPMDCamConfigurationTo0(PMDcfg.rearPmdCamera);

        if (!isPmdConfigRead) {
            return logWarningPmdDefaultValuesUsed();
        }

        isPmdConfigRead = (pepDemoConf.pmdmr == 1U) ? loadPMDCamConfiguration(PMDcfg.rightPmdCamera, jsonPMDFileConfigName) : 
            resetPMDCamConfigurationTo0(PMDcfg.rightPmdCamera);

        //do for the all cases to cover the case when the previous cameras are not available for pmd algoritm
        if (!isPmdConfigRead) {
            return logWarningPmdDefaultValuesUsed();
        }

        return isPmdConfigRead;
    }
    else
    {
        //no CarMaker Warning logs necessary- it's print from loadPepDemoConfiguration
        return isPepDemoConfOk;
    }
}

#pragma warning( pop )