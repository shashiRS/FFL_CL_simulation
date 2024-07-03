#include<pmd_config.h>
#include <json/json.h>
#include<fstream>

class PmdConfigReader{
public:
    // Fill the given configuration structure with information loaded from file (PMD_ALGORITHM)cfg
    bool loadPepDemoConfiguration(pmd::PepDemoConf& cfg, char const * const jsonFileName);

    // Fill the given configuration structure with information loaded from file (PMD_ALGORITHM)cfg
    bool loadPMDCamConfiguration(pmd::PmdCameraConfig& cfg, char const * const jsonFileName);

    // Fill the given configuration structure with information loaded from file (PMD_ALGORITHM)cfg
    bool resetPMDCamConfigurationTo0(pmd::PmdCameraConfig& cfg);

    //Fill the given configuration structure with default value
    void setDefaultValuesPMDConfiguration(pmd::PmdConfig& cfg);

    bool loadPmdAlgorithmConfig(pmd::PmdConfig& PMDcfg, char const * const jsonPMDFileConfigName, char const * const jsonPepDemoConfigFileName);
};