
#********************************************************************
# System Imports
#********************************************************************
import os
import sys
from math import *
from constants import *

#********************************************************************
# Own Imports
#********************************************************************
PROJECT_PATH = os.path.dirname(__file__)
# LOCAL_IMS_SANDBOX = os.path.abspath(os.path.join(PROJECT_PATH,'..','..','..','..','..','..','IMS','AUP_100')) #Adjust accordingly
LOCAL_IMS_SANDBOX = os.path.abspath(os.path.join(PROJECT_PATH,'..','..','..','..','..','..','Sandbox_AUP')) #Adjust accordingly AlBu
IMS_PATH = os.path.abspath(os.path.join(LOCAL_IMS_SANDBOX,'02_System','04_System_Design','02_System_Requirements','03_Use_Cases'))
PDW_TEST_CATALOGUE_NAME  = "Scenario_Catalogue_PDW.xlsm"
TESTCATALOGUE_PATH = os.path.abspath(os.path.join(IMS_PATH, PDW_TEST_CATALOGUE_NAME))
TESTREPORT_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..', '..', 'Test_Results', 'Reports_DWF', 'PDW'))
CM_PRJ_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..', '..', 'CarMaker'))
ERGOUTPUT_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'SimOutput', TEST_CASE_SET_NAME))
PDWGENERATED_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'Data', 'TestRun', 'DWF', 'PDW', '02_Generated'))
PDWGENERATED_TEMP_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'Data', 'TestRun', 'DWF', 'PDW', 'temp'))
PDWFACTORYGENERATED_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'Data', 'TestRun', 'DWF', 'PDW', '03_Generated_Factory'))
PDWTESTRUN_SUBFOLDER = ""
ext_SRC_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..'))
src_pyCMData_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..', 'pycmData')) # legacy pyBase for TestRun generation
src_pyCMBaseScenarios_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..', 'cmBaseScenarios', 'PDW'))
# new VSP_pyBase for test execution
src_pyCMCommon_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..', '..', '..', '..', 'contrib', 'VSP_pyBase', 'pycmCommon'))
src_pyCMEval_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..', '..', '..', '..', 'contrib', 'VSP_pyBase', 'pycmEval'))
src_pyUtils_PATH = os.path.abspath(os.path.join(PROJECT_PATH, '..', '..', '..', '..', 'contrib', 'VSP_pyBase', 'pyUtils'))

sys.path.append(PROJECT_PATH)
sys.path.append(ext_SRC_PATH)
sys.path.append(src_pyCMData_PATH)
sys.path.append(src_pyCMCommon_PATH)
sys.path.append(src_pyCMEval_PATH)
sys.path.append(src_pyUtils_PATH)

# Define list of json files to be parsed in order to read the values of the parameters used in the description of tests in the catalogue
PARAM_FILES_LIST = [
    os.path.abspath(os.path.join(PROJECT_PATH,'..','..','..','..','..','appdemo_drvwarnsm','src','platform','data','FC_DrvWarnSM_Params_config.json')),
    os.path.abspath(os.path.join(PROJECT_PATH,'..','..','..','..','..','mf_pdwarnproc','src','platform','data','FC_PDCP_Params_config.json')),
    os.path.abspath(os.path.join(PROJECT_PATH,'..','..','..','..','..','mf_drvwarnsm_core','src','platform','data','FC_DrvWarnSMCore_Params_config.json')),
    os.path.abspath(os.path.join(PROJECT_PATH,'..','..','..','..','..','mf_common','src','platform','data','AP_Sim','Vehicle_Params_config.json'))
    ]
