
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
pyTestWHP_PATH = os.path.dirname(__file__)
LOCAL_IMS_SANDBOX = os.path.abspath(os.path.join(pyTestWHP_PATH,'..','..','..','..','..','..','IMS','AUP_100')) #Adjust accordingly
#LOCAL_IMS_SANDBOX = os.path.abspath(os.path.join(pyTestWHP_PATH,'..','..','..','..','..','..','Sandbox_AUP')) #Adjust accordingly AlBu
IMS_PATH = os.path.abspath(os.path.join(LOCAL_IMS_SANDBOX,'02_System','04_System_Design','02_System_Requirements','03_Use_Cases'))
WHP_TEST_CATALOGUE_NAME  = "Scenario_Catalogue_WHP.xlsm"
TESTCATALOGUE_PATH = os.path.abspath(os.path.join(IMS_PATH, WHP_TEST_CATALOGUE_NAME))
TESTREPORT_PATH = os.path.abspath(os.path.join(pyTestWHP_PATH, '..', '..', 'Test_Results', 'Reports_DWF', 'WHP'))
CM_PRJ_PATH = os.path.abspath(os.path.join(pyTestWHP_PATH, '..', '..', 'CarMaker'))
ERGOUTPUT_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'SimOutput', TEST_CASE_SET_NAME))
WHPGENERATED_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'Data', 'TestRun', 'DWF', 'WHP', '02_Generated'))
WHPGENERATED_TEMP_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'Data', 'TestRun', 'DWF', 'WHP', 'temp'))
ext_SRC_PATH = os.path.abspath(os.path.join(pyTestWHP_PATH, '..'))
src_pyCMData_PATH = os.path.abspath(os.path.join(pyTestWHP_PATH, '..', 'pycmData')) # legacy pyBase for TestRun generation
# new VSP_pyBase for test execution
src_pyCMCommon_PATH = os.path.abspath(os.path.join(pyTestWHP_PATH, '..', '..', '..', '..', 'contrib', 'VSP_pyBase', 'pycmCommon'))
src_pyCMEval_PATH = os.path.abspath(os.path.join(pyTestWHP_PATH, '..', '..', '..', '..', 'contrib', 'VSP_pyBase', 'pycmEval'))
src_pyUtils_PATH = os.path.abspath(os.path.join(pyTestWHP_PATH, '..', '..', '..', '..', 'contrib', 'VSP_pyBase', 'pyUtils'))

sys.path.append(pyTestWHP_PATH)
sys.path.append(ext_SRC_PATH)
sys.path.append(src_pyCMData_PATH)
sys.path.append(src_pyCMCommon_PATH)
sys.path.append(src_pyCMEval_PATH)
sys.path.append(src_pyUtils_PATH)

# Define list of json files to be parsed in order to read the values of the parameters used in the description of tests in the catalogue
PARAM_FILES_LIST = [
    os.path.abspath(os.path.join(pyTestWHP_PATH,'..','..','..','..','..','appdemo_drvwarnsm','src','platform','data','FC_DrvWarnSM_Params_config.json')),
    os.path.abspath(os.path.join(pyTestWHP_PATH,'..','..','..','..','..','mf_pdwarnproc','src','platform','data','FC_PDCP_Params_config.json')),
    os.path.abspath(os.path.join(pyTestWHP_PATH,'..','..','..','..','..','mf_drvwarnsm_core','src','platform','data','FC_DrvWarnSMCore_Params_config.json')),
    os.path.abspath(os.path.join(pyTestWHP_PATH,'..','..','..','..','..','mf_common','src','platform','data','AP_Sim','Vehicle_Params_config.json'))
    ]
