
#********************************************************************
# System Imports
#********************************************************************
import os
import sys
from math import *



#********************************************************************
# Own Imports
#********************************************************************
AP_PROJECT_PATH = os.path.dirname(__file__)
LOCAL_IMS_SANDBOX = os.path.abspath(os.path.join(AP_PROJECT_PATH,'..','..','..','..','..','..','Sandbox_AUP')) #Adjust accordingly
IMS_PATH = os.path.abspath(os.path.join(LOCAL_IMS_SANDBOX,'02_System','04_System_Design','02_System_Requirements','03_Use_Cases'))
CM_PRJ_PATH = os.path.abspath(os.path.join(AP_PROJECT_PATH, '..', '..', 'CarMaker'))
APGENERATED_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'Data', 'TestRun', 'AP', '04_Generated'))
APGENERATED_TEMP_PATH = os.path.abspath(os.path.join(CM_PRJ_PATH, 'Data', 'TestRun', 'AP', 'temp'))
src_pyCMData_PATH = os.path.abspath(os.path.join(AP_PROJECT_PATH, '..', 'pycmData'))
src_pyCMBaseScenarios_PATH = os.path.abspath(os.path.join(AP_PROJECT_PATH, '..', 'cmBaseScenarios'))

sys.path.append(AP_PROJECT_PATH)
sys.path.append(src_pyCMData_PATH)
sys.path.append(src_pyCMBaseScenarios_PATH)
