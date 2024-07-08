#====================================================================
# System Imports
#====================================================================
import os
import sys
#import math
#import shutil
#from itertools import product
#import csv
#import numpy as np
#import scipy.interpolate
#import string

#====================================================================
# Own Imports
#====================================================================
#DF
sys.path.append( os.path.join(os.path.dirname(__file__), '..', '..', '..', 'contrib', 'VSP_pyBase', 'pycmEval'))
from cm_testrun import cmTestRun

EVALPATH = os.path.dirname(__file__)
PATH_selectedVariant       = os.path.abspath(os.path.join(EVALPATH,'..', '..','..', 'scripts', 'selected_variant.txt'))

# Check, if selected_variant.txt file exists. In case file is not available, use base.
if os.path.isfile(PATH_selectedVariant):
    variant = open(PATH_selectedVariant,'r')
    selectedVariant = variant.readline().strip()
    variant.close()
else:
    selectedVariant = "base"

NO_ERROR = 0
TEST_SCENARIO_ERROR = 10

#====================================================================
# Parameter
#====================================================================
TestCaseFolders                 = ['01_Perpendicular_Left', '02_Perpendicular_Right', '03_Parallel_Left', '04_Parallel_Right']
TestCaseFolder                  = os.path.join(os.path.dirname(__file__),'..', 'CarMaker', 'Data', 'TestRun', 'AP', '04_Generated')
if selectedVariant == 'entry':
    TestCaseFolders_Regression  = ['ScanOnSI_Base',
                                   'US_Scenarios_Base',
                                   'CUS_Regression_Vehicle',
                                   #'2021_G2_03_02_PillarRoadSideCorner',
                                   '2021_G2_03_02_SingleVehicleAndCurb',
                                   '2021_G2_03_04_MisalingmentOrientation',
                                   '2021_G2_03_04_MisalingmentScanning',
                                   '2021_G2_03_04_MisalingmentShiftOnRoad'
                                   ]   # not active: 'DynamicReplanning'
    TestCaseFolders_SmallRegres = ['SmallRegression_Base','SmallRegression_Cus',
                                   #'MoCo_Functional_Testing'
                                  ]
elif selectedVariant == 'performance':
    TestCaseFolders_Regression  = [#'ScanOn_Prem',           NOT active because covered by according tests with SI; will be activated for full regression tests
                                   'ScanOnSI_Base',
                                   'ScanOnSI_Prem',
                                   'US_Scenarios_Base',
                                   'US_Scenarios_Prem',
                                   '2021_G2_03_02_PillarRoadSideCorner',
                                   '2021_G2_03_02_SingleVehicleAndCurb',
                                   '2021_G2_03_04_MisalingmentOrientation',
                                   '2021_G2_03_04_MisalingmentScanning',
                                   '2021_G2_03_04_MisalingmentShiftOnRoad',
                                   '2021_G2_03_02_PartlyDetectedLines',
                                   '2021_G2_07_AngledParkingMarkersOnly',
                                   'AngledParking_Prem',
                                   'AngledParking_SI_Prem',
                                   #'Wheelstopper_Perf',      NOT active because temp. wheelstopper interface line workaround not supported by SISurrogate 
                                   'Wheelstopper_SI_Perf',
                                   #'SI_AngledParkingWithRoadsideLimiter',  TODO: In-dapth analysis why AUPSim_UC_AngRight_ST-0803_F fails
                                   'SI_HorizontalLinesRoadsideCurbside',
                                   'SI_PillarCurbSideCorner',
                                   'SI_SmallObject',
                                   'FirstStrokeFwd',
                                   ]   # not active: 'DynamicReplanning'
    TestCaseFolders_SmallRegres = ['SmallRegression_Base','SmallRegression_Prem','SmallRegression_Perf',
                                   #'MoCo_Functional_Testing'
                                  ]
else:
    TestCaseFolders_Regression  = [#'ScanOn_Prem',          NOT active because covered by according tests with SI; will be activated for full regression tests
                                   'ScanOnSI_Base',
                                   'ScanOnSI_Prem',
                                   'US_Scenarios_Base',
                                   'US_Scenarios_Prem',
                                   '2021_G2_03_02_PillarRoadSideCorner',
                                   '2021_G2_03_02_SingleVehicleAndCurb',
                                   '2021_G2_03_04_MisalingmentOrientation',
                                   '2021_G2_03_04_MisalingmentScanning',
                                   '2021_G2_03_04_MisalingmentShiftOnRoad',
                                   '2021_G2_03_02_PartlyDetectedLines',
                                   '2021_G2_07_AngledParkingMarkersOnly',
                                   'AngledParking_Prem',
                                   'AngledParking_SI_Prem',
                                   'FirstStrokeFwd'
                                   ]   # not active: 'DynamicReplanning', #'MemoryParking'
    TestCaseFolders_SmallRegres =   ['SmallRegression_Base','SmallRegression_Prem',
                                   #'MoCo_Functional_Testing'
                                  ] # ['SmallRegression_MemPark','SmallRegression_Base','SmallRegression_Prem']
# MemoryParking
TestCaseFolders_MP_SmallRegres = ['SmallRegression_MemPark']


TestCaseFolder_Regression       = os.path.join(os.path.dirname(__file__),'..', 'CarMaker', 'Data', 'TestRun', 'AP', '06_Regression')
TestCaseFolderTCE               = os.path.join(os.path.dirname(__file__),'..', 'CarMaker', 'Data', 'TestRun', 'TCE')
TestCaseFolderRequirementTest   = os.path.join(os.path.dirname(__file__),'..', 'CarMaker', 'Data', 'TestRun', 'AP', '08_RequirementTest')
#TestCaseFoldersLSCA             = ['BrakingStatic', 'SteeringProposal', 'VirtualWall']
TestCaseFolderLSCA              = os.path.join(os.path.dirname(__file__),'..', 'CarMaker', 'Data', 'TestRun', 'LSCA', '01_Generated')
TestCaseFolderSI                = os.path.join(os.path.dirname(__file__),'..', 'CarMaker', 'Data', 'TestRun', 'SI', '01_Generated')
TestCaseFolderSI_Regression     = os.path.join(os.path.dirname(__file__),'..', 'CarMaker', 'Data', 'TestRun', 'SI', '02_Regression')
#====================================================================
# Local Functions
#====================================================================

#====================================================================
# TestRun Factories
#====================================================================

def GetTestCases(testCaseSet, CMPrjDir):
    errorCode = NO_ERROR
    TestCases = []
    if testCaseSet == "AP_AutoTestExecution":
        for folder in TestCaseFolders:
            TestCases = TestCases + prepareListOfTests(TestCaseFolder, folder)
    elif testCaseSet == "TCE_AutoTestExecution":
        TestCases = prepareListOfTests(TestCaseFolderTCE)
    elif testCaseSet == "AP_RequirementTest":
        TestCases = prepareListOfTests(TestCaseFolderRequirementTest)

    elif testCaseSet == "LSCA_AutoTestExecution":
        TestCases = TestCases + prepareListOfTests(TestCaseFolderLSCA)
        # for folder in TestCaseFoldersLSCA:
            # TestCases = TestCases + prepareListOfTests(TestCaseFoldersLSCA, folder)
    elif testCaseSet == "AP_RegressionTests":
        for folder in TestCaseFolders_Regression:
            TestCases = TestCases + prepareListOfTests(TestCaseFolder_Regression, folder)
    elif testCaseSet == "AP_SmallRegressionTests":
        for folder in TestCaseFolders_SmallRegres:
            TestCases = TestCases + prepareListOfTests(TestCaseFolder_Regression, folder)
    elif testCaseSet == "SI_AutoTestExecution":
        for folder in TestCaseFolders:
            TestCases = TestCases + prepareListOfTests(TestCaseFolderSI, folder)
    elif testCaseSet == "SI_RegressionTests":
        TestCases = TestCases + prepareListOfTests(TestCaseFolderSI_Regression)
    elif testCaseSet == "MP_SmallRegressionTests":
        for folder in TestCaseFolders_MP_SmallRegres:
            TestCases = TestCases + prepareListOfTests(TestCaseFolder_Regression, folder)
    else:
   
        errorCode = pow(2,TEST_SCENARIO_ERROR) #set error bit with number TEST_SCENARIO_ERROR
        raise SystemError("TestCaseSet {0} unknown".format(testCaseSet))
       
    return (TestCases, errorCode)
    
def prepareListOfTests (testCaseFolder, folder = ''):
    testCases = []
    for filename in os.listdir(os.path.join(testCaseFolder, folder)):
        filePath = os.path.join(testCaseFolder, folder) + "\\" + filename
        if os.path.splitext(filename)[-1] == '' or 'testrun' in os.path.splitext(filename)[-1]:
            testCases.append({"caseType": "useCase", "cmTestRun": cmTestRun(filename, filePath)})
    return testCases

def find_all_files(path):
    """Search for all mf sil txts in the Reports path"""
    testCases = []

    # Use os.walk to go into subfolders
    for folder, subfolders, files in os.walk(path):
        if folder != path:
            for file in files:
                if os.path.splitext(file)[-1] == '' or 'testrun' in os.path.splitext(file)[-1]:
                    testCases.append(os.path.join(folder, file))

    return testCases
