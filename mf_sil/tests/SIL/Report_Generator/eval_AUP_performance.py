#====================================================================
# System Imports
#====================================================================
import os
import sys
#import math
#import copy
import logging

# workaround: avoid that using STET will disable all existing loggers
@property
def disabled(self):
    return self._disabled

@disabled.setter
def disabled(self, disabled):
    self._disabled = False

logging.Logger.disabled = disabled

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s [%(name)s] %(message)s', datefmt='%d-%m-%Y %H:%M:%S')
logger = logging.getLogger("eval_AUP_performance")

from datetime import datetime
from optparse import OptionParser
#import numpy as np
#import xml.etree.ElementTree as ET
#====================================================================
# Simulation Imports
#====================================================================
PYBASE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'contrib', 'VSP_pyBase')
sys.path.append( os.path.join(PYBASE_PATH, 'pycmEval'))
sys.path.append( os.path.join(PYBASE_PATH, 'pyHTMLReport') )
sys.path.append( os.path.join(PYBASE_PATH, 'pyUtils') )

from cm_eval_runner         import cCMEvalTestRunner
from cm_testrun             import cmTestRun
from cm_eval_erg_parser     import cErgFile
from cm_eval_erg_calculator import cErgCalculator
import eval_AUP_testcases

EVALPATH = os.path.dirname(os.path.abspath(__file__))
PATH_AUP_import_signal_list = os.path.abspath(os.path.join(EVALPATH, "ImportList.txt"))
PATH_LSCA_import_signal_list = os.path.abspath(os.path.join(EVALPATH, "ImportList_LSCA.txt"))
PATH_SI_import_signal_list  = os.path.abspath(os.path.join(EVALPATH, "ImportList_SI.txt"))

#====================================================================
#GLOBAL VARIABLES
#====================================================================
testCaseSetNameDefault        = "AP_AutoTestExecution"
testCaseSetRegressionDef      = "AP_RegressionTests"
testCaseSetSmallRegressionDef = "AP_SmallRegressionTests"
testCaseSetNameDefaultReq     = "AP_RequirementTest"         
testCaseSetNameDefaultTCE     = "TCE_AutoTestExecution"
maxExecutionTime_AUP          = 420
maxExecutionTime_TCE          = 9000
maxSimulationTime_AUP         = 250


#====================================================================
# Parameters
#====================================================================
'''Function to read all signals (CM Quantities) from a txt file and return the list with the signals'''
def importSignalList(PATH_import_signal_list):
    with open(PATH_import_signal_list) as import_list:
        '''signals inside import_list can be found: <SW_signal_name> [architecture_name]
        <SW_signal_name> - signal as it is found in the SW
        [architecture_name] - name which should be used in the test specification; optional
        key words of the dictionary structures are searched inside [architecture_name]'''
        '''Read all signals from txt file and save the signals in Quantities_Import list'''
        Quantities_Import = []
        '''structures - dictionary with keyword: structure name in the import list. The signals inside needs to be multiplied
                             value: maximum index number '''
        structures = {"envModelPort":8, "TRJPLA":10}
        for line in import_list:
            if len(line) > 0:
                '''if it is not an empty line, add the <SW_signal_name> in the list'''
                Quantities_Import.append(line.rstrip("\n").split()[0])
                ''' if [architecture_name] exists, use the dictionary to search if the signals should be multiplied'''
                if len(line.split())>1:
                    for struct in structures.keys():
                        if struct in line.split()[1]:
                            for i in range(1,structures[struct]):
                                updated_line = line.rstrip("\n").split()[0].replace("_0.","_"+str(i)+".",1)
                                Quantities_Import.append(updated_line)
    return Quantities_Import

TCEQuantities = [
"AP.odoDebugPort.distanceRA_m",
"AP.odoDebugPort.distanceFA_m",
"AP.tireCircEstimationPort.estimTyreCircFl_m",
"AP.tireCircEstimationPort.estimTyreCircFr_m",
"AP.tireCircEstimationPort.estimTyreCircRl_m",
"AP.tireCircEstimationPort.estimTyreCircRr_m",
"Vhcl.Distance",
"Vhcl.v",
"Time",
"AP.tireCircEstimationPort.estimTyreCircVarFl_m2",
"AP.tireCircEstimationPort.estimTyreCircVarFr_m2",
"AP.tireCircEstimationPort.estimTyreCircVarRl_m2",
"AP.tireCircEstimationPort.estimTyreCircVarRr_m2",
"AP.tceDebugPort.circEst_FL_m",
"AP.tceDebugPort.circEst_FR_m",
"AP.tceDebugPort.circEst_RL_m",
"AP.tceDebugPort.circEst_RR_m",
"AP.tireCircEstimationPort.cmCirc_FL_m",
"AP.tireCircEstimationPort.cmCirc_FR_m",
"AP.tireCircEstimationPort.cmCirc_RL_m",
"AP.tireCircEstimationPort.cmCirc_RR_m",
"AP.tceDebugPort.sampleVeloExtRef_kph",
"AP.tceDebugPort.sampleVeloWhl_FL_kph",
"AP.tceDebugPort.sampleVeloWhl_FR_kph",
"AP.tceDebugPort.sampleVeloWhl_RL_kph",
"AP.tceDebugPort.sampleVeloWhl_RR_kph",
"AP.tceDebugPort.sampleReject_FL_nu",
"AP.tceDebugPort.sampleReject_FR_nu",
"AP.tceDebugPort.sampleReject_RL_nu",
"AP.tceDebugPort.sampleReject_RR_nu"
]

CMPrjDir                   = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..','CarMaker')

def change_CM_config_line_endings():
    '''
    PI-2402: change the line endings of OutputQuantities and SimParameters from windows
    endings to Linux endings
    '''
    WINDOWS_LINE_ENDING = b'\r\n'
    UNIX_LINE_ENDING = b'\n'
    files = [os.path.join(CMPrjDir, "Data", "Config", "OutputQuantities"),
             os.path.join(CMPrjDir, "Data", "Config", "SimParameters")]
    for file_path in files:
        with open(file_path, 'rb') as open_file:
            content = open_file.read()
            content = content.replace(WINDOWS_LINE_ENDING, UNIX_LINE_ENDING)
        with open(file_path, 'wb') as open_file:
            open_file.write(content)

def CalcPerformance(testCaseSetName):
    if "AP" in testCaseSetName:
        os.environ["CARMAKER_GENERATE_JSON"] = "1"
        os.environ["CARMAKER_GENERATE_JSON_ROOT"] = os.path.join(CMPrjDir, "SimOutput")
        Quantities =  importSignalList(PATH_AUP_import_signal_list)
        maxExecTime = maxExecutionTime_AUP
        maxTimeInSimulationSeconds = maxSimulationTime_AUP
    elif "TCE" in testCaseSetName:
        Quantities = TCEQuantities
        maxTimeInSimulationSeconds = None
        maxExecTime = maxExecutionTime_TCE
    elif "LSCA" in testCaseSetName:
        Quantities = importSignalList(PATH_LSCA_import_signal_list)
        maxTimeInSimulationSeconds = None
        maxExecTime = maxExecutionTime_AUP
    elif "SI" in testCaseSetName:
        Quantities = importSignalList(PATH_SI_import_signal_list)
        maxTimeInSimulationSeconds = None
        maxExecTime = maxExecutionTime_AUP
    elif "MP" in testCaseSetName:
        Quantities = importSignalList(PATH_AUP_import_signal_list)
        maxTimeInSimulationSeconds = None
        maxExecTime = maxExecutionTime_AUP

    logger.info("Generating Activation Performance Report")

    # Retrieve all TestCases and TestRuns
    (testCasesBase, errorCodeScenarios) = eval_AUP_testcases.GetTestCases(testCaseSetName, CMPrjDir)
    testRunsBase       = [currTestCase["cmTestRun"] for currTestCase in testCasesBase]
    # Run all TestRuns
    isRunInJenkinsCI = os.getenv('IPGHOME', "NONE").startswith(r"C:\cip_tools")
    testRunner         = cCMEvalTestRunner(CMPrjDir, Quantities, numberOfCarMakerInstances = 4, sampleRate = '10ms')
    ergFilePaths = testRunner.runEvaluations(testCaseSetName, testRunsBase, maxTimeInSimulationSeconds=maxTimeInSimulationSeconds,
                                             headlessMode=True, useCoreLicense=isRunInJenkinsCI, returnErgPaths=True,
                                             maxExecTimeSeconds=maxExecTime, nRetries=240, retryTimeSeconds=30)
    change_CM_config_line_endings()
    return errorCodeScenarios

if __name__ == "__main__":
    ### Parse Command Line Options
    optparser = OptionParser(usage="usage: %prog [options]")
    optparser.add_option("-c", "--cases", dest='testCaseSetName', default=testCaseSetNameDefault, help="TestCase-Set. For possible values look at testcase file")
    args      = optparser.parse_args()[0]
    ### Calculate TestCase Performance
    errorCode = CalcPerformance(args.testCaseSetName)
    #temporary solution to to upload artifacts if the tests are failed.
    #exit(errorCode)
