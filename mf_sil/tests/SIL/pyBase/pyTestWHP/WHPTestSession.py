#********************************************************************
# System Imports
#*******************************************************************
import sys
import os
import warnings
import logging
logging.getLogger("STET").setLevel(logging.ERROR)
logging.getLogger("evaluate").setLevel(logging.ERROR)
from collections import namedtuple, defaultdict
from itertools import product
from time import sleep, time

#********************************************************************
# Own Imports
#*******************************************************************
from pyBase_path_cfg import *
from constants import *
import openpyxl, jdcal
import WHPExcelReader, WHPTestCase

from STET import run_evaluation
from cmdata_testrun import cCMTestRun
from cmdata_road import cCMStraight, cCMTurnRight, cCMTurnLeft, cCMRoadAttributes
from cmdata_sensors import cCMSSASensors, cCMBdySensors, cCMDASensors, cCMRPSensors
from cmdata_drivman import cCMDrivManList
from cm_testrun import cmTestRun
from cm_eval_runner import cCMEvalTestRunner

# The permitted road categories
cmTestRunCateg = [
    "Perpendicular"
    ]

# Flags for performing different stages of the test session
doRegen = True
doRun = False
doEval = False

def runCarMakerScenario(situationID, testFileName, testRunPath):

    testRunsBase = [cmTestRun(testFileName, testRunPath)]
    isRunInJenkinsCI = os.getenv('IPGHOME', "NONE").startswith(r"C:\cip_tools")
    testRunner   = cCMEvalTestRunner(CM_PRJ_PATH, WHP_QUANTITIES, numberOfCarMakerInstances = 1, sampleRate = '10ms')
    ergFilePaths = testRunner.runEvaluations(TEST_CASE_SET_NAME, testRunsBase, maxTimeInSimulationSeconds=MAX_SIMULATION_TIME_S, headlessMode=True,
                                             useCoreLicense=isRunInJenkinsCI, returnErgPaths=True, maxExecTimeSeconds=MAX_EXECUTION_TIME_S, nRetries=240, retryTimeSeconds=30)
    #if testRunner.getErrorCode():
    #    print("Error code occured while executing test scenario {}".format(situationID))

def performTestSession():

    xlsReader = WHPExcelReader.WHPExcelReader()
    firstRowToRead = xlsReader.firstTestRow
    lastRowToRead = xlsReader.lastTestRow 

    # measurements - dictionary with key - measurement name, value - path to measurement
    measurements = defaultdict(list)
    #evaluations - dict with key - measurement name, value - evaluation for the measurement
    evaluations = defaultdict(list)

    for row in range(firstRowToRead, lastRowToRead):
    #for row in range(2, 3):

        # check if current row contains a valid test situation to be generated
        if not xlsReader.existsScenario(row): # the row does not contain a test situation
            continue
        if not xlsReader.toBeGenerated(row): # test specified not to be generated
            continue
        scenario = xlsReader.extractScenario(row)
        
        cat = scenario["Category"]
        if not cat in cmTestRunCateg: # unknown test category
            print("Test scenario {} not generated: unknown category {}".format(scenario["Situation ID"], cat))
            continue

        testFolderName = WHPGENERATED_TEMP_PATH
        testFileName = "WHPSim_TestRun_{}".format(scenario["Situation ID"])
        testRunPath = os.path.join(testFolderName, testFileName)

        if doRegen:
            # generate CarMaker test scenario
            TCaseWHP = WHPTestCase.WHPTestCase(scenario, xlsReader)
            WHPTestRun = cCMTestRun(name=testFileName)
            TCaseWHP.WHPTestRunFactory(WHPTestRun)
            WHPTestRun.save(testFolderName)

        if doRun:
            # run CarMaker scenario
            print("Run CarMaker scenario {}".format(scenario["Situation ID"]))
            runCarMakerScenario(scenario["Situation ID"], testFileName, testRunPath)

        if doEval:
            #check if erg file exists
            ergFile = os.path.abspath(os.path.join(ERGOUTPUT_PATH, testFileName+MEAS_EXTENTION))
            if os.path.isfile(ergFile):
                if scenario["Evaluation"] != "None":
                    # prepare data to run evaluations 
                    measurements[testFileName].append(ergFile)
                    evaluations[testFileName] = ('WHP\n\nTest Case\n\n\n\n\n\n'+scenario["Evaluation"]+'\n\n', [])
                else:
                    print("Test {} has no evaluation criteria. Please check test catalogue.".format(scenario["Situation ID"]))

    if doRun:
        # close CarMaker
        cCMEvalTestRunner._killCarMakerProcesses()

    if doEval:
        # run evaluations using STET
        try:
            result = run_evaluation(measurements = measurements,
                                    testcases = evaluations,
                                    work_dir = TESTREPORT_PATH,
                                    report_title = TEST_CASE_SET_NAME,
                                    display_log = True,
                                    pictures = False)
        except Exception as e:
            print("Exception occurred during evaluation = {}".format(e))

def calcExecTime(startTime):
    execTime = time() - startTime
    if execTime > 3600:
        hours = int(execTime // 3600)
        seconds = execTime % 3600
        minutes = int(seconds // 60)
        seconds = int(seconds % 60)
        showExecTime = "{}h{}min{}s".format(hours, minutes, seconds)
    elif execTime > 60:
        minutes = int(execTime // 60)
        seconds = int(execTime % 60)
        showExecTime = "{}min{}s".format(minutes, seconds)
    else:
        showExecTime = "{:.2f}s".format(execTime)
    return showExecTime

if __name__ == "__main__":
    try:  
        # Parse arguments. Available options:
        #   -regen: only regenerate test scenarios, no execution, no evaluation
        #   -eval: only evaluate last test scenarios
        #   -full: regenerate, execute and evaluate
        doRegen = False
        doRun = False
        doEval = False
        args = sys.argv
        if len(args) == 2:
            startTime = time()
            if args[1] == "-eval":
                doEval = True
            elif args[1] == "-regen":
                doRegen = True
                print("Start test generation")
                print("WHP test catalogue:           {}".format(TESTCATALOGUE_PATH))
                print("Generated CarMaker scenarios: {}".format(WHPGENERATED_PATH))
            elif args[1] == "-full":
                doRegen = True
                doRun = True
                doEval = True
                print("Start test generation")
                print("WHP test catalogue:           {}".format(TESTCATALOGUE_PATH))
                print("Generated CarMaker scenarios: {}".format(WHPGENERATED_PATH))
            else:
                print("Unknown argument for WHPTestSession.py")
                sys.exit(0)
            performTestSession()
            print("All done. Execution time: {}\n".format(calcExecTime(startTime)))
        else:
            print("Unexpected number of arguments for WHPTestSession.py")

    except Exception as e:
        print("Exception = {}".format(e))
        input("Exception in executing WHPTestSession.py.\nHit enter To Close...")
