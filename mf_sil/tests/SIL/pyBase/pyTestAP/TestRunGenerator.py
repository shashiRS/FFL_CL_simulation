#********************************************************************
# System Imports
#*******************************************************************
import sys
import os
import warnings
from collections import namedtuple
from itertools import product


#********************************************************************
# Own Imports
#*******************************************************************
from pyBase_path_cfg import *
import openpyxl, jdcal
import AP_ExcelReader, ParkingTestCase, TCETestCase
from cmTestRunSimple import cmTestRunSimple
from cmdata_testrun import cCMTestRun
from cmdata_road import cCMStraight, cCMTurnRight, cCMTurnLeft, cCMRoadAttributes
from cmdata_sensors import cCMSSASensors, cCMBdySensors, cCMDASensors, cCMRPSensors
from cmdata_drivman import cCMDrivManList

# Base scenario
baseScenarioPathTCE     = os.path.join(src_pyCMBaseScenarios_PATH, 'TCEScenario')
baseScenarioPathTCECity = os.path.join(src_pyCMBaseScenarios_PATH, 'TCEScenarioCity')
# Generation folder
testRunFolderTCE    = "07_TCE"


#********************************************************************
# Configuration tables
#********************************************************************
# cmTestRunConfigType is named tuple containing for each limiter type:
# testRunName = prefix for name of the CarMaker scenario
# testRunFolder = name for CarMaker scenario folder
cmTestRunConfigType = namedtuple("cmTestRunConfigType", "testRunName testRunFolder")

cmTestRunConfig = {                      #        testRunName                    testRunFolder               
    "Perpendicular-Right"   : cmTestRunConfigType("_PerpRight_",   "02_Perpendicular_Right"),
    "Perpendicular-Left"    : cmTestRunConfigType("_PerpLeft_",    "01_Perpendicular_Left" ),
    "Parallel-Right"        : cmTestRunConfigType("_ParRight_",    "04_Parallel_Right"     ),
    "Parallel-Left"         : cmTestRunConfigType("_ParLeft_",     "03_Parallel_Left"      ),
    "Angled-Right"          : cmTestRunConfigType("_AngRight_",    "06_Angled_Right"       ),
    "Angled-Left"           : cmTestRunConfigType("_AngLeft_",     "05_Angled_Left"        )
    }


#********************************************************************
# Main Program
#********************************************************************
def GenerateCMTestRun():
    xlsReader = AP_ExcelReader.AP_ExcelReader()
    firstRowToRead = xlsReader.firstTestRow
    lastRowToRead = xlsReader.lastTestRow 

    print("Generating CM testRuns ...")
    
    for row in range(firstRowToRead, lastRowToRead):
        if xlsReader.existsScenario(row) == True: # the row contains a test situation
            if xlsReader.toBeGenerated(row) == True:
                scenarioIdx = 0 # scenario variation index
                scenarios = xlsReader.extractScenarios(row)
                
                for scenario in scenarios:
                    scenarioIdx += 1
                    if len(scenarios) <= 1:
                        scenarioSuffix = ""
                    else:
                        scenarioSuffix = "_" + str(scenarioIdx).rjust(2, '0') # rjust(2, '0') forces the index to be a 2 digit number <=> '{:0>2}'.format(str(scenarioIdx))
                       
                    cat = scenario["Category"]
                    if cat in list(cmTestRunConfig.keys()):
                        parkingDirection = scenario["Maneuver"].lower().replace(' ', '') # change to lowercase and remove blanks from cell
                        maneuverList = parkingDirection.split("&") # extract enumeration
                        
                        for maneuver in maneuverList:
                            if scenario["Non-/Use Case"] == "Use Case":
                                useCasePrefix = "UC"
                            else:
                                useCasePrefix = "NUC"
                        
                            scenario["Maneuver"] = maneuver
                            maneuverSuffix = "_" + maneuver[0].upper() # use the first letter to indicate parking direction
                                
                            # TCaseAP = ParkingTestCase.AP_ParkingTestCase(scenario, xlsReader.limHeight)
                            TCaseAP = ParkingTestCase.AP_ParkingTestCase(scenario, xlsReader)
                            APTestRun = cCMTestRun(name="{}{}{}{}{}{}".format("AUPSim_", useCasePrefix, cmTestRunConfig[cat].testRunName, scenario["Situation ID"], maneuverSuffix, scenarioSuffix))
                            # APTestRun.Road.Definition.DigRoad = cmTestRunConfig[cat].roadName
                            TCaseAP.AP_TestRunFactory(APTestRun)
                            APTestRun.save(os.path.join(APGENERATED_TEMP_PATH, cmTestRunConfig[cat].testRunFolder))
                        
def GenerateCMTestRun_TCE():
    print("Generating CM testRuns for TCE...")
   
    # Ego vehicle
    egoVelocityList = [10, 30, 50, 75, 100, 125, 150, 200] # km/h TODO: fill this array with pertinent values
    roadRadiusList = [-200, 0, 200] # m TODO: fill this array with pertinent values
    roadGradientList = [-5, 0, 5] # % - longitudinal slope TODO: fill this array with pertinent values
    roadSlopeList = [-3, 0, 3] # % - lateral slope TODO: fill this array with pertinent values
    roadCamberList = [-0.01, 0, 0.01] # m/m TODO: fill this array with pertinent values
    
    for currSetup in product(egoVelocityList, roadRadiusList, roadGradientList, roadSlopeList):
        currEgoVelocity = currSetup[0]
        currRoadRadius = currSetup[1]
        currRoadGradient = currSetup[2]
        currRoadSlope = currSetup[3]
        currName = os.path.splitext(os.path.basename(baseScenarioPathTCE))[0] + "_vEgo{0}_rR{1}_longS{2}_latS{3}".format(currEgoVelocity, currRoadRadius, currRoadGradient, currRoadSlope) + ".testrun"
        testRun = cmTestRunSimple(currName, baseScenarioPathTCE)
        TCETestCase.FactoryTestRunTCE(testRun, vEgo=currEgoVelocity, rRadius=currRoadRadius, rGrad=currRoadGradient, rSlope=currRoadSlope)
        
        outputFolder = os.path.join(APGENERATED_PATH, testRunFolderTCE)
        if not os.path.exists(outputFolder):
            os.makedirs(outputFolder)
        testRun.save(outputFolder)
		
    egoVelocityList = [30, 50] # km/h TODO: fill this array with pertinent values
    
    for currSetup in product(egoVelocityList):
        currEgoVelocity = currSetup[0]
        currName = os.path.splitext(os.path.basename(baseScenarioPathTCECity))[0] + "_vEgo{0}".format(currEgoVelocity) + ".testrun"
        testRun = cmTestRunSimple(currName, baseScenarioPathTCECity)
        TCETestCase.FactoryTestRunTCECity(testRun, vEgo=currEgoVelocity)
        
        outputFolder = os.path.join(APGENERATED_PATH, testRunFolderTCE)
        if not os.path.exists(outputFolder):
            os.makedirs(outputFolder)
        testRun.save(outputFolder)

if __name__ == "__main__":
    try:
        args = sys.argv
        if len(args) == 1:
            print("Starting TestRunGenerator.py")
            GenerateCMTestRun()
            print("Finished TestRunGenerator.py")
        elif args[1] == "TCE":
            print("Starting TestRunGenerator.py for TCE")
            GenerateCMTestRun_TCE()
            print("Finished TestRunGenerator.py for TCE")
        else:
            print("Unknown argument for TestRunGenerator.py")
        
    except Exception as e:
        print (e)
        input("Exception in executing TestRunGenerator.py.\nHit enter To Close...")