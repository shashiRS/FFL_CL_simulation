#********************************************************************
# System Imports
#********************************************************************
import sys
import imp
imp.reload(sys)
# sys.setdefaultencoding('utf8')
import PDWExcelReader
import math
from collections import namedtuple
#import numpy as np
import re
import copy
from pyBase_path_cfg import *

#********************************************************************
# Constants
#********************************************************************
MPS_TO_KPH              = 3.6 # conversion factor from m/s to km/h
KPH_TO_MPS              = 1 / 3.6 # conversion factor from km/h to m/s

# Colors
CM_COLOR_GREY           = [0.74, 0.74, 0.74]
CM_COLOR_WHITE          = [1.0, 1.0, 1.0]
CM_COLOR_RED            = [1.0, 0.0, 0.0]
CM_COLOR_BROWN          = [0.65, 0.165, 0.165]
CM_COLOR_YELLOW         = [1.0, 1.0, 0.0]

# 
CM_ACC_DIST_TH          = 0.5 # percentage of traveled distance with Car.ax >= in Forwards and Backwards maneuvers
CM_MAX_BRK_DIST         = 5 # m TODO: verify maximum distance required to stop vehicle from ~27km/h
CM_LANE_WIDTH           = 3.5 # m TODO: lane width to be read from test catalogue in case it becomes variable parameter
CM_CIRC_OBST_NUM_SEG    = 20 # number of segments considered for a circular obstacle contour (set as divisor of 180°)
CM_HMI_BTN_DURATION     = 0.1 # s duration for maintaining HMI button pressed

# Parking slots
CM_NO_PARK_SLOTS        = 5 # number of available parking slots on each side
CM_PARK_AREA_START_X    = 50 # m longitudinal offset for the start of parking area
CM_PARK_AREA_START_Y    = -3.5 # m lateral offset for the start of parking area
CM_PARK_PERP_H          = 2.75 # m horizontal side of perpendicular parking slot
CM_PARK_PERP_V          = 6 # m vertical side of perpendicular parking slot
CM_PARK_PAR_H           = 6 # m horizontal side of parallel parking slot
CM_PARK_PAR_V           = 2.75 # m vertical side of parallel parking slot
CM_PARK_ANG_H           = 3.89 # m horizontal side of angled parking slot
CM_PARK_ANG_V           = 6.2 # m vertical side of angled parking slot
CM_PARK_ANG_PSI         = 45 # ° tilt of angled parking slot

# CarMaker maneuver commands
ManCmdEnableVEDODO    = "\t# Enable VEDODO" + \
                      "\n\tEval first() ? AP.vedodoActive_nu = ($OdometryActive=1)"
ManCmdConfigureLSCA   = "\t# Configure LSCA" + \
                      "\n\tEval first() ? AP.lscaDisabled_nu = ($LscaDisabled=0)" + \
                      "\n\tEval first() ? AP.lscaBrakeDisabled_nu = ($LscaBrakeDisabled=0)" + \
                      "\n\tEval first() ? AP.lscaSteeringProposalDisabled_nu = ($LscaSteeringDisabled=1)" + \
                      "\n\tEval first() ? AP.lscaVirtualWallDisabled_nu = ($LscaVirtualWallDisabled=1)"
ManCmdEnableSI        = "\t# Enable Scene Interpretation" + \
                      "\n\tEval first() ? AP.sceneInterpretationActive_nu=($SceneInterpretationActive=1)"
ManCmdEnableLimitFOV  = "\t# Enable Field of view limit" + \
                      "\n\tEval first() ? AP.enableLimitFieldOfView_nu = ($LimitFieldOfViewEnabled=1)"
ManCmdEnableLatency   = "\t# Enable Latency effect" + \
                      "\n\tEval first() ? AP.enableLatencyEffect_nu = ($LatencyEffectEnabled=1)" + \
                      "\n\tEval first() ? AP.latencyEffectTime_s = ($LatencyEffectTime_s=0.1)"
ManCmdDisableLatency  = "\t# Disable Latency effect" + \
                      "\n\tEval first() ? AP.enableLatencyEffect_nu = ($LatencyEffectEnabled=0)"
ManCmdEnablePDW       = "\t# Enable Parking Distance Warning" + \
                      "\n\tEval first() ? AP.pdwDisabled_nu=($PdwEnabled=0)"
ManCmdDisablePDW      = "\t# Disable Parking Distance Warning" + \
                      "\n\tEval first() ? AP.pdwDisabled_nu=($PdwDisabled=1)"
ManCmdStopTRTime      = "\t# Stop testrun after xx seconds (default: 200)" + \
                      "\n\tEval (Time > 200) ? ManJump(\"END\")"
ManCmdShiftInR        = "\t# Shift in Reverse" + \
                      "\n\tEval DM.ManTime > 0.001 ? DM.SelectorCtrl = -1"
ManCmdEnableAP        = "\t# HMI User input - Toggle AP active" + \
                      "\n\tEval first() ? AP.hmiOutputPort.userActionHeadUnit_nu = 0" + \
                      "\n\tEval DM.ManTime > 0.2 ? AP.hmiOutputPort.userActionHeadUnit_nu = 28" + \
                      "\n\tEval DM.ManTime > 0.3 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStartSelection  = "\t# HMI User input - Start selection" + \
                      "\n\tEval DM.ManTime > 0.1 ? AP.hmiOutputPort.userActionHeadUnit_nu = 17" + \
                      "\n\tEval DM.ManTime > 0.3 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStartParking    = "\t# HMI User input - Start parking" + \
                      "\n\tEval DM.ManTime > 0.9 ? AP.hmiOutputPort.userActionHeadUnit_nu = 18" + \
                      "\n\tEval DM.ManTime > 1.1 ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
ManCmdStopTRNoSteer   = "\t# Stop testrun if no steering angle was requested in first xx seconds (default: 10)" + \
                      "\n\tEval first() ? Qu::maneuveringFlag_nu = 0" + \
                      "\n\tEval ((abs(AP.steerCtrlRequestPort.steerAngReq_rad) > 0.1) || (abs(Vhcl.v) > 0.01)) ? Qu::maneuveringFlag_nu = 1" + \
                      "\n\tEval (DM.ManTime > 10) & maneuveringFlag_nu==0 ? ManJump(\"END\")"
# Forcing the steering wheel to 0 position may cause issues if it is followed by a curved (non-straight) driving sequence. Currently not the case.
ManCmdClearSteerAng   = "\t# Clear steering angle" + \
                      "\n\tDVAwr DM.Steer.Ang Abs -1 0"

#********************************************************************
# Configuration tables
#********************************************************************
# maneuverParamConfigType is a named tuple containing for each keyword:
# manFunc = name of maneuver fill function
# manNoArgs = expected number of arguments for maneuver
# maneuverFillFunc is a dictionary mapping maneuver keywords to maneuver fill functions
maneuverParamConfigType = namedtuple("maneuverParamConfigType", "manFunc manNoArgs")

maneuverFillFunc = {                           # manFunc                  manNoArgs
    "Forwards"     : maneuverParamConfigType("self.fillManeuverForwards",     2),
    "Backwards"    : maneuverParamConfigType("self.fillManeuverBackwards",    2),
    "Standstill"   : maneuverParamConfigType("self.fillManeuverStandstill",   1),
    "FreeRoll"     : maneuverParamConfigType("self.fillManeuverFreeRoll",     1),
    "StartAP"      : maneuverParamConfigType("self.fillManeuverStartAP",      0),
    "MM_HMI_t"     : maneuverParamConfigType("self.fillManeuverCmdHMITime",   2),
    "MM_HMI_d"     : maneuverParamConfigType("self.fillManeuverCmdHMIDist",   2),
    "MM_Gear"      : maneuverParamConfigType("self.fillManeuverCmdGear",      2),
    "MM_EPB"       : maneuverParamConfigType("self.fillManeuverCmdEPB",       2),
    "MM_Speed_t"   : maneuverParamConfigType("self.fillManeuverCmdSpeedTime", 2),
    "MM_Speed_d"   : maneuverParamConfigType("self.fillManeuverCmdSpeedDist", 2),
    "MM_Failure"   : maneuverParamConfigType("self.fillManeuverCmdFailure",   2)
    }
    
# obstacleParamConfigType is a named tuple containing for each keyword:
# obstFunc = name of obstacle fill function
# obstNoArgs = expected number of arguments for obstacle
# obstacleFillFunc is a dictionary mapping obstacle keywords to obstacle fill functions
obstacleParamConfigType = namedtuple("obstacleParamConfigType", "obstFunc obstNoArgs")

obstacleFillFunc = {
    "Box"       : obstacleParamConfigType("self.fillObstacleBox",           7),
    "Cylinder"  : obstacleParamConfigType("self.fillObstacleCylinder",      5),
    "Vehicle"   : obstacleParamConfigType("self.fillObstacleVehicle",       5),
    "Furniture" : obstacleParamConfigType("self.fillObstacleFurniture",     5)
    }

# roadParamConfigType is a named tuple containing for each keyword:
# roadFile = name of the road file
# latH = size for horizontal side of parking slot (CarMaker longitudinal)
# latV = size for vertical side of parking slot (CarMaker lateral)    
roadParamConfigType = namedtuple("roadParamConfigType", "roadFile latH latV")
    
roadParam = {                                   # roadFile              latH             latV
    "Perpendicular" : roadParamConfigType("PDW_Perpendicular.rd5", CM_PARK_PERP_H,  CM_PARK_PERP_V),
    "Parallel"      : roadParamConfigType("PDW_Parallel.rd5",      CM_PARK_PAR_H,   CM_PARK_PAR_V),
    "Angled"        : roadParamConfigType("PDW_Angled.rd5",        CM_PARK_ANG_H,   CM_PARK_ANG_V),
    "Hill"          : roadParamConfigType("PDW_Hill.rd5",          CM_PARK_PERP_H,  CM_PARK_PERP_V), # started from Perpendicular
    "Downhill"      : roadParamConfigType("PDW_Downhill.rd5",      CM_PARK_PERP_H,  CM_PARK_PERP_V)  # started from Perpendicular
}

refPointsCoordsType = namedtuple("refPointsCoordsType", "x y")


class PDWTestCase():
    def __init__(self, data, xlsReader):
        self.data = data
        self.HMIButtons = xlsReader.HMIButtons
        self.Vehicles = xlsReader.Vehicles
        self.StrFurniture = xlsReader.StrFurniture
        self.Parameters = copy.deepcopy(xlsReader.Parameters) # because we need to alter the parameters dict only for this PDWTestCase, but keep a clean copy in xlsReader
        
        self.refPointsList = ["road"]
        for i in range (1, 2 * CM_NO_PARK_SLOTS + 1):
            self.refPointsList.append("P" + str(i))
            
        self.refPointsCoords = dict.fromkeys(self.refPointsList, refPointsCoordsType(0, 0))
   
    def PDWTestRunFactory(self, myTestRun):
        # TestRun description
        myTestRun.Description.value = self.data["Description"] #+ self.getEvaluationCriteria(myTestRun)
        
        # Overwrite changed parameters
        # self.processChangedParams() #ClPr: disabled for now; will add it later if needed
        
        # Ego vehicle data
        self.fillEgoData(myTestRun)
        
        # Road data
        self.fillRoadData(myTestRun)
        
        # # Parking box
        # # select parking box slot (1 of 5 - "radio button") or select parking box slots (1 to 5 - "checkbox")
        # self.fillParkingBoxData(myTestRun) #TODO: update excel and update script for fake EM generation
        
        # Obstacles
        self.fillObstacleData(myTestRun)
            
        # Maneuvers
        self.fillManeuverData(myTestRun)
    
    def processChangedParams(self):
        for line in self.data["Changed parameters"].splitlines():
            if line in ["None", ""]:
                continue
            line = line.replace(" ","") # remove blanks from cell
            lineSplitted = line.split("=")
            if len(lineSplitted) != 2:
                print("Test scenario {}: Unexpected format for changed parameter {}.".format(self.data["Situation ID"], line))
                sys.exit(0)            
            try:
                name = lineSplitted[0]
                val = float(lineSplitted[1])
            except ValueError:
                print("Test scenario {}: Unexpected value for changed parameter {}.".format(self.data["Situation ID"], line))
                sys.exit(0)
            if name in self.Parameters.keys():
                factor = self.Parameters[name].factor
            else:
                factor = 1
            self.Parameters[name] = PDWExcelReader.pdwParamConfigType(val, factor)
            self.setParamValueInSWBuild(name, lineSplitted[1])
    
    def setParamValueInSWBuild(self, paramName, newValue):
        reParamLine = r"\"" + paramName + "\":\s*(?P<paramVal>[\d.-]*)" # json format for one parameter is "PARAM_NAME": value (create named group for value)
        replaceLine = "\"" + paramName + "\": " + newValue
        newJsonText = ""
        regexp = re.compile(reParamLine)
        for path in MOD_PARAM_FILES_LIST:
            with open(path, 'r+') as jsonFile:
                jsonText = jsonFile.read()
                newJsonText = regexp.sub(replaceLine, jsonText)
                if newJsonText:
                    break
        if newJsonText:
            with open(path, 'w') as jsonFile:
                jsonFile.write(newJsonText)
            #print("Parameter {} was modified in {}".format(paramName, path))
        else:
            print("Parameter {} from Parameters sheet not found in SW build. Please check if name is correct or if list of json files from pyBase_path_cfg.py must be extended.".format(paramName))
            sys.exit(0)

    def fillEgoData(self, myTestRun):
        egoType = "VW_PassatVariant_20TSI_DSG_LLVW_AP_C_GenSteerAng.car"
        myTestRun.Vehicle._srcFile = egoType
        
        # Tires
        myTestRun.Tire._srcFile = "Continental_ContiPremiumContact5_215_55R17_94W_MF-Tyre_62"
        
    def fillRoadData(self, myTestRun):
        cat = self.data["Category"]
        if cat in list(roadParam.keys()):
            myTestRun.Road.Definition.DigRoad = roadParam[cat].roadFile
            if self.data["Start Position"] == "None":
                self.data["Start Position"] = "0"
            myTestRun.Road.VhclStartPos.value = self.data["Start Position"]
                
            if cat in ["Perpendicular", "Hill", "Downhill"]:
                # P1 is set in the middle of the horizontal far side of parking slot
                x_temp = CM_PARK_AREA_START_X + CM_PARK_PERP_H / 2
                y_temp = CM_PARK_AREA_START_Y - CM_PARK_PERP_V
            elif cat == "Parallel":
                # P1 is set in the middle of the vertical near (left in CarMaker) side of parking slot
                x_temp = CM_PARK_AREA_START_X
                y_temp = CM_PARK_AREA_START_Y - CM_PARK_PAR_V / 2
            else:
                # P1 is set in the middle of the horizontal far side of parking slot
                x_temp = CM_PARK_AREA_START_X + CM_PARK_ANG_H / 2 + math.tan(math.radians(CM_PARK_ANG_PSI)) * CM_PARK_ANG_V
                y_temp = CM_PARK_AREA_START_Y - CM_PARK_ANG_V
                
            self.refPointsCoords["P1"] = refPointsCoordsType(x_temp, y_temp)
            
            for i in range(1, 6):
                x_temp = self.refPointsCoords["P1"].x + (i - 1) * roadParam[cat].latH
                y_temp = self.refPointsCoords["P1"].y
                self.refPointsCoords["P" + str(i)] = refPointsCoordsType(x_temp, y_temp)
                x_temp = self.refPointsCoords["P" + str(i)].x
                y_temp = -self.refPointsCoords["P" + str(i)].y
                self.refPointsCoords["P" + str(i + 5)] = refPointsCoordsType(x_temp, y_temp)
        else:
            print("Test scenario {}: Road file not available for category {}.".format(self.data["Situation ID"], cat))
            sys.exit(0)
            
    def fillObstacleData(self, myTestRun):
        if self.data["Obstacles"] != "None":
            index = -1
            reDelimiters = '|'.join(map(re.escape, ["(", ")", ","]))
            reArithmeticOps = '|'.join(map(re.escape, ["+", "-", "*", "/"]))
            
            for line in self.data["Obstacles"].splitlines():
                line = line.replace(" ","") # remove blanks from cell
                if line.count('(') != 1 or line.count(')') != 1 or not line.endswith(')'): # verify if a line is in the correct format Keyword(args)\n
                    print("Test scenario {}: Obstacle syntax error in line {}\nCorrect syntax is ObstacleKeyword(Args)'\\n'".format(self.data["Situation ID"], line))
                    sys.exit(0)
                obstArgs = re.split(reDelimiters,line) # split line with separators: '(', ')', ','
                del obstArgs[-1] # remove last element from list - always '' because last character in line is ')'
                
                if not obstArgs[0] in obstacleFillFunc.keys():
                    print("Test scenario {}: Keyword {} not handled.".format(self.data["Situation ID"], obstArgs[0]))
                    sys.exit(0)
                
                if len(obstArgs[1:]) != obstacleFillFunc[obstArgs[0]].obstNoArgs:
                    print("Test scenario {}: Incorrect number of arguments for obstacle {}".format(self.data["Situation ID"], line))
                    sys.exit(0)
                    
                if not obstArgs[1].upper() in [x.upper() for x in self.refPointsList]:
                    print("Test scenario {}: Incorrect reference point for obstacle {}".format(self.data["Situation ID"], line))
                    sys.exit(0)
                    
                for idx, obstArg in enumerate(obstArgs[2:], start=2): # check if any of the obstacle arguments contains a parameter
                    found = False
                    formulaArgs = re.split(reArithmeticOps, obstArg) # argument could be a formula containing a parameter, need to split by arithmetic operators
                    for arg in formulaArgs:
                        if arg in self.Parameters.keys():
                            val = self.Parameters[arg].value * self.Parameters[arg].factor
                            obstArg = obstArg.replace(arg, str(val)) # replace parameter name with its value
                            found = True
                    if found:
                        obstArgs[idx] = str(eval(obstArg)) # evaluate formula if a parameter was found in the obstacle arguments

                index = eval(obstacleFillFunc[obstArgs[0]].obstFunc)(myTestRun, obstArgs[1:], index) # call corresponding obstacle function indicated by the keyword (obstArgs[0]) with arguments (myTestRun, obstArgs[1:], index)
            
    def getObstStartPos(self, refPoint, longOffset, latOffset):
        if refPoint.lower() == "road":
            return (longOffset, latOffset)
        
        index = int(refPoint[1:])
        sign = 1
        if index > CM_NO_PARK_SLOTS: # lateral offset must be subtracted if parking on the left side of the road
            sign = -1
            
        return(self.refPointsCoords[refPoint].x + longOffset, self.refPointsCoords[refPoint].y + sign * latOffset)
            
    def fillObstacleBox(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Box" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = "Cuboid"
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = " "
        myTestRun.Traffic.Traffic[index].Color.value = CM_COLOR_GREY
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [float(obstArgs[3]), float(obstArgs[4]), float(obstArgs[5])]
        myTestRun.Traffic.Traffic[index].Init.Orientation.value = [0.0, 0.0, float(obstArgs[6])]
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[1]), float(obstArgs[2]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index
        
    def fillObstacleCylinder(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Cyl" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = "Cylinder"
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = " "
        myTestRun.Traffic.Traffic[index].Color.value = CM_COLOR_GREY
        
        contour = ""
        radius = float(obstArgs[3])
        for i in range(0, 181, int(180 / CM_CIRC_OBST_NUM_SEG)): # for (i = 0°; i <= 180°; i+= segmentSize)
            circlePx = round(radius + radius * math.cos(math.radians(i)), 3)
            if i == 180:
                circlePy = 0 # this is added because sin(180°) != 0 in python (e-16 value)
            else:
                circlePy = round(radius * math.sin(math.radians(i)), 3)
            contour = "\t" + str(circlePx) + " " + str(circlePy) + "\n" + contour
        
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [0.01, 0.01, float(obstArgs[4])] # length and width are disregarded because the size is set via countour values; non-zero values must be used to avoid CarMaker error
        myTestRun.Traffic.Traffic[index].Basics.Contour.Contour.value = contour
        myTestRun.Traffic.Traffic[index].Basics.Contour.Mirror.value = 1
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[1]) - radius, float(obstArgs[2]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index
        
    def fillObstacleVehicle(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Vhcl" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = obstArgs[1]
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = self.Vehicles[obstArgs[1]].movieGeometry
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [self.Vehicles[obstArgs[1]].length, self.Vehicles[obstArgs[1]].width, self.Vehicles[obstArgs[1]].height]
        myTestRun.Traffic.Traffic[index].Init.Orientation.value = [0.0, 0.0, float(obstArgs[4])]
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[2]), float(obstArgs[3]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index
        
    def fillObstacleFurniture(self, myTestRun, obstArgs, index):
        index += 1
        myTestRun.Traffic.add()
        myTestRun.Traffic.Traffic[index].Name.value = "Obst" + '{:02d}'.format(index)
        myTestRun.Traffic.Traffic[index].Info.value = obstArgs[1]
        myTestRun.Traffic.Traffic[index].Movie.Geometry.value = self.StrFurniture[obstArgs[1]].movieGeometry
        myTestRun.Traffic.Traffic[index].Basics.Dimension.value = [self.StrFurniture[obstArgs[1]].length, self.StrFurniture[obstArgs[1]].width, self.StrFurniture[obstArgs[1]].height]
        myTestRun.Traffic.Traffic[index].Init.Orientation.value = [0.0, 0.0, float(obstArgs[4])]
        myTestRun.Traffic.Traffic[index].DetectMask.value = "1 1"
        myTestRun.Traffic.Traffic[index].Route.value = "0 1"

        (startX, startY) = self.getObstStartPos(obstArgs[0], float(obstArgs[2]), float(obstArgs[3]))
        myTestRun.Traffic.Traffic[index].Init.Road.value = str(startX) + " " + str(startY + CM_LANE_WIDTH / 2)
        
        return index
    
    def fillManeuverData(self, myTestRun):
        self.fillManeuverInit(myTestRun)

        index = 0
        reDelimiters = '|'.join(map(re.escape, ["(", ")", ","]))
        reArithmeticOps = '|'.join(map(re.escape, ["+", "-", "*", "/"]))
        
        for line in self.data["Maneuvers"].splitlines():
            line = line.replace(" ","") # remove blanks from cell
            if line.count('(') != 1 or line.count(')') != 1 or not line.endswith(')'): # verify if a line is in the correct format Keyword(args)\n
                print("Test scenario {}: Maneuver syntax error in line {}\nCorrect syntax is ManeuverKeyword(Args)'\\n'".format(self.data["Situation ID"], line))
                sys.exit(0)
            manArgs = re.split(reDelimiters,line) # split line with separators: '(', ')', ','
            del manArgs[-1] # remove last element from list - always '' because last character in line is ')'
            if manArgs[1] == '': # for handling the maneuvers with no arguments, another '' element must be removed from the list
                del manArgs[-1]

            if not manArgs[0] in maneuverFillFunc.keys():
                print("Test scenario {}: Keyword {} not handled.".format(self.data["Situation ID"], manArgs[0]))
                sys.exit(0)
            
            if len(manArgs[1:]) != maneuverFillFunc[manArgs[0]].manNoArgs:
                print("Test scenario {}: Incorrect number of arguments for maneuver {}".format(self.data["Situation ID"], line))
                sys.exit(0)

            for idx, manArg in enumerate(manArgs[1:], start=1): # check if any of the manuever arguments contains a parameter
                found = False
                formulaArgs = re.split(reArithmeticOps, manArg) # argument could be a formula containing a parameter, need to split by arithmetic operators
                for arg in formulaArgs:
                    if arg in self.Parameters.keys():
                        val = self.Parameters[arg].value * self.Parameters[arg].factor
                        manArg = manArg.replace(arg, str(val)) # replace parameter name with its value
                        found = True
                if found:
                    manArgs[idx] = str(eval(manArg)) # evaluate formula if a parameter was found in the manuever arguments

            index = eval(maneuverFillFunc[manArgs[0]].manFunc)(myTestRun, manArgs[1:], index) # call corresponding maneuver function indicated by the keyword (manArgs[0]) with arguments (myTestRun, manArgs[1:], index)
            
        self.fillManeuverEnd(myTestRun, index)

    def fillManeuverInit(self, myTestRun):
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[0].Info.value = "Init"
        myTestRun.DrivMan.DrivManList[0].Label.value = ""
        myTestRun.DrivMan.DrivManList[0].TimeLimit.value = 0.5
        myTestRun.DrivMan.DrivManList[0].Cmds.value = ManCmdEnableVEDODO + "\n" + \
                                                      ManCmdConfigureLSCA + "\n" + \
                                                      ManCmdEnableLimitFOV + "\n" + \
                                                      ManCmdEnableLatency + "\n" + \
                                                      ManCmdEnableSI + "\n" + \
                                                      ManCmdEnablePDW + "\n" + \
                                                      "\t\"\"\n" + \
                                                      ManCmdEnableAP + "\n" + \
                                                      "\t\"\"\n" + \
                                                      ManCmdClearSteerAng
        myTestRun.DrivMan.DrivManList[0].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[0].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[0].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[0].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[0].BrakePark.value = "0 0 0.2 abs"
            
    def fillManeuverForwards(self, myTestRun, manArgs, index):
        # Speed Control = VelControl 'Speed' 'Max. Deviation' 'Sensitivity' 'Premature end when final speed is reached' 'Manual Gear Shifting' 'Manumatic'
        # Stop Vehicle = Stop 'Deceleration' 'Stop Distance'
        
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Forwards (Car.ax >= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "VelControl " + manArgs[1] + " 0.0 1.0 0 1 0"
        
        dist = float(manArgs[0])
        if dist <= 2 * CM_MAX_BRK_DIST:
            dist1 = dist * CM_ACC_DIST_TH
            dist2 = dist * (1 - CM_ACC_DIST_TH)
        else:
            dist1 = dist - CM_MAX_BRK_DIST
            dist2 = CM_MAX_BRK_DIST
            
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t# Driving forwards with constant velocity complete" + \
                                                          "\n\tEval first() ? Qu::StartPos = Vhcl.sRoad" + \
                                                          "\n\tEval first() ? Qu::ManDuration = DM.ManTime" + \
                                                          "\n\tEval first() ? Qu::ManDistance = 0" + \
                                                          "\n\tEval first() ? Qu::HMIBtnStart = -1" + \
                                                          "\n\tEval Vhcl.sRoad >= StartPos + " + str(dist1) + " ? (ManDuration = DM.ManTime; ManDistance = Vhcl.sRoad - StartPos; ManJump(\"+1\"))" 
        
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Forwards (Car.ax <= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Stop 0 " + str(dist2)
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t\"\"\n" + \
                                                          "\t# Driving forwards phase complete" + \
                                                          "\n\tEval Car.v < 0.01 ? ManJump(\"+1\")"
        
        return index
    
    def fillManeuverBackwards(self, myTestRun, manArgs, index):
        # Drive backwards = Backward 'Acceleration' 'Speed'
        # Stop Vehicle = Stop 'Deceleration' 'Stop Distance'
        
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Backwards (Car.ax >= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Backward 2 " + manArgs[1]
        
        dist = float(manArgs[0])
        if dist <= 2 * CM_MAX_BRK_DIST:
            dist1 = dist * CM_ACC_DIST_TH
            dist2 = dist * (1 - CM_ACC_DIST_TH)
        else:
            dist1 = dist - CM_MAX_BRK_DIST
            dist2 = CM_MAX_BRK_DIST
        
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t\"\"\n" + \
                                                          "\t# Driving backwards with constant velocity complete" + \
                                                          "\n\tEval first() ? Qu::StartPos = Vhcl.sRoad" + \
                                                          "\n\tEval first() ? Qu::ManDuration = DM.ManTime" + \
                                                          "\n\tEval first() ? Qu::ManDistance = 0" + \
                                                          "\n\tEval first() ? Qu::HMIBtnStart = -1" + \
                                                          "\n\tEval Vhcl.sRoad <= StartPos - " + str(dist1) + " ? (ManDuration = DM.ManTime; ManDistance = StartPos - Vhcl.sRoad; ManJump(\"+1\"))"

        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Backwards (Car.ax <= 0)"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Stop 0 " + str(dist2)
        myTestRun.DrivMan.DrivManList[index].Cmds.value = "\t\"\"\n" + \
                                                          "\t# Driving backwards phase complete" + \
                                                          "\n\tEval Car.v < 0.01 ? ManJump(\"+1\")"
        
        return index
    
    def fillManeuverStandstill(self, myTestRun, manArgs, index):
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Standstill"
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = float(manArgs[0])
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].BrakePark.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdClearSteerAng + "\n" + \
                                                          "\t\"\"\n" + \
                                                          "\tEval first() ? Qu::HMIBtnStart = -1"
        return index

    def fillManeuverFreeRoll(self, myTestRun, manArgs, index):
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "FreeRoll"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdStopTRTime + "\n" + \
                                                          "\t\"\"\n" + \
                                                          "\t# Rolling for a specified distance" + \
                                                          "\n\tEval first() ? Qu::StartPos = Vhcl.sRoad" + \
                                                          "\n\tEval first() ? Qu::HMIBtnStart = -1" + \
                                                          "\n\tEval abs(StartPos - Vhcl.sRoad) >= " + manArgs[0] + " ? (DM.Brake = 0.7; ManJump(\"+1\"))"

        return index

    def fillManeuverStartAP(self, myTestRun, manArgs, index):
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "Start parking"
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 1.4
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Clutch.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Gas.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Brake.value = "1 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].BrakePark.value = "0 0 0.2 abs"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdStopTRTime + "\n" + \
                                                          "\t\"\"" + \
                                                          "\n\tEval first() ? Qu::HMIBtnStart = -1\n" + \
                                                          ManCmdStartSelection + \
                                                          "\t\n" + \
                                                          ManCmdStartParking
        index += 1
        myTestRun.DrivMan.DrivManList.add()
        myTestRun.DrivMan.DrivManList[index].Info.value = "AP control"
        myTestRun.DrivMan.DrivManList[index].EndCondition.value = "AP.headUnitVisualizationPort.screen_nu == 5 || AP.headUnitVisualizationPort.screen_nu == 6"
        myTestRun.DrivMan.DrivManList[index].LongDyn.value = "Manual"
        myTestRun.DrivMan.DrivManList[index].Cmds.value = ManCmdStopTRNoSteer + "\n" + \
                                                          "\t\"\"\n" + \
                                                          ManCmdStopTRTime

        return index

    def fillManeuverCmdHMITime(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value in ["Standstill", "FreeRoll", "AP control"]:
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first(DM.ManTime >= " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                               "\n\tEval ((DM.ManTime >= " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
        else:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\tEval first(DM.ManTime >= " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                                   "\n\tEval ((DM.ManTime >= " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                                   "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                                   "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first((ManDuration <= " + manArgs[0] + ") && (DM.ManTime + ManDuration >= " + manArgs[0] + ")) ? HMIBtnStart = DM.ManTime + ManDuration" + \
                                                               "\n\tEval ((DM.ManTime + ManDuration >= " + manArgs[0] + ") && (DM.ManTime + ManDuration <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime + ManDuration > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"

        return index
        
    def fillManeuverCmdHMIDist(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") != -1:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\tEval first(Vhcl.sRoad >= StartPos + " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                                   "\n\tEval ((Vhcl.sRoad >= StartPos + " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                                   "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                                   "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first((ManDistance <= " + manArgs[0] + ") && (Vhcl.sRoad >= StartPos + " + manArgs[0] + ")) ? HMIBtnStart = DM.ManTime + ManDuration" + \
                                                               "\n\tEval ((Vhcl.sRoad >= StartPos + " + manArgs[0] + ") && (DM.ManTime + ManDuration <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime + ManDuration > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
        elif myTestRun.DrivMan.DrivManList[index].Info.value.find("Backwards") != -1:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\tEval first(Vhcl.sRoad <= StartPos - " + manArgs[0] + ") ? HMIBtnStart = DM.ManTime" + \
                                                                   "\n\tEval ((Vhcl.sRoad <= StartPos - " + manArgs[0] + ") && (DM.ManTime <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                                   "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                                   "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\tEval first((ManDistance <= " + manArgs[0] + ") && (Vhcl.sRoad <= StartPos - " + manArgs[0] + ")) ? HMIBtnStart = DM.ManTime + ManDuration" + \
                                                               "\n\tEval ((Vhcl.sRoad <= StartPos - " + manArgs[0] + ") && (DM.ManTime + ManDuration <= HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? AP.hmiOutputPort.userActionHeadUnit_nu = ($" + self.HMIButtons[manArgs[1]] + ")" + \
                                                               "\n\tEval first((HMIBtnStart >= 0) && (DM.ManTime + ManDuration > HMIBtnStart + " + str(CM_HMI_BTN_DURATION) + ")) ? HMIBtnStart =  -1" + \
                                                               "\n\tEval (HMIBtnStart == -1) ? AP.hmiOutputPort.userActionHeadUnit_nu = 0"
        else:
            print("Test scenario {}: MM_HMI_d not supported in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)

        return index
    
    def fillManeuverCmdGear(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value not in ["Standstill", "FreeRoll"]:
            print("Test scenario {}: MM_Gear not supported in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)
        if manArgs[1].upper() == "P":
            if myTestRun.DrivMan.DrivManList[index].Info.value.find("FreeRoll") != -1:
                print("Test scenario {}: MM_Gear(P) not possible in FreeRoll maneuver. Make a Standstill maneuver with MM_Gear(P) instead.".format(self.data["Situation ID"]))
                sys.exit(0)
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Park" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = -9"
        elif manArgs[1].upper() == "R":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Reverse" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = -1"
        elif manArgs[1].upper() == "N":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Neutral" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = 0"
        elif manArgs[1].upper() == "D":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# Shift gear in Drive" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.SelectorCtrl = 1"
        else:
            print("Test scenario {}: Gear {} not handled.".format(self.data["Situation ID"], manArgs[1]))
            sys.exit(0)
        
        return index
    
    def fillManeuverCmdEPB(self, myTestRun, manArgs, index):
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Standstill") == -1:
            print("Test scenario {}: MM_EPB not supported in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)

        if manArgs[1].upper() == "ON":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# EPB ON" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.BrakePark = 1"
        elif manArgs[1].upper() == "OFF":
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# EPB OFF" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.BrakePark = 0"
        else:
            print("Test scenario {}: EPB state {} is invalid.".format(self.data["Situation ID"], manArgs[1]))
            sys.exit(0)
        
        return index
        
    def fillManeuverCmdSpeedTime(self, myTestRun, manArgs, index):
        if (myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") == -1) and (myTestRun.DrivMan.DrivManList[index].Info.value.find("Backwards") == -1):
            print("Test scenario {}: MM_Speed_t not relevant in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)

        myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\t\"\"" + \
                                                               "\n\t# Change constant velocity threshold" + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? DM.v.Trgt = " + str(round(float(manArgs[1]) * KPH_TO_MPS, 3))
        
        return index
        
    def fillManeuverCmdSpeedDist(self, myTestRun, manArgs, index):
        if (myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") == -1) and (myTestRun.DrivMan.DrivManList[index].Info.value.find("Backwards") == -1):
            print("Test scenario {}: MM_Speed_d not relevant in {} maneuver.".format(self.data["Situation ID"], myTestRun.DrivMan.DrivManList[index].Info.value))
            sys.exit(0)
            
        if myTestRun.DrivMan.DrivManList[index].Info.value.find("Forwards") != -1:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\t\"\"" + \
                                                                   "\n\t# Change constant velocity threshold" + \
                                                                   "\n\tEval Vhcl.sRoad >= StartPos + " + manArgs[0] + " ? DM.v.Trgt = " + str(round(float(manArgs[1]) * KPH_TO_MPS, 3))
        else:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\t\"\"" + \
                                                                   "\n\t# Change constant velocity threshold" + \
                                                                   "\n\tEval Vhcl.sRoad <= StartPos - " + manArgs[0] + " ? DM.v.Trgt = " + str(round(float(manArgs[1]) * KPH_TO_MPS, 3))    
        
        return index

    def fillManeuverCmdFailure(self, myTestRun, manArgs, index):
        if manArgs[1].upper() == "ON":
            state = 1
        elif manArgs[1].upper() == "OFF":
            state = 0
        else:
            print("Test scenario {}: PDW Failure state {} is invalid.".format(self.data["Situation ID"], manArgs[1]))
            sys.exit(0)

        if myTestRun.DrivMan.DrivManList[index].Info.value in ["Standstill", "FreeRoll", "AP control"]:
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# PDW failure " + manArgs[1].upper() + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? AP.pdwFailure_nu=" + str(state)
        else:
            myTestRun.DrivMan.DrivManList[index - 1].Cmds.value += "\n\t# PDW failure " + manArgs[1].upper() + \
                                                               "\n\tEval DM.ManTime >= " + manArgs[0] + " ? AP.pdwFailure_nu=" + str(state)
            myTestRun.DrivMan.DrivManList[index].Cmds.value += "\n\t# PDW failure " + manArgs[1].upper() + \
                                                               "\n\tEval ((ManDuration < " + manArgs[0] + ") && ((DM.ManTime + ManDuration) >= " + manArgs[0] +")) ? AP.pdwFailure_nu=" + str(state)
        return index

    def fillManeuverEnd(self, myTestRun, index):
        index += 1
        myTestRun.DrivMan.DrivManList[index].Info.value = "End testrun"
        myTestRun.DrivMan.DrivManList[index].Label.value = "END"
        myTestRun.DrivMan.DrivManList[index].TimeLimit.value = 0.1
